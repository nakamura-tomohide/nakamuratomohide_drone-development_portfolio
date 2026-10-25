#include "Phase/VisionNavigate/VisionNavigate.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>

// コンストラクタ
VisionNavigate::VisionNavigate(const Config& config)
    : config_(config) {}

void VisionNavigate::start_search(float north_m, float east_m, float down_m) {
    // タイムアウト計測はフェーズ全体で1回だけ（ゴール見失いによる再捜索のたびに
    // リセットすると、見失い→再発見を繰り返すだけでいつまでもタイムアウトしなくなるため）。
    if (!timeout_timer_started_) {
        search_start_time_    = std::chrono::steady_clock::now();
        timeout_timer_started_ = true;
    }

    center_north_m_  = north_m;
    center_east_m_   = east_m;
    // 高度はAGLアンカー(vert_carrot_down_m_)側で一貫管理する。ここで生のdown_mを
    // 起点高度としてラッチし直すと、EKF(down_m)がドリフトしていた場合に「見失う→
    // 再捜索」のたびにドリフトした高度を新しい維持目標として承認してしまい、
    // 実高度がラチェット式にずれていく（2026-08-30フライトで確認）。
    search_lobe_     = SearchLobe::Right;
    orbit_point_     = 0;
    search_radius_m_ = config_.search_radius_initial_m;
    wp_wait_elapsed_s_ = 0.0f;
    miss_elapsed_s_    = 0.0f;
    low_alt_elapsed_s_ = 0.0f;
    off_center_elapsed_s_ = 0.0f;
    few_goals_hold_    = false;
    few_goals_elapsed_s_ = 0.0f;
    nav_anchor_valid_ = false; // 次にImageNavへ入った位置で取り直す
    offset_filter_initialized_ = false; // 再捜索・再発見時はフィルタも作り直す（古い値との平均で反応が遅れないように）
    target_locked_   = false; // 再捜索なので目標地点もリセットし、次に捉えた時点で取り直す

    status_.mode                    = Mode::Search;
    status_.search_radius_m         = search_radius_m_;
    status_.lobe                    = search_lobe_;
    status_.target_position_locked  = false;
    status_.distance_to_target_m    = -1.0f;
    status_.within_target_radius    = false;

    // フレーム取得が一度も成功しない場合でも安全なように、開始位置でホバリングする
    // 指令を初期値にしておく。これが無いとPositionVelocityNedCmd{}のデフォルト値
    // (NED原点(0,0,0)へ速度0で向かう指令)が送られ続け、原点に向かって水平移動・
    // 降下してしまう（起点が原点から離れていれば水平に暴走、down_m=0のため
    // 高度によらず地面付近まで降下しようとする）重大な安全バグになる。
    status_.cmd = PositionVelocityNedCmd{};
    status_.cmd.north_m = north_m;
    status_.cmd.east_m  = east_m;
    write_vertical_cmd(down_m); // AGLアンカー初期化前は生down_m、初期化後はcarrotを維持

    search_started_ = true;
}

void VisionNavigate::update(const cv::Mat& frame, float north_m, float east_m, float down_m, float altitude_m, bool altitude_valid,
                            float yaw_deg, double lat_deg, double lon_deg) {
    if (!search_started_) {
        std::cerr << "VisionNavigate: Search not started. Call start_search() first." << std::endl;
        return;
    }
    if (status_.finished) {
        return; // 完了後はホバリング指令を維持
    }

    // 強制着陸中は他の判定より優先し、GPSゴール座標へ向かうことだけを行う
    if (forced_land_active_) {
        update_forced_land(north_m, east_m, down_m);
        return;
    }

    // 捜索開始からsearch_timeout_sを超えても完了しない場合、ゴール捜索を諦めて
    // GPSゴール座標(goal_lat_deg, goal_lon_deg)へ強制的に向かう（捜索半径に上限が
    // 無いため、ゴールを検出できないまま延々と捜索範囲が広がり続けるのを防ぐ）。
    if (timeout_timer_started_) {
        const float elapsed_s = std::chrono::duration<float>(
            std::chrono::steady_clock::now() - search_start_time_).count();
        if (elapsed_s >= config_.search_timeout_s) {
            std::cout << "VisionNavigate: 捜索が" << config_.search_timeout_s
                      << "秒を超えました。GPSゴール座標へ強制着陸します。" << std::endl;
            forced_land_active_ = true;
            status_.mode         = Mode::ForcedLand;

            // GPSNavigate::goal_to_NED()と同じ手法: home基準ではなく、今この瞬間の
            // 自機のGPS座標・NED座標を対応点(アンカー)としてゴール座標をNEDに変換する
            // （PX4が内部的に使うNED原点とhomeが厳密には一致しない問題を回避するため）。
            const double anchor_lat_rad = lat_deg * M_PI / 180.0;
            const double delta_lat_rad  = (config_.goal_lat_deg - lat_deg) * M_PI / 180.0;
            const double delta_lon_rad  = (config_.goal_lon_deg - lon_deg) * M_PI / 180.0;
            constexpr double EARTH_RADIUS_M = 6378137.0;
            forced_land_north_m_ = north_m + static_cast<float>(delta_lat_rad * EARTH_RADIUS_M);
            forced_land_east_m_  = east_m  + static_cast<float>(delta_lon_rad * EARTH_RADIUS_M * std::cos(anchor_lat_rad));

            update_forced_land(north_m, east_m, down_m);
            return;
        }
    }

    // 鉛直制御(AGLアンカー): 「今のdown_mを基準に、実測AGLがtarget_agl_m_になるdown」を
    // 毎tick計算し直してcarrot(vert_carrot_down_m_)を更新する。撮影失敗・単発検出漏れで
    // このupdate()が途中returnするtickでも、鉛直setpointだけは新しいcarrotで送り続けたい
    // ので、モード分岐より前のここで確定させておく（そうしたtickでは水平は従来通り前回
    // 指令を維持）。EKF(down_m)のドリフトはupdate_vertical_target()内で吸収する。
    const auto now = std::chrono::steady_clock::now();
    dt_s_ = last_update_.has_value()
              ? std::clamp(std::chrono::duration<float>(now - *last_update_).count(), 0.0f, 1.0f)
              : 0.2f; // 初回はINTERVAL_MS(200ms)相当を仮定
    last_update_ = now;
    update_vertical_target(down_m, altitude_m, altitude_valid, dt_s_);
    write_vertical_cmd(down_m);

    // 降下完了判定は、このtickの検出結果が出揃ってから update_image_nav() の末尾で行う
    // （高度が低いだけでなく「ゴールが十分な数見えていて、かつ機体直下に来ている」ことも
    // Nフレーム連続で満たすまで確定しない。詳細は update_image_nav() のコメント参照）。

    // 画像が無いtickでは判断を変えず、前回の指令を維持する
    // （撮影失敗を「ゴール消失」と誤認して捜索に戻らないため）
    if (frame.empty()) {
        return;
    }

    auto candidates = detect_goals(frame, altitude_m);
    status_.goal_visible = !candidates.empty();
    status_.goal_count   = static_cast<int>(candidates.size());

    // デバッグ可視化用に楕円だけ抜き出しておく（GoalCandidate自体はprivateなので）
    status_.debug_goal_ellipses.clear();
    for (const auto& cand : candidates) {
        status_.debug_goal_ellipses.push_back(cand.ellipse);
    }

    // モード遷移。画像航法へ入るのは重心が信用できる個数(min_goals_full以上)を
    // 検出できたときだけ。1〜2個だけの検出（誤検出や部分視野）で画像航法へ移ると、
    // 端に偏った重心を追ってパターンから外れる。足りない間は捜索(八の字)を続ける。
    if (status_.mode == Mode::Search && static_cast<int>(candidates.size()) >= config_.min_goals_full) {
        status_.mode = Mode::ImageNav;
        miss_elapsed_s_ = 0.0f;
        few_goals_elapsed_s_ = 0.0f;
        // 水平setpointのアンカーをこの位置でラッチ。以降は指令速度を積分して進める
        // （生の現在位置に毎tick足す方式だと、テレメトリ遅延時に目標が機体を追って滑る）。
        nav_anchor_north_m_ = north_m;
        nav_anchor_east_m_  = east_m;
        nav_anchor_valid_   = true;
        std::cout << "VisionNavigate: ゴール" << candidates.size() << "個発見。画像航法に切替。" << std::endl;
    } else if (status_.mode == Mode::ImageNav) {
        if (candidates.empty()) {
            miss_elapsed_s_ += dt_s_;
            if (miss_elapsed_s_ >= config_.lost_goal_debounce_s) {
                // 既定秒数連続で見失った → 現在位置を新しい起点として捜索をやり直す
                std::cout << "VisionNavigate: ゴールを見失いました。捜索に戻ります。" << std::endl;
                start_search(north_m, east_m, down_m);
            } else {
                // 単発の検出漏れは無視し、前回の指令を維持したままこのtickは終える
                // （update_image_navをcandidates空で呼ぶと重心計算がゼロ除算になるため）
                return;
            }
        } else {
            miss_elapsed_s_ = 0.0f;
        }
    }

    if (status_.mode == Mode::Search) {
        update_search(north_m, east_m, down_m);
    } else {
        update_image_nav(frame, candidates, north_m, east_m, down_m, yaw_deg, altitude_m, altitude_valid);
    }
}

// ---- 捜索（拡大する八の字: 右→左→前→後）----
// 各ロブは中心を起点からd_north/d_east方向にsearch_radius_m_だけずらした円で、
// 起点がちょうど円周上に来るようtheta_startを選んである。
void VisionNavigate::lobe_offsets(SearchLobe lobe, float& d_north, float& d_east, float& theta_start) const {
    const float half_pi = static_cast<float>(M_PI) / 2.0f;
    switch (lobe) {
        case SearchLobe::Right: d_north = 0.0f;  d_east = 1.0f;  theta_start = -half_pi; return;
        case SearchLobe::Left:  d_north = 0.0f;  d_east = -1.0f; theta_start = half_pi;  return;
        case SearchLobe::Front: d_north = 1.0f;  d_east = 0.0f;  theta_start = static_cast<float>(M_PI); return;
        case SearchLobe::Back:  d_north = -1.0f; d_east = 0.0f;  theta_start = 0.0f;     return;
    }
    d_north = d_east = theta_start = 0.0f;
}

VisionNavigate::SearchLobe VisionNavigate::next_lobe(SearchLobe lobe) {
    switch (lobe) {
        case SearchLobe::Right: return SearchLobe::Left;
        case SearchLobe::Left:  return SearchLobe::Front;
        case SearchLobe::Front: return SearchLobe::Back;
        case SearchLobe::Back:  return SearchLobe::Right;
    }
    return SearchLobe::Right;
}

void VisionNavigate::search_waypoint(float& wp_north, float& wp_east) const {
    float d_north, d_east, theta_start;
    lobe_offsets(search_lobe_, d_north, d_east, theta_start);
    float angle = theta_start
                  + 2.0f * static_cast<float>(M_PI) * orbit_point_ / config_.search_n_points;
    wp_north = center_north_m_ + d_north * search_radius_m_ + search_radius_m_ * std::cos(angle);
    wp_east  = center_east_m_  + d_east  * search_radius_m_ + search_radius_m_ * std::sin(angle);
}

void VisionNavigate::update_search(float north_m, float east_m, float down_m) {
    float wp_north, wp_east;
    search_waypoint(wp_north, wp_east);

    // 現在のウェイポイントに十分近づいたら次のポイントへ進める。
    // 位置制御の精度が足りず到達判定が出ないまま張り付き続けることがあるため、
    // 一定秒数(search_wp_timeout_s)待っても到達しなければ強制的に進める
    // （じっと停止したまま捜索が全く進行しなくなるのを防ぐ）。
    wp_wait_elapsed_s_ += dt_s_;
    const bool arrived_wp = std::hypot(wp_north - north_m, wp_east - east_m) <= config_.search_wp_arrival_thresh_m;
    const bool timed_out  = wp_wait_elapsed_s_ >= config_.search_wp_timeout_s;
    if (arrived_wp || timed_out) {
        orbit_point_++;
        wp_wait_elapsed_s_ = 0.0f;
        if (orbit_point_ >= config_.search_n_points) {
            orbit_point_ = 0;
            bool cycle_done = (search_lobe_ == SearchLobe::Back);
            search_lobe_ = next_lobe(search_lobe_);
            status_.lobe = search_lobe_;
            if (cycle_done) {
                // 右→左→前→後の1サイクル完了 → 次サイクルは半径を広げる
                search_radius_m_ += config_.search_radius_step_m;
                status_.search_radius_m = search_radius_m_;
            }
        }
        search_waypoint(wp_north, wp_east);
    }

    status_.cmd = PositionVelocityNedCmd{};
    status_.cmd.north_m = wp_north;
    status_.cmd.east_m  = wp_east;
    write_vertical_cmd(down_m);           // 高度はAGLアンカーのcarrotで維持（EKFドリフト非依存）
    status_.cmd.yaw_deg = 0.0f;           // 画像上端=北の前提を保つためヨーは常に0
}

// ---- 画像航法 ----
void VisionNavigate::update_image_nav(const cv::Mat& frame, const std::vector<GoalCandidate>& candidates,
                                      float north_m, float east_m, float down_m, float yaw_deg,
                                      float altitude_m, bool altitude_valid) {
    // 画像中心はフレームごとに実サイズから計算する
    const float img_cx = frame.cols / 2.0f;
    const float img_cy = frame.rows / 2.0f;

    // 全ゴールの重心の平均を制御目標にする
    cv::Point2f group_center(0.0f, 0.0f);
    for (const auto& cand : candidates)
        group_center += cand.centroid;
    group_center *= 1.0f / static_cast<float>(candidates.size());

    // 真下カメラ・ヨー0前提: 画像上端=機体前方(北)
    // ox > 0(右) → 東に移動、oy > 0(下) → 南に移動、が基準の取り付けの場合の対応。
    const float raw_ox = group_center.x - img_cx;
    const float raw_oy = group_center.y - img_cy;

    // 検出個数が min_goals_full 未満のtickは重心を制御に使わない。「検出できたボール
    // だけの平均」はパターン中心ではなく端に偏り、そのまま降下するとFOVが縮んで遠い側の
    // ボールが先に切れ、さらに端へ寄る正のフィードバックで中心から外れる（実機で確認）。
    // 未満の間は追わずにその場ホバリングで待ち、降下も完了判定も止める。EMAフィルタも
    // 更新しない（部分重心で filtered_ox_/oy_ を汚さない）。
    const int  goal_count = static_cast<int>(candidates.size());
    const bool enough_goals = goal_count >= config_.min_goals_full;
    // 検出数が一時的に落ちても、直近まで十分見えていてキャッシュ済みの重心があるなら、
    // few_goals_grace_s 秒だけは前回の重心(filtered_ox_/oy_)で制御を続ける
    // （一瞬の検出漏れでホバリング/捜索へ転げ落ちてドリフト・振れを起こさない）。
    // 猶予は enough_goals での復帰時のみ再充電する（下で few_goals_elapsed_s_ = 0）。
    const bool use_cached_offset = !enough_goals && offset_filter_initialized_
                                   && few_goals_elapsed_s_ < config_.few_goals_grace_s;
    if (enough_goals) {
        few_goals_elapsed_s_ = 0.0f;
    } else if (use_cached_offset) {
        few_goals_elapsed_s_ += dt_s_; // filtered_ox_/oy_ は据え置きで下の制御へそのまま進む
    } else {
        // 猶予も尽きた（or まだ一度も十分に見えていない）→ その場でホバリング待機。
        status_.offset_x_px = raw_ox; // 監視用（この値は制御に使っていない）
        status_.offset_y_px = raw_oy;
        low_alt_elapsed_s_ = 0.0f;    // 完了デバウンスは振り出しに戻す
        off_center_elapsed_s_ = 0.0f;
        if (!few_goals_hold_) {
            few_goals_hold_ = true;
            // 待機に入った瞬間の位置をラッチし、待機中はこの固定点を出し続ける
            // （毎tick生の現在位置を代入すると、テレメトリ遅延で目標が機体を追って滑る）。
            hold_north_m_ = nav_anchor_valid_ ? nav_anchor_north_m_ : north_m;
            hold_east_m_  = nav_anchor_valid_ ? nav_anchor_east_m_  : east_m;
            std::cout << "VisionNavigate: 検出" << goal_count << "個 (<" << config_.min_goals_full
                      << ") が続くため重心を追わずホバリングで待機。" << std::endl;
        }
        status_.cmd = PositionVelocityNedCmd{};
        status_.cmd.north_m = hold_north_m_;
        status_.cmd.east_m  = hold_east_m_;
        status_.cmd.yaw_deg = 0.0f;
        write_vertical_cmd(down_m);   // 高度はAGLアンカーcarrotで維持
        return;
    }
    if (few_goals_hold_ && enough_goals) {
        few_goals_hold_ = false;
        std::cout << "VisionNavigate: 検出" << goal_count << "個。画像航法を再開。" << std::endl;
    }

    // 検出ノイズ（照明・部分遮蔽・誤検出によるox,oyの飛び）を平滑化するEMAフィルタ。
    // これを掛けずに毎ティック生値をそのままsetpoint計算に使うと、速度指令が
    // 暴れて高度方向にも振動が漏れる（水平にふらつく→機体が傾く→鉛直推力成分が
    // 減って高度も一緒に揺れる）。
    if (!offset_filter_initialized_) {
        filtered_ox_ = raw_ox;
        filtered_oy_ = raw_oy;
        offset_filter_initialized_ = true;
    } else if (enough_goals) {
        filtered_ox_ += config_.offset_smoothing_alpha * (raw_ox - filtered_ox_);
        filtered_oy_ += config_.offset_smoothing_alpha * (raw_oy - filtered_oy_);
    }
    // 猶予中(use_cached_offset)は filtered_ox_/oy_ を据え置き、前回の重心で制御を続ける
    // （部分重心で filtered_* を汚さない）。
    const float ox = filtered_ox_;
    const float oy = filtered_oy_;
    status_.offset_x_px = ox; // テレメトリ用。以降の制御はここから実距離換算した値を使う
    status_.offset_y_px = oy;

    // ピクセルオフセットをLiDAR高度(altitude_m)を使って実際の水平距離[m]に変換する。
    // ピンホールカメラモデル: 焦点距離[px] = (画像幅[px]/2) / tan(水平画角/2)。
    // 高度h、焦点距離fのとき、中心からoxピクセルズレた点の実距離は h * (ox/f)。
    // 同じピクセルオフセットでも高度によって実際のズレ量は変わるため、これをせずに
    // ピクセル値へ直接ゲインを掛けると、高度が高いほど反応が弱すぎ、低いほど過敏になる
    // （高度依存の不安定さの一因になっていた）。
    const float focal_len_px  = focal_length_px(frame.cols);
    const float alt_for_scale = std::max(altitude_m, 0.1f); // 地面すれすれでスケールが暴れないよう下限を設ける
    const float ox_m = alt_for_scale * ox / focal_len_px;
    const float oy_m = alt_for_scale * oy / focal_len_px;

    // 目標地点(NED)の記録: ゴールが画像中心付近(デッドバンド内)に来た最初のタイミングで、
    // その時の機体位置を「目標地点」として1回だけ記録する。以降はここからの水平距離を
    // 都度計算してStatusに出す（テレメトリ表示用。制御ロジックへは影響しない）。
    if (!target_locked_ && std::hypot(ox_m, oy_m) <= config_.center_dead_m) {
        target_locked_  = true;
        target_north_m_ = north_m;
        target_east_m_  = east_m;
        status_.target_position_locked = true;
    }
    if (target_locked_) {
        status_.distance_to_target_m = std::hypot(north_m - target_north_m_, east_m - target_east_m_);
        status_.within_target_radius = status_.distance_to_target_m <= config_.target_arrival_thresh_m;
    }

    // 画像のx軸(右方向)が実際にコンパス方位でどちらを向いているかは、
    // 「カメラの機体取り付けズレ(camera_mount_rotation_deg)」と「機体の実ヨー角(yaw_deg)」の
    // 両方で決まる。以前はヨーが常に0（コマンド通りに収束済み）という前提で
    // camera_mount_rotation_deg分だけ回転させていたが、実際にはVisionNavigate切替直後など
    // GPSNavigateからの回頭がまだ完了していない期間はヨーが0から大きくズレていることがあり
    // （実測で60〜70度以上ズレていたケースを確認）、その間は補正方向が丸ごと間違った角度で
    // 計算されてしまっていた。そこで実測のyaw_degも合算した角度で回転させる。
    // 導出: カメラを上空から見て時計回りにθ(=camera_mount_rotation_deg+yaw_deg)回すと、
    // センサのx軸(画像右方向)はコンパス方位で(東+θ)、y軸(画像下方向)は(南+θ)を向く。これを
    // sin/cosで東西南北成分に分解し、元の「v_east=f(ox), v_north=-f(oy)」の形に合わせて整理すると、
    // 単純に(ox,oy)を数式 x'=x cosθ - y sinθ, y'=x sinθ + y cosθ で回転させるのと一致する
    // （画像はy軸下向きだが、この式をそのまま適用すればよい。符号を導出済みなので変更不要。
    // メートル換算後の値でも同じ回転行列がそのまま使える）。
    float rox_m = ox_m, roy_m = oy_m;
    const float theta_deg = config_.camera_mount_rotation_deg + yaw_deg;
    if (theta_deg != 0.0f) {
        const float theta = theta_deg * static_cast<float>(M_PI) / 180.0f;
        const float c = std::cos(theta), s = std::sin(theta);
        rox_m = ox_m * c - oy_m * s;
        roy_m = ox_m * s + oy_m * c;
    }

    const float v_east  = offset_to_velocity(rox_m);
    const float v_north = -offset_to_velocity(roy_m);

    // 水平オフセットが大きいほど降下速度を落とす。ゴールを大きく外れたまま降下すると
    // 狙った着地点から外れたり障害物に近づくリスクが上がるため、center_dead_m以内なら
    // 全速、descend_offset_max_m以上ズレていたら降下速度0（高度維持のみ）で水平補正を
    // 優先し、その間は線形に減速する。
    const float offset_mag_m = std::hypot(ox_m, oy_m);
    float descend_scale = 1.0f;
    if (offset_mag_m > config_.center_dead_m) {
        if (offset_mag_m >= config_.descend_offset_max_m) {
            descend_scale = 0.0f;
        } else {
            descend_scale = 1.0f - (offset_mag_m - config_.center_dead_m)
                            / (config_.descend_offset_max_m - config_.center_dead_m);
        }
    }
    const float descend_v = config_.descend_v_m_s * descend_scale;

    // 水平setpoint: 生の現在位置に毎tick足すのではなく、ImageNav開始位置でラッチした
    // アンカー(nav_anchor_*)へ指令速度を積分して進める。テレメトリ遅延で現在位置が
    // 数tick凍結・ジャンプしても、setpointは指令速度ぶんしか動かないのでドリフト・
    // 振れが乗りにくい（旧: north_m + v*1.0秒 だと、遅延した現在地に毎tick足すため
    // 目標が機体を追って滑り、SITLで水平ドリフト・発散振動を起こしていた）。
    // アンカーが実測位置からnav_leash_m以上離れたら引き戻す（機体が指令に追随できて
    // いないときの積分ワインドアップ防止）。
    if (!nav_anchor_valid_) { // 安全策（通常はSearch→ImageNav遷移時にラッチ済み）
        nav_anchor_north_m_ = north_m;
        nav_anchor_east_m_  = east_m;
        nav_anchor_valid_   = true;
    }
    nav_anchor_north_m_ += v_north * dt_s_;
    nav_anchor_east_m_  += v_east  * dt_s_;
    nav_anchor_north_m_ = std::clamp(nav_anchor_north_m_,
                                     north_m - config_.nav_leash_m, north_m + config_.nav_leash_m);
    nav_anchor_east_m_  = std::clamp(nav_anchor_east_m_,
                                     east_m  - config_.nav_leash_m, east_m  + config_.nav_leash_m);
    status_.cmd.north_m   = nav_anchor_north_m_;
    status_.cmd.east_m    = nav_anchor_east_m_;
    // 速度は送らない（GPSNavigate::update()と同じ方針）。PX4のPositionControlは
    // 位置setpointと速度setpointを同時に送ると、位置誤差から計算した速度を
    // 送った速度に単純加算する（PositionControl.cpp:130-138、
    // vel_sp_position = (pos_sp - pos)*gain_pos_p を addIfNotNanVector3f で
    // _vel_spに加算）。nav_anchorはv_north/v_eastをdt積分して毎tick前進させて
    // おり、位置だけで意図した速度をPX4自身の位置制御ループ経由で実現できる
    // ため、ここでv_north/v_eastをさらに速度setpointとして送ると二重に加算され、
    // 実速度が意図の2倍以上(実測max 0.7-0.8m/s、上限0.3m/sに対して)に膨らみ、
    // 約7.5秒周期の持続振動を起こしていた（2026-09-16 SITL試験で確認）。
    status_.cmd.north_m_s = 0.0f;
    status_.cmd.east_m_s  = 0.0f;
    status_.cmd.yaw_deg   = 0.0f;
    // 鉛直: 生のdown_mへ直接descendを乗せず、維持目標の対地高度(target_agl_m_)を
    // descend_vぶん下げるだけにする。実際のsetpointはupdate()冒頭で更新済みの
    // AGLアンカーcarrot(vert_carrot_down_m_)で、この降下分は次tickのcarrotへ反映される
    // （200ms・descend_v≦0.2m/sなら遅れは数cmで無視できる）。EKF(down_m)がドリフトしても
    // 対地高度基準で降下できる。
    write_vertical_cmd(down_m);
    descend_target_agl(descend_v, dt_s_);

    // ---- 降下完了判定 ----
    // 「規定高度まで下がった」だけでなく「ゴール重心が機体直下(finish_center_m以内)に
    // 来ている」ことも hover_alt_debounce_s 秒連続で満たしたときだけ完了とする。
    // ここに来ている時点で enough_goals（十分な数を検出）は満たしている。高度だけで
    // 完了させると、横にズレたまま Land へ引き渡してしまう（2026-08-30フライトで
    // ゴール中心から約1.3mズレて着地）。altitude_valid=false のtickはカウンタを
    // 進めも戻しもしない（LiDAR不調中の誤確定・誤リセットを防ぐ）。
    //
    // 高度は生の altitude_m ではなく、AGLアンカーのレート制限済み状態から復元した
    // 「フィルタ済みAGL」で見る。生 altitude_m は無フィルタで、TFmini-Sの単発異常値
    // (0.3m等)がそのまま入る（2026-08-30フライトでは 0.34→2.92→1.44m の3連発が
    // hover_alt_m=3.0のデバウンスを通過して早期完了した）。agl_filtered_m は
    // vert_target_down_m_ 経由で alt_max_rate_m_s / vert_speed_m_s のレート制限が
    // 掛かっているため、1tickで alt_max_rate_m_s*dt 以上は動かず、ノイズ列では
    // hover_alt_m を割り込めない。定常状態では真のAGLに一致する
    // （導出: vert_target_down_m_ ≒ down_m - (target_agl_m_ - agl) より
    //  agl ≒ target_agl_m_ + (vert_carrot_down_m_ - down_m)）。
    //
    // 判定は「hover_alt_m 以下」ではなく「hover_alt_m + finish_alt_tol_m 以下」。
    // 降下は設計上ちょうど hover_alt_m に収束するため、そこで agl_filtered_m が
    // hover_alt_m を微妙に跨いで揺れ、"Nフレーム連続で <= hover_alt_m" が永久に
    // 成立せず search_timeout_s まで完了しない事象が出た（2026-09-01 SITL）。
    // 収束先付近に幅を持たせて、hover_alt_m 付近に落ち着いたら確実に完了させる。
    const bool centered = offset_mag_m <= config_.finish_center_m;
    const float agl_filtered_m = target_agl_m_ + (vert_carrot_down_m_ - down_m);
    const bool low_alt = vert_initialized_
                         && agl_filtered_m <= config_.hover_alt_m + config_.finish_alt_tol_m;
    // センタリングが一瞬(finish_center_grace_s以内)外れただけでlow_alt_elapsed_s_を
    // 全リセットすると、水平方向がハンチングしている間ずっとカウンタが振り出しに
    // 戻り続け、完了までに実時間で大きく遅れる事象を確認した（2026-09-16 SITL試験）。
    // few_goals_grace_sと同じ発想で、外れが猶予以内の間はlow_alt_elapsed_s_を
    // 凍結するだけに留め、猶予を超えて外れ続けた場合だけ本当にリセットする。
    if (altitude_valid) {
        if (centered && low_alt) {
            off_center_elapsed_s_ = 0.0f;
            low_alt_elapsed_s_ += dt_s_;
            if (low_alt_elapsed_s_ >= config_.hover_alt_debounce_s) {
                status_.cmd = PositionVelocityNedCmd{};
                status_.cmd.north_m = north_m;
                status_.cmd.east_m  = east_m;
                write_vertical_cmd(down_m);
                status_.finished = true;
                std::cout << "VisionNavigate: フィルタ済みAGL " << agl_filtered_m << "m (生LiDAR "
                          << altitude_m << "m) / ゴール重心オフセット " << offset_mag_m
                          << "m でゴール上に到達。降下完了。" << std::endl;
                return;
            }
        } else {
            off_center_elapsed_s_ += dt_s_;
            if (off_center_elapsed_s_ >= config_.finish_center_grace_s) {
                low_alt_elapsed_s_ = 0.0f;
            }
            // 猶予以内はlow_alt_elapsed_s_を凍結（増やしも0に戻しもしない）。
        }
    }
}

// ---- 強制着陸（捜索タイムアウト時のフェイルセーフ）----
// GPSゴール座標(forced_land_north_m_, forced_land_east_m_)へ向けて水平移動する。
// 高度はタイムアウト発生時点のものを維持し、水平方向で目標地点に十分近づいたら
// finishedを立てる。実際の降下・着地はVisionNavigatePhaseが引き渡すLandフェーズが行う。
void VisionNavigate::update_forced_land(float north_m, float east_m, float down_m) {
    status_.cmd = PositionVelocityNedCmd{};
    status_.cmd.north_m = forced_land_north_m_;
    status_.cmd.east_m  = forced_land_east_m_;
    status_.cmd.down_m  = down_m; // 高度は維持。到達後はLandフェーズが実際の降下を担当する
    status_.cmd.yaw_deg = 0.0f;

    const float dist = std::hypot(forced_land_north_m_ - north_m, forced_land_east_m_ - east_m);
    if (dist <= config_.forced_land_arrival_thresh_m) {
        status_.finished = true;
        std::cout << "VisionNavigate: 強制着陸目標地点に到達。着陸します。" << std::endl;
    }
}

// ---- 鉛直制御(AGLアンカー方式。GPSNavigate::update()の鉛直部分と同じ考え方) ----
// PX4へ送る鉛直setpoint(cmd.down_m)はEKFフレームの絶対位置なので、基準は生のdown_mで
// なければならない。一方「あと何m上下すべきか」= (target_agl_m_ - 実測AGL) は差分なので
// フレーム非依存。この差分だけを毎tick今のdown_mから動かすことで、down_m(EKF)が
// ドリフトしても実測AGL基準で高度を維持できる（ドリフトぶんはdown_mと目標で相殺される）。
void VisionNavigate::update_vertical_target(float down_m, float altitude_m, bool altitude_valid, float dt_s) {
    if (!vert_initialized_) {
        // 一度も信頼できるAGLを得ていない間は初期化を保留し、生のdown_mを維持する
        // （従来動作）。ここで無理にhover_alt_m等で初期化すると、初回tickでいきなり
        // 大きな高度差を目標にしてしまう。
        if (!altitude_valid) {
            return;
        }
        target_agl_m_       = altitude_m; // 今の対地高度をそのまま維持目標にする
        vert_target_down_m_ = down_m;
        vert_carrot_down_m_ = down_m;
        // クランプの基準点(対地高度0に対応するdown)をここで1回だけ固定する。
        ground_level_down_ref_ = down_m + target_agl_m_;
        vert_initialized_   = true;
        return;
    }

    // 「今のdown_mを基準に、実測AGLがtarget_agl_m_になるdown」を計算し直す。
    if (altitude_valid) {
        const float desired_down_m = down_m - (target_agl_m_ - altitude_m);
        const float raw_delta_m    = desired_down_m - vert_target_down_m_;
        // 不感帯: LiDARノイズ程度の誤差では目標を動かさない
        if (std::fabs(raw_delta_m) >= config_.alt_deadband_m) {
            // レート制限: LiDARの一発の異常値で目標が瞬時に大きく飛ぶのを防ぐ
            const float max_step_m = config_.alt_max_rate_m_s * dt_s;
            vert_target_down_m_ += std::clamp(raw_delta_m, -max_step_m, max_step_m);
        }
    }
    // altitude_valid=falseの間はvert_target_down_m_を直近の値のまま維持する
    // （LiDARが信頼できない間はPX4のEKF高度に委ねるのと実質同じ挙動）。

    // 最終安全網: vert_target_down_m_がtarget_agl_m_からalt_max_deviation_m以上
    // 外れることを機械的に禁止する。GPSNavigate::clamp_target_down_m()と同じ考え方で
    // 被害を頭打ちにする（2026-09-15 Black Rock飛行の事後対策）。
    // 基準は固定のground_level_down_ref_からtarget_agl_m_(降下中は変化しうる)を
    // 引いた値であって、実測down_mそのものではない。down_mを直接基準にすると、
    // 実高度がわずかにドリフトしただけでクランプ範囲自体がその方向へ追従し、
    // 目標を際限なく押し上げる正のフィードバックになってしまう
    // （2026-09-16 SITL試験で確認: 6mから45m超まで暴走）。
    const float target_down_for_agl = ground_level_down_ref_ - target_agl_m_;
    const float lower = target_down_for_agl - config_.alt_max_deviation_m;
    const float upper = target_down_for_agl + config_.alt_max_deviation_m;
    vert_target_down_m_ = std::clamp(vert_target_down_m_, lower, upper);

    // carrotでvert_target_down_m_へ滑らかに追従（cmd.down_mの変化速度をvert_speed_m_s以下に抑える）
    const float dz_c = vert_target_down_m_ - vert_carrot_down_m_;
    const float step_m = std::min(config_.vert_speed_m_s * dt_s, std::fabs(dz_c));
    vert_carrot_down_m_ += std::copysign(step_m, dz_c);
}

// 画像航法の降下: 維持目標の対地高度をdescend_v_m_sぶん下げる（下限hover_alt_m）。
// 実際のsetpointへの反映はupdate_vertical_target()経由（carrot）。
void VisionNavigate::descend_target_agl(float descend_v_m_s, float dt_s) {
    if (!vert_initialized_) {
        return;
    }
    target_agl_m_ = std::max(config_.hover_alt_m, target_agl_m_ - descend_v_m_s * dt_s);
}

// status_.cmdの鉛直成分を書き込む。AGLアンカーが初期化済みならcarrotを、まだなら
// fallback_down_m(生のdown_m)をそのまま使う。速度成分は常に0（GPSNavigateと同じく
// 鉛直はposition駆動。carrot自体がvert_speed_m_sでレート制限済み）。
void VisionNavigate::write_vertical_cmd(float fallback_down_m) {
    status_.cmd.down_m   = vert_initialized_ ? vert_carrot_down_m_ : fallback_down_m;
    status_.cmd.down_m_s = 0.0f;
}

// 実距離オフセット[m]から水平速度を計算（デッドバンド・クランプ付き）
float VisionNavigate::offset_to_velocity(float offset_m) const {
    if (std::abs(offset_m) < config_.center_dead_m)
        return 0.0f;
    float v = config_.kp * offset_m;
    return std::clamp(v, -config_.max_horiz_v_m_s, config_.max_horiz_v_m_s);
}

// ---- ゴール検出（赤マスク → 輪郭 → 楕円フィット）----
// 焦点距離[px] = (画像幅[px]/2) / tan(水平画角/2)。update_image_nav()のpx→m換算と同じ式。
float VisionNavigate::focal_length_px(int image_width_px) const {
    const float fov_rad = config_.camera_horizontal_fov_deg * static_cast<float>(M_PI) / 180.0f;
    return (static_cast<float>(image_width_px) / 2.0f) / std::tan(fov_rad / 2.0f);
}

std::vector<VisionNavigate::GoalCandidate> VisionNavigate::detect_goals(const cv::Mat& img, float altitude_m) const {
    // Step0: 高度に応じた動的min_areaを計算する。
    // ゴール球は実半径goal_radius_m(0.1m)固定なので、見かけの面積は高度の2乗に
    // 反比例して小さくなる（apparent_radius_px = focal_length_px × goal_radius_m / altitude_m）。
    // 固定値のmin_areaだと、低空では緩すぎてノイズを拾いやすく、高空では厳しすぎて
    // 検出できなくなる（実際に高度3.5m基準の固定値では、高度5mで検出不能になった事例あり）。
    // 地面すれすれでスケールが暴れないよう下限を設けるのはupdate_image_nav()と同じ。
    const float alt_for_scale      = std::max(altitude_m, 0.1f);
    const float focal_len_px       = focal_length_px(img.cols);
    const float apparent_radius_px = focal_len_px * config_.goal_radius_m / alt_for_scale;
    const double dynamic_min_area  = M_PI * apparent_radius_px * apparent_radius_px * config_.min_area_margin;

    // Step1: 赤マスク生成
    cv::Mat hsv;
    cv::cvtColor(img, hsv, cv::COLOR_BGR2HSV);

    cv::Mat mask1, mask2, mask;
    const int s_min = config_.red_min_saturation;
    const int v_min = config_.red_min_value;
    cv::inRange(hsv, cv::Scalar(0, s_min, v_min), cv::Scalar(10, 255, 255), mask1);
    cv::inRange(hsv, cv::Scalar(170, s_min, v_min), cv::Scalar(180, 255, 255), mask2);
    mask = mask1 | mask2;

    // モルフォロジーでノイズ除去。CLOSE(5x5)で穴埋め、OPEN(3x3)で細かいノイズだけ除去する。
    // OPENは収縮→膨張なので、CLOSEと同じ5x5カーネルを使うと、実機の小さい/遠いボール
    // (直径10px前後)が収縮の段階でほぼ消えてしまい、面積判定に届く前に候補から
    // 落ちる（2026-09-05 実機フレームで確認）。OPENだけ3x3に落として、小さい本物の
    // ボールを誤って消さないようにする。
    cv::Mat close_kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
    cv::Mat open_kernel  = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, close_kernel);
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, open_kernel);

    // Step2: 連結成分（輪郭）の検出
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    // Step3: 面積フィルタ → 楕円フィッティング → アスペクト比フィルタ
    std::vector<GoalCandidate> candidates;

    for (const auto& contour : contours) {
        double area = cv::contourArea(contour);
        if (area < dynamic_min_area)
            continue;

        cv::Moments mo = cv::moments(contour);
        cv::Point2f centroid(
            static_cast<float>(mo.m10 / mo.m00),
            static_cast<float>(mo.m01 / mo.m00));

        GoalCandidate cand;
        cand.area     = area;
        cand.centroid = centroid;

        if (area >= config_.large_area) {
            // 十分大きい → 形フィルタをスキップして無条件で採用
            cand.ellipse   = cv::minAreaRect(contour);
            cand.fit_score = 1.0;
            candidates.push_back(cand);
            continue;
        }

        // 楕円フィッティングには輪郭点が5点以上必要
        if (contour.size() < 5)
            continue;

        cand.ellipse = cv::fitEllipse(contour);

        // アスペクト比チェック（細長すぎるものを除外）
        float  major  = std::max(cand.ellipse.size.width, cand.ellipse.size.height);
        float  minor  = std::min(cand.ellipse.size.width, cand.ellipse.size.height);
        double aspect = (minor > 0) ? major / minor : 999.0;
        if (aspect > config_.max_aspect)
            continue;

        // フィット精度: 輪郭面積と楕円面積の比（近いほど1に近い）
        double ellipse_area = CV_PI * (major / 2.0) * (minor / 2.0);
        double ratio        = (ellipse_area > 0) ? area / ellipse_area : 0;
        cand.fit_score      = (ratio <= 1.0) ? ratio : 1.0 / ratio;

        if (cand.fit_score < config_.min_fit_score)
            continue;

        candidates.push_back(cand);
    }

    // Step4: フィット精度の高い上位max_goals個をゴールとして採用
    std::sort(candidates.begin(), candidates.end(),
              [](const GoalCandidate& a, const GoalCandidate& b)
              { return a.fit_score > b.fit_score; });

    if (static_cast<int>(candidates.size()) > config_.max_goals)
        candidates.resize(config_.max_goals);

    return candidates;
}
