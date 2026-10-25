#pragma once
#include "SystemState/SystemState.hpp"
#include <opencv2/opencv.hpp>
#include <vector>
#include <chrono>
#include <optional>

// 画像航法: ゴール捜索（拡大する八の字）→ 発見後に画像航法で降下 → hover_alt_mまで降りたら完了。
// カメラは真下向き・ヨー0固定を目標にコマンドするが、実際のヨーが0に収束しきっていない
// 期間（モード切替直後など）もあるため、画像→東西南北変換には実測のyaw_degを使う
// （詳細はupdate_image_nav()参照）。
// MAVSDKには触らない。撮影も呼び出し側の担当で、update()に最新フレームを渡してもらう。
class VisionNavigate {
public:
    // ForcedLand: 捜索がsearch_timeout_sを超えても完了しない場合のフェイルセーフ。
    // GPSゴール座標へ向かい、到達したらその場でLandへ引き渡す。
    enum class Mode { Search, ImageNav, ForcedLand };
    // 八の字捜索の現在のロブ（右→左→前→後の順に切り替える）
    enum class SearchLobe { Right, Left, Front, Back };

    struct Config {
        // 捜索（八の字: 右→左→前→後で1サイクル。毎回起点に戻る）
        float search_radius_initial_m    = 1.0f;  // 最初の八の字の半径 [m]
        float search_radius_step_m       = 1.0f;  // 1サイクルごとに広げる半径 [m]
        int   search_n_points            = 12;    // 1ロブあたりのウェイポイント数
        float search_wp_arrival_thresh_m = 1.0f;  // ウェイポイント到達とみなす距離 [m]
        // 同じウェイポイントを狙い続けてこの秒数経っても到達判定が出ない場合、
        // 強制的に次のウェイポイントへ進める（位置制御精度に関わらず捜索が確実に
        // 進行するための安全装置）。以前はtick数(想定1tick=200ms)で数えていたが、
        // 実機では撮影(rpicam-still)が1回あたり約900ms〜1sかかり実質1tick≒1.1秒
        // だった（2026-09-07 実測）ため、tick数依存だと撮影速度が変わるたびに
        // この秒数も無意識に変わってしまう。実時間(dt_s_の積算)で管理することで
        // 撮影方式が変わっても狙った秒数のまま動くようにする。
        float search_wp_timeout_s = 16.5f; // 旧15tick×実測1.1秒/tickと同じ実時間
        // 画像航法
        // カメラの水平画角[度]。ピクセルオフセットをLiDAR高度から実際の水平距離[m]に
        // 変換するのに使う（詳細はupdate_image_nav()参照）。Raspberry Pi Camera Module 3
        // Wideのカタログ値。実際のキャプチャモード（切り出し・スケーリング）によって
        // 多少ズレる可能性があるため、既知のサイズ・距離の物体で実測してキャリブレーション
        // することを推奨。
        float camera_horizontal_fov_deg = 102.0f;
        // 比例ゲイン [1/s]（v[m/s] = kp * 実距離オフセット[m]）。
        // 以前はピクセルオフセットに直接掛けていたが、それだと同じピクセルオフセットでも
        // 高度が高いほど実際のズレは大きいのに反応が変わらない（低高度では逆に過敏になる）
        // という高度依存の問題があった。実距離換算後の値に掛けることで高度によらず
        // 一貫した反応になる。
        float kp               = 0.25f;
        float max_horiz_v_m_s  = 0.3f;   // 水平速度上限 [m/s]
        float descend_v_m_s    = 0.2f;   // 降下速度 [m/s]（正=下方向=NED Z正）
        float center_dead_m    = 0.15f;  // デッドバンド（実距離換算後）[m]
        // ゴールが画像中心からこれ以上ズレている間は、水平補正を優先して降下速度を0にする
        // （ゴールを大きく外れたまま降下すると、狙った着地点から外れたり障害物に近づく
        // リスクが上がるため）。center_dead_m以内なら降下速度はdescend_v_m_s全開。
        // center_dead_m〜descend_offset_max_mの間は線形に減速する。
        float descend_offset_max_m = 1.5f;
        float hover_alt_m     = 1.0f;   // この高度[m]まで降下したら完了
        // ゴール重心オフセット(ox, oy)に掛ける指数移動平均(EMA)の重み。
        // 検出ノイズ(照明・部分遮蔽・誤検出)がそのまま速度setpointの暴れに
        // 直結し、高度方向にも振動として漏れてしまうため平滑化する。
        // filtered = alpha*raw + (1-alpha)*filtered_prev。1.0でフィルタ無効（生値をそのまま使う）。
        float offset_smoothing_alpha = 0.3f;
        // 目標地点（ゴールが画像中心付近=デッドバンド内で最初に捉えられた瞬間の機体位置）
        // からの水平距離がこの値[m]以内かどうかをStatusに出す（テレメトリ表示用。制御へは影響しない）。
        float target_arrival_thresh_m = 0.5f;
        // カメラの取り付け向き補正。「画像上端=機体前方」からのズレを、上空から見て
        // 時計回り(北→東→南→西の向き)に測った角度[度]で指定する。
        // 例: カメラが基準位置から時計回りに45度ズレて付いている場合は45.0f。
        float camera_mount_rotation_deg = 45.0f;
        // 画像航法中にゴールを見失った状態が何秒続いたら捜索モードに戻すか。
        // 0秒だと単発の検出漏れでも即座に捜索へ戻ってしまいフェーズが暴れるため、
        // 数秒は前回の指令を維持して粘る猶予を持たせる。以前はtick数(3)で数えて
        // いたが、実機の実測tick周期(約1.1秒/tick、2026-09-07)から実時間へ変換
        // (dt_s_の積算)。撮影速度が変わってもこの秒数のまま動く。
        float lost_goal_debounce_s = 3.3f; // 旧3tick×実測1.1秒/tickと同じ実時間
        // 降下完了判定のデバウンス。LiDARは紐・虫・埃・反射などで瞬間的に
        // 誤って近距離を返すことがあるため、1tickだけhover_alt_m以下を
        // 検出しても即座には完了とせず、この秒数だけ連続でhover_alt_m以下が
        // 続いた場合にのみ「降下完了」と確定する（実時間管理、理由は上と同じ）。
        float hover_alt_debounce_s = 3.3f; // 旧3tick×実測1.1秒/tickと同じ実時間
        // 捜索フェーズ開始からこの秒数を超えてもhover_alt_mまでの降下が完了しない場合、
        // ゴール捜索を諦めてGPSゴール座標(goal_lat_deg, goal_lon_deg)へ強制的に向かい、
        // 到達したらLandへ引き渡すフェイルセーフ。捜索半径に上限が無いため、
        // ゴールを検出できないまま延々と捜索範囲が広がり続けるのを防ぐ。
        float  search_timeout_s = 180.0f; // 3分
        double goal_lat_deg = 0.0;        // MissionConfig.target_lat_degと同じ値を渡す想定
        double goal_lon_deg = 0.0;        // MissionConfig.target_lon_degと同じ値を渡す想定
        // 強制着陸目標地点からこの距離[m]以内に入ったら「到達」とみなしLandへ引き渡す
        float  forced_land_arrival_thresh_m = 1.0f;
        // ゴール検出
        double large_area    = 50000.0;
        double max_aspect    = 3.0;
        double min_fit_score = 0.5;
        int    max_goals     = 4;
        // ゴール球の実半径[m]（直径20cm固定）。detect_goals()内で高度から
        // 動的にmin_areaを計算するのに使う（詳細はdetect_goals()参照）。
        float  goal_radius_m   = 0.10f;
        // 動的min_areaにかける安全マージン係数(0〜1)。レンダリングノイズ・
        // 部分陰影による見かけ面積の目減りを見込んで理論値より少し緩くする。
        float  min_area_margin = 0.7f;

        // ---- 鉛直制御(AGLアンカー方式): GPSNavigate::update()の鉛直ロジックと同じ ----
        // 「今のdown_mを基準に、LiDAR実測AGLがtarget_agl_m_になるdown」を毎tick計算し直し、
        // EKF(down_m)がドリフトしてもLiDAR実測AGLで高度を守り続ける。捜索・画像航法とも
        // このアンカー済み目標(vert_carrot_down_m_)を鉛直setpointに使う。値はGPSNavigate用
        // (gps_altitude_deadband_m 等)と揃えてある。
        float alt_deadband_m   = 0.05f; // これ未満のAGL誤差はLiDARノイズとみなし目標を動かさない
        float alt_max_rate_m_s = 0.5f;  // 高度目標を動かす最大速度[m/s]（LiDAR一発の異常値対策）
        float vert_speed_m_s   = 0.5f;  // 目標へ滑らかに追従するキャロット速度[m/s]

        // 重心(group_center)を制御に信用する最小検出個数。これ未満のtickは重心が
        // 端に偏るので追わず、その場ホバリングで待つ（降下・完了判定も止める）。
        int   min_goals_full   = 3;
        // 降下完了とみなすゴール重心の水平許容[m]（center_dead_mより少し緩め）。
        float finish_center_m  = 0.3f;
        // 降下完了とみなすフィルタ済みAGLの許容幅[m]。hover_alt_m ちょうどに収束した
        // ときに境界の揺れで完了デバウンスが通らない事象への対策（詳細はupdate_image_nav()）。
        float finish_alt_tol_m = 0.1f;
        // センタリング(finish_center_m以内)が一瞬外れても、連続してこの秒数以内なら
        // 完了デバウンス(low_alt_elapsed_s_)を全リセットしない猶予（詳細は
        // update_image_nav()の降下完了判定コメント参照。few_goals_grace_sと同じ発想）。
        float finish_center_grace_s = 1.0f;
        // vert_target_down_m_が「今のdown_mを基準にした対地target_agl_m_」からこの幅[m]を
        // 超えて外れることを機械的に禁止する最終安全網（GPSNavigate::Config::
        // gps_altitude_max_deviation_mと同じ考え方。詳細はupdate_vertical_target()参照）。
        float alt_max_deviation_m = 1.5f;

        // ---- 水平setpointのアンカー方式（ドリフト・振れ対策） ----
        // 画像航法の水平位置setpointを「毎tickの生の現在位置＋速度*ルックアヘッド」で
        // 作ると、テレメトリ遅延時に目標が機体を追いかけて滑る（＝ホバリングなのに
        // ドリフト）。代わりにImageNav開始位置をアンカーとしてラッチし、以降は指令速度を
        // 積分してアンカーを進める。アンカーが実測位置からnav_leash_m以上離れたら引き戻す
        // （積分ワインドアップ防止）。検出数不足でホバリング待機に入ったtickはアンカーを
        // 凍結し、その固定点を出し続ける。
        float nav_leash_m = 0.6f;
        // 検出数が min_goals_full 未満に落ちても、直近まで十分見えていればこの秒数だけは
        // 前回の重心で制御を継続する（一瞬の検出漏れでホバリング/捜索へ転げ落ちて
        // ドリフト・振れを起こさないための猶予）。以前はtick数(3)で数えていたが、
        // 実機の実測tick周期(約1.1秒/tick、2026-09-07)から実時間へ変換(dt_s_の積算)。
        float few_goals_grace_s = 3.3f; // 旧3tick×実測1.1秒/tickと同じ実時間

        // ---- 赤マスクの彩度・明度下限（detect_goals）----
        // 実機の生フレーム実測(2026-09-05)で、ボールの縁(ボケ・アンチエイリアス)は
        // 彩度が中心のハイライト部より大きく落ちる(実測でS=47〜128)ことを確認した。
        // 100だと縁が全部弾かれて中心の小さな塊しか残らず、輪郭面積が理論値より
        // 目減りして高度・距離のわずかな違いで検出が明滅する。色相(H:0-10/170-180)は
        // そのままなので、この値を下げても芝生・ロープ等(色相が違う)を拾うリスクは低い。
        int   red_min_saturation = 70;
        int   red_min_value      = 70;
    };

    struct Status {
        Mode  mode{Mode::Search};
        bool  goal_visible{false};
        int   goal_count{0};
        float offset_x_px{}, offset_y_px{};  // 画像中心からのゴール重心オフセット（右・下が正、EMA平滑化後）
        // 直近のdetect_goals()で検出したゴール候補の楕円（デバッグ可視化用。フレームが
        // 空でdetect_goals()自体が呼ばれなかったtickでは前回の値がそのまま残る）
        std::vector<cv::RotatedRect> debug_goal_ellipses{};
        float search_radius_m{};             // 現在の八の字の半径
        SearchLobe lobe{SearchLobe::Right};  // 捜索中の現在のロブ（画像航法中は最後の値のまま）
        PositionVelocityNedCmd cmd{};        // 位置+速度setpoint（捜索中は速度0）
        bool  finished{false};               // hover_alt_mまで降下完了

        // 目標地点(NED)からの水平距離。target_position_lockedがfalseの間（＝ゴールを
        // 一度も画像中心付近で捉えていない間）は距離が定義できないためdistance_to_target_mは-1。
        bool  target_position_locked{false};
        float distance_to_target_m{-1.0f};
        bool  within_target_radius{false};   // distance_to_target_m <= target_arrival_thresh_m
    };

    explicit VisionNavigate(const Config& config);

    // 捜索の起点を設定する。フェーズ開始時に1回呼ぶ。
    // （画像航法中にゴールを見失った場合は、内部で現在位置を起点に自動で呼び直す）
    void start_search(float north_m, float east_m, float down_m);
    bool is_search_started() const { return search_started_; }

    // 毎tick呼ぶ。frameは最新のカメラ画像。start_search()が呼ばれていない場合何もしない。
    // altitude_m はLiDAR優先・フォールバックはground_down_ref_m基準のNED down差分。
    // altitude_valid は altitude_m がLiDAR実測の対地高度として信頼できるか（呼び出し側で
    // valid かつ測距下限超え かつ is_reliable() を満たしたか）。鉛直制御(AGLアンカー)の
    // 目標補正はこれがtrueのtickだけ行い、falseの間は直近の目標を維持する（GPSNavigate
    // ::update()のaltitude_agl_validと同じ扱い）。
    // yaw_deg は実際の機体ヨー角[度]（北=0、時計回り正）。画像航法へ切り替わった直後など
    // コマンドしたヨー0度にまだ収束していない期間でも、画像→東西南北変換をこの実測値で
    // 補正するために使う（詳細はupdate_image_nav()参照）。
    // lat_deg, lon_deg は現在の自機GPS座標。search_timeout_s超過時、GPSゴール座標を
    // NEDに変換する際の対応点(アンカー)として使う（GPSNavigateと同じ手法。詳細はcpp参照）。
    void update(const cv::Mat& frame, float north_m, float east_m, float down_m, float altitude_m, bool altitude_valid,
                float yaw_deg, double lat_deg, double lon_deg);

    bool   is_finished() const { return status_.finished; }
    Status get_status() { return status_; }

    // デバッグ表示用。
    int orbit_point() const { return orbit_point_; }

private:
    struct GoalCandidate {
        cv::RotatedRect ellipse;  // フィットした楕円
        cv::Point2f     centroid; // 重心
        double          area;     // 輪郭面積
        double          fit_score; // 楕円フィット精度（1に近いほど良い）
    };

    // altitude_mから高度連動のmin_areaを計算して使う（詳細は.cppのコメント参照）。
    std::vector<GoalCandidate> detect_goals(const cv::Mat& img, float altitude_m) const;
    float offset_to_velocity(float offset_m) const;
    // ピンホールカメラモデルの焦点距離[px]。画像幅とconfig_.camera_horizontal_fov_degから
    // 計算する。detect_goals()（動的min_area用）とupdate_image_nav()（px→m換算用）で共用。
    float focal_length_px(int image_width_px) const;

    // 各ロブの円の中心オフセット方向と、起点が円周上に来る開始角度
    void lobe_offsets(SearchLobe lobe, float& d_north, float& d_east, float& theta_start) const;
    static SearchLobe next_lobe(SearchLobe lobe);

    // 現在のロブ・ポイント番号に対応するウェイポイントを計算
    void search_waypoint(float& wp_north, float& wp_east) const;

    void update_search(float north_m, float east_m, float down_m);
    void update_image_nav(const cv::Mat& frame, const std::vector<GoalCandidate>& candidates,
                          float north_m, float east_m, float down_m, float yaw_deg,
                          float altitude_m, bool altitude_valid);
    // GPSゴール座標へ向けて水平移動し、到達したらstatus_.finishedを立てる
    void update_forced_land(float north_m, float east_m, float down_m);

    // ---- 鉛直制御(AGLアンカー方式。GPSNavigate::update()の鉛直部分と同じ) ----
    // 「今のdown_mを基準に、実測AGL(altitude_m)がtarget_agl_m_になるdown」を毎tick
    // 計算し直し(不感帯alt_deadband_m・レート制限alt_max_rate_m_s付き)、そこへcarrotを
    // vert_speed_m_s以下で追従させる。EKF(down_m)がドリフトしても、LiDAR実測AGLで
    // 高度目標を守り続ける。altitude_valid=falseの間はvert_target_down_m_を維持する。
    // 一度も信頼できるAGLを得ていない間は初期化を保留する(vert_initialized_=false)。
    void update_vertical_target(float down_m, float altitude_m, bool altitude_valid, float dt_s);
    // 画像航法の降下: 維持目標の対地高度をdescend_v_m_s分だけ下げる(下限hover_alt_m)。
    void descend_target_agl(float descend_v_m_s, float dt_s);
    // status_.cmdの鉛直成分を書き込む。AGLアンカーが初期化済みならcarrotを、
    // まだならfallback_down_m(=生のdown_m)をそのまま使う。速度成分は常に0
    // (GPSNavigateと同じくposition駆動)。
    void write_vertical_cmd(float fallback_down_m);

    Config config_;
    Status status_{};
    bool   search_started_{false};

    // 八の字捜索の状態（起点の水平位置。高度はAGLアンカー側で管理するのでdownは持たない）
    float      center_north_m_{}, center_east_m_{};
    SearchLobe search_lobe_{SearchLobe::Right};
    int        orbit_point_{0};
    float      search_radius_m_{};
    // 現在のウェイポイントを狙い始めてから何秒経ったか（タイムアウト判定用。dt_s_の積算）
    float      wp_wait_elapsed_s_{0.0f};

    // 画像航法中、ゴールを連続で見失っている秒数（lost_goal_debounce_sとの比較用。dt_s_の積算）
    float      miss_elapsed_s_{0.0f};

    // 画像航法中、「hover_alt_m以下 かつ ゴール重心が直下(finish_center_m以内)」を
    // 連続で満たしている秒数（hover_alt_debounce_sとの比較用。dt_s_の積算）。
    float      low_alt_elapsed_s_{0.0f};
    // センタリングが連続して外れている秒数（finish_center_grace_sとの比較用。dt_s_の
    // 積算）。猶予以内ならlow_alt_elapsed_s_を凍結するだけでリセットしない。満たせば0に戻す。
    float      off_center_elapsed_s_{0.0f};

    // 検出個数がmin_goals_full未満で「重心を追わずホバリング待機」に入っているか。
    // ログを状態遷移時だけ出すためのフラグ。
    bool       few_goals_hold_{false};
    // 検出数不足が続いた秒数（few_goals_grace_sとの比較用。dt_s_の積算）。十分な検出で0に戻す。
    float      few_goals_elapsed_s_{0.0f};
    // ホバリング待機に入った瞬間の機体位置をラッチ（待機中はこの固定点を指令し続ける）。
    float      hold_north_m_{}, hold_east_m_{};

    // 水平setpointのアンカー（ImageNav開始位置でラッチ→指令速度を積分して進める）。
    // nav_anchor_valid_=false の間は次のImageNav tickで現在位置から初期化する。
    // start_search()でリセットする。
    float      nav_anchor_north_m_{}, nav_anchor_east_m_{};
    bool       nav_anchor_valid_{false};

    // ゴール重心オフセットのEMAフィルタ状態（検出ノイズ平滑化用）
    float      filtered_ox_{0.0f}, filtered_oy_{0.0f};
    bool       offset_filter_initialized_{false};

    // 目標地点(NED)。target_locked_がtrueになった瞬間の機体位置を保持する。
    bool       target_locked_{false};
    float      target_north_m_{}, target_east_m_{};

    // 捜索タイムアウト（フェーズ全体で1回だけ計測。ゴール見失いによる再捜索では
    // リセットしない）。start_search()の最初の呼び出し時にのみ計測開始する。
    std::chrono::steady_clock::time_point search_start_time_{};
    bool       timeout_timer_started_{false};

    // 強制着陸（タイムアウト後、GPSゴール座標へ向かっている間）の状態
    bool       forced_land_active_{false};
    float      forced_land_north_m_{}, forced_land_east_m_{};

    // ---- 鉛直制御(AGLアンカー方式)の状態 ----
    // target_agl_m_       : 維持したい対地高度[m]。初回に実測AGLで初期化し、画像航法の
    //                       降下ぶんだけhover_alt_mまで下げていく。
    // vert_target_down_m_ : 上記AGLを満たすEKFフレームdown。毎tick down_m を基準に
    //                       計算し直す(不感帯・レート制限付き)。EKFドリフトはここで吸収。
    // vert_carrot_down_m_ : vert_target_down_m_ へvert_speed_m_s以下で追従する実際の
    //                       鉛直setpoint。cmd.down_m にはこれを送る。
    // vert_initialized_   : 一度でも信頼できるAGLで初期化できたか。falseの間は生のdown_mを
    //                       そのまま鉛直setpointにする(従来動作)。再捜索でもリセットしない。
    float      target_agl_m_{};
    float      vert_target_down_m_{};
    float      vert_carrot_down_m_{};
    bool       vert_initialized_{false};
    // 初期化時点の「対地高度0(地表)に対応するdown」。vert_initialized_になった瞬間に
    // 1回だけ ground_level_down_ref_ = down_m_init + target_agl_m_init として固定する。
    // 最終安全網(update_vertical_target()のクランプ)の基準に使う。実測down_mを直接
    // 基準にすると、実高度がわずかにドリフトしただけでクランプ範囲自体がその方向へ
    // 追従してしまい、目標を際限なく押し上げる正のフィードバックになる
    // （2026-09-16 SITL試験で確認: 6mから45m超まで暴走）。target_agl_m_はdescend_target_agl()
    // で意図的に変化しうるため、クランプ時は毎回 ground_level_down_ref_ - target_agl_m_
    // を計算し直す。
    float      ground_level_down_ref_{};
    // update()呼び出し間隔[s]。撮影(rpicam-still)待ちで200msから伸びうるので実測する。
    std::optional<std::chrono::steady_clock::time_point> last_update_{};
    float      dt_s_{0.2f};
};
