// 画像航法: ゴール捜索 → 発見後に画像航法で降下
// カメラは真下向き。画像上端 = 機首方向と仮定。
// NED座標系: X=北(前), Y=東(右), Z=下(降下=正)

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <opencv2/opencv.hpp>
#include <thread>
#include <unistd.h> // usleep
#include <vector>

using namespace mavsdk;
using namespace std::chrono_literals;

// ---- 撮影間隔 ----
static const int INTERVAL_MS    = 200;  // 画像航法の撮影間隔 [ms]
static const int SEARCH_WAIT_MS = 2000; // 捜索: 各ウェイポイントでの待機時間 [ms]

// ---- 制御パラメータ（画像航法）----
static const float KP          = 0.002f; // 比例ゲイン [m/s per pixel]
static const float MAX_HORIZ_V = 1.0f;  // 水平速度上限 [m/s]
static const float DESCEND_V   = 0.3f;  // 降下速度 [m/s]（正=下方向=NED Z正）
static const float CENTER_DEAD = 20.0f; // デッドバンド [px]
static const float HOVER_ALT_M = 1.0f;  // この高度[m]まで降下したら降下を止めてホバリングする

// ---- 捜索パラメータ ----
static const float SEARCH_RADIUS_STEP = 0.3f;  // 1周ごとに広げる半径 [m]
static const float MAX_SEARCH_RADIUS  = 5.0f;  // 最大捜索半径 [m]
static const int   SEARCH_N_POINTS    = 12;    // 1周あたりのウェイポイント数（30°刻み）

// ---- 検出パラメータ ----
static const double MIN_AREA      = 500.0;
static const double LARGE_AREA    = 50000.0;
static const double MAX_ASPECT    = 3.0;
static const double MIN_FIT_SCORE = 0.5;
static const int    MAX_GOALS     = 4;

// 状態
enum class State { SEARCH, IMAGE_NAV };

struct GoalCandidate
{
    cv::RotatedRect ellipse; // フィットした楕円
    cv::Point2f centroid;    // 重心
    double area;             // 輪郭面積
    double fitScore;         // 楕円フィット精度（1に近いほど良い）
};

std::vector<GoalCandidate> detectGoals(const cv::Mat &img)
{
    // ---- Step1: 赤マスク生成 ----
    cv::Mat hsv;
    cv::cvtColor(img, hsv, cv::COLOR_BGR2HSV); // 加工前、加工後、加工処理

    cv::Mat mask1, mask2, mask;
    cv::inRange(hsv, cv::Scalar(0, 100, 100), cv::Scalar(10, 255, 255), mask1);
    cv::inRange(hsv, cv::Scalar(170, 100, 100), cv::Scalar(180, 255, 255), mask2);
    mask = mask1 | mask2;

    // モルフォロジーでノイズ除去
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);

    // ---- Step2: 連結成分（輪郭）の検出 ----
    std::vector<std::vector<cv::Point>> contours; // 1つの点→その集合が一つの輪郭→その集合で全体の輪郭　二重のコンテナを作る
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    // ---- Step3: 面積フィルタ → 楕円フィッティング → アスペクト比フィルタ ----
    std::vector<GoalCandidate> candidates; // さっき作ったGoalCandidate型のデータを何個でも入れられる箱（動的配列）

    for (const auto &contour : contours) // contours の中身を1つずつ取り出して contour に入れながらループ
    {
        double area = cv::contourArea(contour); // 輪郭面積

        // 小さすぎる → ノイズ
        if (area < MIN_AREA)
            continue;

        // 重心をモーメントで計算
        cv::Moments mo = cv::moments(contour);
        cv::Point2f centroid( // centroidを定義
            static_cast<float>(mo.m10 / mo.m00),
            static_cast<float>(mo.m01 / mo.m00)); // double→floatに変換
        // cv::Point2f は float を要求しますが、mo.m10 / mo.m00 の結果は double

        GoalCandidate cand;       // GoalCandidate型の変数を作る
        cand.area = area;         // area メンバに面積を入れる
        cand.centroid = centroid; // centroid メンバに重心を入れる

        if (area >= LARGE_AREA)
        {
            // 十分大きい → 形フィルタをスキップして無条件で採用
            cand.ellipse = cv::minAreaRect(contour); // 外接する最小の矩形
            cand.fitScore = 1.0;
            candidates.push_back(cand); /*std.vectorに要素を末尾に追加
            ↑vector    ↑末尾に追加  ↑これを  */
            continue;
        }

        // 楕円フィッティングには輪郭点が5点以上必要
        if (contour.size() < 5) // coutour.size()/輪郭の点列、点の数
            continue;

        cand.ellipse = cv::fitEllipse(contour);

        // アスペクト比チェック（細長すぎるものを除外）
        float major = std::max(cand.ellipse.size.width, cand.ellipse.size.height);
        float minor = std::min(cand.ellipse.size.width, cand.ellipse.size.height); // 長軸と短軸を取り出す
        double aspect = (minor > 0) ? major / minor : 999.0;
        /*
        // if-else で書いた場合
        if (minor > 0) {
            aspect = major / minor;
        } else {
            aspect = 999.0;
        }*/
        if (aspect > MAX_ASPECT)
            continue;

        // フィット精度: 輪郭面積と楕円面積の比（近いほど1に近い）
        double ellipseArea = CV_PI * (major / 2.0) * (minor / 2.0);
        double ratio = (ellipseArea > 0) ? area / ellipseArea : 0;
        cand.fitScore = (ratio <= 1.0) ? ratio : 1.0 / ratio;

        if (cand.fitScore < MIN_FIT_SCORE)
            continue;

        candidates.push_back(cand); // すべての基準を満たしたものをcandに追加
    }

    // ---- Step4: フィット精度の高い上位MAX_GOALSをゴールとして採用 ----
    std::sort(candidates.begin(), candidates.end(),              // vectorの最初と最後を示す(ソートの範囲)
              [](const GoalCandidate &a, const GoalCandidate &b) //[]:その場で使い捨ての関数
              {
                  return a.fitScore > b.fitScore;
                  // aの方が大きければ true → a を前に置く
              });

    if (static_cast<int>(candidates.size()) > MAX_GOALS) // 戻り値の型が size_t（符号なし整数）なので、intに変換
        candidates.resize(MAX_GOALS);                    // vector の要素数を指定した数(MAX_GOALS)に変更するメンバ関数

    return candidates;
}

// ピクセルオフセットから水平速度を計算（デッドバンド・クランプ付き）
static float pixelToVelocity(float offset_px)
{
    if (std::abs(offset_px) < CENTER_DEAD)
        return 0.0f;
    float v = KP * offset_px;
    return std::clamp(v, -MAX_HORIZ_V, MAX_HORIZ_V);
}

int main()
{
    // ---- MAVSDKセットアップ ----
    Mavsdk::Configuration config(ComponentType::CompanionComputer);
    Mavsdk mavsdk(config);

    ConnectionResult conn = mavsdk.add_any_connection("udp://:14540"); // simulation:"udp://:14540", drone:"serial:///dev/serial0:921600"
    if (conn != ConnectionResult::Success)
    {
        std::cerr << "接続失敗: " << conn << std::endl;
        return 1;
    }

    std::cout << "PX4接続待機中..." << std::endl;
    // "drone" に命名: std::shared_ptr<System> system だと ::system() と名前衝突するため
    std::shared_ptr<System> drone;
    mavsdk.subscribe_on_new_system([&]()
                                   {
        auto systems = mavsdk.systems();
        if (!systems.empty() && systems[0]->has_autopilot())
            drone = systems[0]; });
    for (int i = 0; i < 10 && !drone; ++i)
        std::this_thread::sleep_for(1s);
    if (!drone)
    {
        std::cerr << "PX4が見つかりません。" << std::endl;
        return 1;
    }

    auto action    = Action{drone};
    auto offboard  = Offboard{drone};
    auto telemetry = Telemetry{drone};

    // テレメトリが安定するまで待つ
    std::this_thread::sleep_for(1s);

    const float imgCx = 640.0f / 2.0f;
    const float imgCy = 480.0f / 2.0f;

    // ---- 初期チェック: 起動時点で赤が見えるか確認 ----
    // Offboard開始前に済ませる（開始直後にsetpointを途切れさせてPX4のOffboardタイムアウトを招かないため）
    State state = State::SEARCH; // デフォルトは捜索
    {
        int ret = ::system("rpicam-still -t 1 --nopreview --width 640 --height 480 -o /tmp/frame.jpg 2>/tmp/rpicam_err.log");
        if (ret == 0)
        {
            cv::Mat frame = cv::imread("/tmp/frame.jpg");
            if (!frame.empty() && !detectGoals(frame).empty())
            {
                state = State::IMAGE_NAV;
                std::cout << "起動時にゴールを発見。画像航法を開始します。" << std::endl;
            }
        }
    }
    if (state == State::SEARCH)
        std::cout << "ゴール未検出。捜索を開始します。Ctrl+C で終了。" << std::endl;

    // ---- Offboardモード開始 ----
    // 初期セットポイントを送ってから有効化する（PX4の要件）
    Offboard::VelocityNedYaw initVel{};
    offboard.set_velocity_ned(initVel);
    std::this_thread::sleep_for(100ms);

    auto startResult = offboard.start();
    if (startResult != Offboard::Result::Success)
    {
        std::cerr << "Offboard開始失敗: " << startResult << std::endl;
        return 1;
    }

    // ---- 捜索の中心位置を記録（NED座標）----
    // telemetry.position_velocity_ned(): 離陸地点を原点とした North/East/Down [m]
    auto initNed  = telemetry.position_velocity_ned();
    float centerN = initNed.position.north_m;
    float centerE = initNed.position.east_m;
    float centerD = initNed.position.down_m; // 捜索中はこの高度を維持

    float searchRadius = SEARCH_RADIUS_STEP; // 現在の捜索半径
    int   orbitPoint   = 0;                  // 現在の周回ポイント番号

    int frameCount = 0;

    while (true)
    {
        // ---- rpicam-still で写真を撮影 ----
        // -t 1: 起動待機(timeout)を最短にして即座に撮影, --nopreview: プレビュー無し, --width/height: 解像度固定
        int ret = ::system("rpicam-still -t 1 --nopreview --width 640 --height 480 -o /tmp/frame.jpg 2>/tmp/rpicam_err.log");
        if (ret != 0)
        {
            std::cerr << "警告: 撮影失敗。スキップします。詳細は /tmp/rpicam_err.log を確認してください。" << std::endl;
            usleep(INTERVAL_MS * 1000); // マイクロ秒なので×1000
            continue;
        }

        // ---- 画像を読み込む ----
        cv::Mat frame = cv::imread("/tmp/frame.jpg");
        if (frame.empty())
        {
            std::cerr << "警告: 画像読み込み失敗。スキップします。" << std::endl;
            usleep(INTERVAL_MS * 1000);
            continue;
        }

        auto candidates = detectGoals(frame);

        // ---- 捜索中にゴールを発見 → 画像航法に切替 ----
        if (state == State::SEARCH && !candidates.empty())
        {
            state = State::IMAGE_NAV;
            std::cout << "ゴール発見！画像航法に切替。" << std::endl;
        }

        // ---- 捜索モード ----
        if (state == State::SEARCH)
        {
            // 次のウェイポイントへ移動（位置制御）
            // angle: 円周上の角度。0 → 北、π/2 → 東
            float angle = 2.0f * static_cast<float>(CV_PI) * orbitPoint / SEARCH_N_POINTS;
            Offboard::PositionNedYaw wp{};
            wp.north_m = centerN + searchRadius * std::cos(angle);
            wp.east_m  = centerE + searchRadius * std::sin(angle);
            wp.down_m  = centerD;  // 出発高度を維持（down は上昇=負、降下=正）
            wp.yaw_deg = 0.0f;
            offboard.set_position_ned(wp);

            std::cout << "[捜索] 半径=" << searchRadius << "m"
                      << "  ポイント " << (orbitPoint + 1) << "/" << SEARCH_N_POINTS
                      << "  N=" << wp.north_m << " E=" << wp.east_m << std::endl;

            orbitPoint++;
            if (orbitPoint >= SEARCH_N_POINTS)
            {
                orbitPoint = 0;
                searchRadius += SEARCH_RADIUS_STEP;
                if (searchRadius > MAX_SEARCH_RADIUS)
                {
                    std::cout << "最大捜索半径 " << MAX_SEARCH_RADIUS
                              << "m に到達。終了します。" << std::endl;
                    break;
                }
            }

            // ウェイポイントに近づくまで待機
            usleep(SEARCH_WAIT_MS * 1000);
            continue;
        }

        // ---- 画像航法モード ----
        // 高度制御は気圧計で推算された高度（relative_altitude_m）を使用（LiDAR統合は別途対応）
        // ※ RP2040Bridge/Store等でもこの値をpressure_relative_altitude_mとして気圧高度扱いしている
        float currentAlt = telemetry.position().relative_altitude_m; // 離陸地点からの相対高度[m]

        // HOVER_ALT_M まで降下したら、その場でホバリングして終了する
        if (currentAlt <= HOVER_ALT_M)
        {
            std::cout << "高度 " << currentAlt << "m に到達。その場でホバリングして終了します。" << std::endl;
            Offboard::VelocityNedYaw hoverVel{};
            offboard.set_velocity_ned(hoverVel);
            offboard.stop();
            return 0;
        }

        // 着地検知（安全フォールバック）: 想定より早く接地した場合はOffboardを止めてdisarmする
        if (telemetry.landed_state() == Telemetry::LandedState::OnGround)
        {
            std::cout << "着地を検知しました。Offboard停止・disarmします。" << std::endl;
            offboard.stop();
            auto disarmResult = action.disarm();
            if (disarmResult != Action::Result::Success)
                std::cerr << "Disarm失敗: " << disarmResult << std::endl;
            return 0;
        }

        Offboard::VelocityNedYaw vel{};
        vel.down_m_s = DESCEND_V; // 常に降下

        if (!candidates.empty())
        {
            // ---- Step5: 重心・方向・距離の計算 ----
            cv::Point2f groupCenter(0.0f, 0.0f);                         // groupCenterを定義, float型で
            for (const auto &cand : candidates)                          /*auto:型を自動推論、　&:参照, cand:各要素受け取る変数名, const:ループ内でcandを変更しない,
                                         candidates:ループする対象のコンテナ（配列・ベクトルなど）*/
                groupCenter += cand.centroid;                            // 座標の合計を出す
            groupCenter *= 1.0f / static_cast<float>(candidates.size()); // 全重心の平均

            // ---- Step6: 重心の重心を制御目標として速度を計算 ----
            float ox = groupCenter.x - imgCx; // 右が正
            float oy = groupCenter.y - imgCy; // 下が正

            // 真下カメラ: 画像上端=機首方向
            // ox > 0(右) → 東に移動 → east_m_s 正
            // oy > 0(下) → 機後ろ方向 → north_m_s 負
            vel.east_m_s  = pixelToVelocity(ox);
            vel.north_m_s = -pixelToVelocity(oy);

            std::cout << "[" << frameCount << "] 画像航法"
                      << "  ゴール=" << candidates.size() << "個"
                      << "  オフセット=(" << static_cast<int>(ox)
                      << ", " << static_cast<int>(oy) << ")px"
                      << "  速度 N=" << vel.north_m_s
                      << " E=" << vel.east_m_s
                      << " D=" << vel.down_m_s << " m/s" << std::endl;
        }
        else
        {
            // ゴール未検出: その場でホバリングしながら降下
            std::cout << "[" << frameCount << "] 画像航法: ゴール未検出。ホバリング降下。" << std::endl;
        }

        offboard.set_velocity_ned(vel);
        frameCount++;

        // ---- 次の撮影まで待つ ----
        usleep(INTERVAL_MS * 1000); // マイクロ秒なので×1000
    }

    offboard.stop();
    return 0;
}
