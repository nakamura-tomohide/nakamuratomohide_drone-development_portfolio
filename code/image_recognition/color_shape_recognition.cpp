#include <algorithm>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

// ---- パラメータ ----
static const double MIN_AREA = 500.0;     // これより小さい → ノイズ
static const double LARGE_AREA = 50000.0; // これより大きい → 形フィルタをスキップ
static const double MAX_ASPECT = 3.0;     // 楕円の長軸/短軸の上限
static const double MIN_FIT_SCORE = 0.5;  // 楕円フィット精度の下限（0〜1）
static const int MAX_GOALS = 4;           // 検出するゴール数の上限
// static const:プログラム全体で定数になる

struct GoalCandidate // 複数の関数をひとまとめにした型
{
    cv::RotatedRect ellipse; // フィットした楕円
    cv::Point2f centroid;    // 重心
    double area;             // 輪郭面積
    double fitScore;         // 楕円フィット精度（1に近いほど良い）
};

int main(int argc, char *argv[]) // コマンドライン関数を受け取るための仕組み
{
    std::string imagePath = (argc > 1) ? argv[1] : "images/IMG_8827.jpg";
    // （引数があれば）?それを使う:"無ければデフォルト"

    cv::Mat img = cv::imread(imagePath, cv::IMREAD_COLOR);
    if (img.empty())
    {
        std::cerr << "エラー: 画像を読み込めませんでした: " << imagePath << std::endl;
        return -1;
    }
    std::cout << "画像サイズ: " << img.cols << "x" << img.rows << std::endl;

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
        float minor = std::min(cand.ellipse.size.width, cand.ellipse.size.height); // 長軸と単軸を取り出す
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

    // ---- Step5: 重心・方向・距離の計算と描画 ----
    cv::Mat result = img.clone();
    cv::Point2f imageCenter(img.cols / 2.0f, img.rows / 2.0f); // imageCenterを定義

    // 画像中心に十字線を描画（基準点の可視化）
    cv::drawMarker(result, imageCenter, cv::Scalar(200, 200, 200),
                   cv::MARKER_CROSS, 20, 1);

    std::cout << "\n検出ゴール数: " << candidates.size() << std::endl;

    for (int i = 0; i < static_cast<int>(candidates.size()); i++)
    {
        const GoalCandidate &cand = candidates[i];

        // 楕円と重心を描画
        cv::ellipse(result, cand.ellipse, cv::Scalar(0, 255, 0), 2);     // cand.ellipse(フィットさせた楕円)をresult画像に描く
        cv::circle(result, cand.centroid, 5, cv::Scalar(0, 0, 255), -1); // cand.centroid:輪郭の重心座標

        // 画像中心から重心への矢印
        cv::arrowedLine(result, imageCenter, cand.centroid,
                        cv::Scalar(255, 100, 0), 2);

        // 画像中心を原点とした重心の相対位置（右・下が正）
        cv::Point2f offset = cand.centroid - imageCenter; // 座標は右下が正
        double distance = cv::norm(offset);
        double angleDeg = std::atan2(offset.y, offset.x) * 180.0 / CV_PI; // offset方向の角度をラジアンで取得

        // ラベル描画
        std::string label = "Goal" + std::to_string(i + 1);
        cv::putText(result, label,
                    cv::Point(cand.centroid.x + 8, cand.centroid.y - 8),
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);

        std::cout << "Goal " << (i + 1) << ":"
                  << "  重心=(" << static_cast<int>(cand.centroid.x)
                  << ", " << static_cast<int>(cand.centroid.y) << ")"
                  << "  オフセット=(" << static_cast<int>(offset.x)
                  << ", " << static_cast<int>(offset.y) << ")"
                  << "  距離=" << static_cast<int>(distance) << "px"
                  << "  角度=" << static_cast<int>(angleDeg) << "deg"
                  << "  fitScore=" << cand.fitScore
                  << std::endl;
    }

    // ---- Step6: 重心の重心を計算 ----
    if (!candidates.empty())
    {
        cv::Point2f groupCenter(0.0f, 0.0f);                         // groupCenterを定義, float型で
        for (const auto &cand : candidates)                          /*auto:型を自動推論、　&:参照, cand:各要素受け取る変数名, const:ループ内でcandを変更しない,
                                     candidates:ループする対象のコンテナ（配列・ベクトルなど）*/
            groupCenter += cand.centroid;                            // 座標の合計を出す
        groupCenter *= 1.0f / static_cast<float>(candidates.size()); // 全重心の平均

        // 画像に描画
        cv::circle(result, groupCenter, 8, cv::Scalar(0, 255, 255), -1); // 黄色の点
        cv::putText(result, "CENTER", cv::Point(groupCenter.x + 10, groupCenter.y - 10),
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 255), 2);

        // 画像中心からの相対位置
        cv::Point2f offset = groupCenter - imageCenter;
        std::cout << "\n重心の重心: (" << static_cast<int>(groupCenter.x)
                  << ", " << static_cast<int>(groupCenter.y) << ")"
                  << "  オフセット=(" << static_cast<int>(offset.x)
                  << ", " << static_cast<int>(offset.y) << ")"
                  << "  距離=" << static_cast<int>(cv::norm(offset)) << "px"
                  << std::endl;
    }

    if (!cv::imwrite("color_shape_result.jpeg", result))
    {
        std::cerr << "エラー: 画像の保存に失敗しました。" << std::endl;
        return -1;
    }
    std::cout << "\n結果を color_shape_result.jpeg に保存しました。" << std::endl;
    return 0;
}
