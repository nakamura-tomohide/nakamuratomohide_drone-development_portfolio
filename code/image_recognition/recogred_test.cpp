#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>

int main()
{
    cv::Mat redimg = cv::imread("images/redimage.jpeg", cv::IMREAD_COLOR); // 一旦BGRで取り込む
    if (redimg.empty())
    {
        std::cerr << "エラー: 画像を読み込めませんでした。パスを確認してください。" << std::endl;
        return -1;
    }

    cv::Size imgSize = redimg.size();
    int imgWidth = imgSize.width;
    int imgHeight = imgSize.height;
    std::cout << "Image size: " << imgWidth << "x" << imgHeight << std::endl;

    // HSV色空間に変換
    cv::Mat hsv;                                  // Matオブジェクトを定義
    cv::cvtColor(redimg, hsv, cv::COLOR_BGR2HSV); //(元画像、加工後画像、加工処理)

    // inRange によって Hue が特定の範囲にある領域の mask を取得(二値化)
    cv::Mat mask1, mask2, mask_sum; // RedはHueにおいて0を跨ぐ
    cv::inRange(hsv, cv::Scalar(0, 100, 100), cv::Scalar(10, 255, 255), mask1);
    cv::inRange(hsv, cv::Scalar(170, 100, 100), cv::Scalar(180, 255, 255), mask2);
    mask_sum = mask1 | mask2;
    double max;
    double min;
    cv::minMaxLoc(mask_sum, &min, &max); // 最小値と最大値の確認
    /*&:その変数が保存されている場所（アドレス）を教えてあげる
    なぜこれが必要かというと、cv::minMaxLoc という関数は一度に2つ以上の結果（最小値と最大値）を返したいから
    普通の関数は1つしか答えを返せない*/
    std::cout << min << std::endl;
    std::cout << max << std::endl;

    cv::Mat res;
    cv::bitwise_and(redimg, redimg, res, mask_sum); // maskによって切り抜き

    // 描画
    // 抽出した輪郭データを保存する容器
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    // Vec4i は「4個の整数(int)のセット」という意味で、各輪郭につき [次の輪郭, 前の輪郭, 最初の子, 親] というインデックス番号が格納される
    cv::findContours(mask_sum, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    // mask_sum（白黒画像）から輪郭を探して contours に格納します

    // OpenCVの関数で一気に描画
    cv::drawContours(redimg, contours, -1, cv::Scalar(0, 255, 0), 2);

    // 輪郭の情報をコンソールに表示（確認用）
    std::cout << "見つかった輪郭の数: " << contours.size() << std::endl;
    // 第1引数: 保存するパス（ファイル名）
    // 第2引数: 保存したいMatオブジェクト（今回は結果(res)）
    if (cv::imwrite("recogred.jpeg", res))
    {
        std::cout << "マスク処理後画像を保存しました！" << std::endl;
    }
    else
    {
        std::cerr << "エラー: 画像の保存に失敗しました。" << std::endl;
    }

    // 描画結果（輪郭が描かれた画像）を保存
    if (cv::imwrite("recogred_with_contours.jpeg", redimg))
    {
        std::cout << "輪郭を描画した画像を保存しました！" << std::endl;
    }
    else
    {
        std::cerr << "エラー: 画像の保存に失敗しました。" << std::endl;
    }
    return 0;
}