#include <opencv2/opencv.hpp>
#include <iostream>

int main()
{
    // OpenCVのバージョンを表示して動作確認
    std::cout << "OpenCV Version: " << CV_VERSION << std::endl;

    // 画像を読み込む（ファイルパスは適宜変更）
    cv::Mat img = cv::imread("test.jpg");

    if (img.empty())
    {
        std::cout << "画像が読み込めませんでした。" << std::endl;
        return -1;
    }

    std::cout << "画像読み込み成功！ 幅: " << img.cols << " 高さ: " << img.rows << std::endl;
    return 0;
}