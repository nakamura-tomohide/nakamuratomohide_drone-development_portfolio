#include <iostream>
#include <opencv2/opencv.hpp>
int main()
{
    // image.pngをimgに代入
    cv::Mat img = cv::imread("images/itsme.jpg"); //(ディレクトリ/ファイル)
    if (img.empty())
    {
        std::cerr << "エラー: 画像を読み込めませんでした。パスを確認してください。" << std::endl;
        return -1;
    }

    // imgを白黒にしてimg_binに代入
    cv::Mat img_bin;
    cv::cvtColor(img, img_bin, cv::COLOR_BGR2GRAY);
    //(元画像、加工後画像、加工処理)

    // チャンネル数の確認
    std::cout << "img     : " << img.channels() << " channel" << std::endl; //\nとほぼ同じ
    // 3個のはず(RGB)
    std::cout << "img_bin : " << img_bin.channels() << " channel" << std::endl;
    // 1個のはず（グレースケール）

    // 第1引数: 保存するパス（ファイル名）
    // 第2引数: 保存したいMatオブジェクト（今回は白黒にしたimg_bin）
    if (cv::imwrite("output_gray.jpg", img_bin))
    {
        std::cout << "画像を保存しました！" << std::endl;
    }
    else
    {
        std::cerr << "エラー: 画像の保存に失敗しました。" << std::endl;
    }

    return 0;
}