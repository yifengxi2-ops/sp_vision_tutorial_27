#include <iostream>
#include <opencv2/opencv.hpp>

// ======================= 作业 =======================
// 1. 读取 ../assets/demo.jpg
// 2. 使用 cvtColor 把图像转为灰度图（颜色空间：BGR2GRAY）
// 3. 使用 imwrite 把灰度图保存为 gray.jpg
// 4. 在灰度图上用 circle 画一个圆，标记你要"瞄准"的位置
// 5. 显示灰度图，按任意键退出
// ====================================================

int main()
{
    // TODO: 在这里完成你的代码
    // 1. 读取 ../assets/demo.jpg
    cv::Mat img_origin=cv::imread("assets/demo.jpg");
    cv::Mat img_grey;
    // 2. 使用 cvtColor 把图像转为灰度图（颜色空间：BGR2GRAY）
    cv::cvtColor(img_origin,img_grey,cv::COLOR_BGR2GRAY);
    // 3. 使用 imwrite 把灰度图保存为 gray.jpg
    cv::imwrite("gray.jpg",img_grey);
    // 4. 在灰度图上用 circle 画一个圆，标记你要"瞄准"的位置
    cv::circle(img_grey, cv::Point(img_grey.cols/2, img_grey.rows/2), 80, cv::Scalar(0, 0, 255));
    // 5. 显示灰度图，按任意键退出
    imshow("grey",img_grey);
    cv::waitKey(0);

    return 0;
}
