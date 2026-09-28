#include "io/camera.hpp"
#include "tasks/apriltag_detector.hpp"
#include "opencv2/opencv.hpp"
#include "tools/img_tools.hpp"

int main()
{
    // 初始化相机、apriltag类
    Camera camera;
    auto_charge::AprilTagDetector detector("./configs/yolo.yaml");

    while (1) {
        // 调用相机读取图像
        cv::Mat img = camera.read();
        if (img.empty()) continue;

        // 调用检测器识别opencv标志
        auto tags = detector.detect(img);

        // 把识别到的标志画出来：闭合四边形 + id
        for (const auto & tag : tags) {
            tools::draw_points(img, tag.corners, {0, 255, 0});
            tools::draw_text(img, std::to_string(tag.id), tag.center, {0, 255, 0});
        }

        // 显示图像
        cv::resize(img, img, cv::Size(640, 480));
        cv::imshow("img", img);
        if (cv::waitKey(1) == 'q') {
            break;
        }
    }

    return 0;
}