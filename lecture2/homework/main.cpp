#include "io/camera.hpp"
#include "tasks/yolo.hpp"
#include "opencv2/opencv.hpp"
#include "tools/img_tools.hpp"

int main()
{
    // 初始化相机、yolo类
    Camera camera;
    auto_aim::YOLO yolo("./configs/yolo.yaml", false);
    int frame_count = 0;
    while (1) {
        // 调用相机读取图像
        cv::Mat img = camera.read();
        if (img.empty()) continue;
        // 调用yolo识别装甲板
        auto armors = yolo.detect(img, frame_count++);
        for (const auto & armor : armors) {
            // 装甲板关键点
            tools::draw_points(img, armor.points, {0, 255, 0});
            // 方框下方写出颜色+名称，如 "bluefour"（armor.color / armor.name 就是下标）
            auto text = auto_aim::COLORS[armor.color] + auto_aim::ARMOR_NAMES[armor.name];
            tools::draw_text(
              img, text, {armor.box.x, armor.box.y + armor.box.height + 25}, {0, 255, 0},3);
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
