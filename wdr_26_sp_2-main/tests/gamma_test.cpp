#include <fmt/core.h>

#include <fstream>
#include <yaml-cpp/yaml.h>

#include <chrono>
#include <opencv2/opencv.hpp>

#include "io/camera.hpp"
#include "tasks/auto_aim/detector.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

const std::string keys =
    "{help h usage ? |                        | 输出命令行参数说明 }"
    "{@config-path   | configs/demo_rw.yaml   | yaml配置文件的路径}"
    "{target-gray    | 80                     | 目标数字区域灰度值 }"
    "{tradition      | false                  | 是否使用传统方法识别}"
    "{gamma          | true                   | 是否启用gamma校正}";

//==============================================================================
// Gamma变换相关函数 shanxiadesu
//==============================================================================

cv::Mat applyGamma(const cv::Mat& src, double gamma) {
    cv::Mat dst;
    cv::Mat lut(1, 256, CV_8UC1);
    for (int i = 0; i < 256; i++) {
        lut.at<uchar>(i) = cv::saturate_cast<uchar>(pow(i / 255.0, gamma) * 255.0);
    }
    cv::LUT(src, lut, dst);
    return dst;
}

double estimateIllumination(const cv::Mat& gray_img) {
    cv::Mat hist;
    int histSize = 256;
    float range[] = { 0, 256 };
    const float* histRange = { range };
    cv::calcHist(&gray_img, 1, 0, cv::Mat(), hist, 1, &histSize, &histRange);

    double maxVal = 0;
    int maxIdx = 0;
    for (int i = 0; i < 200; i++) {   
        if (hist.at<float>(i) > maxVal) {
            maxVal = hist.at<float>(i);
            maxIdx = i;
        }
    }
    return maxIdx;
}

double estimateDigitRegionGray(const cv::Mat& gray_img, double lightbar_threshold) {
    cv::Mat lightbar_mask;
    cv::threshold(gray_img, lightbar_mask, lightbar_threshold, 255, cv::THRESH_BINARY);

    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
    cv::dilate(lightbar_mask, lightbar_mask, kernel);

    cv::Mat digit_region_mask;
    cv::dilate(lightbar_mask, digit_region_mask, kernel, cv::Point(-1, -1), 10);
    digit_region_mask = digit_region_mask - lightbar_mask;

    if (cv::countNonZero(digit_region_mask) > 100) {
        return cv::mean(gray_img, digit_region_mask)[0];
    }
    return estimateIllumination(gray_img);
}

double calculateAdaptiveGamma(double current_gray, double target_gray) {
    if (current_gray <= 0 || target_gray <= 0)
        return 1.0;
    double ratio = target_gray / current_gray;
    double gamma = 1.0 / ratio;
    gamma = std::max(0.3, std::min(3.0, gamma));
    return gamma;
}

int main(int argc, char* argv[]) {
    // 读取命令行参数
    cv::CommandLineParser cli(argc, argv, keys);
    if (cli.has("help")) {
        cli.printMessage();
        return 0;
    }
    auto config_path = cli.get<std::string>(0);
    auto use_tradition = cli.get<bool>("tradition");
    auto target_gray = cli.get<double>("target-gray");
    auto enable_gamma = cli.get<bool>("gamma");

    tools::Exiter exiter;

    // 读取配置文件中的阈值
    auto yaml = YAML::LoadFile(config_path);
    double threshold = yaml["threshold"].as<double>();

    io::Camera camera(config_path);
    auto_aim::Detector detector(config_path, true);
    auto_aim::YOLO yolo(config_path, true);

    fmt::print("========================================\n");
    fmt::print("相机检测测试 (带Gamma校正)\n");
    fmt::print("========================================\n");
    fmt::print("配置文件: {}\n", config_path);
    fmt::print("检测方法: {}\n", use_tradition ? "传统" : "YOLO");
    fmt::print("Gamma校正: {}\n", enable_gamma ? "启用" : "禁用");
    fmt::print("目标灰度: {:.1f}\n", target_gray);
    fmt::print("二值化阈值: {:.1f}\n", threshold);
    fmt::print("========================================\n");
    fmt::print("操作说明:\n");
    fmt::print("  +/-: 调整目标灰度值\n");
    fmt::print("  g:   增大gamma值 (手动模式)\n");
    fmt::print("  h:   减小gamma值 (手动模式)\n");
    fmt::print("  a:   切换自适应/手动模式\n");
    fmt::print("  c:   切换gamma校正开关\n");
    fmt::print("  q:   退出\n");
    fmt::print("========================================\n");

    std::chrono::steady_clock::time_point timestamp;
    auto last_timestamp = std::chrono::steady_clock::now();

    bool adaptive_mode = true;
    double manual_gamma = 1.0;
    int frame_count = 0;

    while (!exiter.exit()) {
        cv::Mat img;
        std::list<auto_aim::Armor> armors;

        camera.read(img, timestamp);
        auto img_dt = tools::delta_time(timestamp, last_timestamp);
        last_timestamp = timestamp;
        if (img.empty())
            break;

        frame_count++;
        cv::Mat detect_img = img; // 用于检测的图像
        double gamma = 1.0;
        double digit_region_gray = 0;

        // Gamma校正
        auto t1 = std::chrono::steady_clock::now();
        if (enable_gamma) {
            // 转换为灰度图估计光照
            cv::Mat gray_img;
            cv::cvtColor(img, gray_img, cv::COLOR_BGR2GRAY);

            // 估计光照
            digit_region_gray = estimateDigitRegionGray(gray_img, threshold);

            // 计算自适应gamma或使用手动gamma
            gamma = adaptive_mode ? calculateAdaptiveGamma(digit_region_gray, target_gray)
                                  : manual_gamma;

            // 应用gamma变换到彩色图像（保留颜色信息用于检测）
            cv::Mat hsv;
            cv::cvtColor(img, hsv, cv::COLOR_BGR2HSV);
            std::vector<cv::Mat> channels;
            cv::split(hsv, channels);
            channels[2] = applyGamma(channels[2], gamma); // V通道
            cv::merge(channels, hsv);
            cv::cvtColor(hsv, detect_img, cv::COLOR_HSV2BGR);
        }
        auto t2 = std::chrono::steady_clock::now();
        std::cout << tools::delta_time(t2, t1) << std::endl;

        auto last = std::chrono::steady_clock::now();

        if (use_tradition)
            armors = detector.detect(detect_img);
        else
            armors = yolo.detect(detect_img);

        auto now = std::chrono::steady_clock::now();
        auto dt = tools::delta_time(now, last);

        // 绘制检测结果
        cv::Mat show_img = detect_img.clone();
        for (const auto& armor: armors) {
            tools::draw_points(show_img, armor.points, { 0, 255, 0 }, 2);
            cv::putText(
                show_img,
                fmt::format("{:.2f}", armor.confidence),
                armor.center,
                cv::FONT_HERSHEY_SIMPLEX,
                0.5,
                cv::Scalar(0, 255, 0),
                1
            );
        }

        // 显示信息
        cv::putText(
            show_img,
            fmt::format("FPS: {:.1f}", 1.0 / dt),
            cv::Point(10, 30),
            cv::FONT_HERSHEY_SIMPLEX,
            0.6,
            cv::Scalar(0, 255, 0),
            2
        );
        cv::putText(
            show_img,
            fmt::format("Detected: {}", armors.size()),
            cv::Point(10, 55),
            cv::FONT_HERSHEY_SIMPLEX,
            0.5,
            cv::Scalar(255, 255, 0),
            1
        );

        if (enable_gamma) {
            cv::putText(
                show_img,
                fmt::format("Gamma: {:.2f} ({})", gamma, adaptive_mode ? "auto" : "manual"),
                cv::Point(10, 80),
                cv::FONT_HERSHEY_SIMPLEX,
                0.5,
                cv::Scalar(0, 255, 255),
                1
            );
            cv::putText(
                show_img,
                fmt::format("Digit Gray: {:.1f} / Target: {:.1f}", digit_region_gray, target_gray),
                cv::Point(10, 105),
                cv::FONT_HERSHEY_SIMPLEX,
                0.5,
                cv::Scalar(0, 255, 255),
                1
            );
        } else {
            cv::putText(
                show_img,
                "Gamma: OFF",
                cv::Point(10, 80),
                cv::FONT_HERSHEY_SIMPLEX,
                0.5,
                cv::Scalar(128, 128, 128),
                1
            );
        }

        cv::resize(show_img, show_img, cv::Size(), 0.5, 0.5);
        cv::namedWindow("Camera Detect Result", cv::WINDOW_NORMAL);
        cv::imshow("Camera Detect Result", show_img);

        auto key = cv::waitKey(1);
        if (key == 'q')
            break;
        if (key == '+' || key == '=') {
            target_gray = std::min(200.0, target_gray + 5.0);
            fmt::print("目标灰度: {:.1f}\n", target_gray);
        }
        if (key == '-') {
            target_gray = std::max(20.0, target_gray - 5.0);
            fmt::print("目标灰度: {:.1f}\n", target_gray);
        }
        if (key == 'h') {
            manual_gamma = std::max(0.3, manual_gamma - 0.1);
            fmt::print("手动gamma: {:.2f}\n", manual_gamma);
        }
        if (key == 'g') {
            manual_gamma = std::min(3.0, manual_gamma + 0.1);
            fmt::print("手动gamma: {:.2f}\n", manual_gamma);
        }
        if (key == 'a') {
            adaptive_mode = !adaptive_mode;
            fmt::print("模式: {}\n", adaptive_mode ? "自适应" : "手动");
        }
        if (key == 'c') {
            enable_gamma = !enable_gamma;
            fmt::print("Gamma校正: {}\n", enable_gamma ? "启用" : "禁用");
        }

        // 每100帧打印信息
        if (frame_count % 100 == 0) {
            tools::logger()->info(
                "Frame {} | FPS: {:.1f} | Gamma: {:.2f} | Detected: {}",
                frame_count,
                1.0 / dt,
                gamma,
                armors.size()
            );
        }
    }

    fmt::print("\n测试结束\n");
    return 0;
}
