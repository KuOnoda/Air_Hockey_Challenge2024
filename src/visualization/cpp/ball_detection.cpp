#include <opencv2/opencv.hpp>
#include <iostream>
#include <tuple>
#include <vector>

class CameraTracker
{
public:
    CameraTracker(int dev_num = -1, int capture_time = 5000, bool use_camera = true,
                  int search_range_devices = 100, int threshold_of_contour_length = 100, int frequency = 200)
        : dev_num(dev_num), capture_time(capture_time), use_camera(use_camera),
          search_range_devices(search_range_devices), threshold_of_contour_length(threshold_of_contour_length),
          frequency(frequency), previous_cx(0), previous_cy(0) {}

    void find_camera_device()
    {
        if (dev_num == -1)
        {
            for (int i = 0; i < search_range_devices; ++i)
            {
                cv::VideoCapture cap(i);
                if (cap.isOpened())
                {
                    dev_num = i;
                    cap.release();
                    std::cout << "Camera device number is " << dev_num << std::endl;
                    return;
                }
            }
            std::cerr << "Could not find a camera device within " << search_range_devices
                      << " devices. Please confirm the cable is correctly connected." << std::endl;
            exit(1);
        }
    }

    void setup_camera()
    {
        if (use_camera)
        {
            cap.open(dev_num);
        }
        else
        {
            cap.open("../data/air_hockey.mp4");
        }
        cap.set(cv::CAP_PROP_FPS, frequency);
        cap.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
        cap.set(cv::CAP_PROP_FRAME_HEIGHT, 720);
    }

    std::tuple<int, int, float, float> track_when_it_called()
    {
        cv::Mat frame;
        if (!cap.read(frame))
        {
            std::cerr << "Failed to capture frame." << std::endl;
            return {0, 0, 0.0f, 0.0f};
        }

        // HSV変換による緑色検出
        cv::Mat hsv;
        cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
        cv::Scalar lower_green(30, 50, 50), upper_green(90, 255, 255);
        cv::Mat mask, res;
        cv::inRange(hsv, lower_green, upper_green, mask);
        cv::bitwise_and(frame, frame, res, mask);

        // グレースケールに変換して輪郭検出
        cv::Mat gray;
        cv::cvtColor(res, gray, cv::COLOR_BGR2GRAY);
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(gray, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

        int cx = 0, cy = 0;
        for (const auto &contour : contours)
        {
            if (cv::contourArea(contour) > threshold_of_contour_length)
            {
                cv::Moments moment = cv::moments(contour);
                if (moment.m00 != 0)
                {
                    cx = static_cast<int>(moment.m10 / moment.m00);
                    cy = static_cast<int>(moment.m01 / moment.m00);
                    cv::circle(frame, cv::Point(cx, cy), 5, cv::Scalar(0, 0, 255), -1);
                    cv::putText(frame, "Center: (" + std::to_string(cx) + ", " + std::to_string(cy) + ")",
                                cv::Point(cx + 10, cy - 10), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
                    std::cout << "Center: (" << cx << ", " << cy << ")" << std::endl;
                }
            }
        }

        float velocity_x = static_cast<float>(cx - previous_cx) * 1000 / frequency;
        float velocity_y = static_cast<float>(cy - previous_cy) * 1000 / frequency;
        previous_cx = cx;
        previous_cy = cy;

        cv::imshow("Tracking", frame);
        cv::waitKey(1);

        return {cx, cy, velocity_x, velocity_y};
    }

    void release_resources()
    {
        if (cap.isOpened())
        {
            cap.release();
        }
        cv::destroyAllWindows();
    }

private:
    int dev_num, capture_time, search_range_devices, threshold_of_contour_length, frequency;
    int previous_cx, previous_cy;
    bool use_camera;
    cv::VideoCapture cap;
};

int main()
{
    CameraTracker tracker;
    tracker.find_camera_device();
    tracker.setup_camera();

    while (true)
    {
        auto [cx, cy, vx, vy] = tracker.track_when_it_called();
        std::cout << "Center: (" << cx << ", " << cy << "), Velocity: (" << vx << ", " << vy << ")" << std::endl;
    }

    tracker.release_resources();
    return 0;
}
