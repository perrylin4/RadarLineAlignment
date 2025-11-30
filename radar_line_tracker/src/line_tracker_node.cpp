#include <cstdio>
#include <cstring>
#include <cmath>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "opencv2/core/core_c.h"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

#include "MvCameraControl.h"

//==================== 可调参数集中区域 ====================

// 调试视图开关：1 显示窗口和 FPS，0 只做控制
#define ENABLE_DEBUG_VIEW 1

// 图像处理参数
static const double kThresh              = 20.0;          // 二值化阈值
static const int    kMorphKernelSize     = 3;             // 形态学卷积核尺寸
static const int    kRefineIterations    = 5;             // 线带缩窄迭代次数
static const double kInitialBandWidth    = 15.0;          // 初始线带宽度（像素）

// 误差滤波参数
static const double kErrorFilterAlpha    = 0.3;           // 一阶低通滤波系数（越小越平滑）

// PID 参数
static const double kAngleKp             = 1.0;
static const double kAngleKi             = 0.0;
static const double kAngleKd             = 0.1;
static const double kAngleOutMin         = -1.0;
static const double kAngleOutMax         =  1.0;

static const double kCenterKp            = 0.002;
static const double kCenterKi            = 0.0;
static const double kCenterKd            = 0.0;
static const double kCenterOutMin        = -0.3;
static const double kCenterOutMax        =  0.3;

// 运动控制参数
static const double kMaxLateralSpeed     = 0.2;           // 最大横向速度 linear.y
static const double kCenterMaxRatio      = 0.2;           // 中心误差归一化上限（画面宽度比例）

// 稳定后前进相关参数
static const double kAngleStableThresh   = 0.02;          // 角度稳定阈值（弧度，约 5.7 度）
static const double kCenterStableRatio   = 0.03;          // 中心稳定阈值（画面宽度比例）
static const double kRequiredStableTime  = 2.0;           // 需要连续稳定的时间（秒）
static const double kForwardSpeedWhenStable = 0.05;       // 稳定后缓慢前进的速度 linear.x

// 显示缩放比例
static const double kDisplayScale        = 0.5;           // 调试窗口缩放比例

//=========================================================

class PID {
public:
    PID(double kp, double ki, double kd, double outMin, double outMax)
        : kp_(kp), ki_(ki), kd_(kd),
                    outMin_(outMin), outMax_(outMax),
                    integral_(0.0), prevError_(0.0), firstRun_(true) {}

    void reset() {
        integral_ = 0.0;
        prevError_ = 0.0;
        firstRun_ = true;
    }

    double update(double error, double dt) {
        double p = kp_ * error;

        integral_ += error * dt;
        double i = ki_ * integral_;

        double d = 0.0;
        if (!firstRun_) {
            d = kd_ * (error - prevError_) / (dt > 1e-6 ? dt : 1e-6);
        } else {
            firstRun_ = false;
        }

        prevError_ = error;

        double out = p + i + d;
        if (out > outMax_) out = outMax_;
        if (out < outMin_) out = outMin_;
        return out;
    }

private:
    double kp_, ki_, kd_;
    double outMin_, outMax_;
    double integral_;
    double prevError_;
    bool firstRun_;
};

static void RGB2BGR(unsigned char* pRgbData, unsigned int nWidth, unsigned int nHeight)
{
    if (pRgbData == nullptr) return;

    for (unsigned int j = 0; j < nHeight; j++) {
        for (unsigned int i = 0; i < nWidth; i++) {
            unsigned char red = pRgbData[j * (nWidth * 3) + i * 3];
            pRgbData[j * (nWidth * 3) + i * 3]     = pRgbData[j * (nWidth * 3) + i * 3 + 2];
            pRgbData[j * (nWidth * 3) + i * 3 + 2] = red;
        }
    }
}

class LineTrackerNode : public rclcpp::Node {
public:
    LineTrackerNode()
        : Node("line_tracker_node"),
          handle_(nullptr),
          pData_(nullptr),
          nPayloadSize_(0),
                    anglePid_(kAngleKp,  kAngleKi,  kAngleKd,  kAngleOutMin,  kAngleOutMax),
                    centerPid_(kCenterKp, kCenterKi, kCenterKd, kCenterOutMin, kCenterOutMax)
    {
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        RCLCPP_INFO(this->get_logger(), "LineTrackerNode starting, initializing camera...");

        if (!init_camera()) {
            throw std::runtime_error("Failed to initialize camera");
        }

        double nowSec = static_cast<double>(cv::getTickCount()) / cv::getTickFrequency();
        lastTimeSec_ = nowSec;

#if ENABLE_DEBUG_VIEW
        fps_ = 0.0;
        lastTick_ = cv::getTickCount();
        fpsFrameCounter_ = 0;
        fpsAccumTime_ = 0.0;
#endif

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(33),
            std::bind(&LineTrackerNode::capture_and_control, this));
    }

    ~LineTrackerNode() override
    {
        stop_camera();
    }

private:
    bool init_camera()
    {
        int nRet = MV_OK;

        MV_CC_DEVICE_INFO_LIST stDeviceList;
        memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));

        nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
        if (MV_OK != nRet) {
            RCLCPP_ERROR(this->get_logger(), "Enum Devices fail! nRet [0x%x]", nRet);
            return false;
        }

        if (stDeviceList.nDeviceNum == 0) {
            RCLCPP_ERROR(this->get_logger(), "Find No Devices!");
            return false;
        }

        unsigned int nIndex = 0;
        if (nIndex >= stDeviceList.nDeviceNum) {
            RCLCPP_ERROR(this->get_logger(), "Camera index 0 not available.");
            return false;
        }

        if (false == MV_CC_IsDeviceAccessible(stDeviceList.pDeviceInfo[nIndex], MV_ACCESS_Exclusive)) {
            RCLCPP_ERROR(this->get_logger(), "Camera 0 is not accessible!");
            return false;
        }

        nRet = MV_CC_CreateHandle(&handle_, stDeviceList.pDeviceInfo[nIndex]);
        if (MV_OK != nRet) {
            RCLCPP_ERROR(this->get_logger(), "Create Handle fail! nRet [0x%x]", nRet);
            handle_ = nullptr;
            return false;
        }

        nRet = MV_CC_OpenDevice(handle_);
        if (MV_OK != nRet) {
            RCLCPP_ERROR(this->get_logger(), "Open Device fail! nRet [0x%x]", nRet);
            return false;
        }

        if (MV_GIGE_DEVICE == stDeviceList.pDeviceInfo[nIndex]->nTLayerType) {
            int nPacketSize = MV_CC_GetOptimalPacketSize(handle_);
            if (nPacketSize > 0) {
                nRet = MV_CC_SetIntValue(handle_, "GevSCPSPacketSize", nPacketSize);
                if (MV_OK != nRet) {
                    RCLCPP_WARN(this->get_logger(), "Set Packet Size fail! nRet [0x%x]", nRet);
                }
            } else {
                RCLCPP_WARN(this->get_logger(), "Get Packet Size fail! nRet [0x%x]", nPacketSize);
            }
        }

        nRet = MV_CC_SetEnumValue(handle_, "TriggerMode", 0);
        if (MV_OK != nRet) {
            RCLCPP_ERROR(this->get_logger(), "Set Trigger Mode fail! nRet [0x%x]", nRet);
            return false;
        }

        MVCC_INTVALUE stParam;
        memset(&stParam, 0, sizeof(MVCC_INTVALUE));
        nRet = MV_CC_GetIntValue(handle_, "PayloadSize", &stParam);
        if (MV_OK != nRet) {
            RCLCPP_ERROR(this->get_logger(), "Get PayloadSize fail! nRet [0x%x]", nRet);
            return false;
        }
        nPayloadSize_ = stParam.nCurValue;

        pData_ = static_cast<unsigned char*>(malloc(sizeof(unsigned char) * nPayloadSize_));
        if (nullptr == pData_) {
            RCLCPP_ERROR(this->get_logger(), "Allocate memory failed.");
            return false;
        }
        memset(pData_, 0, sizeof(unsigned char) * nPayloadSize_);

        nRet = MV_CC_SetEnumValue(handle_, "ExposureAuto", 0);
        if (MV_OK != nRet) {
            RCLCPP_WARN(this->get_logger(), "Set ExposureAuto Off fail! nRet [0x%x]", nRet);
        }

        nRet = MV_CC_SetFloatValue(handle_, "ExposureTime", 3000.0f);
        if (MV_OK != nRet) {
            RCLCPP_WARN(this->get_logger(), "Set ExposureTime fail! nRet [0x%x]", nRet);
        }

        nRet = MV_CC_SetFloatValue(handle_, "Gain", 16.0f);
        if (MV_OK != nRet) {
            RCLCPP_WARN(this->get_logger(), "Set Gain fail! nRet [0x%x]", nRet);
        }

        nRet = MV_CC_StartGrabbing(handle_);
        if (MV_OK != nRet) {
            RCLCPP_ERROR(this->get_logger(), "Start Grabbing fail! nRet [0x%x]", nRet);
            return false;
        }

        RCLCPP_INFO(this->get_logger(), "Camera initialized and grabbing started.");
        return true;
    }

    void stop_camera()
    {
        if (handle_ != nullptr) {
            MV_CC_StopGrabbing(handle_);
            MV_CC_CloseDevice(handle_);
            MV_CC_DestroyHandle(handle_);
            handle_ = nullptr;
        }

        if (pData_ != nullptr) {
            free(pData_);
            pData_ = nullptr;
        }
    }

    void capture_and_control()
    {
        if (!handle_ || !pData_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "Camera handle or buffer not ready.");
            return;
        }

        int nRet = MV_OK;
        MV_FRAME_OUT_INFO_EX stFrameInfo;
        memset(&stFrameInfo, 0, sizeof(MV_FRAME_OUT_INFO_EX));

        nRet = MV_CC_GetOneFrameTimeout(handle_, pData_, nPayloadSize_, &stFrameInfo, 1000);
        if (MV_OK != nRet) {
            RCLCPP_WARN(this->get_logger(), "Get Frame fail! nRet [0x%x]", nRet);
            return;
        }

        cv::Mat frame;
        if (PixelType_Gvsp_Mono8 == stFrameInfo.enPixelType) {
            frame = cv::Mat(stFrameInfo.nHeight, stFrameInfo.nWidth, CV_8UC1, pData_);
            cv::cvtColor(frame, frame, cv::COLOR_GRAY2BGR);
        } else if (PixelType_Gvsp_RGB8_Packed == stFrameInfo.enPixelType) {
            RGB2BGR(pData_, stFrameInfo.nWidth, stFrameInfo.nHeight);
            frame = cv::Mat(stFrameInfo.nHeight, stFrameInfo.nWidth, CV_8UC3, pData_);
        } else {
            RCLCPP_WARN(this->get_logger(), "Unsupported pixel format: %ld", stFrameInfo.enPixelType);
            return;
        }

        cv::Mat workImage = frame;

        static const cv::Mat kMorphKernel =
            cv::getStructuringElement(cv::MORPH_RECT,
                                      cv::Size(kMorphKernelSize, kMorphKernelSize));

        cv::Mat grayForBinary;
        cv::cvtColor(workImage, grayForBinary, cv::COLOR_BGR2GRAY);

        cv::Mat binary;
        cv::threshold(grayForBinary, binary, kThresh, 255, cv::THRESH_BINARY);
        cv::morphologyEx(binary, binary, cv::MORPH_OPEN, kMorphKernel);

        cv::Mat refinedMask = binary;
        std::vector<cv::Point> whitePoints;

        const int refineIterations = kRefineIterations;
        double bandWidth = kInitialBandWidth;

        for (int it = 0; it < refineIterations; ++it) {
            whitePoints.clear();
            cv::findNonZero(refinedMask, whitePoints);
            if (whitePoints.size() < 10) {
                break;
            }

            cv::Vec4f lineParam;
            cv::fitLine(whitePoints, lineParam, cv::DIST_L2, 0, 0.01, 0.01);
            float vx = lineParam[0];
            float vy = lineParam[1];
            float x0 = lineParam[2];
            float y0 = lineParam[3];

            double A_band = vy;
            double B_band = -vx;
            double C_band = -(A_band * x0 + B_band * y0);

            cv::Mat nextMask = cv::Mat::zeros(refinedMask.size(), CV_8UC1);
            for (const auto& p : whitePoints) {
                double x = p.x;
                double y = p.y;
                double dist = std::fabs(A_band * x + B_band * y + C_band) /
                              std::sqrt(A_band * A_band + B_band * B_band + 1e-12);
                if (dist <= bandWidth) {
                    nextMask.at<uint8_t>(p) = 255;
                }
            }

            refinedMask = nextMask;
            bandWidth *= 0.6;
        }

        whitePoints.clear();
        cv::findNonZero(refinedMask, whitePoints);

        double rAbs = 0.0;
        double angleError = 0.0;
        double centerError = 0.0;
        bool hasLine = false;

        if (!whitePoints.empty()) {
            cv::Vec4f lineParam;
            cv::fitLine(whitePoints, lineParam, cv::DIST_L2, 0, 0.01, 0.01);
            float vx = lineParam[0];
            float vy = lineParam[1];
            float x0 = lineParam[2];
            // float y0 = lineParam[3];

            float cx = static_cast<float>(workImage.cols) * 0.5f;
            float cy = static_cast<float>(workImage.rows) * 0.5f;
            // float x0c = x0 - cx;
            // float y0c = y0 - cy;

            double sumX = 0.0, sumY = 0.0;
            double sumXX = 0.0, sumYY = 0.0, sumXY = 0.0;
            int N = static_cast<int>(whitePoints.size());
            for (const auto& p : whitePoints) {
                double x = p.x - cx;
                double y = p.y - cy;
                sumX  += x;
                sumY  += y;
                sumXX += x * x;
                sumYY += y * y;
                sumXY += x * y;
            }

            double denomX = N * sumXX - sumX * sumX;
            double denomY = N * sumYY - sumY * sumY;
            double r = 0.0;
            if (denomX > 1e-6 && denomY > 1e-6) {
                double num = N * sumXY - sumX * sumY;
                r = num / std::sqrt(denomX * denomY);
            }

            rAbs = std::fabs(r);

            // 方向归一 + 误差滤波
            double vx_d = static_cast<double>(vx);
            double vy_d = static_cast<double>(vy);
            if (vy_d < 0.0) {
                vx_d = -vx_d;
                vy_d = -vy_d;
            }

            // 相对竖直的偏差角，范围约 (-pi/2, pi/2)
            double rawAngleError = std::atan2(vx_d, vy_d);

            // 中心误差（像素）
            float tMid = 0.0f;
            float lineCenterX = x0 + tMid * vx;
            float imgCenterX  = static_cast<float>(workImage.cols) * 0.5f;
            double rawCenterError = static_cast<double>(lineCenterX - imgCenterX);

            // 一阶低通滤波（指数滑动平均）
            if (!filterInitialized_) {
                filteredAngleError_  = rawAngleError;
                filteredCenterError_ = rawCenterError;
                filterInitialized_   = true;
            } else {
                filteredAngleError_  = kErrorFilterAlpha * rawAngleError  + (1.0 - kErrorFilterAlpha) * filteredAngleError_;
                filteredCenterError_ = kErrorFilterAlpha * rawCenterError + (1.0 - kErrorFilterAlpha) * filteredCenterError_;
            }

            angleError  = filteredAngleError_;
            centerError = filteredCenterError_;

            hasLine = true;
        }

        double nowSec = static_cast<double>(cv::getTickCount()) / cv::getTickFrequency();
        double dtControl = nowSec - lastTimeSec_;
        if (dtControl <= 0.0) dtControl = 1e-2;
        lastTimeSec_ = nowSec;

        geometry_msgs::msg::Twist cmd;
        if (hasLine) {
            double wAngle  = anglePid_.update(angleError, dtControl);
            double wCenter = centerPid_.update(centerError, dtControl);
            double wCmd    = wAngle + wCenter;

            double angleErrAbs  = std::fabs(angleError);
            double centerErrAbs = std::fabs(centerError);

            // 最大横向速度（左右平移），根据底盘能力可调
            const double vYMax = kMaxLateralSpeed;

            // 允许的最大中心误差（画面宽度的一定比例），用于归一化
            const double centerMax = workImage.cols * kCenterMaxRatio;

            // 归一化中心误差到 [0, 1]
            double normCenter = std::min(centerErrAbs / centerMax, 1.0);

            // 误差越大，横移越快；方向由误差符号决定
            double vyCmd = vYMax * normCenter;
            if (centerError > 0) {
                // 线在画面右侧：向右平移（若方向相反可整体取反）
                vyCmd = +vyCmd;
            } else if (centerError < 0) {
                // 线在画面左侧：向左平移
                vyCmd = -vyCmd;
            } else {
                vyCmd = 0.0;
            }

            // 误差稳定判定：当角度和中心误差都足够小，并且持续一段时间后，开始缓慢前进
            const double angleStableThresh  = kAngleStableThresh;                  // 约 5.7 度
            const double centerStableThresh = workImage.cols * kCenterStableRatio; // 画面宽度的 3%
            const double requiredStableTime = kRequiredStableTime;                 // 需要连续稳定一段时间

            if (angleErrAbs < angleStableThresh && centerErrAbs < centerStableThresh) {
                stableTimeAccum_ += dtControl;
            } else {
                stableTimeAccum_ = 0.0;
            }

            double vxCmd = 0.0;
            if (stableTimeAccum_ >= requiredStableTime) {
                // 误差小且稳定一段时间后，给一个很小的前进速度
                vxCmd = kForwardSpeedWhenStable;
            }

            cmd.linear.x  = vxCmd;  // 缓慢前进
            cmd.linear.y  = vyCmd;  // 同时允许左右微调
            cmd.angular.z = wCmd;   // 继续用角度 + 中心 PID 组合的角速度

            RCLCPP_DEBUG(this->get_logger(),
                         "Line OK: r=%.3f angleErr=%.3f centerErr=%.1f vx=%.2f vy=%.2f w=%.2f stableT=%.2f",
                         rAbs, angleError, centerError, vxCmd, vyCmd, wCmd, stableTimeAccum_);
        } else {
            cmd.linear.x  = 0.0;
            cmd.linear.y  = 0.0;
            cmd.angular.z = 0.0;
            stableTimeAccum_ = 0.0;
            RCLCPP_DEBUG(this->get_logger(), "No reliable line, stopping.");
        }

        cmd_pub_->publish(cmd);

#if ENABLE_DEBUG_VIEW
        int64_t nowTick = cv::getTickCount();
        double deltaSec = (nowTick - lastTick_) / cv::getTickFrequency();
        lastTick_ = nowTick;
        if (deltaSec > 0) {
            fpsAccumTime_ += deltaSec;
            fpsFrameCounter_++;

            if (fpsFrameCounter_ >= fpsUpdateInterval_) {
                fps_ = fpsFrameCounter_ / fpsAccumTime_;
                fpsFrameCounter_ = 0;
                fpsAccumTime_ = 0.0;
            }
        }

        char text[64] = {0};
        std::snprintf(text, sizeof(text), "FPS: %.1f", fps_);
        cv::putText(workImage, text, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX,
                    1.0, cv::Scalar(0, 255, 0), 2);

        cv::Mat display;
        double scale = kDisplayScale;
        cv::resize(workImage, display, cv::Size(), scale, scale, cv::INTER_AREA);

        cv::Mat binaryView;
        cv::resize(binary, binaryView, cv::Size(), scale, scale, cv::INTER_AREA);

        // 第三个视图：在二值图的缩放版本上叠加直线和信息
        cv::Mat lineViewDisplay;
        cv::cvtColor(binaryView, lineViewDisplay, cv::COLOR_GRAY2BGR);
        if (hasLine && !whitePoints.empty()) {
            cv::Vec4f lineParam;
            cv::fitLine(whitePoints, lineParam, cv::DIST_L2, 0, 0.01, 0.01);
            float vx = lineParam[0];
            float vy = lineParam[1];
            float x0 = lineParam[2];
            float y0 = lineParam[3];

            float t1 = -1000.0f;
            float t2 =  1000.0f;
            cv::Point2f p1 = cv::Point2f(x0 + t1 * vx, y0 + t1 * vy);
            cv::Point2f p2 = cv::Point2f(x0 + t2 * vx, y0 + t2 * vy);

            p1.x *= scale; p1.y *= scale;
            p2.x *= scale; p2.y *= scale;

            cv::line(lineViewDisplay, p1, p2, cv::Scalar(0, 0, 255), 2);

            char infoText[128] = {0};
            std::snprintf(infoText, sizeof(infoText), "|r| = %.3f", rAbs);
            cv::putText(lineViewDisplay, infoText, cv::Point(10, 25),
                        cv::FONT_HERSHEY_SIMPLEX, 0.6,
                        cv::Scalar(0, 255, 255), 1);

            double angleDeg = angleError * 180.0 / CV_PI;
            char eqText[128] = {0};
            std::snprintf(eqText, sizeof(eqText), "angle = %.1f deg", angleDeg);
            cv::putText(lineViewDisplay, eqText, cv::Point(10, 50),
                        cv::FONT_HERSHEY_SIMPLEX, 0.6,
                        cv::Scalar(0, 255, 0), 1);
        }

        cv::imshow("Camera", display);
        cv::imshow("Binary", binaryView);
        if (!lineViewDisplay.empty()) {
            cv::imshow("LaserLine", lineViewDisplay);
        }

        int key = cv::waitKey(1);
        if (key == 27) {
            RCLCPP_INFO(this->get_logger(), "ESC pressed, shutting down node.");
            rclcpp::shutdown();
        }
#endif
    }

    void* handle_;
    unsigned char* pData_;
    unsigned int nPayloadSize_;

    PID anglePid_;
    PID centerPid_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    double lastTimeSec_;

    // 简单一阶滤波后的误差，用于稳定控制
    double filteredAngleError_  = 0.0;
    double filteredCenterError_ = 0.0;
    bool   filterInitialized_   = false;

    // 误差稳定计时：用于判断何时可以开始缓慢前进
    double stableTimeAccum_     = 0.0;

#if ENABLE_DEBUG_VIEW
    double fps_;
    int64_t lastTick_;
    static constexpr int fpsUpdateInterval_ = 15;
    int fpsFrameCounter_;
    double fpsAccumTime_;
#endif
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    try {
        auto node = std::make_shared<LineTrackerNode>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        std::fprintf(stderr, "Exception in LineTrackerNode: %s\n", e.what());
    }
    rclcpp::shutdown();
    return 0;
}