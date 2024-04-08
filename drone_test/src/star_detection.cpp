#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <drone_test/detection.h>
#include <chrono>

using namespace std;
using namespace cv;

Mat depth_image;
Mat color_image;
Mat mask_image;
vector<Vec4i> lines;
double target = 1100; // 目标深度值
double dt = 1;        // PID控制器采样时间
int status = 0;
double cx = 0;
double last_cx = 0;
Point center_fitline; // 拟合直线的中心点
double last_depth = 0.0;
double last_depth_1 = 1100;
double last_depth_2 = 1100;

// 黑色hsv值
int minH_1 = 0, maxH_1 = 100;
int minS_1 = 3, maxS_1 = 30;
int minV_1 = 0, maxV_1 = 140;

// 红色hsv值
int minH = 0, maxH = 190;
int minS = 80, maxS = 180;
int minV = 68, maxV = 225;
// 滤波器
class Filter
{
public:
    Filter() : sum(0.0) {}

    double getAverage() const
    {
        return sum / data.size();
    }

    void update(double newValue)
    {
        sum += newValue;
        data.push_back(newValue);

        if (data.size() > 5)
        {
            sum -= data[0];
            data.erase(data.begin());
        }
    }

private:
    double sum;
    vector<double> data;
};

// PID控制器类
class PIDController
{
public:
    PIDController() : Kp(0), Ki(0), Kd(0), min_output(0), max_output(0), last_error(0), integral(0) {}

    void setParameters(double kp, double ki, double kd, double minoutput, double maxoutput)
    {
        Kp = kp;
        Ki = ki;
        Kd = kd;
        min_output = minoutput;
        max_output = maxoutput;
    }

    double update(double target, double feedback)
    {
        double error = target - feedback;
        integral += error * dt;
        double derivative = (error - last_error) / dt;
        double output = Kp * error + Ki * integral + Kd * derivative;
        last_error = error;
        output = std::max(output, min_output);
        output = std::min(output, max_output);
        return output;
    }

private:
    double Kp, Ki, Kd;
    double min_output, max_output;
    double last_error;
    double integral;
};

// 获取相机深度图像
void depthCb(const sensor_msgs::ImageConstPtr &msg)
{
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1); // 16位无符号整数单通道图像
        depth_image = cv_ptr->image;
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    waitKey(1);
    // imshow("depth", depth_image);
}

// 获取相机彩色图像
void colorCb(const sensor_msgs::ImageConstPtr &msg)
{
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
        color_image = cv_ptr->image;
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    waitKey(1);
    // imshow("color", color_image);
}

// 剔除端点横坐标值相差大于dis的直线
vector<Vec4i> filterLines(vector<Vec4i> &lines, int dis)
{
    vector<Vec4i> filtered_lines;
    for (size_t i = 0; i < lines.size(); i++)
    {
        Vec4i l = lines[i];
        if (abs(l[0] - l[2]) <= dis)
        {
            filtered_lines.push_back(l);
        }
    }
    return filtered_lines;
}

// 剔除深度值大于target_depth的直线
vector<Vec4i> filterLines_depth(vector<Vec4i> lines, int target_depth)
{
    vector<Vec4i> new_lines;
    for (size_t a = 0; a < lines.size(); a++)
    {
        Vec4i line = lines[a];
        int x1 = line[0], y1 = line[1], x2 = line[2], y2 = line[3];
        int center_x = (x1 + x2) / 2;
        int center_y = (y1 + y2) / 2;
        int depth = depth_image.at<ushort>(center_y, center_x);
        if (depth < target_depth)
        {
            new_lines.push_back(line);
        }
    }
    return new_lines;
}

// 拟合直线
void getLine(Mat &color_image, vector<Vec4i> &lines)
{
    // 获取当前时间点
    auto start_1 = std::chrono::high_resolution_clock::now();
    vector<Point> points;
    for (size_t i = 0; i < lines.size(); i++)
    {
        Vec4i l = lines[i];
        points.push_back(Point(l[0], l[1]));
        points.push_back(Point(l[2], l[3]));
    }

    if (points.size() < 2)
    {
        return;
    }
    // 获取当前时间点
    auto end_1 = std::chrono::high_resolution_clock::now();
    // 计算函数执行的时间
    std::chrono::duration<double> duration_1 = end_1 - start_1;
    double timeInSeconds = duration_1.count();
    if (timeInSeconds > 1)
    {
        ROS_ERROR("get points takes too long");
        ROS_ERROR("timeInSeconds: %f", timeInSeconds);
    }

    Vec4f line_params;
    // 获取当前时间点
    start_1 = std::chrono::high_resolution_clock::now();
    fitLine(points, line_params, DIST_L2, 0, 0.01, 0.01);
    // 获取当前时间点
    end_1 = std::chrono::high_resolution_clock::now();
    // 计算函数执行的时间
    duration_1 = end_1 - start_1;
    timeInSeconds = duration_1.count();
    if (timeInSeconds > 1)
    {
        ROS_ERROR("fitline takes too long");
        ROS_ERROR("timeInSeconds: %f", timeInSeconds);
        ROS_ERROR("params: %f, %f, %f, %f", line_params[0], line_params[1], line_params[2], line_params[3]);
    }

    // 获取当前时间点
    auto start_2 = std::chrono::high_resolution_clock::now();
    double k = line_params[1] / line_params[0];
    // 获取当前时间点
    auto end_2 = std::chrono::high_resolution_clock::now();
    // 计算函数执行的时间
    std::chrono::duration<double> duration_2 = end_2 - start_2;
    timeInSeconds = duration_2.count();
    if (timeInSeconds > 1)
    {
        ROS_ERROR("k takes too long");
        ROS_ERROR("timeInSeconds: %f", timeInSeconds);
    }

    // 获取当前时间点
    start_2 = std::chrono::high_resolution_clock::now();
    int x1 = 0;
    int y1 = k * (x1 - line_params[2]) + line_params[3];
    int x2 = color_image.cols;
    int y2 = k * (x2 - line_params[2]) + line_params[3];
    cx = line_params[2];
    // 获取当前时间点
    end_2 = std::chrono::high_resolution_clock::now();
    // 计算函数执行的时间
    duration_2 = end_2 - start_2;
    timeInSeconds = duration_2.count();
    if (timeInSeconds > 1)
    {
        ROS_ERROR("+- takes too long");
        ROS_ERROR("timeInSeconds: %f", timeInSeconds);
    }

    if (k < 4.0 && k > -4.0)
    {
        cx = last_cx;
    }
    else
    {
        last_cx = cx;
        line(color_image, Point(x1, y1), Point(x2, y2), Scalar(255, 0, 0), 2);
    }
}

// 图像预处理
Mat process(vector<int> color_threshold)
{
    Mat hsv_image;
    cvtColor(color_image, hsv_image, COLOR_BGR2HSV);
    // imshow("hsv", hsv_image);

    Mat threshold_image;
    int h_min = color_threshold[0];
    int h_max = color_threshold[1];
    int s_min = color_threshold[2];
    int s_max = color_threshold[3];
    int v_min = color_threshold[4];
    int v_max = color_threshold[5];

    inRange(hsv_image, Scalar(h_min, s_min, v_min), Scalar(h_max, s_max, v_max), threshold_image);
    // imshow("threshold", threshold_image);

    return threshold_image;
}

// 对直线进行处理
vector<Vec4i> lines_process(Mat src)
{
    Mat structure_close = getStructuringElement(MORPH_RECT, Size(7, 7));
    Mat structure_open = getStructuringElement(MORPH_RECT, Size(3, 3));
    Mat close, open;

    // 闭运算，连接特征点
    morphologyEx(src, close, MORPH_CLOSE, structure_close);
    // morphologyEx(close, close, MORPH_CLOSE, structure_close);
    // imshow("close of line", close);

    // 开运算，消除毛刺和小轮廓
    morphologyEx(close, open, MORPH_OPEN, structure_open);
    // imshow("open of line", open);

    vector<Vec4i> lines;
    // 获取当前时间点
    auto start = std::chrono::high_resolution_clock::now();
    HoughLinesP(open, lines, 1, CV_PI / 180, 80, 100, 10);
    // 获取当前时间点
    auto end = std::chrono::high_resolution_clock::now();
    // 计算函数执行的时间
    std::chrono::duration<double> duration = end - start;
    double timeInSeconds = duration.count();
    if (timeInSeconds > 1)
    {
        ROS_ERROR("HoughLinesP takes too long");
    }

    if (lines.size() > 0)
    {
        lines = filterLines(lines, 40);

        lines = filterLines_depth(lines, 2700);
        for (size_t i = 0; i < lines.size(); i++)
        {
            Vec4i l = lines[i];
            line(color_image, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0, 0, 255), 2);
        }

        // 获取当前时间点
        auto start = std::chrono::high_resolution_clock::now();
        getLine(color_image, lines);
        // 获取当前时间点
        auto end = std::chrono::high_resolution_clock::now();
        // 计算函数执行的时间
        std::chrono::duration<double> duration = end - start;
        double timeInSeconds = duration.count();
        if (timeInSeconds > 1)
        {
            ROS_ERROR("getLine() takes too long");
        }
    }
    return lines;
}

// 对圆进行处理
double get_circle_depth(Mat src)
{
    double erro_circle;
    int max_depth = 0;
    Mat structure_close = getStructuringElement(MORPH_RECT, Size(5, 5));
    Mat structure_open = getStructuringElement(MORPH_RECT, Size(5, 5));
    Mat close, open;

    // 闭运算，连接特征点
    morphologyEx(src, open, MORPH_OPEN, structure_open);
    morphologyEx(open, close, MORPH_CLOSE, structure_close);
    // morphologyEx(src, close, MORPH_CLOSE, structure_close);
    // morphologyEx(close, open, MORPH_OPEN, structure_open);
    // imshow("close", close);
    // imshow("open", open);

    vector<std::pair<int, int>> leftPoints, rightPoints;
    int midY = 0.75 * depth_image.rows / 2;
    int splitX = depth_image.cols / 2;
    imshow("open", open);
    for (int y = midY - 8; y < midY + 8; y++)
    {
        for (int x = 0; x < depth_image.cols; ++x)
        {
            int depthValue = depth_image.at<ushort>(y, x);
            int color_value = open.at<uchar>(y, x);

            if (depthValue > 50 && depthValue < 1800)
            {
                // ROS_INFO("depthValue=%d", depthValue);
                if (x < splitX)
                {
                    leftPoints.push_back(std::make_pair(x, depthValue));
                }
                else
                {
                    rightPoints.push_back(std::make_pair(x, depthValue));
                }
            }
        }
    }
    // ROS_INFO("max_depth=%d", max_depth);

    int sum1 = 0, sum2 = 0;
    for (const auto &point : leftPoints)
        sum1 += point.second;
    for (const auto &point : rightPoints)
        sum2 += point.second;

    int depth1 = (leftPoints.empty() ? 0 : sum1 / leftPoints.size());
    int depth2 = (rightPoints.empty() ? 0 : sum2 / rightPoints.size());

    // ROS_INFO("leftPoints.size() = %d", leftPoints.size());
    // ROS_INFO("rightPoints.size() = %d", rightPoints.size());
    // 添加滤波，如果突然出现一个空点，那么就不要管它
    if (depth1)
    {
        last_depth_1 = depth1;
    }
    else
    {
        depth1 = last_depth_1;
    }
    if (depth2)
    {
        last_depth_2 = depth2;
    }
    else
    {
        depth2 = last_depth_2;
    }
    ROS_INFO("depth1=%d", depth1);
    ROS_INFO("depth2=%d", depth2);
    erro_circle = depth1 - depth2;
    return erro_circle;
}

// 提取直线特定位置的深度值
double getLineDepth_plus(vector<Vec4i> lines, int pos_y)
{
    double sum_all = 0;
    int line_size = lines.size();
    for (size_t a = 0; a < lines.size(); a++)
    {
        int depth_sum = 0;   // 深度值的总和
        int valid_count = 0; // 有效深度值的数量
        Vec4i line = lines[a];
        int x1 = line[0], y1 = line[1], x2 = line[2], y2 = line[3];
        double avg_depth = 0;
        // 判断直线是否跨越中间水平线
        double y1_y2 = (y1 - pos_y) * (y2 - pos_y);
        if (y1_y2 < 0)
        {
            // 计算直线长度和角度
            double line_length = sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
            double line_angle;
            if (x1 == x2)
            {
                line_angle = 3.1415926 / 2;
            }
            else
            {
                line_angle = atan2(y2 - y1, x2 - x1);
            }

            // 获取直线上的点的深度值
            for (double i = 0; i < line_length; i += 1) // 步长
            {
                int x = x1 + i * cos(line_angle);
                int y = y1 + i * sin(line_angle);

                // 判断当前点是否在中间水平线上
                if (abs(y - pos_y) < 5)
                {
                    // 从深度图像中获取深度值
                    int depth = depth_image.at<ushort>(y, x);
                    // 如果深度值在有效范围内，则累加到depth_sum中
                    if (depth > 0 && depth < 3000)
                    {
                        depth_sum = depth_sum + depth;
                        valid_count = valid_count + 1;
                    }
                }
            }

            // 计算平均深度
            // ROS_INFO("valid_count=%d", valid_count);
            if (valid_count != 0)
            {
                avg_depth = double(depth_sum / valid_count);
            }
            else
            {
                avg_depth = 0.0;
            }

            if (!avg_depth)
            {
                line_size = line_size - 1;
            }
            else
            {
                sum_all += avg_depth;
            }
        }
        else
        {
            line_size = line_size - 1;
        }
    }
    if (line_size == 0)
    {
        return last_depth;
    }
    else
    {
        last_depth = sum_all / line_size;
        return sum_all / line_size;
    }
}

// 订阅状态变量
void doOpencv_sub(const drone_test::detection::ConstPtr &msg)
{
    status = msg->start_opencv;
}

Filter filter_erro_circle;
Filter filter_depth;
PIDController pid_controller_depth;
PIDController pid_controller_angle;
PIDController pid_controller_position;

int main(int argc, char **argv)
{
    cout << "start" << endl;
    ros::init(argc, argv, "code_test_3");
    ros::NodeHandle nh;

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber depth_sub = it.subscribe("/d400/aligned_depth_to_color/image_raw", 1, depthCb);
    image_transport::Subscriber color_sub = it.subscribe("/d400/color/image_raw", 1, colorCb);
    // image_transport::Subscriber depth_sub = it.subscribe("/camera/aligned_depth_to_color/image_raw", 2, depthCb);
    // image_transport::Subscriber color_sub = it.subscribe("/camera/color/image_raw", 2, colorCb);
    ros::Publisher detection_pub = nh.advertise<drone_test::detection>("detection_stage1", 1);
    ros::Publisher detection_pub_stage2 = nh.advertise<drone_test::detection>("detection_stage2", 1);
    ros::Subscriber Opencv_sub = nh.subscribe<drone_test::detection>("detection_status", 10, doOpencv_sub);

    ros::Rate rate(20);
    drone_test::detection erro;
    // 初始化PID控制器
    pid_controller_depth.setParameters(0.2, 0.0, 0.2, -300.0, 300.0);
    pid_controller_angle.setParameters(1.0, 0.0, 0.2, -300.0, 300.0);
    pid_controller_position.setParameters(0.4, 0.0, 0.1, -300.0, 300.0);

    namedWindow("Trackbars", WINDOW_NORMAL);
    resizeWindow("Trackbars", 640, 480);

    createTrackbar("Min H", "Trackbars", &minH, 255);
    createTrackbar("Max H", "Trackbars", &maxH, 255);
    createTrackbar("Min S", "Trackbars", &minS, 255);
    createTrackbar("Max S", "Trackbars", &maxS, 255);
    createTrackbar("Min V", "Trackbars", &minV, 255);
    createTrackbar("Max V", "Trackbars", &maxV, 255);

    // createTrackbar("Min H", "Trackbars", &minH_1, 255);
    // createTrackbar("Max H", "Trackbars", &maxH_1, 255);
    // createTrackbar("Min S", "Trackbars", &minS_1, 255);
    // createTrackbar("Max S", "Trackbars", &maxS_1, 255);
    // createTrackbar("Min V", "Trackbars", &minV_1, 255);
    // createTrackbar("Max V", "Trackbars", &maxV_1, 255);
    namedWindow("color", WINDOW_NORMAL);
    startWindowThread();
    while (ros::ok())
    {
        try
        {
            ros::spinOnce();
            if (color_image.size().empty() || depth_image.size().empty())
            {
                ROS_INFO("no image");
                continue;
            }
            // 输出检测颜色代码
            vector<vector<int>> color_thresholds =
                {
                    {minH, maxH, minV, maxV, minS, maxS},            // 红色阈值
                    {minH_1, maxH_1, minV_1, maxV_1, minS_1, maxS_1} // 绿色阈值
                };

            // 模式1，识别黑色杆并保持2m距离
            if (status == 1)
            {
                Mat mask = process(color_thresholds[1]);
                vector<Vec4i> lines = lines_process(mask);
                // 获取特定颜色直线的深度值并输入PID控制器
                if (lines.size())
                {
                    double depth = getLineDepth_plus(lines, depth_image.rows / 2);
                    double dx = pid_controller_angle.update(320, cx);
                    double dy = pid_controller_depth.update(target, depth);
                    ROS_INFO("depth=%lf", depth);
                    ROS_INFO("dx=%lf, dy=%lf", dx, dy);
                    erro.erro_y = dy;
                    erro.erro_x = dx;
                    detection_pub.publish(erro);
                }
            }

            // 模式2，识别红色圆并给出横向偏差
            if (status == 2)
            {
                // 获取特定颜色圆的左右两侧深度值偏差并输入滤波器
                Mat hsv = process(color_thresholds[0]);
                double erro_circle = get_circle_depth(hsv);
                // filter_erro_circle.update(erro_circle);
                // erro_circle = filter_erro_circle.getAverage();
                // imshow("hsv", hsv);
                //    获取直线深度值，横向偏移量
                Mat mask = process(color_thresholds[1]);
                // imshow("mask", mask);
                vector<Vec4i> lines = lines_process(mask);

                double dx, dz;
                double dy = -pid_controller_position.update(0, erro_circle);
                ROS_INFO("dy=%lf", dy);
                if (lines.size())
                {
                    dx = pid_controller_angle.update(320, cx);
                    double depth = getLineDepth_plus(lines, 0.75 * depth_image.rows);
                    // ROS_INFO("depth=%lf", depth);
                    filter_depth.update(depth);
                    dz = filter_depth.getAverage();
                    ROS_INFO("dz=%lf", dz);
                    dz = pid_controller_depth.update(1200, dz); // 保持1.3m距离
                    ROS_INFO("dx=%lf", dx);
                    ROS_INFO("pid dz=%lf", dz);
                }
                else
                {
                    dx = 0;
                    dz = 0;
                }
                erro.erro_y = dy;
                erro.erro_x = dx;
                erro.erro_z = dz;
                detection_pub_stage2.publish(erro);
            }

            // 获取图像尺寸
            int imageWidth = color_image.cols;
            int imageHeight = color_image.rows;

            // 计算十字的中心位置
            int centerX = imageWidth / 2;
            int centerY = imageHeight / 2;

            // 绘制十字
            cv::line(color_image, cv::Point(centerX, 0), cv::Point(centerX, imageHeight), cv::Scalar(0, 0, 255), 2);
            cv::line(color_image, cv::Point(0, centerY * 0.5), cv::Point(imageWidth, centerY * 0.5), cv::Scalar(0, 0, 255), 2);
            cv::line(color_image, cv::Point(0, 0.75 * depth_image.rows), cv::Point(imageWidth, 0.75 * depth_image.rows), cv::Scalar(0, 0, 255), 2);

            imshow("color", color_image);

            // imshow("depth", depth_image);
            if (waitKey(33) == 'q')
            {
                break;
            }
            rate.sleep();
            color_image.release();
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("Could not convert to image!");
        }
    }
    return 0;
}