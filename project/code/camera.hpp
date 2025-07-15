#ifndef CAMERA_H
#define CAMERA_H

#include <zf_common_headfile.h>

#define BEEP "/dev/zf_driver_gpio_beep"
using namespace cv;
using namespace std;
Mat ApplyInversePerspective(const Mat &input);
// 可配置参数
const int image_h = 120;                // 图像高度
const int image_w = 160;                // 图像宽度
const int wrapped_image_h = 100;        // 透视变换后图像高度
const int wrapped_image_w = 114;        // 透视变换后图像宽度
const int INTEGRAL_STEPS = 5;           // 积分步长
const int POINT_DISTANCE_THRESHOLD = 5; // 点间距阈值
const float FLYING_RATIO = 1;           // 飞行占空比
const int CIRCLE_THRESHOLD = 40;        // 圆环检测阈值
const int CIRCLE_ACCURACY = 10;         // 圆环检测精度
const int WIDTH_EXTEND = 90;            // 宽度扩展量
const int WIDTH_EXTEND2 = 70;           // 辅助宽度扩展
extern int debugmode;                   // true调试模式
extern int speed;
extern int ready_count; // 准备计数器

// 数据结构
struct TrackPoint
{
    Point position; // 点的位置
    int status;     // 点的状态（例如：0-无效，1-有效）
};

// circle state
enum CircleState
{
    CIRCLE_INACTIVE,       // 未检测到环岛
    RIGHT_CIRCLE_DETECTED, // 检测到环岛入口
    RIGHT_CIRCLE_INTRY,    // 右环岛入口
    RIGHT_CIRCLE_INSIDE,   // 正在环岛内循迹
    RIGHT_CIRCLE_EXITING,  // 正在离开环岛
    RIGHT_CIRCLE_DONE,
    LEFT_CIRCLE_DETECTED, // 检测到环岛入口
    LEFT_CIRCLE_INTRY,    // 左环岛入口
    LEFT_CIRCLE_INSIDE,   // 正在环岛内循迹
    LEFT_CIRCLE_EXITING,  // 正在离开环岛
    LEFT_CIRCLE_DONE,
    LEFT_TURN,
    RIGHT_TURN,
    STRAIGHT,
    CROSSING,
    UP,
    ZEBRA, // 斑马线
};

struct DetectionResult
{
    bool hasCross = false;         // 是否检测到十字路口
    bool hasZebraCrossing = false; // 是否检测到斑马线
    bool hasCircle = false;        // 是否检测到圆环
    Mat sourceImage;               // 原始图像
    Mat binaryImage;               // 二值化图像
    Mat outputImage;               // 输出图像
    Mat warpedImage;               // 逆透视变换后的图像
};

// 图像预处理：二值化
bool binaryThreshold(const Mat &input, Mat &binary, Mat &output);
VideoCapture cap_init(int camera);
// cv::Mat getLatestFrame(cv::VideoCapture& cap) ;
//  车道线处理类
class LaneProcessor
{
public:
    cv::Mat mapX, mapY;            // 存储像素映射关系（初始化时计算）
    bool isMapInitialized = false; // 是否已初始化映射表
    int startline;                 // 起始线
    int numPoints;
    int lastnumPoints; // 上一帧的点数
    bool lost = false;
    int roiHeight = image_h - 1;
    // member variables
    Point leftJumpPointA;
    Point rightJumpPointA;
    Point leftJumpPointB;
    Point rightJumpPointB;
    bool isleftJumpvalid = false;
    bool isrightJumpvalid = false;
    bool isleftCrossing = false; // 左车道是否在环岛内
    bool isrightCrossing = false; // 右车道是否在环岛内
    bool isleftLanecontinuous = false;
    bool isrightLanecontinuous = false;
    vector<Point> rightvirtualPath;
    vector<Point> leftvirtualPath;
    CircleState circleState = CIRCLE_INACTIVE;
    vector<int> whitePixels;      // 白点分布
    vector<TrackPoint> leftLane;  // 左车道
    vector<TrackPoint> rightLane; // 右车道
    vector<Point> centerLine;     // 中线
    int juli = 0; // 距离计算

    struct ControlParams
    {
        float speed_factor = 1.0f;
        float kp_switch = 1.0f; // 速度切换阈值
        float kd_switch = 1.0f; // 速度切换阈值
        float A_switch = 1.0f;
    };

    ControlParams getControlParams() const
    {
        return current_params;
    }

    // member function
    DetectionResult detect(const cv::Mat &inputImage);
    int juli_caculate(const cv::Mat &binaryImage);
    int speed_decide();//const vector<TrackPoint> &Leftline, const vector<TrackPoint> &Rightline
    void initializeVariables(int image_w, int image_h);
    void detectWhitePixels(const Mat &img, int roiHeight, std::vector<int> &whitePixels);
    bool detectZebraCrossing(const std::vector<int> &whitePixels);
    void detectLanePoints(const Mat &img, int roiHeight, const std::vector<int> &whitePixels,
                          std::vector<TrackPoint> &leftLane, std::vector<TrackPoint> &rightLane, int &leftMissedPoints, int &rightMissedPoints, float &leftMissedRadius, float &rightMissedRadius);
    void drawLanes(Mat &img, int roiHeight, vector<TrackPoint> &leftLane, vector<TrackPoint> &rightLane,
                   vector<Point> &centerLine);

    // 环岛处理
    void processCircle(vector<TrackPoint> &LeftLane,
                       vector<TrackPoint> &RightLane,
                       Mat &img, Mat &output, int &roiHeight, float &leftMissedRadius, float &rightMissedRadiu);
    void resetCircleState();

    // 辅助函数
    bool isLaneContinuous(const vector<TrackPoint> &lane);
    void findrightInflectionPoints(const vector<TrackPoint> &lane,
                                   Point &pointA, Point &pointB,
                                   bool &isValid,bool &isrightCrossing);
    void findleftInflectionPoints(const vector<TrackPoint> &lane, Point &pointA, Point &pointB, bool &isValid,bool &isleftCrossing);
    void generateVirtualPath(const Point2f &start, const Point2f &end,
                             vector<Point> &path,
                             bool isLeftLane);
    bool checkExitCondition(const vector<TrackPoint> &Lane, Mat &img, int roiHeight);
    bool isPathClear(const vector<TrackPoint> &leftLane);
    void mergeVirtualPath(vector<TrackPoint> &lane,
                          const vector<Point> &virtualPath,
                          float minY);
    void smoothPath(vector<Point> &path);
    void linearRegression(
        vector<Point>::iterator begin,
        vector<Point>::iterator end,
        float &k, float &b, float &r_squared);
    Point findSuddenChangePoint(const vector<TrackPoint> &points, bool isLeftLane, int y);
    float calculateLaneSlope(const std::vector<TrackPoint> &lane);
    bool isStraightLane(const std::vector<TrackPoint> &lane, int trend);
    map<int, float> calculateTrackWidthsByY(
        const std::vector<TrackPoint> &leftLane,
        const std::vector<TrackPoint> &rightLane);
    float getTrackWIdthFormY(int y);
    int checkLaneTrend(const std::vector<TrackPoint> &lane);

private:
    int left_straight[120] = {
    42, 42, 42, 42, 42, 43, 43, 43, 44, 44, 44, 44, 45, 45, 45, 46, 46, 46, 46, 47,
    47, 47, 47, 48, 48, 49, 49, 49, 50, 50, 50, 50, 51, 51, 51, 52, 52, 52, 53, 53,
    53, 53, 54, 54, 54, 55, 55, 55, 55, 56, 57, 57, 57, 58, 58, 58, 58, 59, 59, 59,
    59, 60, 60, 60, 60, 60, 61, 62, 62, 62, 62, 63, 63, 64, 64, 64, 65, 65, 65, 65,
    66, 66, 66, 66, 67, 67, 67, 68, 68, 69, 69, 70, 70, 70, 71, 71, 71, 72, 72, 72,
    73, 73, 73, 74, 74, 74, 75, 75, 75, 75, 76, 76, 76, 77, 77, 77, 77, 77, 77, 77,
};
    int right_straight[120] = {
    113, 113, 113, 112, 112, 112, 112, 112, 112, 111,
    111, 111, 111, 110, 110, 110, 110, 110, 109, 109,
    109, 108, 108, 108, 108, 107, 107, 107, 106, 106,
    106, 106, 106, 105, 105, 105, 105, 104, 104, 104,
    104, 103, 103, 103, 102, 102, 102, 101, 101, 101,
    100, 100, 100, 99, 99, 99, 99, 98, 98, 98, 98, 97,
    97, 97, 97, 96, 96, 96, 95, 95, 95, 95, 95, 94, 94,
    94, 93, 93, 93, 93, 92, 92, 92, 92, 92, 91, 91, 91, 
    90, 90, 90, 89, 89, 89, 89, 89, 88, 88, 88, 87, 87,
    87, 86, 86, 86, 86, 85, 85, 85, 84, 84, 84, 83, 83,
    83, 83, 83, 83, 83, 83
};
    ControlParams current_params;

    void updateControlParams()
    {
        switch (circleState)
        {
        case RIGHT_CIRCLE_DETECTED:
            current_params = {1.0f, 5.0f, 15.0f, 0.01f};
            break;
        case RIGHT_CIRCLE_INTRY:
            current_params = {0.725f, 10.0f, 20.0f, 0.1f}; // 速度倍数,Kp0,Kd0,A
            break;
        case RIGHT_CIRCLE_INSIDE:
            current_params = {0.725f, 10.0f, 20.0f, 0.2f};
            break;
        case LEFT_CIRCLE_DETECTED:
            current_params = {1.0f, 5.0f, 15.0f, 0.01f};
            break;
        case LEFT_CIRCLE_INTRY:
            current_params = {0.825f, 10.0f, 20.0f, 0.1f};
            break;
        case LEFT_CIRCLE_INSIDE:
            current_params = {0.725f, 1.0f, 20.0f, 0.2f};
            break;
        case RIGHT_TURN:
            current_params = {0.825f, 38.f, 35.0f, 0.2f};
            break;
        case LEFT_TURN:
            current_params = {0.825f, 38.f, 35.0f, 0.2f};
            break;
        case STRAIGHT:
            current_params = {1.0f,5.f, 10.0f, 0.0f};
            break;
        default:
            current_params = {0.825f, 15.0f, 20.0f, 0.0f}; // 默认参数
        }
    }
};

// 逆透视变换类

#endif