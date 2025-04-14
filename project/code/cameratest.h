#ifndef CAMERA_H
#define CAMERA_H


#include<zf_common_headfile.h>

using namespace cv;
using namespace std;

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
extern int debugmode;                  // true调试模式



// 数据结构
struct TrackPoint
{
    Point position; // 点的位置
    int status;     // 点的状态（例如：0-无效，1-有效）
};

// circle state
enum CircleState
{
    CIRCLE_INACTIVE,    // 未检测到环岛
    CIRCLE_DETECTED,       // 检测到环岛入口
    CIRCLE_APPROACHING, // 正在接近环岛（检测到入口）
    CIRCLE_INSIDE,      // 正在环岛内循迹
    CIRCLE_EXITING,      // 正在离开环岛
    LEFT_TURN,
    RIGHT_TURN,
    CROSSING
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
bool binaryThreshold(const Mat &input, Mat &output);

// 车道线处理类
class LaneProcessor
{
public:

    cv::Mat mapX, mapY;  // 存储像素映射关系（初始化时计算）
    bool isMapInitialized = false;  // 是否已初始化映射表
// member variables
    Point leftJumpPointA;
    Point rightJumpPointA;
    Point leftJumpPointB;
    Point rightJumpPointB;
    bool isleftJumpvalid = false;
    bool isrightJumpvalid= false;
    vector<Point> rightvirtualPath;
    vector<Point> leftvirtualPath;
    CircleState circleState = CIRCLE_INACTIVE;
    vector<int> whitePixels;                       // 白点分布
    vector<TrackPoint> leftLane;  // 左车道
    vector<TrackPoint> rightLane; // 右车道
    vector<TrackPoint> rightcicle;      // right_circle
    vector<TrackPoint> leftcicle;
    vector<Point> centerLine; // 中线


    // member function
    DetectionResult detect(const cv::Mat &inputImage);
    void initializeVariables(int image_w, int image_h);
    void detectWhitePixels(const Mat &img, int roiHeight, std::vector<int> &whitePixels);
    bool detectZebraCrossing(const std::vector<int> &whitePixels,float &leftMissedRadius, float &rightMissedRadius);
    void detectLanePoints(const Mat &img, int roiHeight, const std::vector<int> &whitePixels,
                                 std::vector<TrackPoint> &leftLane, std::vector<TrackPoint> &rightLane,int &leftMissedPoints, int &rightMissedPoints,float &leftMissedRadius, float &rightMissedRadius);
    void drawLanes(Mat &img, int roiHeight,vector<TrackPoint> &leftLane,vector<TrackPoint> &rightLane,
                   vector<Point> &centerLine);

    // 环岛处理
    void processCircle(vector<TrackPoint> &leftLane,
                       vector<TrackPoint> &rightLane,
                       Mat &debugImage,int &leftMissedPoints, float &leftMissedRadius, float &rightMissedRadius);
    void resetCircleState();

    // 辅助函数
    bool isLaneContinuous(const vector<TrackPoint>& lane, float max_deviation);
    bool detectCircleEntry(const vector<TrackPoint> &left,
        const vector<TrackPoint> &right,float &leftMissedRadius, float &rightMissedRadius);
    void findInflectionPoints(const vector<TrackPoint> &lane,
                              Point &pointA, Point &pointB,
                              bool &isValid);
    void generateVirtualPath(const Point2f &start, const Point2f &end,
        vector<Point> &path,
        bool isLeftLane);
    bool checkExitCondition(const vector<TrackPoint>& Lane, Mat& img,int roiHeight);
    bool isPathClear(const vector<TrackPoint>& leftLane);
    void mergeVirtualPath(vector<TrackPoint> &lane,
        const vector<Point> &virtualPath,
        bool isLeftLane);
    void smoothPath(vector<Point> &path);
    void linearRegression(
        vector<Point>::iterator begin,
        vector<Point>::iterator end,
        float& k, float& b, float& r_squared
    );
    Mat ApplyInversePerspective(const cv::Mat& inputImage);
    Point findInflectionPoint(const vector<TrackPoint>& points);

};

// 逆透视变换类

#endif