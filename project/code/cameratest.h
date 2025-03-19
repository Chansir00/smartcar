#ifndef CAMERA_H
#define CAMERA_H

#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>
#include <sys/stat.h> // 用于检查文件夹是否存在
#include <socket.h>
#include<Pers.h>
using namespace cv;
using namespace std;

// 可配置参数
const int image_h = 240;                // 图像高度
const int image_w = 320;                // 图像宽度
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
    CIRCLE_EXITING      // 正在离开环岛
};

struct DetectionResult
{
    bool hasCross = false;         // 是否检测到十字路口
    bool hasZebraCrossing = false; // 是否检测到斑马线
    bool hasCircle = false;        // 是否检测到圆环
    bool PointA_found = false;     // 是否检测到A点
    bool PointC_found = false;     // 是否检测到C点
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
// member variables
    Point2f circlePointA, circlePointB, circlePointC;
    bool PointA_found = false;
    bool PointB_found = false;
    bool PointC_found = false;
    vector<Point> virtualPath;
    CircleState circleState = CIRCLE_INACTIVE;
    int centre;
    vector<int> whitePixels;                       // 白点分布
    vector<TrackPoint> leftLane;  // 左车道
    vector<TrackPoint> rightLane; // 右车道
    vector<TrackPoint> rightcicle;      // right_circle
    vector<TrackPoint> leftcicle;
    vector<Point> centerLine; // left_circle


    // member function
    DetectionResult detect(const cv::Mat &inputImage);
    void initializeVariables(int image_w, int image_h);
    void detectWhitePixels(const Mat &img, int roiHeight, std::vector<int> &whitePixels);
    bool detectZebraCrossing(const std::vector<int> &whitePixels);
    void detectLanePoints(const Mat &img, int roiHeight, const std::vector<int> &whitePixels,
                                 std::vector<TrackPoint> &leftLane, std::vector<TrackPoint> &rightLane,int &leftMissedPoints, int &rightMissedPoints);
    int drawLanes(Mat &img, int roiHeight,const std::vector<TrackPoint> &leftLane, const std::vector<TrackPoint> &rightLane,
                   vector<Point> &centerLine);
    void detectCircle(const Mat &img, const std::vector<TrackPoint> &left,
                             const std::vector<TrackPoint> &right, Mat &result);
    // 环岛处理
    void processCircle(vector<TrackPoint> &leftLane,
                       vector<TrackPoint> &rightLane,
                       Mat &debugImage,int &leftMissedPoints, int &rightMissedPoints,int &roiHeight);
    void resetCircleState();

    // 辅助函数
    bool isLaneContinuous(const std::vector<TrackPoint> &lane);
    bool detectCircleEntry(const vector<TrackPoint> &left,
        const vector<TrackPoint> &right,int &leftMissedPoints, int &rightMissedPoints);
    bool findInflectionPoints(const vector<TrackPoint> &lane,
                              Point2f &pointA, Point2f &pointC);
    void generateVirtualPath(const Point2f &start, const Point2f &end,
        vector<Point> &path,
        bool isLeftLane);
    bool checkExitCondition(const vector<TrackPoint>& Lane, Mat& img,int roiHeight);
    bool isPathClear(const vector<TrackPoint>& leftLane);
    void mergeVirtualPath(vector<TrackPoint> &lane,
        const vector<Point> &virtualPath,
        bool isLeftLane);
    void smoothPath(vector<Point> &path);

};

// 逆透视变换类

#endif