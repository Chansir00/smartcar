#ifndef TEST_H
#define TEST_H
#include <opencv2/opencv.hpp>
#include <iostream>
using namespace cv;
using namespace std;

constexpr int img_w = 160;       // 图像宽度（根据实际情况修改）
constexpr int img_h = 120;       // 图像高度

struct SpecialCorner {
    Point position;    // 拐点坐标
    bool detected;     // 是否检测到
    int frame_counter; // 连续检测帧数（用于滤波）
};

// 全局变量声明
extern SpecialCorner left_down_corner;  // 左下拐点
extern SpecialCorner right_down_corner; // 右下拐点 
extern SpecialCorner right_up_corner;   // 右上拐点（备用）

struct LEFT_EDGE {
    int16_t row;  
    int16_t col;  
    uint8_t flag; 
    uint8_t dir;
};

struct RIGHT_EDGE {
    int16_t row;  
    int16_t col;  
    uint8_t flag;
    uint8_t dir;
};

struct CENTER_LINE {
    int16_t y;  // 行坐标
    int16_t x;  // 列坐标（中线位置）
    uint8_t valid; // 有效性标志 (0:无效 1:双线计算 2:左侧推断 3:右侧推断)
    uint8_t mode;  // 生成模式标识符
};

enum class MidlineMode : uint8_t {
    NORMAL = 0,      // 正常模式
    LEFT_ISLAND,     // 左环岛模式
    RIGHT_ISLAND,    // 右环岛模式
    CROSSROAD        // 十字路口
};

// 全局变量声明
extern LEFT_EDGE L_edge[200];
extern RIGHT_EDGE R_edge[200];
extern CENTER_LINE mid_line[160];
extern int16_t left_edge_map[120];  // 声明为外部变量
extern int16_t right_edge_map[120];
extern uint8_t L_edge_count, R_edge_count;
extern int L_start_y, L_start_x, R_start_y, R_start_x;

// 函数声明
void search_neighborhood(Mat& src);
void zuoyoujidian2(Mat& src);
void showBorder1(Mat& image);
Mat visualizeEdges(const Mat& canvas);
void image_draw_rectan(cv::Mat& mat);
int find_zuoduandian(int row);
int find_youduandian(int row);
void find_midline();
void find_midline(Mat& img);
void find_zuoxiaguaidian(Mat &src);
void find_youxiaguaidian(Mat &src);
void find_youshangguaidian(Mat &src);
void find_zuoshangguaidian(Mat &src);
bool isLeftGrowingUp();

// vector<CornerPoint> zuoguaidian2(LEFT_EDGE* edges, int count, bool is_left_edge, Mat& debug_img);
// double angleBetween(const cv::Point2d& vec1, const cv::Point2d& vec2);

#endif 