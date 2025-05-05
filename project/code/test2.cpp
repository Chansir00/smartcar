#include "test2.h"
// 全局变量定义
uint8_t dire_left, dire_right;
const uint8_t L_search_amount = 200, R_search_amount = 200;
int left_findflag = 0;
int right_findflag = 0;
int left_lose, right_lose;

LEFT_EDGE L_edge[200] = {0};
RIGHT_EDGE R_edge[200] = {0};
CENTER_LINE mid_line[160] = {0};
bool found_left = false;
bool found_right = false;
uint8_t L_edge_count = 0;
uint8_t R_edge_count = 0;
int L_start_y = 0, L_start_x = 0, R_start_y = 0, R_start_x = 0;
int16_t left_edge_map[120] = { -1 };  // 初始化为-1表示无效
int16_t right_edge_map[120] = { -1 }; // 初始化为-1
uint8_t left_dir_map[120] = {0};
uint8_t right_dir_map[120] = {0};


void search_neighborhood(Mat& src) {
    int img_h = src.rows; // 动态尺寸
    int img_w = src.cols;
    L_edge_count = 0;
    R_edge_count = 0;
    left_findflag = right_findflag = 0;

    std::fill(left_edge_map, left_edge_map + img_h, -1);
    std::fill(right_edge_map, right_edge_map + img_h, img_w);

    // jidiansousuo
    zuoyoujidian2(src);
    // 左边界追踪
    //7 0 6       6 0 7
    //4   5       5   4
    //3   2       2   3
    if (left_findflag) {
        L_edge[0] = { (int16_t)L_start_y, (int16_t)L_start_x, 1 };
        int16_t curr_row = L_start_y;
        int16_t curr_col = L_start_x;
        dire_left = 0;
        L_edge_count = 1;

        if (curr_col > left_edge_map[curr_row]) {
            left_edge_map[curr_row] = curr_col;
        }


        for (int i = 1; i < L_search_amount; i++) {
            // 边界检查
            if (curr_col >= img_w/2)break;
            if (curr_row < 0 || curr_row >= img_h-1 || curr_col < 0 || curr_col >= img_w-1) 
            {
                cerr<<"yuejie"<<endl;
                break;
            }

            bool found = false;
            
            // 八邻域检查逻辑
            if (dire_left != 2 && curr_row > 0 && curr_col > 0 &&
                src.at<uint8_t>(curr_row-1, curr_col-1) == 0 &&
                src.at<uint8_t>(curr_row-1, curr_col) == 255) {
                curr_row--; curr_col--;
                dire_left = 7;
                found = true;
            }
            else if (dire_left != 3 && curr_row > 0 && curr_col < src.cols-1 &&
                     src.at<uint8_t>(curr_row-1, curr_col+1) == 0 &&
                     src.at<uint8_t>(curr_row, curr_col+1) == 255) {
                curr_row--; curr_col++;
                dire_left = 6;
                found = true;
            }
            else if (curr_row > 0 &&
                     src.at<uint8_t>(curr_row-1, curr_col) == 0 &&
                     src.at<uint8_t>(curr_row-1, curr_col+1) == 255) {
                curr_row--;
                dire_left = 0;
                found = true;
            }
            else if (dire_left != 5 && curr_col > 0 &&
                     src.at<uint8_t>(curr_row, curr_col-1) == 0 &&
                     src.at<uint8_t>(curr_row-1, curr_col-1) == 255) {
                curr_col--;
                dire_left = 4;
                found = true;
            }
            else if (dire_left != 4 && curr_col < src.cols-1 &&
                     src.at<uint8_t>(curr_row, curr_col+1) == 0 &&
                     src.at<uint8_t>(curr_row+1, curr_col+1) == 255) {
                curr_col++;
                dire_left = 5;
                found = true;
            }
            else if (dire_left != 6 && curr_row < src.rows-1 && curr_col > 0 &&
                     src.at<uint8_t>(curr_row+1, curr_col-1) == 0 &&
                     src.at<uint8_t>(curr_row, curr_col-1) == 255) {
                curr_row++; curr_col--;
                dire_left = 3;
                found = true;
            }
            else if (dire_left != 7 && curr_row < src.rows-1 && curr_col < src.cols-1 &&
                     src.at<uint8_t>(curr_row+1, curr_col+1) == 0 &&
                     src.at<uint8_t>(curr_row+1, curr_col) == 255) {
                curr_row++; curr_col++;
                dire_left = 2;
                found = true;
            }
            if (found) {
                // 检查当前行是否已有更优（更右）的点
                if (curr_col > left_edge_map[curr_row]) { // 只保留更右侧的点
                    left_edge_map[curr_row] = curr_col;
                    left_dir_map[curr_row] = dire_left;
                }
                L_edge[i] = { curr_row, curr_col, 1, dire_left };//y(raw),x(col),1(有效)  
                L_edge_count++;
            }
            else break;
        }
    }

    // 右边界追踪（结构对称，逻辑类似）
    if (right_findflag) {
        R_edge[0] = { (int16_t)R_start_y, (int16_t)R_start_x, 1 };
        int16_t curr_row = R_start_y;
        int16_t curr_col = R_start_x;
        dire_right = 0;
        R_edge_count = 1;

        if (curr_col < right_edge_map[curr_row]) {
            right_edge_map[curr_row] = curr_col;
        }

        for (int i = 1; i < R_search_amount; i++) {
            if (curr_col <= img_w/2 )break;
            if (curr_row < 0 || curr_row >= src.rows || curr_col < 0 || curr_col >= src.cols) {
                cerr<<"youyuejie"<<endl;
                break;
            }

            bool found = false;
            
            // 右边界八邻域检查逻辑
            if (dire_right != 3 && curr_row > 0 && curr_col < src.cols-1 &&
                src.at<uint8_t>(curr_row-1, curr_col+1) == 0 &&
                src.at<uint8_t>(curr_row-1, curr_col) == 255) {
                curr_row--; curr_col++;
                dire_right = 6;
                found = true;
            }
            else if (dire_right != 2 && curr_row > 0 && curr_col > 0 &&
                     src.at<uint8_t>(curr_row-1, curr_col-1) == 0 &&
                     src.at<uint8_t>(curr_row, curr_col-1) == 255) {
                curr_row--; curr_col--;
                dire_right = 7;
                found = true;
            }
            else if (curr_row > 0 &&
                     src.at<uint8_t>(curr_row-1, curr_col) == 0 &&
                     src.at<uint8_t>(curr_row-1, curr_col-1) == 255) {
                curr_row--;
                dire_right = 0;
                found = true;
            }
            else if (dire_right != 4 && curr_col < src.cols-1 &&
                     src.at<uint8_t>(curr_row, curr_col+1) == 0 &&
                     src.at<uint8_t>(curr_row-1, curr_col+1) == 255) {
                curr_col++;
                dire_right = 5;
                found = true;
            }
            else if (dire_right != 5 && curr_col > 0 &&
                     src.at<uint8_t>(curr_row, curr_col-1) == 0 &&
                     src.at<uint8_t>(curr_row+1, curr_col-1) == 255) {
                curr_col--;
                dire_right = 4;
                found = true;
            }
            else if (dire_right != 6 && curr_row < src.rows-1 && curr_col > 0 &&
                     src.at<uint8_t>(curr_row+1, curr_col-1) == 0 &&
                     src.at<uint8_t>(curr_row+1, curr_col) == 255) {
                curr_row++; curr_col--;
                dire_right = 3;
                found = true;
            }
            else if (dire_right != 7 && curr_row < src.rows-1 && curr_col < src.cols-1 &&
                     src.at<uint8_t>(curr_row+1, curr_col+1) == 0 &&
                     src.at<uint8_t>(curr_row, curr_col+1) == 255) {
                curr_row++; curr_col++;
                dire_right = 2;
                found = true;
            }
            if (found) {
                if (curr_col < right_edge_map[curr_row]) {
                    right_edge_map[curr_row] = curr_col;
                    right_dir_map[curr_row] = dire_right;
                }
                R_edge[i] = { curr_row, curr_col, 1, dire_right };
                R_edge_count++;
            }

            else break;
        }
    }
    //cerr << "Left points: " << static_cast<int>(L_edge_count) << "r" << static_cast<int>(R_edge_count)<<endl; 
}

void zuoyoujidian2(Mat& src) {
    int img_h = src.rows; // 动态尺寸
    int img_w = src.cols;
    // 初始化坐标
    L_start_x = L_start_y = R_start_x = R_start_y = -1;
    left_findflag = right_findflag = 0; // 重置标志位

    // for (int row = img_h - 2; row > 0; --row) {
    //     for (int col = img_w / 2; col > 0; --col) {
    //         if (col - 1 < 0) continue; // 防止越界
    //         if (src.at<uchar>(row, col) == 0 && src.at<uchar>(row, col - 1) == 0) {
    //             L_start_y = row;
    //             L_start_x = col;
    //             left_findflag = 1;
    //             break;
    //         }
    //     }
    //     if (left_findflag) break; // 找到后立即退出所有循环
    // }


    for (int row = img_h - 2; row > 0; --row) {
        for (int col = img_w / 2; col > 0; --col) {
            if (col - 1 < 0) continue; // 防止越界
            if (src.at<uchar>(row, col) == 0 && src.at<uchar>(row, col + 1) == 255) {
                L_start_y = row;
                L_start_x = col;
                left_findflag = 1;
                break;
            }
        }
        if (left_findflag) break; // 找到后立即退出所有循环
    }
    // 搜索右基点（当前白，右侧黑）
    // bool found_right = false;
    // for (int row = img_h - 2; row > 0; --row) {
    //     for (int col = img_w / 2; col < img_w - 1; ++col) {
    //         if (col + 1 >= img_w) continue; // 防止越界
    //         if (src.at<uchar>(row, col) == 0 && src.at<uchar>(row, col + 1) == 0) {
    //             R_start_y = row;
    //             R_start_x = col;
    //             right_findflag = 1;
    //             break;
    //         }
    //     }
    //     if (right_findflag) break;
    // }

    bool found_right = false;
    for (int row = img_h - 2; row > 0; --row) {
        for (int col = img_w / 2; col < img_w - 1; ++col) {
            if (col + 1 >= img_w) continue; // 防止越界
            if (src.at<uchar>(row, col) == 0 && src.at<uchar>(row, col - 1) == 255) {
                R_start_y = row;
                R_start_x = col;
                right_findflag = 1;
                break;
            }
        }
        if (right_findflag) break;
    }
    // 绘制基点
    // if (l_start_x != -1 && l_start_y != -1) {
    //     circle(src, Point(l_start_x, l_start_y), 3, Scalar(255, 0, 0), FILLED);
    // }
    // if (r_start_x != -1 && r_start_y != -1) {
    //     circle(src, Point(r_start_x, r_start_y), 3, Scalar(0, 255, 0), FILLED);
    // }
}
    //1.生长方向
void zhijiaoguaidian(LEFT_EDGE* edge, int count,Mat &src)
{
    const int dir_threshold = 3;
    for (int i = 1; i < count; i++) {
        if (!edge[i].flag || !edge[i-1].flag) continue;
        // 计算最小方向差
        int diff = abs(edge[i].dir - edge[i-1].dir);
        int min_diff = std::min(diff, 8 - diff);
        
        if (min_diff > dir_threshold) {
            // 找到拐点（edge[i]）
            circle(src, Point(edge[i].col, edge[i].row), 3, Scalar(255), -1);
        }
    }
}
    //2.基于斜率突变
    void zhijiaoguaidian2(LEFT_EDGE* edge, int count,Mat &src) {
        const double slope_threshold = 1.0; // 斜率变化阈值
        
        vector<Point2d> slopes;
        for (int i = 1; i < count; i++) {
            if (!edge[i].flag || !edge[i-1].flag) continue;
            
            // 计算相邻点斜率
            double dx = edge[i].col - edge[i-1].col;
            double dy = edge[i].row - edge[i-1].row;
            if (fabs(dx) < 1e-5) dx = 0.0001; // 避免除零
            
            double slope = dy / dx;
            slopes.push_back(Point2d(i, slope));
        }
        
        // 检测斜率突变
        for (size_t i = 1; i < slopes.size(); i++) {
            double delta = fabs(slopes[i].y - slopes[i-1].y);
            if (delta > slope_threshold) {
                int idx = static_cast<int>(slopes[i].x);
                circle(src, Point(edge[idx].col, edge[idx].row), 3, Scalar(255), -1);
            }
        }
    }
    //基于向量夹角的直角检测（推荐）
    // void detectCornerByAngle(LEFT_EDGE* edge, int count,Mat &src) {
    //     const double angle_threshold = 80.0; // 角度阈值
        
    //     vector<Point> points;
    //     for (int i = 0; i < count; i++) {
    //         if (edge[i].flag)
    //             points.push_back(Point(edge[i].col, edge[i].row));
    //     }
        
    //     for (size_t i = 1; i < points.size()-1; i++) {
    //         Point prev = points[i-1];
    //         Point curr = points[i];
    //         Point next = points[i+1];
            
    //         // 计算向量
    //         Point2d v1(prev.x - curr.x, prev.y - curr.y);
    //         Point2d v2(next.x - curr.x, next.y - curr.y);
            
    //         // 计算夹角
    //         double dot = v1.x*v2.x + v1.y*v2.y;
    //         double mag1 = sqrt(v1.x*v1.x + v1.y*v1.y);
    //         double mag2 = sqrt(v2.x*v2.x + v2.y*v2.y);
            
    //         if (mag1 < 1e-5 || mag2 < 1e-5) continue;
            
    //         double angle = acos(dot / (mag1*mag2)) * 180 / CV_PI;
    //         if (angle > angle_threshold) {
    //             circle(src, curr, 5, Scalar(255), 2);
    //         }
    //     }
    // }

//改进你的detectCornerByAngle函数：
// void detectCornerByAngle(LEFT_EDGE* edge, int count, Mat &src) {
//     const double ANGLE_THRESHOLD = 75.0; // 更严格的阈值
//     const int MIN_SEGMENT_LENGTH = 5;    // 最小有效线段长度
    
//     vector<Point> points;
//     for (int i = 0; i < count; i++) {
//         if (edge[i].flag && i > MIN_SEGMENT_LENGTH) { // 过滤短线段
//             points.push_back(Point(edge[i].col, edge[i].row));
//         }
//     }
    
//     for (size_t i = 1; i < points.size()-1; ++i) {
//         // 计算前后向量夹角
//         Point2d prev_vec = points[i] - points[i-1];
//         Point2d next_vec = points[i+1] - points[i];
        
//         double angle = angleBetween(prev_vec, next_vec); // 封装角度计算函数
//         if (angle > ANGLE_THRESHOLD) {
//             // 记录拐点特征
//             circle(src, points[i], 8, Scalar(0,255,255), 2); 
//         }
//     }
// }



    void showBorder1(Mat& image) {  // 传入彩色图像矩阵（假设已转为BGR格式）
        // 绘制左边界（蓝色）
        for (int i = 0; i < L_edge_count; ++i) {
            if (L_edge[i].flag) {
                int x = L_edge[i].col;
                int y = L_edge[i].row;
                if (x >= 0 && x < image.cols && y >= 0 && y < image.rows) {
                    circle(image, Point(x, y), 1, Scalar(255, 0, 0), FILLED); // 蓝色点
                }
            }
        }
    
        // 绘制右边界（红色）
        for (int i = 0; i < R_edge_count; ++i) {
            if (R_edge[i].flag) {
                int x = R_edge[i].col;
                int y = R_edge[i].row;
                if (x >= 0 && x < image.cols && y >= 0 && y < image.rows) {
                    circle(image, Point(x, y), 1, Scalar(0, 0, 255), FILLED); // 红色点
                }
            }
        }
    
        // 绘制中线（绿色）
        for (int i = 0; i < 140; ++i) { // 根据实际数组大小调整
            if (mid_line[i].valid) {
                int x = mid_line[i].x;
                int y = mid_line[i].y;
                if (x >= 0 && x < image.cols && y >= 0 && y < image.rows) {
                    circle(image, Point(x, y), 1, Scalar(0, 255, 0), FILLED); // 绿色点
                }
            }
        }
    }


    Mat visualizeEdges(const Mat& canvas) {
        // 创建画布副本防止修改原图
        Mat display;
        if(canvas.channels() == 1)cvtColor(canvas, display, COLOR_GRAY2BGR);
        else canvas.copyTo(display);

        const int point_radius = 3;    // 点半径
        const int point_thickness = 1; // 点线宽
    
        // 绘制左边界（蓝色）
        for (int i = 0; i < L_edge_count; ++i) {
            if (L_edge[i].flag) {
                Point pt(L_edge[i].col, L_edge[i].row);
                circle(display, pt, point_radius, Scalar(255, 0, 0), point_thickness);
            }
        }
    
        // 绘制右边界（红色）
        for (int i = 0; i < R_edge_count; ++i) {
            if (R_edge[i].flag) {
                Point pt(R_edge[i].col, R_edge[i].row);
                circle(display, pt, point_radius, Scalar(0, 0, 255), point_thickness);
            }
        }
    
        // 中线
        for (int row = 0; row < 120; ++row) {  
            if (mid_line[row].valid) {
                Point center(mid_line[row].y, row);  // x=列坐标，y=行坐标
                Scalar color;
                MidlineMode mode = static_cast<MidlineMode>(mid_line[row].mode);
                // 根据模式设置颜色
                switch (mode) {
                case MidlineMode::NORMAL:
                    color = Scalar(0, 255, 0);   // 绿色：正常中线
                    //cerr<<"1"<<endl;
                    break;
                case MidlineMode::LEFT_ISLAND:
                    color = Scalar(0, 255, 255); // 黄色：左环岛
                    break;
                case MidlineMode::RIGHT_ISLAND:
                    color = Scalar(255, 255, 0); // 青色：右环岛
                    break;
                case MidlineMode::CROSSROAD:
                    color = Scalar(255, 0, 255); // 紫色：十字路口
                    break;
                default:
                    color = Scalar(128, 128, 128); // 灰色：未知模式
                }
                // 绘制中线点
                circle(display, center, point_radius, color, FILLED);
            }
        }
        find_zuoxiaguaidian(display);
        find_youxiaguaidian(display);
        find_zuoshangguaidian(display);
        find_youshangguaidian(display);
        // 将结果复制回canvas
        return display;
    }

    void image_draw_rectan(cv::Mat& mat) {
        // 检查输入有效性
        if (mat.empty() || mat.cols < 2) return;
        // 获取图像参数
        const int rows = mat.rows;
        const int cols = mat.cols;
        const int channels = mat.channels();
        // 遍历每一行
        for (int row = 0; row < mat.rows; ++row) {
            mat.at<uchar>(row, 1) = 0;                 
            mat.at<uchar>(row, mat.cols - 2) = 0;      
        }
    }

    int find_zuoduandian(int row) {
        const int col_THRESHOLD = 6;
        const int MIN_CONSECUTIVE_DIR0 = 3; // 连续3行方向为0视为有效
    
        int current_col = left_edge_map[row];
        int prev_col = left_edge_map[row-1];
        int col_diff = abs(current_col - prev_col);
        
        int break_count = 0;
        
        // 基础列差检测
        if (col_diff > col_THRESHOLD) {
            break_count++;
            
            // 方向连续性检测
            int dir0_streak = 0;
            for (int i = row; i >= max(0, row-3); --i) {
                if (left_dir_map[i] == 0) dir0_streak++;
            }
            if (dir0_streak >= MIN_CONSECUTIVE_DIR0) {
                break_count++;
            }
        }
        
        return break_count;
    }

    int find_youduandian(int row) {
        const int col_THRESHOLD = 6;
        const int MIN_CONSECUTIVE_DIR0 = 3; // 连续3行方向为0视为有效
    
        int current_col = right_edge_map[row];
        int prev_col = right_edge_map[row-1];
        int col_diff = abs(current_col - prev_col);
        
        int break_count = 0;
        
        // 基础列差检测
        if (col_diff > col_THRESHOLD) {
            break_count++;
            
            // 方向连续性检测
            int dir0_streak = 0;
            for (int i = row; i >= max(0, row-3); --i) {
                if (right_dir_map[i] == 0) dir0_streak++;
            }
            if (dir0_streak >= MIN_CONSECUTIVE_DIR0) {
                break_count++;
            }
        }
        
        return break_count;
    }

    // void find_midline() {
    //     const int default_width = 80;
    //     const int img_w = 160;
    //     const int img_h = 120;

    //     for (int row = 0; row < img_h; ++row) {
    //         int zuoduandian = find_zuoduandian(row);
    //         int youduandian = find_youduandian(row);
            
    //         if (zuoduandian >= 2 && youduandian >= 2) {
    //             // 进入特殊区域处理（示例）
    //             mid_line[row].valid = 0;
    //             mid_line[row].mode = 4; // 特殊模式
    //             // 这里可以添加环岛/十字的特殊处理逻辑
    //         } 
    //         else if(zuoduandian >=2 && youduandian <2){
    //             //左环岛
    //         }
    //         else if(zuoduandian <2 && youduandian >=2){
    //             // 右环岛
    //         }
    //         else {
    //             //正常巡线
    //         }
    //     }
    // }

    // void find_midline() {
    //     const int default_width = 80;  // 默认扩展宽度
    //     const int img_w = 160;         // 假设图像宽度为120（根据实际替换为全局变量）
    
    //     // 遍历所有行（假设图像最大高度为120）
    //     for (int row = 0; row < img_w; ++row) {
    //         // 获取当前行的左右边界值
    //         int16_t left_col = left_edge_map[row];
    //         int16_t right_col = right_edge_map[row];
    
    //         // 判断左右边界的有效性
    //         bool left_valid = (left_col != -1);        // 左边界有效条件：非-1
    //         bool right_valid = (right_col != img_w);   // 右边界有效条件：非img_w（初始值）
    
    //         // 计算中线逻辑
    //         if (left_valid && right_valid) {
    //             // 两侧有效：中线为左右边界中点
    //             mid_line[row].y = (left_col + right_col) / 2;
    //             mid_line[row].valid = 1;
    //             mid_line[row].mode = 0;  // 模式0：双侧有效
    //         } 
    //         else if (left_valid) {
    //             // 仅左有效：中线 = 左边界 + 默认宽度（限制在图像右侧）
    //             mid_line[row].y = std::min(left_col + default_width, img_w - 1);
    //             mid_line[row].valid = 1;
    //             mid_line[row].mode = 1;  // 模式1：仅左有效
    //         } 
    //         else if (right_valid) {
    //             // 仅右有效：中线 = 右边界 - 默认宽度（限制在图像左侧）
    //             mid_line[row].y = std::max(right_col - default_width, 0);
    //             mid_line[row].valid = 1;
    //             mid_line[row].mode = 2;  // 模式2：仅右有效
    //         } 
    //         else {
    //             // 完全无效：中线无效
    //             mid_line[row].valid = 0;
    //             mid_line[row].mode = 3;  // 模式3：双侧无效
    //         }
    //     }
    // }

    void find_midline(Mat& img) {
        const int DEFAULT_WIDTH = 80;
        const int IMG_W = 160;
        const int IMG_H = 120;
        static int last_valid_y = IMG_W/2; // 持久化保存上一有效行中线
    
        for (int row = 0; row < IMG_H; ++row) {
            // 获取当前边界值和断点信息
            int16_t left_col = left_edge_map[row];
            int16_t right_col = right_edge_map[row];
            
            int left_break = find_zuoduandian(row);
            int right_break = find_youduandian(row);

            // if (left_break >= 2) {
            //     //cv::circle(img, cv::Point(left_col, row), 5, cv::Scalar(0, 255, 255), -1);
            // }
            // 右断点：蓝色圆圈
            // if (right_break >= 2) {
            //     //cv::circle(img, cv::Point(right_col, row), 5, cv::Scalar(255, 255, 0), -1);
            // }
    
            // 模式判断逻辑
            MidlineMode mode;
            if (left_break >= 2 && right_break >= 2) {
                mode = MidlineMode::CROSSROAD;
            } else if (left_break >= 2) {
                mode = MidlineMode::LEFT_ISLAND;
            } else if (right_break >= 2) {
                mode = MidlineMode::RIGHT_ISLAND;
            } else {
                mode = MidlineMode::NORMAL;
            }
    
            // 根据模式计算中线
            switch (mode) {
            case MidlineMode::NORMAL:
                mid_line[row].y = (left_col + right_col) / 2;
                mid_line[row].valid = 1;
                last_valid_y = mid_line[row].y; // 更新历史值
                break;
    
            // case MODE_LEFT_ONLY:
            //     mid_line[row].y = std::min(left_col + DEFAULT_WIDTH, IMG_W - 1);
            //     mid_line[row].valid = 1;
            //     last_valid_y = mid_line[row].y;
            //     break;
    
            // case MODE_RIGHT_ONLY:
            //     mid_line[row].y = std::max(right_col - DEFAULT_WIDTH, 0);
            //     mid_line[row].valid = 1;
            //     last_valid_y = mid_line[row].y;
            //     break;
    
            case MidlineMode::LEFT_ISLAND:  // 左环岛扩大补偿
                mid_line[row].y = right_col + DEFAULT_WIDTH;
                mid_line[row].valid = 1;
                break;
    
            case MidlineMode::RIGHT_ISLAND: // 右环岛扩大补偿
                mid_line[row].y = left_col + DEFAULT_WIDTH;
                mid_line[row].valid = 1;
                break;
    
            case MidlineMode::CROSSROAD:    // 十字路口延续历史轨迹
                mid_line[row].y = last_valid_y;
                mid_line[row].valid = (last_valid_y != -1) ? 1 : 0;
                //cerr<<1<<endl;
                break;
    
            default:  // MODE_BOTH_INVALID
                mid_line[row].valid = 0;
                break;
            }
    
            // 记录模式信息
            mid_line[row].mode = static_cast<uint8_t>(mode);
            // 后处理：防止无效值传播
            if (!mid_line[row].valid && row > 0) {
                mid_line[row].y = mid_line[row-1].y;
            }
        }
    }

// 新增结构体用于存储拐点信息
struct CornerPoint {
    Point position;
    double angle;
    bool is_left; // 标记左/右边界拐点
};

// 主拐点检测函数
// vector<CornerPoint> zuoguaidian2(LEFT_EDGE* edges, int count, bool is_left_edge, Mat& debug_img) {
//     const double ANGLE_THRESHOLD = 75.0;   // 角度阈值（直角附近）
//     const int WINDOW_SIZE = 5;             // 滑动窗口大小
//     const int MIN_SEGMENT_LENGTH = 10;     // 有效线段最小长度
//     const int DIR_CONSISTENCY = 3;         // 方向一致性检查范围

//     vector<CornerPoint> corners;
//     vector<Point> points;

//     // 1. 数据预处理：提取有效点并过滤短线段
//     for (int i = 0; i < count; ++i) {
//         if (edges[i].flag) {
//             points.push_back(Point(edges[i].col, edges[i].row));
//         }
//     }
//     if (points.size() < MIN_SEGMENT_LENGTH) return corners;

//     // 2. 滑动窗口角度检测
//     for (size_t i = WINDOW_SIZE; i < points.size() - WINDOW_SIZE; ++i) {
//         // 计算前后向量
//         Point2d prev_vec = points[i] - points[i - WINDOW_SIZE];
//         Point2d next_vec = points[i + WINDOW_SIZE] - points[i];

//         // 忽略过短的向量
//         if (norm(prev_vec) < 1e-5 || norm(next_vec) < 1e-5) continue;

//         // 计算夹角（角度制）
//         double angle = angleBetween(prev_vec, next_vec);
        
//         // 初步筛选角度
//         if (angle > ANGLE_THRESHOLD) {
//             // 3. 方向一致性验证（避免噪声误判）
//             bool dir_consistent = true;
//             for (int j = i - DIR_CONSISTENCY; j <= i + DIR_CONSISTENCY; ++j) {
//                 if (j < 0 || j >= edges[i].dir) continue;
//                 // 检查方向突变是否符合直角特征
//                 if (abs(edges[j].dir - edges[i].dir) > 2) {
//                     dir_consistent = false;
//                     break;
//                 }
//             }

//             if (dir_consistent) {
//                 // 4. 记录拐点
//                 CornerPoint cp;
//                 cp.position = points[i];
//                 cp.angle = angle;
//                 cp.is_left = is_left_edge;
//                 corners.push_back(cp);

//                 // 调试绘制
//                 if (!debug_img.empty()) {
//                     Scalar color = is_left_edge ? Scalar(0, 255, 255) : Scalar(255, 255, 0);
//                     circle(debug_img, points[i], 5, color, 2);
//                 }
//             }
//         }
//     }

//     return corners;
// }

// double angleBetween(const cv::Point2d& vec1, const cv::Point2d& vec2) {
//     // 1. 计算点积
//     double dot = vec1.x * vec2.x + vec1.y * vec2.y;
    
//     // 2. 计算向量模长
//     double norm1 = std::sqrt(vec1.x * vec1.x + vec1.y * vec1.y);
//     double norm2 = std::sqrt(vec2.x * vec2.x + vec2.y * vec2.y);
    
//     // 3. 处理零向量（模长接近0的情况）
//     if (norm1 < 1e-6 || norm2 < 1e-6) {
//         return 0.0; // 返回0度表示无效计算
//     }
    
//     // 4. 计算余弦值并限制在[-1,1]范围内（防止浮点误差）
//     double cos_theta = dot / (norm1 * norm2);
//     cos_theta = std::max(std::min(cos_theta, 1.0), -1.0);
    
//     // 5. 计算弧度并转换为角度
//     return std::acos(cos_theta) * 180.0 / M_PI;
// }


// void detectInflectionByDirection(Mat &src) {
//     const int DIRECTION_THRESHOLD = 3;  // 方向突变阈值
//     const int WINDOW_SIZE = 3;          // 滑动窗口大小

//     // 遍历左边界有效点
//     for (int i = WINDOW_SIZE; i < L_edge_count - WINDOW_SIZE; ++i) {
//         if (!L_edge[i].flag) continue;

//         // 获取当前及历史方向信息
//         uint8_t curr_dir = L_edge[i].dir;
//         uint8_t prev_dir = L_edge[i-1].dir;

//         // 核心判断逻辑：0/6→7/4的突变
//         if ((prev_dir == 0 || prev_dir == 6) && 
//             (curr_dir == 7 || curr_dir == 4)) {
            
//             // 二次校验：检查窗口内的方向一致性
//             bool valid = true;
//             for (int j = i-WINDOW_SIZE; j < i; ++j) {
//                 if (L_edge[j].dir == 0 || L_edge[j].dir == 6) {
//                     valid = false;
//                     break;
//                 }
//             for (int j = i+WINDOW_SIZE; j > i; --j) {
//                 if (L_edge[j].dir == 7 || L_edge[j].dir == 4 || L_edge[j].dir == 3) {
//                     valid = false;
//                     break;
//                 }

//             }
//             if (valid) {
//                 // 标记拐点（红色圆圈）
//                 circle(src, Point(L_edge[i].col, L_edge[i].row), 
//                       5, Scalar(0, 0, 255), 2);
//                 cerr<<"lie"<<L_edge[i].col<<"hang"<<L_edge[i].row<<endl;
//             }
//         }
//     }
// }
// }
void find_zuoxiaguaidian(Mat &src) {
    const int DIRECTION_THRESHOLD = 3;
    const int WINDOW_SIZE = 5;          // 扩大窗口范围
    const int MIN_CONSECUTIVE = 3;      // 最小连续方向要求

    // 左边界检测
    for (int i = WINDOW_SIZE; i < L_edge_count - WINDOW_SIZE; ++i) {
        if (!L_edge[i].flag) continue;

        // 核心判断逻辑：从向上/右上（0/6）突变为向左相关方向（3/4/7）
        bool direction_change = 
            (L_edge[i-WINDOW_SIZE].dir == 0 || L_edge[i-WINDOW_SIZE].dir == 6) && // 之前方向向上/右上
            (L_edge[i].dir == 3 || L_edge[i].dir == 4 || L_edge[i].dir == 7);     // 当前方向向左相关

        if (direction_change) {
            // 二次校验：检查前后窗口方向一致性
            bool prev_valid = true;
            bool next_valid = true;
            
            // 检查前窗口是否保持向上趋势
            for (int j = i-WINDOW_SIZE; j < i; ++j) {
                if (!(L_edge[j].dir == 0 || L_edge[j].dir == 6)) {
                    prev_valid = false;
                    break;
                }
            }
            
            // 检查后窗口是否保持向左趋势
            for (int j = i; j <= i+WINDOW_SIZE; ++j) {
                if (!(L_edge[j].dir == 3 || L_edge[j].dir == 4 || L_edge[j].dir == 7)) {
                    next_valid = false;
                    break;
                }
            }

            if (prev_valid && next_valid) {
                circle(src, Point(L_edge[i].col, L_edge[i].row), 
                      5, Scalar(0, 200, 255), 2); // 橙色标记拐点
            }
        }
        // if (valid) { // 假设你的有效性判断条件
        //     left_down_corner.position = Point(L_edge[i].col, L_edge[i].row);
        //     left_down_corner.detected = true;
        //     left_down_corner.frame_counter = min(left_down_corner.frame_counter + 1, 5); // 最大连续5帧
        // } else {
        //     left_down_corner.frame_counter = max(left_down_corner.frame_counter - 1, 0);
        //     if (left_down_corner.frame_counter == 0) 
        //         left_down_corner.detected = false;
        // }
    }
}

void find_youxiaguaidian(Mat &src) {
    const int DIRECTION_THRESHOLD = 3;
    const int WINDOW_SIZE = 5;          // 扩大窗口范围
    const int MIN_CONSECUTIVE = 3;      // 最小连续方向要求

    // 左边界检测
    for (int i = WINDOW_SIZE; i < L_edge_count - WINDOW_SIZE; ++i) {
        if (!R_edge[i].flag) continue;

        // 核心判断逻辑：从向上/右上（0/6）突变为向左相关方向（3/4/7）
        bool direction_change = 
            (R_edge[i-WINDOW_SIZE].dir == 0 || R_edge[i-WINDOW_SIZE].dir == 7) && // 之前方向向上/右上
            (R_edge[i].dir == 2 || R_edge[i].dir == 5 || R_edge[i].dir == 6);     // 当前方向向左相关

        if (direction_change) {
            // 二次校验：检查前后窗口方向一致性
            bool prev_valid = true;
            bool next_valid = true;
            
            // 检查前窗口是否保持向上趋势
            for (int j = i-WINDOW_SIZE; j < i; ++j) {
                if (!(R_edge[j].dir == 0 || R_edge[j].dir == 7)) {
                    prev_valid = false;
                    break;
                }
            }
            
            // 检查后窗口是否保持向左趋势
            for (int j = i; j <= i+WINDOW_SIZE; ++j) {
                if (!(R_edge[j].dir == 2 || R_edge[j].dir == 5 || R_edge[j].dir == 6)) {
                    next_valid = false;
                    break;
                }
            }

            if (prev_valid && next_valid) {
                circle(src, Point(R_edge[i].col, R_edge[i].row), 
                      5, Scalar(0, 200, 255), 2); // 橙色标记拐点
            }
        }
    }
    // if (valid) {
    //     right_down_corner.position = Point(R_edge[i].col, R_edge[i].row);
    //     right_down_corner.detected = true;
    //     right_down_corner.frame_counter = min(right_down_corner.frame_counter + 1, 5);
    // } else {
    //     right_down_corner.frame_counter = max(right_down_corner.frame_counter - 1, 0);
    //     if (right_down_corner.frame_counter == 0)
    //         right_down_corner.detected = false;
    // }
}

void find_youshangguaidian(Mat &src) {
    const int DIRECTION_THRESHOLD = 3;
    const int WINDOW_SIZE = 5;          // 扩大窗口范围
    const int MIN_CONSECUTIVE = 3;      // 最小连续方向要求

    // 左边界检测
    for (int i = WINDOW_SIZE; i < L_edge_count - WINDOW_SIZE; ++i) {
        if (!R_edge[i].flag) continue;

        // 核心判断逻辑：从向上/右上（0/6）突变为向左相关方向（3/4/7）
        bool direction_change = 
            (R_edge[i-WINDOW_SIZE].dir == 4 || R_edge[i-WINDOW_SIZE].dir == 3) && // 之前方向向上/右上
            (R_edge[i].dir == 0 || R_edge[i].dir == 7);     // 当前方向向左相关

        if (direction_change) {
            // 二次校验：检查前后窗口方向一致性
            bool prev_valid = true;
            bool next_valid = true;
            
            // 检查前窗口是否保持向上趋势
            for (int j = i-WINDOW_SIZE; j < i; ++j) {
                if (!(R_edge[j].dir == 4 || R_edge[i-WINDOW_SIZE].dir == 3)) {
                    prev_valid = false;
                    break;
                }
            }
            
            // 检查后窗口是否保持向左趋势
            for (int j = i; j <= i+WINDOW_SIZE; ++j) {
                if (!(R_edge[j].dir == 0 || R_edge[j].dir == 7)) {
                    next_valid = false;
                    break;
                }
            }

            if (prev_valid && next_valid) {
                circle(src, Point(R_edge[i].col, R_edge[i].row), 
                      5, Scalar(0, 200, 255), 2); // 橙色标记拐点
            }
        }
    }
}

void find_zuoshangguaidian(Mat &src) {
    const int DIRECTION_THRESHOLD = 3;
    const int WINDOW_SIZE = 5;          // 扩大窗口范围
    const int MIN_CONSECUTIVE = 3;      // 最小连续方向要求

    // 左边界检测
    for (int i = WINDOW_SIZE; i < L_edge_count - WINDOW_SIZE; ++i) {
        if (!L_edge[i].flag) continue;

        // 核心判断逻辑：从向上/右上（0/6）突变为向左相关方向（3/4/7）
        bool direction_change = 
            (L_edge[i-WINDOW_SIZE].dir == 5 || L_edge[i-WINDOW_SIZE].dir == 2) && // 之前方向向上/右上
            (L_edge[i].dir == 0 || L_edge[i].dir == 6);     // 当前方向向左相关

        if (direction_change) {
            // 二次校验：检查前后窗口方向一致性
            bool prev_valid = true;
            bool next_valid = true;
            
            // 检查前窗口是否保持向上趋势
            for (int j = i-WINDOW_SIZE; j < i; ++j) {
                if (!(L_edge[j].dir == 5 || L_edge[j].dir == 2)) {
                    prev_valid = false;
                    break;
                }
            }
            
            // 检查后窗口是否保持向左趋势
            for (int j = i; j <= i+WINDOW_SIZE; ++j) {
                if (!(L_edge[j].dir == 0 || L_edge[j].dir == 6)) {
                    next_valid = false;
                    break;
                }
            }

            if (prev_valid && next_valid) {
                circle(src, Point(L_edge[i].col, L_edge[i].row), 
                      5, Scalar(0, 200, 255), 2); // 橙色标记拐点
            }
        }
    }
}
bool isLeftGrowingUp() {
    const float UP_DIR_RATIO = 0.8f; // 60%以上方向为0/6
    int up_count = 0;
    
    for (int i = 0; i < L_edge_count; ++i) {
        if (L_edge[i].dir == 0 || L_edge[i].dir == 6) { // 0:正上 6:右上
            up_count++;
        }
    }
    return (up_count > L_edge_count * UP_DIR_RATIO);
}



// #include <vector>
// #include <algorithm>

// // 坐标点结构体
// struct Corner {
//     float x;
//     float y;
// };

// // 车道线信息
// struct Line {
//     float curvature; // 曲率值
//     bool valid;     // 是否有效
// };

// // 状态枚举
// enum class State {
//     Normal,
//     CrossPre,
//     CrossConfirmed,
//     RoundaboutPre,
//     RoundaboutConfirmed
// };

// class StateMachine {
// private:
//     State current_state = State::Normal;
//     int cross_pre_count = 0;
//     int roundabout_pre_count = 0;
    
//     // 参数配置
//     const int N = 5;  // 十字确认阈值
//     const int M = 3;  // 环岛确认阈值
//     const float curvature_threshold = 0.1f; // 曲率判定阈值
//     const int image_width = 640;            // 图像宽度（根据摄像头分辨率调整）

// public:   //传入角点集合和左车道线信息
//     void update(const std::vector<Corner>& corners, const Line& left_line) {
//         switch (current_state) { //默认普通进入普通处理
//             case State::Normal:
//                 handle_normal_state(corners, left_line);
//                 break;
//             case State::CrossConfirmed: //进入十字处理
//                 if (check_cross_exit(corners)) {
//                     current_state = State::Normal;
//                 }
//                 break;
//             case State::RoundaboutConfirmed: //进入环岛处理
//                 if (check_roundabout_exit(corners, left_line)) {
//                     current_state = State::Normal;
//                 }
//                 break;
//             // 其他状态暂不处理特殊逻辑
//             default:
//                 break;
//         }
//     }

// private:
//     void handle_normal_state(const std::vector<Corner>& corners, const Line& left_line) {
//         if (check_cross_pre(corners)) { //正确一次count+1
//             if (++cross_pre_count >= N) { 
//                 current_state = State::CrossConfirmed; //确认
//                 cross_pre_count = 0;
//             }
//         } else if (check_roundabout_pre(corners, left_line)) { //正确一次count+1
//             if (++roundabout_pre_count >= M) {
//                 current_state = State::RoundaboutConfirmed; //确认
//                 roundabout_pre_count = 0;
//             }
//         } else {
//             cross_pre_count = 0;
//             roundabout_pre_count = 0;
//         }
//     }

//     // 检查十字预判条件
//     bool check_cross_pre(){
//         return left_down_corner.frame_counter >= 3 && right_down_corner.frame_counter >= 3;
//     }

//     // 检查环岛预判条件
//     bool check_roundabout_pre(){
//         return right_down_corner.frame_counter >= 3 && isLeftGrowingUp();
//     }

//     // 检查十字退出条件
//     bool check_cross_exit(){
//         return left_down_corner.frame_counter < 1 || right_down_corner.frame_counter < 1;
//     }

//     // 检查环岛退出条件
//     bool check_roundabout_exit(const std::vector<Corner>& corners, const Line& left_line) const {
//         return right_down_corner.frame_counter < 1; 
//     }
// };