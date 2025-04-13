#include "cameratest.h"

// 车道检测逻辑
DetectionResult LaneProcessor::detect(const Mat &inputImage)
{
    int leftMissedPoints, rightMissedPoints;
    float leftMissedRadius, rightMissedRadius;
    DetectionResult result;
    result.outputImage = inputImage.clone();
    // 预处理图像
    binaryThreshold(inputImage, result.binaryImage);
    result.warpedImage = ApplyInversePerspective(result.binaryImage);
    //  计算ROI区域的高度
    int roiHeight = image_h;

    // 计算白点分布
    detectWhitePixels(result.binaryImage, roiHeight, whitePixels);

    // 检测车道点
    detectLanePoints(result.binaryImage, roiHeight, whitePixels, leftLane, rightLane, leftMissedPoints, rightMissedPoints, leftMissedRadius, rightMissedRadius);
    processCircle(leftLane, rightLane, result.binaryImage, roiHeight, leftMissedRadius, rightMissedRadius);
    cerr << circleState << endl;
    // 每帧结束时检查重置
    if (circleState == CIRCLE_INACTIVE)
    {
        resetCircleState();
    }
    // 检测斑马线
    result.hasZebraCrossing = detectZebraCrossing(whitePixels, leftMissedRadius, rightMissedRadius);

    // 绘制检测结果

    drawLanes(result.outputImage, roiHeight, leftLane, rightLane, centerLine);

    // 如果检测到斑马线，添加文本标注
    if (result.hasZebraCrossing)
    {
        putText(result.binaryImage, "ZEBRA CROSSING", Point(20, 40),
                FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 0, 255), 2);
    }
    return result;
}
int circleflag = 1;
int countcircle = 0;
// 检测圆环
void LaneProcessor::processCircle(vector<TrackPoint> &leftLane,
                                  vector<TrackPoint> &rightLane,
                                  Mat &img, int &roiHeight, float &leftMissedRadius, float &rightMissedRadius)
{

    if (countcircle == 100)
    {
        circleflag = 1;
        countcircle = 0;
    }
    cerr << "circleflag: " << circleflag << endl;
    if (circleflag == 0)
    {
        countcircle++;
        return;
    }
    findInflectionPoints(rightLane, rightJumpPointA, rightJumpPointB, isrightJumpPointA, isrightJumpPointB,false);
    findInflectionPoints(leftLane, leftJumpPointA, leftJumpPointB, isleftJumpPointA, isleftJumpPointB,true);
    switch (circleState)
    {
    case CIRCLE_INACTIVE:
    {

        // 先通过左车道单调性和右车道丢点率判断是否可能进入环岛
        if (isLaneContinuous(leftLane, 0.1f) && isrightJumpPointB)
        {
            // circleState = CIRCLE_DETECTED;
        }
        else if (rightMissedRadius > 0.9 && leftMissedRadius < 0.3 && leftLane.size() < 0.6 * image_h)
        {
            circleState = RIGHT_TURN;
        }
        else if (rightMissedRadius < 0.3 && leftMissedRadius > 0.9 && leftLane.size() < 0.6 * image_h)
        {
            circleState = LEFT_TURN;
        }
        else if (rightMissedRadius < 0.3 && leftMissedRadius < 0.3 && leftLane.size() > 0.6 * image_h)
        {
            circleState = CROSSING;
        }
        else
        {
            circleState = CIRCLE_INACTIVE;
        }
    }
    break;

    case CIRCLE_DETECTED:
    {
        if (1)
            ;
        {
            circle(img, rightJumpPointA, 5, Scalar(0, 0, 0), -1); // 半径 1，颜色黑色，厚度 -1 表示填充
            circle(img, rightJumpPointB, 5, Scalar(0, 0, 0), -1); // 半径 1，颜色黑色，厚度 -1 表示填充
            // 满足三个特征：A 点（正向变化且另一端平缓）、圆弧、C 点（负向变化且另一端平缓）
            // 从左车道中寻找一个有效的连接点（例如第一个有效点）
            Point2f leftStart = leftLane[image_h / 2].position;
            // 生成虚拟路径：从左车道的连接点延伸到 C 点
            vector<Point> virtualPath;
            // isLeftLane = true ;//表示生成时向左偏移，具体根据你的车道配置而定
            // generateVirtualPath(leftStart, circlePointC, virtualPath, true);
            cerr << "virtualPath: " << virtualPath.size() << endl;
            // 绘制调试信息
            polylines(img, virtualPath, false, Scalar(255, 0, 0), 2);
            // 将生成的虚拟路径合并到左车道中，实现车道线拉线
            mergeVirtualPath(leftLane, virtualPath, true);
            // 状态转移到环内循迹（根据需要也可以直接进入 INSIDE 状态）
            // if (circlePointC.x < 30)
            // {
            //     circleState = CIRCLE_INSIDE;
            // }
            // circleState = CIRCLE_INSIDE;
        }
    }
    break;
    case CIRCLE_INSIDE:
    {
        if (checkExitCondition(leftLane, img, roiHeight))
        {
            circleState = CIRCLE_EXITING;
        }
    }
    break;

    case CIRCLE_EXITING:
    {
        checkExitCondition(leftLane, img, roiHeight);
        // circle(img, circlePointB, 5, Scalar(0, 0, 0), -1); // 半径 1，颜色黑色，厚度 -1 表示填充
        Point2f endPoint = Point2f(160, 0);
        // circle(img, rightLane[239].position, 5, Scalar(0, 0, 0), -1); // 半径 1，颜色黑色，厚度 -1 表示填充
        for (int i = roiHeight - 1; i > roiHeight - rightLane.size(); i--)
        {
            if (rightLane[i - 1].position.x != image_h - 1 && rightLane[i].position.x == image_h - 1 && rightLane[i + 1].position.x == image_h - 1 && rightLane[i + 2].position.x == image_h - 1)
            {
                endPoint = rightLane[i].position;
            }
        }
        circle(img, endPoint, 5, Scalar(0, 0, 0), -1); // 半径 1，颜色黑色，厚度 -1 表示填充
        // generateVirtualPath(circlePointB, endPoint, virtualPath, true);
        polylines(img, virtualPath, false, Scalar(255, 0, 0), 2);
        mergeVirtualPath(leftLane, virtualPath, true);
        if (isPathClear(rightLane))
        {
            circleState = CIRCLE_INACTIVE;
            circleflag = 0;
            virtualPath.clear();
        }
    }
    break;
    case LEFT_TURN:
    {
        if (leftLane.size() > 0.6 * image_h && leftMissedRadius < 0.7)
        {
            circleState = CIRCLE_INACTIVE;
        }
    }
    break;
    case RIGHT_TURN:
    {
        if (rightLane.size() > 0.6 * image_h && rightMissedRadius < 0.7)
        {
            circleState = CIRCLE_INACTIVE;
        }
    }
    break;
    case CROSSING:
    {
        if (rightLane.size() < 0.6 * image_h && (leftMissedRadius < 0.7 || rightMissedRadius < 0.7))
        {
            circleState = CIRCLE_INACTIVE;
        }
    }
    break;
    default:
        break;
    }
    cerr << "rightMissedRadius: " << rightMissedRadius << endl;
    cerr << "leftMissedRadius: " << leftMissedRadius << endl;
}
// initializeVariables
void LaneProcessor::initializeVariables(int image_w, int image_h)
{
    leftJumpPointA = Point(-1, -1);
    rightJumpPointA = Point(-1, -1);
    leftJumpPointB = Point(-1, -1);
    rightJumpPointB = Point(-1, -1);
    // 初始化 whitePixels，大小为 image_w，默认值为 0
    whitePixels = std::vector<int>(image_w, 0);

    // 初始化 leftLane，大小为 image_h，每个元素为 TrackPoint{Point(-1, -1), 0}
    leftLane = std::vector<TrackPoint>(image_h, {Point{-1, -1}, 0});

    // 初始化 rightLane，大小为 image_h，每个元素为 TrackPoint{Point(-1, -1), 0}
    rightLane = std::vector<TrackPoint>(image_h, {Point{-1, -1}, 0});

    // 初始化 rightcicle，大小为 5，每个元素为 TrackPoint{Point(-1, -1), 0}
    rightcicle = std::vector<TrackPoint>(5, {Point{-1, -1}, 0});

    // 初始化 leftcicle，大小为 5，每个元素为 TrackPoint{Point(-1, -1), 0}
    leftcicle = std::vector<TrackPoint>(5, {Point{-1, -1}, 0});

    // 初始化 centerLine，大小为 image_h，每个元素为 TrackPoint{Point(-1, -1), 0}
    centerLine = std::vector<Point>(image_h, Point{0, 0});

    // 初始化 virtualPath，大小为 image_h，每个元素为 Point{0, 0}
    virtualPath = std::vector<Point>(image_h, Point{0, 0});

    // 初始化 circleState
    circleState = CIRCLE_INACTIVE;
}
// 图像预处理：二值化
bool binaryThreshold(const Mat &input, Mat &output)
{
    if (input.empty())
    {
        cerr << "错误：输入图像为空！" << endl;
        return false;
    }

    Mat img, blurred;
    cvtColor(input, img, COLOR_BGR2GRAY);
    GaussianBlur(img, blurred, Size(5, 5), 0);
    threshold(blurred, output, 0, 255, THRESH_BINARY | THRESH_OTSU);
    return true;
}
// 检测每列白色像素数量
void LaneProcessor::detectWhitePixels(const Mat &img, int roiHeight, std::vector<int> &whitePixels)
{
    whitePixels.assign(img.cols, 0); // 初始化 whitePixels 为 0
    for (int x = 0; x < img.cols; x++)
    {
        for (int y = roiHeight - 1; y >= 0; y--)
        {
            if (img.at<uchar>(y, x) != 0)
                whitePixels[x]++;
            else
                break;
        }
    }
}

// 检测斑马线
bool LaneProcessor::detectZebraCrossing(const std::vector<int> &whitePixels, float &leftMissedRadius, float &rightMissedRadius)
{
    // 条件1: 检查白线变化次数
    int changes = 0;
    for (size_t i = 1; i < whitePixels.size(); i++)
    {
        if (abs(whitePixels[i] - whitePixels[i - 1]) >= 60)
            changes++;
    }

    // 条件2: 检查最大白线数量是否大于180
    int maxWhitePixels = *std::max_element(whitePixels.begin(), whitePixels.end());
    if (maxWhitePixels <= 180)
        return false;

    // 条件3: 检查左右车道线是否连续
    bool isLeftLaneContinuous = isLaneContinuous(leftLane, leftMissedRadius);
    bool isRightLaneContinuous = isLaneContinuous(rightLane, rightMissedRadius);
    if (!isLeftLaneContinuous || !isRightLaneContinuous)
        return false;

    // 条件4: 检查是否存在10个左右车道线距离中线小于10
    int closeToCenterCount = 0;
    for (size_t i = 0; i < leftLane.size(); i++)
    {
        if (abs(leftLane[i].position.x - rightLane[i].position.x) < 30)
            closeToCenterCount++;
    }
    if (closeToCenterCount < 10)
        return false;

    // 所有条件都满足，判断为斑马线
    return changes >= 5;
}

bool LaneProcessor::isLaneContinuous(const vector<TrackPoint> &lane, float max_deviation)
{
    // 1. 数据有效性检查
    if (lane.size() < 10)
        return false;

    // 2. 提取有效点（过滤x=0/image_h-1等无效值）
    vector<Point> valid_points;
    for (const auto &pt : lane)
    {
        if (pt.position.x > 1.0f && pt.position.x < image_h - 1)
        { // 留1像素安全边界
            valid_points.emplace_back(pt.position.x, pt.position.y);
        }
    }
    if (valid_points.size() < 5)
        return false; // 有效点太少直接返回

    // 3. 滑动窗口检测连续性
    const int window_size = 5; // 滑动窗口大小
    int valid_windows = 0;     // 有效连续窗口数

    for (size_t i = 0; i <= valid_points.size() - window_size; ++i)
    {
        float k, b, r2;
        linearRegression(
            valid_points.begin() + i,
            valid_points.begin() + i + window_size,
            k, b, r2);

        // R²>0.85表示线性显著，且斜率变化合理（过滤突变）
        if (r2 > 0.85f && abs(k) < 2.0f)
        {
            valid_windows++;
        }
    }

    // 4. 连续性比例计算
    float continuity_ratio = static_cast<float>(valid_windows) /
                             (valid_points.size() - window_size + 1);
    return continuity_ratio > (1.0f - max_deviation);
}
// 检测车道点
void LaneProcessor::detectLanePoints(const Mat &binaryImage, int roiHeight,
                                     const std::vector<int> &whitePixels,
                                     std::vector<TrackPoint> &leftLane,
                                     std::vector<TrackPoint> &rightLane,
                                     int &leftMissedPoints, int &rightMissedPoints,
                                     float &leftMissedRadius, float &rightMissedRadius)
{
    // 初始化丢点计数器
    leftMissedPoints = 0;
    rightMissedPoints = 0;

    // 初始化左右车道的最大值
    int maxLeft[2] = {-1, -1}, maxRight[2] = {-1, -1};

    // 寻找最长左白条
    for (size_t i = 0; i < whitePixels.size(); i++)
    {
        if (whitePixels[i] > maxLeft[1])
        {
            maxLeft[0] = i;
            maxLeft[1] = whitePixels[i];
        }
    }

    // 寻找最长右白条
    for (int i = whitePixels.size() - 1; i >= 0; i--)
    {
        if (whitePixels[i] > maxRight[1])
        {
            maxRight[0] = i;
            maxRight[1] = whitePixels[i];
        }
    }

    const int cols = binaryImage.cols;

    // 检查 maxLeft 和 maxRight 是否有效
    if (maxLeft[0] == -1 || maxRight[0] == -1)
    {
        cerr << "未找到有效的车道线起始位置" << endl;
        return;
    }
    // 记录最长白条的数量，作为赛道的长度
    int LeftTrackLength = maxLeft[1];
    int RightTrackLength = maxRight[1];
    // 初始化 leftLane 和 rightLane
    leftLane.clear();
    rightLane.clear();
    leftLane.resize(LeftTrackLength, {Point(-1, -1), 0});
    rightLane.resize(RightTrackLength, {Point(-1, -1), 0});

    // 检测左车道点
    for (int y = roiHeight - 1; y >= roiHeight - LeftTrackLength; y--)
    {
        bool pointFound = false;

        // 从左到右扫描
        for (int x = maxLeft[0]; x >= 0; x--)
        {
            // 检查是否为车道边界点（避免越界）
            if (x > 0 && binaryImage.at<uchar>(y, x) != 0 &&
                binaryImage.at<uchar>(y, x - 1) == 0)
            {
                // 存储车道点
                leftLane[y] = {Point(x, y), 0};
                pointFound = true;
                break; // 找到点后跳出循环
            }
        }
        // 如果未找到点，默认以图像左边界为赛道边界
        if (!pointFound)
        {
            leftLane[y] = {Point(0, y), 0};
            leftMissedPoints++;
        }
    }

    // 检测右车道点
    for (int y = roiHeight - 1; y >= roiHeight - RightTrackLength; y--)
    {
        bool pointFound = false;

        // 从右到左扫描
        for (int x = maxRight[0]; x < cols; x++)
        {
            // 检查是否为车道边界点（避免越界）
            if (x < cols - 1 && binaryImage.at<uchar>(y, x) != 0 &&
                binaryImage.at<uchar>(y, x + 1) == 0)
            {
                // 存储车道点
                rightLane[y] = {Point(x, y), 0};
                pointFound = true;
                break; // 找到点后跳出循环
            }
        }
        // 如果未找到点，默认以图像右边界为赛道边界
        if (!pointFound)
        {
            rightLane[y] = {Point(cols - 1, y), 0};
            rightMissedPoints++;
        }
    }
    leftMissedRadius = float(leftMissedPoints) / LeftTrackLength;
    rightMissedRadius = float(rightMissedPoints) / RightTrackLength;
}
// 绘制车道线
void LaneProcessor::drawLanes(Mat &image, int roiHeight,
                              vector<TrackPoint> &leftLane,
                              vector<TrackPoint> &rightLane,
                              vector<Point> &centerLine)
{
    if (leftLane.empty() || rightLane.empty())
    {
        cerr << "左车道或右车道数据为空，无法绘制车道线" << endl;
        return;
    }
    size_t numPoints = min(leftLane.size(), rightLane.size());
    // cerr<<"numPoints"<<numPoints<<endl;
    const int estimatedLaneWidth = 80; // 假设车道宽度为 100 像素，你可以动态计算
    // 清空之前的中线数据
    centerLine.clear();

    bool isleftJumpPoint = false;
    bool isrightJumpPoint = false;

    // 绘制左车道
    for (size_t i = roiHeight - 1; i > roiHeight - numPoints; i--)
    {
        if (abs(leftLane[i].position.x - leftLane[i - 1].position.x) > 10 && abs(leftLane[i + 1].position.x - leftLane[i].position.x) > 10)
        {
            leftLane[i].position.x = leftLane[i - 1].position.x;
        }
        circle(image, leftLane[i].position, 1, Scalar(255, 0, 0), FILLED);
    }
    // 绘制右车道
    for (size_t i = roiHeight - 1; i > roiHeight - numPoints; i--)
    {
        if (abs(rightLane[i].position.x - rightLane[i - 1].position.x) > 10 && abs(rightLane[i + 1].position.x - rightLane[i].position.x) > 10)
        {
            rightLane[i].position.x = rightLane[i - 1].position.x;
        }
        circle(image, rightLane[i].position, 1, Scalar(0, 0, 255), FILLED);
    }
    circle(image, leftJumpPointA, 1, Scalar(0, 0, 0), FILLED);
    circle(image, rightJumpPointA, 1, Scalar(0, 0, 0), FILLED);
    circle(image, leftJumpPointB, 1, Scalar(0, 255, 0), FILLED);
    circle(image, rightJumpPointB, 1, Scalar(0, 255, 0), FILLED);
    // 绘制中线
    if (!leftLane.empty() && !rightLane.empty())
    {
        // 确保左右车道点的数量一致

        for (size_t i = roiHeight - 1; i > roiHeight - numPoints; i--)
        {
            const Point &leftPoint = leftLane[i].position;
            const Point &rightPoint = rightLane[i].position;

            // 跳过无效点
            if (leftPoint.x == -1 || leftPoint.y == -1 ||
                rightPoint.x == -1 || rightPoint.y == -1)
            {
                continue;
            }
            if (circleState == RIGHT_TURN)
            {
                Point center(
                    (leftPoint.x + estimatedLaneWidth),
                    (leftPoint.y));
                if (center.x < image.cols)
                    centerLine.push_back(center);
            }
            // 计算中点
            else if (circleState == LEFT_TURN)
            {
                Point center(
                    (rightPoint.x - estimatedLaneWidth),
                    (rightPoint.y));
                if (center.x > 0)
                    centerLine.push_back(center);
            }
            else if (circleState == CROSSING)
            {
                // 检测跳变点
                // if (rightJumpPoint.x != -1 && leftJumpPoint.x != -1)
                // {
                //     if (leftPoint.y <= leftJumpPoint.y || rightPoint.y <= rightJumpPoint.y)
                //     {
                //         circle(image, rightJumpPoint, 2, Scalar(0, 0, 0), FILLED);
                //         circle(image, leftJumpPoint, 2, Scalar(0, 0, 0), FILLED);
                //         Point temp(
                //             (leftPoint.x + rightPoint.x) / 2,
                //             (leftPoint.y + rightPoint.y) / 2);
                //         Point end(80, 119);
                //         generateVirtualPath(end, temp, virtualPath, true);
                //         centerLine.insert(centerLine.end(), virtualPath.begin(), virtualPath.end());
                //         break;
                //     }
                // }
                Point center(
                    (leftPoint.x + rightPoint.x) / 2,
                    (leftPoint.y + rightPoint.y) / 2);
                centerLine.push_back(center);
            }
            else
            {
                Point center(
                    (leftPoint.x + rightPoint.x) / 2,
                    (leftPoint.y + rightPoint.y) / 2);
                centerLine.push_back(center);
            }
        }

        cerr << "centerLine size: " << centerLine.size() << endl;
        // 只有当中线有点时才绘制
        if (!centerLine.empty())
        {
            // 绘制中线

            polylines(image, centerLine, false, Scalar(0, 255, 0), 1);
        }
        else
        {
            cerr << "未检测到有效的中线点" << endl;
        }
    }
    else
    {
        cerr << "左右车道线数据为空" << endl;
    }
}

// 环岛入口检测（步骤1）
bool LaneProcessor::detectCircleEntry(const vector<TrackPoint> &left,
                                      const vector<TrackPoint> &right, float &leftMissedRadius, float &rightMissedRadius)
{
    // 条件1：左侧单调性检查
    bool leftMonotonic = isLaneContinuous(left, 0.1f);
    bool rightMonotonic = isLaneContinuous(right, 0.1f);
    cerr << "leftMonotonic: " << leftMonotonic << endl;
    cerr << "rightMonotonic: " << rightMonotonic << endl;
    // 综合判定（阈值需实际调整）
    return leftMonotonic && rightMissedRadius > 0.3f && rightMissedRadius < 0.6f && left.size() > 220;
}

// 参数设置
const int JUMP_THRESHOLD = 10;      // 跳变点数的阈值
const int SLOPE_CHECK_WINDOW = 3;   // 检查跳变点数的窗口
const int SMALL_JUMP_THRESHOLD = 5; // 跳变点数平缓的阈值
// 拐点检测（步骤2/5）
void LaneProcessor::findInflectionPoints(const vector<TrackPoint> &lane,
                                         Point &pointA, Point &pointB,
                                         bool &pointA_found, bool &pointB_found,bool isleft)
{
    // 参数定义
    const int JUMP_WINDOW = 5;          // 跳变检测窗口大小
    int MIN_JUMP = 10;            // 拐点最小跳变阈值（绝对值）
    const int CONTINUITY_THRESHOLD = 2; // 连续性检测阈值
    const int MIN_DISTANCE = 20;        // 两个拐点间最小距离
    int sign = 1; // 用于判断跳变方向
    if(!isleft)
        sign = -1;
    // 初始化标志位
    pointA_found = false;
    pointB_found = false;

    // 第一阶段：从末端向前搜索第一个拐点（A点）
    if (!pointA_found)
    {
        for (int i = lane.size() - JUMP_WINDOW - 1; i >= JUMP_WINDOW; i--)
        {
            if (lane[i].position.x == -1)
                continue; // 跳过无效点

            // 计算当前点与前一点的跳变（绝对值）
            int currentJump = sign*(lane[i].position.x - lane[i - 1].position.x);

            // 如果跳变足够大
            if (currentJump >= MIN_JUMP)
            {
                // 检查跳变后的连续性（右侧点）
                bool rightContinuous = true;
                for (int j = i + 1; j < i + JUMP_WINDOW; j++)
                {
                    if (abs(lane[j].position.x - lane[j - 1].position.x) > CONTINUITY_THRESHOLD)
                    {
                        rightContinuous = false;
                        break;
                    }
                }

                // 检查跳变前的连续性（左侧点）
                bool leftContinuous = true;
                for (int j = i - 1; j > i - JUMP_WINDOW; j--)
                {
                    if (abs(lane[j].position.x - lane[j - 1].position.x) > CONTINUITY_THRESHOLD)
                    {
                        leftContinuous = false;
                        break;
                    }
                }

                // 合格条件：大跳变且至少一侧连续
                if ((leftContinuous || rightContinuous))
                {
                    pointA = lane[i].position;
                    pointA_found = true;
                    break; // 找到第一个符合条件的点就退出
                }
            }
        }
    }

    // 第二阶段：从起点向后搜索第二个拐点（B点）
    if (!pointB_found)
    {
        for (int i = JUMP_WINDOW; i < (int)lane.size() - JUMP_WINDOW; i++)
        {
            // 如果已经找到A点，确保B点与A点保持足够距离
            if (pointA_found && abs(pointA.y - lane[i].position.y) < MIN_DISTANCE)
            {
                continue;
            }

            if (lane[i].position.x == -1)
                continue; // 跳过无效点

            // 计算当前点与前一点的跳变（绝对值）
            int currentJump = abs(lane[i].position.x - lane[i - 1].position.x);

            // 如果跳变足够大
            if (currentJump >= MIN_JUMP)
            {
                // 检查跳变后的连续性（右侧点）
                bool rightContinuous = true;
                for (int j = i + 1; j < i + JUMP_WINDOW; j++)
                {
                    if (abs(lane[j].position.x - lane[j - 1].position.x) > CONTINUITY_THRESHOLD)
                    {
                        rightContinuous = false;
                        break;
                    }
                }

                // 检查跳变前的连续性（左侧点）
                bool leftContinuous = true;
                for (int j = i - 1; j > i - JUMP_WINDOW; j--)
                {
                    if (abs(lane[j].position.x - lane[j - 1].position.x) > CONTINUITY_THRESHOLD)
                    {
                        leftContinuous = false;
                        break;
                    }
                }

                // 合格条件：大跳变且至少一侧连续
                if ((leftContinuous || rightContinuous))
                {
                    pointB = lane[i-1].position;
                    pointB_found = true;
                    break; // 找到第一个符合条件的点就退出
                }
            }
        }
    }

    // 输出调试信息
    cerr << "A点坐标: " << pointA << (pointA_found ? " (已找到)" : " (未找到)") << endl;
    cerr << "B点坐标: " << pointB << (pointB_found ? " (已找到)" : " (未找到)") << endl;
}
void LaneProcessor::generateVirtualPath(const Point2f &start, const Point2f &end,
                                        vector<Point> &path,
                                        bool isLeftLane)
{
    path.clear();

    // 计算行数差（必须为整数）
    const int rowDiff = static_cast<int>(end.y) - static_cast<int>(start.y);
    if (rowDiff == 0)
    {
        // 如果两点在同一行，直接连接
        path.push_back(Point(start.x, start.y));
        path.push_back(Point(end.x, end.y));
        return;
    }

    // 确定步数（等于行数差的绝对值）
    const int STEPS = abs(rowDiff);
    path.reserve(STEPS + 1);

    // 计算中间控制点（保持平滑）
    const Point2f midPoint = (start + end) * 0.5f;
    const float offsetX = (isLeftLane ? 0 : 0); // 根据车道方向调整偏移量
    const Point2f controlPoint(midPoint.x + offsetX, midPoint.y);

    // 确保每行只生成一个点
    for (int i = 0; i <= STEPS; ++i)
    {
        // 计算当前行（整数）
        const int currentRow = (rowDiff > 0)
                                   ? (static_cast<int>(start.y) + i)
                                   : (static_cast<int>(start.y) - i);

        // 计算归一化参数（基于行数）
        float t = (STEPS > 0) ? static_cast<float>(i) / STEPS : 0.0f;

        // 二次贝塞尔曲线插值
        float x = (1 - t) * (1 - t) * start.x + 2 * (1 - t) * t * controlPoint.x + t * t * end.x;

        // 确保每行只有一个点
        path.emplace_back(static_cast<int>(x), currentRow);
    }

    // 去重（确保没有重复行）
    auto last = std::unique(path.begin(), path.end(),
                            [](const Point &a, const Point &b)
                            {
                                return a.y == b.y; // 如果y坐标相同则认为重复
                            });
    path.erase(last, path.end());

    // 最终平滑处理（可选）
    smoothPath(path);
}
void LaneProcessor::smoothPath(vector<Point> &path)
{
    if (path.size() < 3)
        return;

    vector<Point> smoothedPath;
    smoothedPath.reserve(path.size());
    smoothedPath.push_back(path.front()); // 保留起点

    for (size_t i = 1; i < path.size() - 1; ++i)
    {
        int x = (path[i - 1].x + path[i].x + path[i + 1].x) / 3;
        int y = (path[i - 1].y + path[i].y + path[i + 1].y) / 3;
        smoothedPath.emplace_back(x, y);
    }

    smoothedPath.push_back(path.back()); // 保留终点
    path.swap(smoothedPath);
}

void LaneProcessor::mergeVirtualPath(vector<TrackPoint> &lane,
                                     const vector<Point> &virtualPath,
                                     bool isLeftLane)
{
    if (virtualPath.empty())
        return;

    // 1. 找到左车道的有效连接点（跳过无效点x=-1）
    auto it = std::find_if(lane.begin(), lane.end(), [](const TrackPoint &tp)
                           { return tp.position.x != -1; });

    // 2. 如果左车道完全无效，直接替换为虚拟路径
    if (it == lane.end())
    {
        lane.clear();
        lane.reserve(virtualPath.size());
        for (const auto &point : virtualPath)
        {
            lane.emplace_back(TrackPoint{point}); // 假设置信度为0
        }
        return;
    }

    // 3. 计算连接点索引
    int connectIndex = std::distance(lane.begin(), it);

    // 4. 覆盖或扩展车道数据
    size_t i = 0;
    for (; i < virtualPath.size() && connectIndex + i < lane.size(); ++i)
    {
        lane[connectIndex + i].position = virtualPath[i];
    }

    // 5. 若虚拟路径更长，扩展车道数据
    for (; i < virtualPath.size(); ++i)
    {
        lane.emplace_back(TrackPoint{virtualPath[i]});
    }

    // 6. 移除原左车道的多余点（可选）
    // if (connectIndex + virtualPath.size() < lane.size())
    // {
    //     lane.erase(lane.begin(),lane.begin() + connectIndex);
    // }
}

bool LaneProcessor::checkExitCondition(const vector<TrackPoint> &Lane, Mat &img, int roiHeight)
{
    // 遍历车道点，检查变化趋势
    for (size_t i = roiHeight - 2; i > roiHeight - Lane.size() + 1; i--)
    {
        // 计算 x 坐标的变化
        int dxPrev = Lane[i].position.x - Lane[i - 1].position.x;
        int dxNext = Lane[i + 1].position.x - Lane[i].position.x;
        bool ispreSmallJump = true;
        bool isnextSmallJump = true;
        // 判断变化趋势是否相反
        if (dxPrev * dxNext < 0)
        {

            for (size_t j = i + SLOPE_CHECK_WINDOW; j > i; j--)
            {
                if (abs(Lane[j].position.x - Lane[j - 1].position.x) > SMALL_JUMP_THRESHOLD)
                {
                    ispreSmallJump = false;
                }
            }

            for (size_t j = i; j > i - SLOPE_CHECK_WINDOW; j--)
            {
                if (abs(Lane[j].position.x - Lane[j - 1].position.x) > SMALL_JUMP_THRESHOLD)
                {
                    isnextSmallJump = false;
                }
            }

            if (ispreSmallJump && isnextSmallJump)
            {
                leftJumpPointA = Lane[i].position;
                return true;
            }
        }
    }
    return false;
}

bool LaneProcessor::isPathClear(const vector<TrackPoint> &leftLane)
{
    int flag = 0;
    // 条件1：车道恢复有效点比例
    for (int i = 0; i < SLOPE_CHECK_WINDOW + 2; i++)
    {
        if (leftLane[i].position.x != 239 && leftLane[i].position.x > 160 && leftLane[i + 1].position.x - leftLane[i].position.x < 3)
        {
            flag++;
        }
    }
    if (flag >= 4)
    {
        return true;
    }
    return false;
}
void LaneProcessor::resetCircleState()
{
    circleState = CIRCLE_INACTIVE;
    leftJumpPointA = Point(-1, -1);
    rightJumpPointA = Point(-1, -1);
    leftJumpPointB = Point(-1, -1);
    rightJumpPointB = Point(-1, -1);
    isleftJumpPointA = false;
    isrightJumpPointA = false;
    isleftJumpPointB = false;
    isrightJumpPointB = false;
    virtualPath.clear();
}

// 最小二乘法线性回归（核心算法）
void LaneProcessor::linearRegression(
    vector<Point>::iterator begin,
    vector<Point>::iterator end,
    float &k, float &b, float &r_squared)
{
    size_t n = std::distance(begin, end);
    if (n == 0)
        return;

    // 计算均值
    float sum_x = 0, sum_y = 0;
    for (auto it = begin; it != end; ++it)
    {
        sum_x += it->x;
        sum_y += it->y;
    }
    float mean_x = sum_x / n;
    float mean_y = sum_y / n;

    // 计算协方差和方差
    float cov_xy = 0, var_x = 0, var_y = 0;
    for (auto it = begin; it != end; ++it)
    {
        float dx = it->x - mean_x;
        float dy = it->y - mean_y;
        cov_xy += dx * dy;
        var_x += dx * dx;
        var_y += dy * dy;
    }

    // 计算斜率和截距
    k = cov_xy / var_x;
    b = mean_y - k * mean_x;

    // 计算决定系数R²
    r_squared = (var_x == 0 || var_y == 0) ? 0 : (cov_xy * cov_xy) / (var_x * var_y);
}

#include <opencv2/opencv.hpp>

cv::Mat ApplyInversePerspective(const cv::Mat &inputImage)
{
    const int RESULT_ROW = 100;
    const int RESULT_COL = 114;

    // 定义逆透视矩阵（double 类型）
    cv::Mat change_un_Mat = (cv::Mat_<double>(3, 3) << -1.508309, 2.023527, -138.070108, 0.072206, 0.613826, -102.201813, 0.001605, 0.023339, -2.756092);

    // 创建输出图像（结果图，大小为 RESULT_ROW x RESULT_COL）
    cv::Mat result(RESULT_ROW, RESULT_COL, inputImage.type(), cv::Scalar(0));

    for (int j = 0; j < RESULT_ROW; ++j)
    {
        for (int i = 0; i < RESULT_COL; ++i)
        {

            // 齐次坐标变换计算原图上的坐标
            double x = (change_un_Mat.at<double>(0, 0) * i +
                        change_un_Mat.at<double>(0, 1) * j +
                        change_un_Mat.at<double>(0, 2));
            double y = (change_un_Mat.at<double>(1, 0) * i +
                        change_un_Mat.at<double>(1, 1) * j +
                        change_un_Mat.at<double>(1, 2));
            double w = (change_un_Mat.at<double>(2, 0) * i +
                        change_un_Mat.at<double>(2, 1) * j +
                        change_un_Mat.at<double>(2, 2));

            int src_x = static_cast<int>(x / w);
            int src_y = static_cast<int>(y / w);

            // 边界检查
            if (src_x >= 0 && src_x < inputImage.cols && src_y >= 0 && src_y < inputImage.rows)
            {
                result.at<uchar>(j, i) = inputImage.at<uchar>(src_y, src_x);
            }
            else
            {
                result.at<uchar>(j, i) = 0; // 黑色填充
            }
        }
    }

    return result;
}
#include <opencv2/opencv.hpp>

Mat LaneProcessor::ApplyInversePerspective(const cv::Mat &inputImage)
{
    const int RESULT_ROW = 100;
    const int RESULT_COL = 114;

    // 定义逆透视矩阵（double 类型）
    cv::Mat change_un_Mat = (cv::Mat_<double>(3, 3) << -1.330861, 1.785465, -124.138197,
                             0.063711, 0.541612, -94.298591,
                             0.001416, 0.020594, -2.463509);

    // 创建输出图像（结果图，大小为 RESULT_ROW x RESULT_COL）
    cv::Mat result(RESULT_ROW, RESULT_COL, inputImage.type(), cv::Scalar(0));

    for (int j = 0; j < RESULT_ROW; ++j)
    {
        for (int i = 0; i < RESULT_COL; ++i)
        {

            // 齐次坐标变换计算原图上的坐标
            double x = (change_un_Mat.at<double>(0, 0) * i +
                        change_un_Mat.at<double>(0, 1) * j +
                        change_un_Mat.at<double>(0, 2));
            double y = (change_un_Mat.at<double>(1, 0) * i +
                        change_un_Mat.at<double>(1, 1) * j +
                        change_un_Mat.at<double>(1, 2));
            double w = (change_un_Mat.at<double>(2, 0) * i +
                        change_un_Mat.at<double>(2, 1) * j +
                        change_un_Mat.at<double>(2, 2));

            int src_x = static_cast<int>(x / w);
            int src_y = static_cast<int>(y / w);

            // 边界检查
            if (src_x >= 0 && src_x < inputImage.cols && src_y >= 0 && src_y < inputImage.rows)
            {
                result.at<uchar>(j, i) = inputImage.at<uchar>(src_y, src_x);
            }
            else
            {
                result.at<uchar>(j, i) = 0; // 黑色填充
            }
        }
    }

    return result;
}
Point LaneProcessor::findInflectionPoint(const vector<TrackPoint> &points)
{
    int n = points.size();
    if (n < 3)
        return {-1, -1}; // 至少3个点才能判断拐点

    // 从倒数第二个点往前遍历（排除边界）
    for (int i = n - 2; i > 0; --i)
    {
        double dy1 = points[i].position.x - points[i - 1].position.x; // 当前点和前一个点差值
        double dy2 = points[i + 1].position.x - points[i].position.x; // 后一个点和当前点差值

        // 判断趋势是否发生变化（一个正一个负）
        if (dy1 * dy2 < 0)
        {
            return points[i].position; // 当前点是拐点
        }
    }

    return {0, 0}; // 没找到拐点
}
