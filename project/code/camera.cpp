#include "cameratest.h"

// 车道检测逻辑
DetectionResult LaneProcessor::detect(const Mat &inputImage)
{
    int leftMissedPoints, rightMissedPoints;
    DetectionResult result;
    result.outputImage = inputImage.clone();
    // 预处理图像
    binaryThreshold(inputImage, result.binaryImage);
    //result.warpedImage = applyInversePerspectiveTransform(result.binaryImage);
    result.warpedImage = result.binaryImage;
    // 计算ROI区域的高度
    int roiHeight = image_h * FLYING_RATIO;

    // 计算白点分布
    detectWhitePixels(result.warpedImage, roiHeight, whitePixels);

    // 检测车道点
    detectLanePoints(result.warpedImage, roiHeight, whitePixels, leftLane, rightLane, leftMissedPoints, rightMissedPoints);
    processCircle(leftLane, rightLane, result.warpedImage, leftMissedPoints, rightMissedPoints);
    cerr << circleState << endl;
    // 每帧结束时检查重置
    if (circleState == CIRCLE_INACTIVE)
    {
        resetCircleState();
    }
    // 检测斑马线
    result.hasZebraCrossing = detectZebraCrossing(whitePixels);

    // 绘制检测结果

    centre = drawLanes(result.warpedImage, roiHeight, leftLane, rightLane, centerLine);

    // 如果检测到斑马线，添加文本标注
    if (result.hasZebraCrossing)
    {
        putText(result.warpedImage, "ZEBRA CROSSING", Point(20, 40),
                FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 0, 255), 2);
    }

    return result;
}

// 检测圆环
void LaneProcessor::processCircle(vector<TrackPoint> &leftLane,
                                  vector<TrackPoint> &rightLane,
                                  Mat &img, int &leftMissedPoints, int &rightMissedPoints)
{
    switch (circleState)
    {
    case CIRCLE_INACTIVE:
        // 先通过左车道单调性和右车道丢点率判断是否可能进入环岛
        if (detectCircleEntry(leftLane, rightLane, leftMissedPoints, rightMissedPoints))
        {
            circleState = CIRCLE_DETECTED;
        }
        break;

    case CIRCLE_DETECTED:
        if (findInflectionPoints(rightLane, circlePointA, circlePointC))
            ;
        {
            circle(img, circlePointA, 5, Scalar(0, 0, 0), -1); // 半径 1，颜色黑色，厚度 -1 表示填充
            circle(img, circlePointC, 5, Scalar(0, 0, 0), -1); // 半径 1，颜色黑色，厚度 -1 表示填充
            // 满足三个特征：A 点（正向变化且另一端平缓）、圆弧、C 点（负向变化且另一端平缓）
            // 从左车道中寻找一个有效的连接点（例如第一个有效点）
            Point2f leftStart;
            bool found = false;
            leftStart = leftLane[120].position;
            // 生成虚拟路径：从左车道的连接点延伸到 C 点
            vector<Point> virtualPath;
            // isLeftLane = true 表示生成时向左偏移，具体根据你的车道配置而定
            //generateVirtualPath(leftStart, circlePointC, virtualPath, true);
            //cerr << "virtualPath: " << virtualPath.size() << endl;
            // 绘制调试信息
            // polylines(img, virtualPath, false, Scalar(255, 0, 0), 2);
            // 将生成的虚拟路径合并到左车道中，实现车道线拉线
            //mergeVirtualPath(leftLane, virtualPath, true);

            // 状态转移到环内循迹（根据需要也可以直接进入 INSIDE 状态）
            // circleState = CIRCLE_INSIDE;
        }
        break;
    case CIRCLE_INSIDE:
    {
        vector<Point> innerPath;
        // 利用右侧车道或环岛边缘作为参考（此处示例右车道偏移）
        for (auto &p : rightLane)
        {
            if (p.position.x != -1)
            {
                innerPath.push_back(p.position + Point(15, 0));
            }
        }
        polylines(img, innerPath, false, Scalar(200, 0, 200), 2);

        // 检测出口条件（checkExitCondition() 的实现需自行确认）
        if (checkExitCondition(leftLane, img))
        {
            circleState = CIRCLE_EXITING;
        }
    }
    break;

    case CIRCLE_EXITING:
    {
        generateVirtualPath(circlePointC, circlePointA, virtualPath, true);
        polylines(img, virtualPath, false, Scalar(0, 200, 200), 2);
        if (isPathClear(leftLane))
        {
            circleState = CIRCLE_INACTIVE;
            virtualPath.clear();
        }
    }
    break;

    default:
        break;
    }
}
// initializeVariables
void LaneProcessor::initializeVariables(int image_w, int image_h)
{
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

    // 初始化 circlePointA, circlePointB, circlePointC
    circlePointA = Point2f(-1, -1);
    circlePointB = Point2f(-1, -1);
    circlePointC = Point2f(-1, -1);

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
bool LaneProcessor::detectZebraCrossing(const std::vector<int> &whitePixels)
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
    bool isLeftLaneContinuous = isLaneContinuous(leftLane);
    bool isRightLaneContinuous = isLaneContinuous(rightLane);
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

bool LaneProcessor::isLaneContinuous(const std::vector<TrackPoint> &lane)
{
    bool isMonotonic = false;
    int left_count = 0;
    for (size_t i = 0; i < lane.size(); i++)
    {
        if (lane[i].position.x == 0 || lane[i].position.x == -1)
            continue;
        int prevX = lane[i].position.x;
        if (abs(lane[i + 1].position.x - prevX) < 5)
        { // 允许的波动阈值
            left_count++;
        }
    }
    if (left_count > 120)
        isMonotonic = true;
    return isMonotonic;
}
// 检测车道点
void LaneProcessor::detectLanePoints(const Mat &binaryImage, int roiHeight,
                                     const std::vector<int> &whitePixels,
                                     std::vector<TrackPoint> &leftLane,
                                     std::vector<TrackPoint> &rightLane,
                                     int &leftMissedPoints, int &rightMissedPoints)
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
}
// 绘制车道线
int LaneProcessor::drawLanes(Mat &image, int roiHeight,
                             const vector<TrackPoint> &leftLane,
                             const vector<TrackPoint> &rightLane,
                             vector<Point> &centerLine)
{
    // 清空之前的中线数据
    centerLine.clear();

    // 绘制左车道
    for (const auto &point : leftLane)
    {
        if (point.position.x != -1 && point.position.y != -1)
        {
            circle(image, point.position, 2, Scalar(255, 0, 0), FILLED);
        }
    }

    // 绘制右车道
    for (const auto &point : rightLane)
    {
        if (point.position.x != -1 && point.position.y != -1)
        {
            circle(image, point.position, 2, Scalar(0, 0, 255), FILLED);
        }
    }

    // 绘制中线
    if (!leftLane.empty() && !rightLane.empty())
    {
        // 确保左右车道点的数量一致
        size_t numPoints = min(leftLane.size(), rightLane.size());

        for (size_t i = 0; i < numPoints; i++)
        {
            const Point &leftPoint = leftLane[i].position;
            const Point &rightPoint = rightLane[i].position;

            // 跳过无效点
            if (leftPoint.x == -1 || leftPoint.y == -1 ||
                rightPoint.x == -1 || rightPoint.y == -1)
            {
                continue;
            }

            // 计算中点
            Point center(
                (leftPoint.x + rightPoint.x) / 2,
                (leftPoint.y + rightPoint.y) / 2);
            centerLine.push_back(center);
        }

        // 只有当中线有点时才绘制
        if (!centerLine.empty())
        {
            // // 按 y 坐标排序，确保点顺序正确
            // sort(centerLine.begin(), centerLine.end(),
            //      [](const Point &a, const Point &b)
            //      {
            //          return a.y < b.y; // 从近到远排序
            //      });

            // 绘制中线
            polylines(image, centerLine, false, Scalar(0, 255, 0), 2);

            // 输出中线中点坐标
            int mid_point = centerLine.size() / 2;
            if (mid_point < centerLine.size())
            {
                cerr << "中线中点坐标: " << centerLine[mid_point] << endl;
                cerr << centerLine.size() << endl;
            }
            return centerLine[mid_point].x;
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
    return 0;
}

// 环岛入口检测（步骤1）
bool LaneProcessor::detectCircleEntry(const vector<TrackPoint> &left,
                                      const vector<TrackPoint> &right, int &leftMissedPoints, int &rightMissedPoints)
{
    // 条件1：左侧单调性检查
    bool leftMonotonic = isLaneContinuous(left);
    // cerr<<"leftsize: "<<left.size()<<endl;
    cerr << "leftMonotonic: " << leftMonotonic << endl;
    // cerr << left[240].position.x << endl;
    //  条件2：右侧丢点率检查
    float leftRatio = leftMissedPoints / (float)left.size();
    float rightRatio = rightMissedPoints / (float)right.size();
    cerr << "lostRatio: " << rightRatio << endl;

    // 综合判定（阈值需实际调整）
    return leftMonotonic && (rightRatio > 0.3f) && (leftRatio < 0.2f);
}

// 参数设置
const int JUMP_THRESHOLD = 10;      // 跳变点数的阈值
const int SLOPE_CHECK_WINDOW = 5;   // 检查跳变点数的窗口
const int SMALL_JUMP_THRESHOLD = 3; // 跳变点数平缓的阈值
// 拐点检测（步骤2/5）
bool LaneProcessor::findInflectionPoints(const vector<TrackPoint> &lane,
                                         Point2f &pointA, Point2f &pointC)
{

    // 从索引 239 开始查找 A 点
    size_t startIndex = 239;
    size_t candidateAIndex = startIndex;
    bool foundA = false;
    int maxJumpA = 10;
    // cerr << lane[239].position << endl;
    for (size_t i = startIndex; i > startIndex - lane.size(); i--)
    {
        // 计算当前点的跳变点数
        if (lane[i].position.x == -1)
            continue;
        int jump = lane[i - 1].position.x - lane[i].position.x;

        if (jump > maxJumpA)
        {
            // 检查左侧五个点的跳变点数是否很小
            bool isSmallJump = true;
            for (size_t j = i - SLOPE_CHECK_WINDOW; j < i; ++j)
            {
                if (abs(lane[j].position.x - lane[j - 1].position.x) > SMALL_JUMP_THRESHOLD)
                {
                    isSmallJump = false;
                    break;
                }
            }

            if (isSmallJump)
            {
                maxJumpA = jump;
                candidateAIndex = i;
                foundA = true;
                pointA = lane[candidateAIndex].position;
            }
        }
    }

    // if (!foundA)
    // {
    //     cerr << "未找到有效的 A 点" << endl;
    //     return false;
    // }

    // 得到 A 点位置

    // 从 A 点之后查找 C 点
    size_t candidateCIndex = candidateAIndex - 10;
    bool foundC = false;
    int maxJumpC = -10;
    // cerr << candidateCIndex << endl;
    for (size_t i = startIndex - lane.size(); i < candidateCIndex; i++)
    {
        // cerr << i << endl;
        if (lane[i].position.x == -1)
            continue;
        // 计算当前点的跳变点数
        int jump = lane[i].position.x - lane[i + 1].position.x;
        if (jump < maxJumpC)
        {
            // 检查右侧五个点的跳变点数是否很小
            bool isSmallJump = true;
            // cerr << isSmallJump << endl;
            for (size_t j = i; j > i - SLOPE_CHECK_WINDOW; j--)
            {
                if (abs(lane[j].position.x - lane[j - 1].position.x) > SMALL_JUMP_THRESHOLD)
                {
                    isSmallJump = false;
                    break;
                }
            }

            if (isSmallJump)
            {
                maxJumpC = jump;
                candidateCIndex = i;
                foundC = true;
                pointC = lane[candidateCIndex].position;
            }
        }
    }

    // if (!foundC)
    // {
    //     cerr << "未找到有效的 C 点" << endl;
    //     return false;
    // }

    // 得到 C 点位置

    cerr << "A 点坐标: " << pointA << endl;
    cerr << "C 点坐标: " << pointC << endl;

    return foundA && foundC;
}
void LaneProcessor::generateVirtualPath(const Point2f &start, const Point2f &end,
                                        vector<Point> &path,
                                        bool isLeftLane)
{
    path.clear();
    constexpr float LANE_WIDTH =0; // 车道宽度（根据实际环岛道路调整）
    const float distance = std::hypot(end.x - start.x, end.y - start.y);
    const int MIN_STEPS = 10;  // 最小插值点数
    const int MAX_STEPS = 100; // 最大插值点数

    // 动态计算步数：每米至少2个点，避免过密或过疏
    int STEPS = static_cast<int>(distance * 2.0f);
    STEPS = std::clamp(STEPS, MIN_STEPS, MAX_STEPS);

    path.reserve(STEPS + 1);

    // 使用贝塞尔曲线插值生成路径（假设拐点为控制点）

    Point2f midPoint = (start + end) * 0.5f;
    float offsetX = 2.0f; // X方向偏移量
    float offsetY = 2.0f; // Y方向偏移量
    const Point2f controlPoint =Point2f(midPoint.x + offsetX, midPoint.y + offsetY);
    for (int i = 0; i <= STEPS; ++i)
    {
        float t = static_cast<float>(i) / STEPS;
        // 二次贝塞尔曲线插值
        float x = (1 - t) * (1 - t) * start.x + 2 * (1 - t) * t * controlPoint.x + t * t * end.x;
        float y = (1 - t) * (1 - t) * start.y + 2 * (1 - t) * t * controlPoint.y + t * t * end.y;

        // 车道偏移调整：进入右车道拐点需向右偏移
        const float offset = (isLeftLane ? 1 : -1) * LANE_WIDTH;
        x += offset;

        path.emplace_back(x, y);
    }

    // 使用三次样条插值平滑路径
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


// // 平滑连接点附近的路径
// void LaneProcessor::smoothTransition(vector<TrackPoint> &lane,
//                                      int connectIndex, int virtualPathSize)
// {
//     const int SMOOTH_WINDOW = 3; // 平滑窗口大小
//     int start = max(connectIndex - SMOOTH_WINDOW, 0);
//     int end = min(connectIndex + virtualPathSize + SMOOTH_WINDOW, (int)lane.size());

//     for (int i = start; i < end; ++i)
//     {
//         int sumX = 0, sumY = 0, count = 0;
//         for (int j = max(i - SMOOTH_WINDOW, 0); j <= min(i + SMOOTH_WINDOW, (int)lane.size() - 1); ++j)
//         {
//             sumX += lane[j].position.x;
//             sumY += lane[j].position.y;
//             count++;
//         }
//         lane[i].position.x = sumX / count;
//         lane[i].position.y = sumY / count;
//     }
// }
bool LaneProcessor::checkExitCondition(const vector<TrackPoint> &leftLane, Mat &img)
{
    // 条件1：左侧车道点恢复连续
    int continuousCount = 0;
    for (size_t i = 1; i < leftLane.size(); ++i)
    {
        if (abs(leftLane[i].position.x - leftLane[i - 1].position.x) < 5)
        {
            if (++continuousCount > 15)
                return true;
        }
        else
        {
            continuousCount = 0;
        }
    }

    // 条件2：特征点C检测（结合视觉特征）
    vector<Point2f> corners;
    goodFeaturesToTrack(img, corners, 20, 0.01, 30);

    // 检测右下方角点
    for (const auto &pt : corners)
    {
        if (pt.x > img.cols * 0.6 && pt.y > img.rows * 0.7)
        {
            circlePointC = pt;
            return true;
        }
    }

    return false;
}

bool LaneProcessor::isPathClear(const vector<TrackPoint> &leftLane)
{
    // 条件1：左侧车道恢复有效点比例
    int validCount = 0;
    for (const auto &p : leftLane)
    {
        if (p.position.x != -1)
            validCount++;
    }
    if (validCount / (float)leftLane.size() < 0.7)
        return false;

    // 条件2：虚拟路径与真实路径偏差
    if (!virtualPath.empty())
    {
        Point lastVirtual = virtualPath.back();
        Point lastReal = leftLane.back().position;
        return norm(lastVirtual - lastReal) < 20;
    }
    return false;
}
void LaneProcessor::resetCircleState()
{
    circleState = CIRCLE_INACTIVE;
    circlePointA = Point2f(-1, -1);
    circlePointB = Point2f(-1, -1);
    circlePointC = Point2f(-1, -1);
    virtualPath.clear();
}