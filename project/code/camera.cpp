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
    //result.warpedImage = ApplyInversePerspective(result.binaryImage);
    //  计算ROI区域的高度
    int roiHeight = image_h;

    // 计算白点分布
    detectWhitePixels(result.binaryImage, roiHeight, whitePixels);

    // 检测车道点
    detectLanePoints(result.binaryImage, roiHeight, whitePixels, leftLane, rightLane, leftMissedPoints, rightMissedPoints, leftMissedRadius, rightMissedRadius);
    processCircle(leftLane, rightLane, result.binaryImage, roiHeight, leftMissedRadius, rightMissedRadius);

    // 每帧结束时检查重置
    if (circleState == CIRCLE_INACTIVE)
    {
        resetCircleState();
    }
    // 检测斑马线
    //result.hasZebraCrossing = detectZebraCrossing(whitePixels, leftMissedRadius, rightMissedRadius);

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
void LaneProcessor::processCircle(vector<TrackPoint> &LeftLane,
                                  vector<TrackPoint> &RightLane,
                                  Mat &img, int &roiHeight, float &leftMissedRadius, float &rightMissedRadius)
{

    if (countcircle == 50)
    {
        circleflag = 1;
        countcircle = 0;
    }
    if (circleflag == 0)
    {
        countcircle++;
        return;
    }
    findInflectionPoints(rightLane, rightJumpPointA, rightJumpPointB, isrightJumpvalid);
    findInflectionPoints(LeftLane, leftJumpPointA, leftJumpPointB, isleftJumpvalid);
    isleftLanecontinuous = isLaneContinuous(LeftLane);
    isrightLanecontinuous = isLaneContinuous(RightLane);
    cerr << "circleflag: " << circleflag << endl;
    cerr << "isleftJumpvalid: " << isleftJumpvalid << endl;
    cerr << "isrightJumpvalid: " << isrightJumpvalid << endl;
    cerr << "leftJumpPointA: " << leftJumpPointA << endl;
    cerr << "leftJumpPointB: " << leftJumpPointB << endl;
    cerr << "rightJumpPointA: " << rightJumpPointA << endl;
    cerr << "rightJumpPointB: " << rightJumpPointB << endl;
    cerr << "isleftLanecontinuous: " << isleftLanecontinuous << endl;
    cerr << "isrightLanecontinuous: " << isrightLanecontinuous << endl;
    cerr << circleState << endl;
    cerr << "rightMissedRadius: " << rightMissedRadius << endl;
    cerr << "leftMissedRadius: " << leftMissedRadius << endl;

    switch (circleState)
    {
    case CIRCLE_INACTIVE:
    {
        gpio_set_level(BEEP, 0x0);

        // 先通过左车道单调性和右车道丢点率判断是否可能进入环岛
        if (!isleftJumpvalid&&leftJumpPointB.x==-1&&rightJumpPointB.x!=-1&&isleftLanecontinuous&&leftMissedRadius<0.1&&rightMissedRadius<0.5 && numPoints>0.67 * image_h)
        {
            circleState = RIGHT_CIRCLE_DETECTED;
        }
        else if(!isrightJumpvalid&&rightJumpPointB.x==-1&&leftJumpPointB.x!=-1&&isrightLanecontinuous&&rightMissedRadius<0.1&&leftMissedRadius<0.5 && numPoints>0.67 * image_h)
        {
            circleState = LEFT_CIRCLE_DETECTED;
        }
        else if (rightMissedRadius > 0.9 && leftMissedRadius < 0.1 && numPoints < 0.6 * image_h)
        {
            circleState = RIGHT_TURN;
        }
        else if (rightMissedRadius < 0.1 && leftMissedRadius > 0.9 && numPoints< 0.6 * image_h)
        {
            circleState = LEFT_TURN;
        }

        else if (((isrightJumpvalid&&isleftJumpvalid)||(isrightJumpvalid&&leftJumpPointB.x!=-1)||(isleftJumpvalid&&rightJumpPointB.x!=-1))&&!isleftLanecontinuous&&!isrightLanecontinuous&&numPoints>0.6*image_h)
        {
            circleState = CROSSING;
        }
        else
        {
            circleState = CIRCLE_INACTIVE;
        }
    }
    break;

    case RIGHT_CIRCLE_DETECTED:
    {
        gpio_set_level(BEEP, 0x1);
        // if (rightJumpPointB.x < 40)
        // {
        //     circleState = CIRCLE_INSIDE;
        // }
        if(isrightLanecontinuous||(leftJumpPointA.x!=-1))
        {
            circleState = CIRCLE_INACTIVE;
            break;
        }
        if (rightJumpPointB.x!=-1 && rightJumpPointB.y<=60)
        {
            generateVirtualPath(RightLane[RightLane.size()-1].position,rightJumpPointB,rightvirtualPath,true);
            mergeVirtualPath(RightLane,rightvirtualPath,true);
        }
        else if(rightJumpPointB.x!=-1 && rightJumpPointB.y>60&&!isrightJumpvalid)
        {
            Point2f leftStart = LeftLane[LeftLane.size()].position;
            generateVirtualPath(leftStart, rightJumpPointB, leftvirtualPath, true);
            mergeVirtualPath(LeftLane, leftvirtualPath, true);
        }

    }
    break;
    case LEFT_CIRCLE_DETECTED:
    {
        gpio_set_level(BEEP, 0x1);
        // if (leftJumpPointB.x < 40)
        // {
        //    circleState = CIRCLE_INSIDE;
        // }
        if(isleftLanecontinuous||(rightJumpPointA.x!=-1))
        {
            circleState = CIRCLE_INACTIVE;
            break;
        }
        if (leftJumpPointB.x!=-1 && leftJumpPointB.y<=60)
        {
            generateVirtualPath(LeftLane[LeftLane.size()-1].position,leftJumpPointB,leftvirtualPath,true);
            mergeVirtualPath(LeftLane,leftvirtualPath,true);
        }
        else if(leftJumpPointB.x!=-1 && leftJumpPointB.y>60&&!isleftJumpvalid)
        {
            Point2f rightStart = RightLane[RightLane.size()].position;
            generateVirtualPath(rightStart, leftJumpPointB, rightvirtualPath, true);
            mergeVirtualPath(RightLane, rightvirtualPath, true);
        }

    }
    case CIRCLE_INSIDE:
    {
        // if (checkExitCondition(leftLane, img, roiHeight))
        // {
            //circleState = CIRCLE_EXITING;
        // }
    }
    break;

    case CIRCLE_EXITING:
    {
        checkExitCondition(LeftLane, img, roiHeight);
        // circle(img, circlePointB, 5, Scalar(0, 0, 0), -1); // 半径 1，颜色黑色，厚度 -1 表示填充
        Point2f endPoint = Point2f(160, 0);
        // circle(img, rightLane[239].position, 5, Scalar(0, 0, 0), -1); // 半径 1，颜色黑色，厚度 -1 表示填充
        for (int i = roiHeight - 1; i > roiHeight - RightLane.size(); i--)
        {
            if (RightLane[i - 1].position.x != image_h - 1 && RightLane[i].position.x == image_h - 1 && RightLane[i + 1].position.x == image_h - 1 && RightLane[i + 2].position.x == image_h - 1)
            {
                endPoint = RightLane[i].position;
            }
        }
        circle(img, endPoint, 5, Scalar(0, 0, 0), -1); // 半径 1，颜色黑色，厚度 -1 表示填充
        // generateVirtualPath(circlePointB, endPoint, virtualPath, true);
        polylines(img, leftvirtualPath, false, Scalar(255, 0, 0), 2);
        mergeVirtualPath(LeftLane, leftvirtualPath, true);
        if (isPathClear(RightLane))
        {
            circleState = CIRCLE_INACTIVE;
            circleflag = 0;
            leftvirtualPath.clear();
        }
    }
    break;
    case LEFT_TURN:
    {
        if (numPoints>0.6* image_h || leftMissedRadius < 0.9)
        {
            circleState = CIRCLE_INACTIVE;
        }
    }
    break;
    case RIGHT_TURN:
    {
        if (numPoints> 0.6 * image_h || rightMissedRadius < 0.9)
        {
            circleState = CIRCLE_INACTIVE;
        }
    }
    break;
    case CROSSING:
    {
        if ((rightJumpPointB.y>70&&leftJumpPointB.y>70)||(isleftLanecontinuous&&isrightLanecontinuous)||(!isrightJumpvalid&&!isleftJumpvalid))
        {
            circleState = CIRCLE_INACTIVE;
            break;
        }
        if(isrightJumpvalid&&isleftJumpvalid)
        {
            leftJumpPointA = leftJumpPointA;
        }
        else if (isrightJumpvalid)
        {
            leftJumpPointA.x = rightJumpPointA.x - 70;
            leftJumpPointA.y = rightJumpPointA.y;
            leftJumpPointB.x = rightJumpPointB.x - 40;
            leftJumpPointB.y = rightJumpPointB.y;
        }
        else if (isleftJumpvalid)
        {
            rightJumpPointA.x = leftJumpPointA.x + 70;
            rightJumpPointA.y = leftJumpPointA.y;
            rightJumpPointB.x = leftJumpPointB.x + 40;
            rightJumpPointB.y = leftJumpPointB.y;
        }
        else
        {
            leftJumpPointA = leftLane[leftLane.size() - 1].position;
            rightJumpPointA = rightLane[rightLane.size() - 1].position;
        }
        generateVirtualPath(rightJumpPointA, rightJumpPointB, rightvirtualPath, true);
        generateVirtualPath(leftJumpPointA, leftJumpPointB, leftvirtualPath, true);
        mergeVirtualPath(LeftLane, leftvirtualPath, true);
        mergeVirtualPath(RightLane, rightvirtualPath, true);
    }
    break;
    default:
        break;
    }

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
    leftvirtualPath = std::vector<Point>(image_h, Point{0, 0});
    rightvirtualPath = std::vector<Point>(image_h, Point{0, 0});

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

bool LaneProcessor::isLaneContinuous(const vector<TrackPoint>& lane) 
{
    // 1. 数据有效性检查
    if (lane.size() < 2) // 至少需要2个点才能比较
        return false;

    // 2. 检查所有相邻点的x差值
    for (size_t i = lane.size()-1; i > startline+1; i--) 
    {
        // 跳过无效点（x=-1或其他标志）
        if (lane[i-1].position.x < 0)
            continue;
            
        // 计算相邻点x坐标差值
        float delta = abs(lane[i].position.x - lane[i-1].position.x);
        
        // 只要有一个差值>5就直接返回不连续
        if (delta > 5.0f) 
        {
            cerr << "车道线不连续，点" << i << "和点" << (i-1) << "的x坐标差值为" << delta << endl;
            return false;
        }
    }


    return true;
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
    for (size_t i = 15; i < whitePixels.size()-15; i++)
    {
        if (whitePixels[i] > maxLeft[1])
        {
            maxLeft[0] = i;
            maxLeft[1] = whitePixels[i];
        }
    }

    // 寻找最长右白条
    for (int i = whitePixels.size() - 15; i >= 15; i--)
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
    numPoints = min(LeftTrackLength, RightTrackLength);
    startline = roiHeight - numPoints+1;
    // 初始化 leftLane 和 rightLane
    leftLane.clear();
    rightLane.clear();
    leftLane.resize(roiHeight-1, {Point(-1, -1), 0});
    rightLane.resize(roiHeight-1, {Point(-1, -1), 0});

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
    circle(image, leftJumpPointA, 3, Scalar(0, 0, 0), FILLED);
    circle(image, rightJumpPointA, 3, Scalar(0, 0, 0), FILLED);
    circle(image, leftJumpPointB, 3, Scalar(0, 255, 0), FILLED);
    circle(image, rightJumpPointB, 3, Scalar(0, 255, 0), FILLED);
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
            cerr<<"centerLine: " << centerLine[numPoints/2] << endl;
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
    bool leftMonotonic = isLaneContinuous(left);
    bool rightMonotonic = isLaneContinuous(right);
    cerr << "leftMonotonic: " << leftMonotonic << endl;
    cerr << "rightMonotonic: " << rightMonotonic << endl;
    // 综合判定（阈值需实际调整）
    return leftMonotonic && rightMissedRadius > 0.3f && rightMissedRadius < 0.6f && left.size() > 220;
}

// 参数设置
const int JUMP_THRESHOLD = 10;      // 跳变点数的阈值
const int SLOPE_CHECK_WINDOW = 2;   // 检查跳变点数的窗口
const int SMALL_JUMP_THRESHOLD = 5; // 跳变点数平缓的阈值
// 拐点检测（步骤2/5）
void LaneProcessor::findInflectionPoints(const vector<TrackPoint> &lane,
                                         Point &pointA, Point &pointB, bool &isvalid)
{
    isvalid = false;
    bool pointA_found = false;
    bool pointB_found = false;
    pointA = Point(-1, -1);
    pointB = Point(-1, -1);
    // 从索引 239 开始查找 A 点
    int startIndex = 119;
    size_t candidateAIndex = startIndex;
    int maxJumpA = 10;
    int pre_error = 0;
    int error = 0;
    // cerr << lane[239].position << endl;
    if (!pointA_found)
    {
        for (int i = startIndex-2; i > 40; i--)
        {
            //cerr<<lane[i].position<<endl;
            // 计算当前点的跳变点数
            if (lane[i].position.x == -1 )
                continue;
            pre_error = lane[i+2].position.x - lane[i ].position.x;
            error = lane[i].position.x - lane[i-1].position.x;


            if (pre_error*error<0&&pre_error!=error)
            {
                // 
                bool isSmallJump = true;
                for (size_t j = i + SLOPE_CHECK_WINDOW; j > i; j--)
                {
                    if (abs(lane[j].position.x - lane[j - 1].position.x) > SMALL_JUMP_THRESHOLD)
                    {
                        isSmallJump = false;
                        break;
                    }
                }

                if (isSmallJump)
                {
                    // maxJumpA = jump;
                    candidateAIndex = i;
                    pointA_found = true;
                    pointA = lane[candidateAIndex].position;
                    break;
                }
            }
            else if(abs(error)>maxJumpA)
            {
                // 检查右侧五个点的跳变点数是否很小
                bool isSmallJump = true;
                for (size_t j = i + SLOPE_CHECK_WINDOW; j > i; j--)
                {
                    if (abs(lane[j].position.x - lane[j - 1].position.x) > SMALL_JUMP_THRESHOLD)
                    {
                        isSmallJump = false;
                        break;
                    }
                }

                if (isSmallJump)
                {
                    // maxJumpA = jump;
                    candidateAIndex = i;
                    pointA_found = true;
                    pointA = lane[candidateAIndex].position;
                    break;
                }
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
    size_t candidateCIndex = 50;
    int maxJumpC = 5;
    // cerr << candidateCIndex << endl;
    for (size_t i = startline; i < 100; i++)
    {
        // cerr << i << endl;
        if (lane[i].position.x == -1)
            continue;
        // 计算当前点的跳变点数
        int jump = abs(lane[i].position.x - lane[i + 1].position.x);
        if (jump > maxJumpC)
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
                pointB_found = true;
                pointB = lane[candidateCIndex].position;
                break;
            }
        }
    }

    // if (!foundC)
    // {
    //     cerr << "未找到有效的 C 点" << endl;
    //     return false;
    // }

    // 得到 C 点位置
    if (pointA_found && pointB_found)
    {
        if (pointA.x!=-1&&pointB.x!=-1&&abs(pointA.x - pointB.x) >1 && abs(pointA.x - pointB.x) < 50 && abs(pointA.y - pointB.y) > 5)
            isvalid = true;
    }
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
    {
        return;
    }

    // 遍历虚拟车道的每个点
    for (const auto &virtualPoint : virtualPath)
    {
        // 在原车道中查找是否有相同 y 坐标的点
        auto it = std::find_if(lane.begin(), lane.end(),
                               [&virtualPoint](const TrackPoint &tp)
                               {
                                   return tp.position.y == virtualPoint.y;
                               });

        // 如果找到相同 y 坐标的点，就用虚拟点替换它
        if (it != lane.end())
        {
            it->position = virtualPoint; // 只替换坐标，保留其他属性（如置信度）
        }
    }
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
    leftvirtualPath.clear();
    rightvirtualPath.clear();
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


