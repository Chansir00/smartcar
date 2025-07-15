#include "camera.hpp"

VideoCapture cap_init(int camera)
{
    ips200_init("/dev/fb0");
    VideoCapture cap;

    // 最多尝试 2 次（例如 camera=0 失败后尝试 camera=1）
    for (int i = 0; i < 2; ++i)
    {
        cap.open(camera + i, CAP_V4L2); // 尝试打开 camera+i
        if (cap.isOpened())
        {
            // 成功打开，设置分辨率
            cap.set(CAP_PROP_FRAME_WIDTH, 160);
            cap.set(CAP_PROP_FRAME_HEIGHT, 120);
            // cap.set(CAP_PROP_FPS, 60);
            return cap; // 返回成功打开的摄像头
        }
    }

    // 如果全部尝试失败，输出错误信息
    return cap; // 返回未成功打开的 cap（isOpened() 为 false）
}

// int LaneProcessor::juli_caculate(const vector<Point> &Leftline, const vector<Point> &Rightline)
// {
//     int left_count = 0;
//     int right_count = 0;
//     for(int i = 0; i < 120; i++)
//     {
//         if(Leftline[i].x < left_straight[i])
//         {
//             left_count++;
//         }
//         if(Rightline[i].x > right_straight[i])
//         {
//             right_count++;
//         }
//     }
//     if(left_count > right_count)
//     {
//         juli = right_count;
//     }else{
//         juli = left_count;
//     }
//     clamp(juli, 0, 119);
//     cerr << "juli:" << juli << endl;
//     return juli;
// }
int LaneProcessor::juli_caculate(const cv::Mat &binaryImage)
{
    int left_count = 0;
    int right_count = 0;
    
    // 确保二值图像有效
    if (binaryImage.empty() || binaryImage.rows != 120 || binaryImage.cols != 160) {
        cerr << "错误：二值图像无效！" << endl;
        return 0;
    }
    // 从图像底部（第119行）开始向上扫描到顶部（第0行）
    for (int i = 0; i <= 119; i++)
    {
        // 获取当前行对应的左参考列
        int left_col = left_straight[i];
        if (binaryImage.at<uchar>(i, left_col) == 255) {
            left_count++;
        }
      
        int right_col = right_straight[i];
        if (binaryImage.at<uchar>(i, right_col) == 255) {
            right_count++;
        }
    }
    
    // 计算最终距离值
    if (left_count > right_count) {
        juli = right_count;
    } else {
        juli = left_count;
    }
    
    // 确保距离值在合理范围内
    clamp(juli, 0, 119);
    cerr << "juli:" << juli << endl;
    return juli;
}
int LaneProcessor::speed_decide()//const vector<TrackPoint> &Leftline, const vector<TrackPoint> &Rightline
{
    int a = juli;
    if (a <= 80){
        return 0;
    }
    else if (a > 80){
        return 1;
    }
}

// 车道检测逻辑
DetectionResult LaneProcessor::detect(const Mat &inputImage)
{
    int leftMissedPoints, rightMissedPoints;
    float leftMissedRadius, rightMissedRadius;
    DetectionResult result;
    binaryThreshold(inputImage, result.binaryImage, result.outputImage); // 二值化处理

    // 预处理图像

    // result.warpedImage = ApplyInversePerspective(result.binaryImage);
    //    计算ROI区域的高度

    // 计算白点分布
    detectWhitePixels(result.binaryImage, roiHeight, whitePixels);
    juli = juli_caculate(result.binaryImage);
    // 检测车道点
    detectLanePoints(result.binaryImage, roiHeight, whitePixels, leftLane, rightLane, leftMissedPoints, rightMissedPoints, leftMissedRadius, rightMissedRadius);
    processCircle(leftLane, rightLane, result.binaryImage, result.outputImage, roiHeight, leftMissedRadius, rightMissedRadius);
    updateControlParams();

    // 每帧结束时检查重置
    if (circleState == CIRCLE_INACTIVE)
    {
        resetCircleState();
    }
    // 绘制检测结果
    drawLanes(result.outputImage, roiHeight, leftLane, rightLane, centerLine);

    return result;
}



int circleflag = 1;
int countcircle = 0;
bool slow_down = false;
int small_count = 0;
int zebra_count = 0;
int lefttrend = 0;
int righttrend = 0;
int crossing = 0;
// 检测圆环
void LaneProcessor::processCircle(vector<TrackPoint> &LeftLane,
                                  vector<TrackPoint> &RightLane,
                                  Mat &img, Mat &output, int &roiHeight, float &leftMissedRadius, float &rightMissedRadius)
{

    if (countcircle == 50)
    {
        circleflag = 1;
        countcircle = 0;
    }
    if (circleflag == 0)
    {
        countcircle++;
    }
    if(crossing>0)
    {
        crossing--;
    }
    int img_devided = 110;
    findrightInflectionPoints(rightLane, rightJumpPointA, rightJumpPointB, isrightJumpvalid,isrightCrossing);
    findleftInflectionPoints(LeftLane, leftJumpPointA, leftJumpPointB, isleftJumpvalid,isleftCrossing);
    isleftLanecontinuous = isLaneContinuous(LeftLane);
    isrightLanecontinuous = isLaneContinuous(RightLane);
    lefttrend = checkLaneTrend(LeftLane);
    righttrend = checkLaneTrend(RightLane);
    Point righthemisphere = Point(-1, -1);
    Point lefthemisphere = Point(-1, -1);
    bool isleftstraight = false;
    bool isrightstraight = false;
    isleftstraight = isStraightLane(LeftLane, -1);
    isrightstraight = isStraightLane(RightLane, 1);
    // float leftscope = 0;
    // float rightscope = 0;
    // leftscope = calculateLaneSlope(LeftLane);
    // rightscope = calculateLaneSlope(RightLane);
    // map<int, float>lenth = calculateTrackWidthsByY(LeftLane, RightLane);
    righthemisphere = findSuddenChangePoint(RightLane, 0, rightJumpPointA.y);
    lefthemisphere = findSuddenChangePoint(LeftLane, 1, leftJumpPointA.y);
    // cerr << "circleflag: " << circleflag << endl;
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
    cerr << "isleftstraight: " << isleftstraight << endl;
    cerr << "isrightstraight: " << isrightstraight << endl;
    // cerr << "imu963ra_acc_z: " << imu963ra_acc_z << endl;
    cerr << "numPoints: " << numPoints << endl;
    cerr << "lastnumPoints: " << lastnumPoints << endl;
    cerr << "small_count: " << small_count << endl;
    cerr << "lefttrend: " << lefttrend << endl;
    cerr << "righttrend: " << righttrend << endl;
    if (numPoints < 20)
        lost = true;
    if (detectZebraCrossing(whitePixels)&&ready_count>200)
    {
        circleState = ZEBRA;
        gpio_set_level(BEEP, 0x1);
    }
    switch (circleState)
    {
    case CIRCLE_INACTIVE:
    {
        gpio_set_level(BEEP, 0x0);
        // 先通过左车道单调性和右车道丢点率判断是否可能进入环岛
        if (circleflag == 1 && leftJumpPointA.x == -1  && leftMissedRadius < 0.6 &&rightMissedRadius<0.7&& isrightJumpvalid && isleftstraight && !isrightstraight && numPoints >= img_devided)
        {
            circleState = RIGHT_CIRCLE_DETECTED;
        }
        else if (circleflag == 1 && rightJumpPointA.x == -1  && rightMissedRadius < 0.6 &&leftMissedRadius<0.7&& isrightstraight && !isleftstraight && isleftJumpvalid && numPoints >= img_devided)
        {
            circleState = LEFT_CIRCLE_DETECTED;
        }
        else if (((isrightCrossing && isleftCrossing) || (isrightCrossing && leftJumpPointA.x != -1&&leftMissedRadius>0.6) || (rightMissedRadius>0.6&&isleftCrossing && rightJumpPointA.x != -1) || (rightJumpPointB.y >= 30 && leftJumpPointB.y >= 30&&rightJumpPointB.x>=80&&leftJumpPointB.x<=80)) && numPoints > img_devided)
        {
            circleState = CROSSING;
        }
        else if ((isleftstraight && lefttrend == 1 && !isrightstraight&&leftMissedRadius<=0.7 && righttrend!=1 )  &&numPoints<=img_devided)
        {
            circleState = RIGHT_TURN;
        }
        else if ((isrightstraight && righttrend == -1 &&!isleftstraight&& rightMissedRadius<=0.7 && lefttrend!=-1) && numPoints <= img_devided)
        {
            circleState = LEFT_TURN;
        }
        else if (small_count > 0.9 * numPoints && numPoints >= 110 && leftMissedRadius < 0.5 && rightMissedRadius < 0.5 && isleftLanecontinuous&& isrightLanecontinuous&&lefttrend==1 &&righttrend==-1)
        {
            circleState = STRAIGHT;
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
        //  if (rightJumpPointB.x < 40)
        //  {
        //      circleState = CIRCLE_INSIDE;
        //  }
        //  if (isrightLanecontinuous&&isrightLanecontinuous)
        //  {
        //      circleState = CIRCLE_INACTIVE;
        //      break;
        //  }
        Point endPoint(159, 110); // 初始化为无效值
        // for(int i = roiHeight-1;i>startline;i--)
        // {
        //     if (RightLane[i].position.x < 159)
        //     {
        //         endPoint = RightLane[i+1].position;
        //         break;  // 找到第一个就退出循环
        //     }
        // }
        circle(output, righthemisphere, 2, Scalar(0, 0, 0), 2);
        if (rightJumpPointA.x != -1&&rightJumpPointA.y>=20&&rightJumpPointA.x>=rightJumpPointB.x)
        {
            generateVirtualPath(rightJumpPointB, rightJumpPointA, rightvirtualPath, true);
            // if (righthemisphere.x != -1 && righthemisphere.x < rightJumpPointA.x)
            // {
            //     // generateVirtualPath(rightJumpPointA, righthemisphere, rightvirtualPath, true);
            // }
        }
        else if(rightJumpPointB.x<=rightJumpPointA.x)
        {
            generateVirtualPath(rightJumpPointB, endPoint, rightvirtualPath, true);
        }
        mergeVirtualPath(RightLane, rightvirtualPath, -1);
        if (rightJumpPointB.x != -1 && rightJumpPointB.y >= 14)
        {
            //circleState = RIGHT_CIRCLE_INTRY;
            circleState = CIRCLE_INACTIVE;
        }
    }
    break;
    case RIGHT_CIRCLE_INTRY:
    {
        // if (hemisphere.x!=-1&&hemisphere.y>=30)
        // {
        //     prehemisphere = hemisphere;
        //     generateVirtualPath(RightLane[RightLane.size() - 1].position, hemisphere, rightvirtualPath, true);
        //     mergeVirtualPath(RightLane, rightvirtualPath,-1);
        // }
        Point endpoint(150, 0); // 初始化为无效值
        if (numPoints < img_devided && rightJumpPointB.x == -1  && isleftstraight)
        {
            circleState = RIGHT_CIRCLE_INSIDE;
        }
        generateVirtualPath(leftLane[leftLane.size() - 1].position,endpoint, leftvirtualPath, true);
        mergeVirtualPath(LeftLane, leftvirtualPath, -1);
        // mergeVirtualPath(RightLane, rightvirtualPath, rightJumpPointB.y);
    }
    break;
    case RIGHT_CIRCLE_INSIDE:
    {
        if (leftJumpPointA.x != -1 && !isleftstraight && !isleftLanecontinuous)
        {
            circleState = RIGHT_CIRCLE_EXITING;
        }
    }
    break;

    case RIGHT_CIRCLE_EXITING:
    {
        Point endPoint(159, 0); // 初始化为无效值
        // for (int i = startline; i < roiHeight - 1; i++) {
        //     if (RightLane[i].position.x >= 159) {
        //         endPoint = RightLane[i].position;
        //         break;  // 找到第一个就退出循环
        //     }
        // }
        circle(img, endPoint, 2, Scalar(0, 0, 0), 2);
        if (leftJumpPointA.y >=20 && leftJumpPointA.x != 0)
        {
            generateVirtualPath(endPoint, leftJumpPointA, leftvirtualPath, true);
        }
        else
        {
            generateVirtualPath(endPoint, leftLane[leftLane.size() - 1].position, leftvirtualPath, true);
        }

        mergeVirtualPath(LeftLane, leftvirtualPath, -1);

        if (numPoints >= img_devided && isleftstraight && rightJumpPointB.x != -1)
        {
            circleState = RIGHT_CIRCLE_DONE;
        }
    }
    break;
    case RIGHT_CIRCLE_DONE:
    {
        if (rightJumpPointB.x != -1)
        {
            generateVirtualPath(rightJumpPointB, rightLane[rightLane.size() - 1].position, rightvirtualPath, true);
            mergeVirtualPath(rightLane, rightvirtualPath, -1);
        }
        if (rightJumpPointB.x == -1 || rightJumpPointB.x > 120 || rightJumpPointB.y >= 60)
        {
            circleState = CIRCLE_INACTIVE;
            circleflag = 0;
            rightvirtualPath.clear();
        }
    }
    break;
    case LEFT_CIRCLE_DETECTED:
    {
        gpio_set_level(BEEP, 0x1);
        //  if (leftJumpPointB.x < 40)
        //  {
        //      circleState = CIRCLE_INSIDE;
        //  }
        //  if (isleftLanecontinuous&&isleftLanecontinuous)
        //  {
        //      circleState = CIRCLE_INACTIVE;
        //      break;
        //  }

        circle(output, lefthemisphere, 2, Scalar(0, 0, 0), 2);
        if (leftJumpPointA.x != -1&&leftJumpPointA.y>=20&&leftJumpPointA.x<=leftJumpPointB.x)
        {
            generateVirtualPath(leftJumpPointB, leftJumpPointA, leftvirtualPath, true);
            // if (lefthemisphere.x != -1 && lefthemisphere.x > leftJumpPointA.x)
            // {
            //     // generateVirtualPath(leftJumpPointA, lefthemisphere, leftvirtualPath, true);
            // }
            // else
            // {
            //     generateVirtualPath(leftJumpPointB, leftLane[startline + 1].position, leftvirtualPath, true);
            // }
        }
        else if(leftJumpPointB.x >= leftJumpPointA.x)
        {
            generateVirtualPath(leftJumpPointB, leftLane[leftLane.size() - 10].position, leftvirtualPath, true);
        }
        mergeVirtualPath(LeftLane, leftvirtualPath, -1);
        if (leftJumpPointB.x != -1 && leftJumpPointB.y >= 14)
        {
            //circleState = LEFT_CIRCLE_INTRY;
            circleState = CIRCLE_INACTIVE;
        }
    }
    break;
    case LEFT_CIRCLE_INTRY:
    {
        // if (hemisphere.x!=-1&&hemisphere.y>=30)
        // {
        //     prehemisphere = hemisphere;
        //     generateVirtualPath(LeftLane[LeftLane.size() - 1].position, hemisphere, leftvirtualPath, true);
        //     mergeVirtualPath(LeftLane, leftvirtualPath,-1);
        // }
        Point endpoint(10, 0); // 初始化为无效值
        if (numPoints < img_devided && leftJumpPointB.x == -1  && isrightstraight)
        {
            circleState = LEFT_CIRCLE_INSIDE;
        }
        generateVirtualPath(rightLane[rightLane.size() - 1].position,endpoint, rightvirtualPath, true);
        mergeVirtualPath(RightLane, rightvirtualPath, -1);
    }
    break;
    case LEFT_CIRCLE_INSIDE:
    {
        if (rightJumpPointA.x != -1 && !isrightstraight && !isrightLanecontinuous)
        {
            circleState = LEFT_CIRCLE_EXITING;
        }
    }
    break;

    case LEFT_CIRCLE_EXITING:
    {
        Point endPoint(0, 0); // 初始化为无效值
        // for (int i = startline; i < roiHeight - 1; i++) {
        //     if (LeftLane[i].position.x <= 0) {
        //         endPoint = LeftLane[i].position;
        //         break;  // 找到第一个就退出循环
        //     }
        // }
        circle(img, endPoint, 2, Scalar(0, 0, 0), 2);
        if (rightJumpPointA.y >=20 && rightJumpPointA.x != (image_w - 1))
        {
            generateVirtualPath(endPoint, rightJumpPointA, rightvirtualPath, true);
        }
        else
        {
            generateVirtualPath(endPoint, rightLane[rightLane.size() - 1].position, rightvirtualPath, true);
        }

        mergeVirtualPath(RightLane, rightvirtualPath, -1);

        if (numPoints >= img_devided && isrightstraight && leftJumpPointB.x != -1)
        {
            circleState = LEFT_CIRCLE_DONE;
        }
    }
    break;
    case LEFT_CIRCLE_DONE:
    {
        if (leftJumpPointB.x != -1)
        {
            generateVirtualPath(leftJumpPointB, leftLane[leftLane.size() - 1].position, leftvirtualPath, true);
            mergeVirtualPath(leftLane, leftvirtualPath, -1);
        }
        if (leftJumpPointB.x == -1 || leftJumpPointB.y >= 60||leftJumpPointB.x < 60)
        {
            circleflag = 0;
            circleState = CIRCLE_INACTIVE;
            leftvirtualPath.clear();
        }
    }
    break;
    case LEFT_TURN:
    {
        gpio_set_level(BEEP, 0x1);
        if (numPoints > 110|| small_count > 0.9*numPoints)
        {
            circleState = CIRCLE_INACTIVE;
        }
        else if (isleftstraight && lefttrend == 1 &&leftMissedRadius<=0.7 && righttrend!=1 && numPoints <= img_devided ||(leftMissedRadius<0.1&&rightMissedRadius>0.9))
        {
            circleState = RIGHT_TURN;
        }
    }
    break;
    case RIGHT_TURN:
    {
        gpio_set_level(BEEP, 0x1);
        if (numPoints >110 ||small_count > 0.9*numPoints)
        {
            circleState = CIRCLE_INACTIVE;
        }
        else if (isrightstraight && righttrend == -1 && rightMissedRadius<=0.7 && lefttrend!=-1&&numPoints <= img_devided||(rightMissedRadius<0.1&&leftMissedRadius>0.9))
        {
            circleState = LEFT_TURN;
        }
    }
    break;
    case CROSSING:
    {
        if ((rightJumpPointB.x == -1 && leftJumpPointB.x == -1) || (rightJumpPointB.y > 100 || leftJumpPointB.y > 100) || isleftstraight || isrightstraight)
        {
            crossing = 100;
            circleState = CIRCLE_INACTIVE;
            break;
        }
        if (rightJumpPointA.x != -1 && rightJumpPointB.x != -1 && leftJumpPointA.x != -1 && leftJumpPointB.x != -1)
        {
            leftJumpPointA = leftJumpPointA;
        }
        else if (rightJumpPointB.x < rightJumpPointA.x && rightJumpPointB.x > 1 && (leftJumpPointA.x != -1 || leftJumpPointB.x != -1))
        {
            // float widthA = getTrackWIdthFormY(rightJumpPointA.y);
            // float widthB = getTrackWIdthFormY(rightJumpPointB.y);
            // leftJumpPointA.x = rightJumpPointA.x - widthA;
            // leftJumpPointA.y = rightJumpPointA.y;
            // leftJumpPointB.x = rightJumpPointB.x - widthB;
            // leftJumpPointB.y = rightJumpPointB.y;
            if (leftJumpPointB.x == -1)
            {
                leftJumpPointB.y = rightJumpPointB.y;
                leftJumpPointB.x = leftJumpPointA.x;
            }
            else if (leftJumpPointA.x == -1)
            {
                leftJumpPointA.y = rightJumpPointA.y;
                leftJumpPointA.x = leftJumpPointB.x;
            }
        }
        else if (leftJumpPointA.x < leftJumpPointB.x && leftJumpPointA.x > 1 && (rightJumpPointA.x != -1 || rightJumpPointB.x != -1))
        {
            // float widthA = getTrackWIdthFormY(leftJumpPointA.y);
            // float widthB = getTrackWIdthFormY(leftJumpPointB.y);
            // rightJumpPointA.x = leftJumpPointA.x + widthA;
            // rightJumpPointA.y = leftJumpPointA.y;
            // rightJumpPointB.x = leftJumpPointB.x + widthB;
            // rightJumpPointB.y = leftJumpPointB.y;
            if (rightJumpPointB.x == -1)
            {
                rightJumpPointB.y = leftJumpPointB.y;
                rightJumpPointB.x = rightJumpPointA.x;
            }
            else if (rightJumpPointA.x == -1)
            {
                rightJumpPointA.y = leftJumpPointA.y;
                rightJumpPointA.x = rightJumpPointB.x;
            }
        }
        else if (rightJumpPointB.y > 30 && leftJumpPointB.y >= 30 && rightJumpPointB.x>=80&&leftJumpPointB.x<=80)
        {
            leftJumpPointA = leftLane[leftLane.size() - 1].position;
            rightJumpPointA = rightLane[rightLane.size() - 1].position;
        }
        else
        {
            circleState = CIRCLE_INACTIVE;
            break;
        }
        cerr << "leftJumpPointA: " << leftJumpPointA << endl;
        cerr << "leftJumpPointB: " << leftJumpPointB << endl;
        cerr << "rightJumpPointA: " << rightJumpPointA << endl;
        cerr << "rightJumpPointB: " << rightJumpPointB << endl;
        generateVirtualPath(rightJumpPointA, rightJumpPointB, rightvirtualPath, true);
        generateVirtualPath(leftJumpPointA, leftJumpPointB, leftvirtualPath, true);
        mergeVirtualPath(LeftLane, leftvirtualPath, -1);
        mergeVirtualPath(RightLane, rightvirtualPath, -1);
    }
    break;
    case UP:
    {
        if (imu963ra_acc_z < 0.5)
        {
            circleState = CIRCLE_INACTIVE;
        }
    }
    break;
    case ZEBRA:
    {
        gpio_set_level(BEEP, 0x1);
        if(zebra_count<20)
        {
            zebra_count++;
        }
        else
        {
            lost = true;
        }
    }
    break;
    case STRAIGHT:
    {
        gpio_set_level(BEEP, 0x1);
        if (numPoints < 110||rightJumpPointB.x!=-1||leftJumpPointB.x!=-1)
        {
            circleState = CIRCLE_INACTIVE;
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

    // 初始化 centerLine，大小为 image_h，每个元素为 TrackPoint{Point(-1, -1), 0}
    centerLine = std::vector<Point>(image_h, Point{0, 0});

    // 初始化 virtualPath，大小为 image_h，每个元素为 Point{0, 0}
    leftvirtualPath = std::vector<Point>(image_h, Point{0, 0});
    rightvirtualPath = std::vector<Point>(image_h, Point{0, 0});

    // 初始化 circleState
    circleState = CIRCLE_INACTIVE;
}
// 图像预处理：二值化
bool binaryThreshold(const Mat &input, Mat &binary, Mat &output)
{
    if (input.empty())
    {
        cerr << "错误：输入图像为空！" << endl;
        return false;
    }
    Rect roi(0, 34, image_w, image_h -34); // 定义ROI区域
    Mat croppedImage = input(roi);
    //  调整大小为160x120
    Mat resizedImage;
    resize(croppedImage, resizedImage, cv::Size(160, 120));
    output = resizedImage.clone();
    Mat img, blurred;
    cvtColor(resizedImage, img, COLOR_BGR2GRAY);
    GaussianBlur(img, blurred, Size(3, 3), 0);
    threshold(blurred, binary, 0, 255, THRESH_BINARY | THRESH_OTSU);
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
    // 白线检测区间为第60到100列（包括100）
    const int startCol = 60;
    const int endCol = 100;

    // 条件1: 在指定列中检查跳变次数
    int changes = 0;
    for (int i = startCol + 1; i <= endCol; ++i)
    {
        if (i >= (int)whitePixels.size()) break; // 防止越界
        //cerr<< "whitePixels[" << i << "]: " << whitePixels[i] << endl;
        if (abs(whitePixels[i] - whitePixels[i - 1]) > 30)
            changes++;
    }
    
    // 跳变次数不足3次，不是斑马线
    if (changes < 3)
        return false;
    // 所有条件满足，判断为斑马线

    return true;
}


bool LaneProcessor::isLaneContinuous(const vector<TrackPoint> &lane)
{
    // 1. 数据有效性检查
    if (lane.size() < 2)
    { // 至少需要2个点才能比较
        return false;
    }

    // 2. 检查所有相邻点的x差值
    for (size_t i = startline + 1; i < roiHeight - 2; i++)
    {
        // 跳过无效点（x=-1或其他标志）
        if (lane[i - 1].position.x < 0)
        {
            continue;
        }

        // 计算相邻点x坐标差值
        float delta = lane[i].position.x - lane[i - 1].position.x;
        if (abs(delta) > 5)
        {
            cerr << "车道线不连续，点" << i - 1 << "和点" << i << "之间的x差值为" << delta << endl;
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
    for (int i = 10; i < 150; i++)
    {
        if (whitePixels[i] < maxLeft[1]) 
            continue; // 必须比当前最大值大

        // 边界检查（要能访问 i-3 到 i+3）
        if (i < 3 || i > (int)whitePixels.size() - 4)
            continue;

        // 检查左右三列是否稳定（差值 ≤ 10）
        bool stable = true;
        for (int offset = -3; offset <= 3; offset++)
        {
            if (offset == 0)
                continue; // 排除自身
            if (std::abs(whitePixels[i] - whitePixels[i + offset]) > 10)
            {
                stable = false;
                break;
            }
        }

        if (stable)
        {
            maxLeft[0] = i;
            maxLeft[1] = whitePixels[i];
            maxRight[0] = i;
            maxRight[1] = whitePixels[i];
        }
    }
    //cerr << "maxLeft: " << maxLeft[0] << ", count: " << maxLeft[1] << endl;
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
    lastnumPoints = numPoints;
    numPoints = min(LeftTrackLength, RightTrackLength);
    // if (numPoints > 110)
    // {
    //     numPoints = 110;
    // }
    startline = roiHeight - numPoints + 1;
    // 初始化 leftLane 和 rightLane
    leftLane.clear();
    rightLane.clear();
    leftLane.resize(roiHeight - 1, {Point(-1, -1), 0});
    rightLane.resize(roiHeight - 1, {Point(-1, -1), 0});

    // 检测左车道点
    for (int y = roiHeight; y >= roiHeight - numPoints; y--)
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
    for (int y = roiHeight; y >= roiHeight - numPoints; y--)
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
    const int estimatedLaneWidth = 0; // 假设车道宽度为 100 像素，你可以动态计算
    // 清空之前的中线数据
    centerLine.clear();

    bool isleftJumpPoint = false;
    bool isrightJumpPoint = false;
    small_count = 0;
    // 绘制左车道
    for (size_t i = roiHeight; i > roiHeight - numPoints; i--)
    {
        // if (abs(leftLane[i].position.x - leftLane[i - 1].position.x) > 10 && abs(leftLane[i + 1].position.x - leftLane[i].position.x) > 10)
        // {
        //     leftLane[i].position.x = leftLane[i - 1].position.x;
        // }
        circle(image, leftLane[i].position, 1, Scalar(255, 0, 0), FILLED);
        // cerr << "leftLane[" << i << "]: " << leftLane[i].position << endl;
    }
    // 绘制右车道
    for (size_t i = roiHeight; i > roiHeight - numPoints; i--)
    {
        // if (abs(rightLane[i].position.x - rightLane[i - 1].position.x) > 10 && abs(rightLane[i + 1].position.x - rightLane[i].position.x) > 10)
        // {
        //     rightLane[i].position.x = rightLane[i - 1].position.x;
        // }
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

        for (size_t i = roiHeight; i > roiHeight - numPoints; i--)
        {
            const Point &leftPoint = leftLane[i].position;
            const Point &rightPoint = rightLane[i].position;
            Point center = Point(-1, -1); // 初始化中线点为无效值
            // 跳过无效点
            if (leftPoint.x == -1 || leftPoint.y == -1 ||
                rightPoint.x == -1 || rightPoint.y == -1)
            {
                continue;
            }
            if (circleState == RIGHT_TURN)
            {
                center.x = (leftPoint.x + rightPoint.x) / 2 + estimatedLaneWidth;
                center.y = rightPoint.y;
                if (center.x >= image.cols)
                {
                    center.x = image.cols - 1;
                }
                centerLine.push_back(center);
            }
            // 计算中点
            else if (circleState == LEFT_TURN)
            {
                center.x = (leftPoint.x + rightPoint.x) / 2 - estimatedLaneWidth;
                center.y = rightPoint.y;
                if (center.x <= 0)
                {
                    center.x = 0;
                }
                centerLine.push_back(center);
            }
            else if (circleState == RIGHT_CIRCLE_INTRY)
            {
                center.x = (leftPoint.x + 159) / 2;
                center.y = leftPoint.y;
                centerLine.push_back(center);
            }
            else if (circleState == LEFT_CIRCLE_INTRY)
            {
                center.x = (rightPoint.x + 0) / 2;
                center.y = rightPoint.y;
                centerLine.push_back(center);
            }
            else
            {
                center.x = (leftPoint.x + rightPoint.x) / 2;
                center.y = (leftPoint.y + rightPoint.y) / 2;
                centerLine.push_back(center);
            }
            if (abs(center.x - 80) < 10)
            {
                small_count++;
            }
        }
        // cerr << "centerLine size: " << centerLine.size() << endl;
        // 只有当中线有点时才绘制
        // for(auto &point : centerLine)
        // {
        //     cerr << "中线点: " << point << endl;
        // }
        if (!centerLine.empty())
        {
            // 绘制中线

            polylines(image, centerLine, false, Scalar(0, 255, 0), 1);
            cerr << "centerLine: " << centerLine[numPoints / 2] << endl;
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

// 参数设置
const int JUMP_THRESHOLD = 10;      // 跳变点数的阈值
const int SLOPE_CHECK_WINDOW = 1;   // 检查跳变点数的窗口
const int SMALL_JUMP_THRESHOLD = 5; // 跳变点数平缓的阈值
// 拐点检测（步骤2/5）
void LaneProcessor::findrightInflectionPoints(const vector<TrackPoint> &lane,
                                              Point &pointA, Point &pointB, bool &isvalid,bool &isrightCrossing)
{
    isvalid = false;
    isrightCrossing = false;
    bool pointA_found = false;
    bool pointB_found = false;
    pointA = Point(-1, -1);
    pointB = Point(-1, -1);
    // 从索引 239 开始查找 A 点
    int startIndex = 114;
    size_t candidateAIndex = startIndex;
    int maxJumpA = 8;
    int pre_error = 0;
    int error = 0;

    size_t candidateCIndex = 20;
    int maxJumpC = 7;
    // find pointB
    for (size_t i = startline+1 ; i < roiHeight - 5; i++)
    {
        // cerr << i << endl;
        if (lane[i].position.x == -1)
            continue;
        // 计算当前点的跳变点数
        int jump = lane[i].position.x - lane[i + 1].position.x;
        if (jump < -maxJumpC)
        {
            // 检查右侧五个点的跳变点数是否很小
            bool isSmallJump = true;
            cerr << isSmallJump << endl;
            // for (size_t j = i; j > i - SLOPE_CHECK_WINDOW; j--)
            // {
            //     if (lane[i - 1].position.x == -1)
            //         continue;
            //     if (lane[i-1].position.x-lane[i].position.x>0)
            //     {
            //         isSmallJump = false;
            //         break;
            //     }
            // }

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
    // find Point A
    if (!pointA_found)
    {
        for (int i = roiHeight - 5; i > candidateCIndex + 2; i--)
        {
            // cerr<<lane[i].position<<endl;
            //  计算当前点的跳变点数
            if (lane[i].position.x == -1)
                continue;
            pre_error = lane[i + 1].position.x - lane[i].position.x;
            error = lane[i].position.x - lane[i - 1].position.x;

            if (pre_error >= 0 && error < 0)
            {
                // 检查跳变后局部平稳性
                bool isSmallJump = true;
                for (size_t j = i + SLOPE_CHECK_WINDOW; j > i; j--)
                {
                    if (abs(lane[j].position.x - lane[j - 1].position.x) > SMALL_JUMP_THRESHOLD)
                    {
                        isSmallJump = false;
                        break;
                    }
                }

                // 新增判断：后面三个点x坐标是否都小于当前跳变点x
                bool isDescending = true;
                if (i + 3 < lane.size()) // 防止越界
                {
                    int current_x = lane[i].position.x;
                    for (int k = 1; k <= 3; ++k)
                    {
                        if (lane[i - k].position.x < current_x)
                        {
                            isDescending = false;
                            break;
                        }
                    }
                }

                if (isSmallJump && isDescending)
                {
                    candidateAIndex = i;
                    pointA_found = true;
                    pointA = lane[candidateAIndex].position;
                    break;
                }
            }
            else if (error < -maxJumpA)
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
    // if (!foundC)
    // {
    //     cerr << "未找到有效的 C 点" << endl;
    //     return false;
    // }

    // 得到 C 点位置
    if (pointA_found && pointB_found)
    {
        if (pointA.x >= 60 && pointB.x >= 60 && pointA.y - pointB.y >= 3 && pointA.y >= 2  && pointB.x < pointA.x)
            isvalid = true;
        if (pointA.x-pointB.x <50&&pointB.y>10&&pointA.y-pointB.y>3)
        {
            isrightCrossing = true; // 右侧车道线交叉
        }
    }
    
}
void LaneProcessor::findleftInflectionPoints(const vector<TrackPoint> &lane,
                                             Point &pointA, Point &pointB, bool &isvalid,bool &isleftCrossing)
{
    isvalid = false;
    isleftCrossing = false;
    bool pointA_found = false;
    bool pointB_found = false;
    pointA = Point(-1, -1);
    pointB = Point(-1, -1);

    int startIndex = 114;
    size_t candidateAIndex = startIndex;
    int maxJumpA = 8;
    int pre_error = 0;
    int error = 0;

    // 找 B 点
    size_t candidateCIndex = startline;
    int maxJumpC = 7;

    for (size_t i = startline+1 ; i < roiHeight - 5; i++)
    {
        if (lane[i].position.x == -1)
            continue;

        int jump = lane[i + 1].position.x - lane[i].position.x; // 左车道：x 增大

        if (jump < -maxJumpC)
        {
            bool isSmallJump = true;
            // for (size_t j = i; j >= i - SLOPE_CHECK_WINDOW; j--)
            // {
            //     if (lane[j - 1].position.x - lane[j].position.x < 0)
            //     {
            //         isSmallJump = false;
            //         break;
            //     }
            // }

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
    if (!pointA_found)
    {
        for (int i = roiHeight - 5; i > candidateCIndex + 2; i--)
        {
            if (lane[i].position.x == -1)
                continue;
            pre_error = lane[i + 1].position.x - lane[i].position.x;
            error = lane[i].position.x - lane[i - 1].position.x;

            // 判断趋势突变，并确保是左车道（x 变大）
            if (pre_error <= 0 && error > 0)
            {
                // 检查跳变后局部平稳性
                bool isSmallJump = true;
                for (size_t j = i + SLOPE_CHECK_WINDOW; j > i; j--)
                {
                    if (abs(lane[j].position.x - lane[j - 1].position.x) > SMALL_JUMP_THRESHOLD)
                    {
                        isSmallJump = false;
                        break;
                    }
                }

                // 新增判断：后面三个点x坐标是否都小于当前跳变点x
                bool isDescending = true;
                if (i + 3 < lane.size()) // 防止越界
                {
                    int current_x = lane[i].position.x;
                    for (int k = 2; k <= 3; ++k)
                    {
                        if (lane[i - k].position.x > current_x)
                        {
                            isDescending = false;
                            break;
                        }
                    }
                }

                if (isSmallJump && isDescending)
                {
                    candidateAIndex = i;
                    pointA_found = true;
                    pointA = lane[candidateAIndex].position;
                    break;
                }
            }
            else if (error > maxJumpA) // 大跳变向左
            {
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
                    candidateAIndex = i;
                    pointA_found = true;
                    pointA = lane[candidateAIndex].position;
                    break;
                }
            }
        }
    }

    if (pointA_found && pointB_found)
    {
        if (pointA.x <= 100 && pointB.x <= 100 && pointA.y - pointB.y >= 3 && pointA.y >= 2  && pointB.x > pointA.x)
            isvalid = true;
        if (pointB.x-pointA.x <50&&pointB.y>10&&pointA.y-pointB.y>3)
        {
            isleftCrossing = true; // 左侧车道线交叉
        }
    }
}
void LaneProcessor::generateVirtualPath(const Point2f &start, const Point2f &end,
                                        vector<Point> &path,
                                        bool isLeftLane)
{
    path.clear();
    if (start.x < 0 || start.y < 0 || end.x < 0 || end.y < 0)
    {
        cerr << "起点或终点坐标无效，无法生成虚拟路径" << endl;
        return;
    }
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
    // smoothPath(path);
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
                                     float minY = -1.0f)
{
    // 情况1：virtualPath 为空，直接裁剪 lane 中 y < minY 的点
    if (virtualPath.empty())
    {
        if (minY >= -1.0f)
        {
            // 使用 erase-remove 惯用法高效删除 y < minY 的点
            lane.erase(
                std::remove_if(lane.begin(), lane.end(),
                               [minY](const TrackPoint &tp)
                               {
                                   return tp.position.y < minY;
                               }),
                lane.end());
        }
        return; // 无需后续处理
    }

    // 情况2：virtualPath 非空，合并数据
    for (const auto &virtualPoint : virtualPath)
    {
        auto it = std::find_if(lane.begin(), lane.end(),
                               [&virtualPoint](const TrackPoint &tp)
                               {
                                   return tp.position.y == virtualPoint.y;
                               });

        if (it != lane.end())
        {
            it->position = (virtualPoint.y >= minY) ? virtualPoint : Point(-1, -1); // 裁剪
        }
    }

    // 额外处理原车道中 y < minY 的点（如果 minY 有效）
    if (minY >= -1.0f)
    {
        for (auto &tp : lane)
        {
            if (tp.position.y <= minY)
            {
                tp.position = Point(-1, -1); // 裁剪
            }
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

Point LaneProcessor::findSuddenChangePoint(const vector<TrackPoint> &points, bool isLeftLane, int y)
{
    // 从第2个点到倒数第3个点遍历（因为要访问 i-2 和 i+2）

    for (int i = y - 1; i > startline + 1; i--)
    {
        if (points[i].position.x == 119 || points[i].position.x == 0)
        {
            continue; // 跳过无效点
        }
        double x_prev2 = points[i - 1].position.x;
        double x_next2 = points[i + 1].position.x;
        double x_curr = points[i].position.x;
        if (isLeftLane)
        {
            if (x_prev2 <= x_curr && x_next2 < x_curr)
            {
                return points[i].position; // 当前点是突变点
            }
        }
        else
        {
            if (x_prev2 >= x_curr && x_next2 > x_curr)
            {
                return points[i].position;
            }
        }
    }
    return {-1, -1}; // 没找到突变点
}
float LaneProcessor::calculateLaneSlope(const std::vector<TrackPoint> &lane)
{
    if (lane.size() < 2)
        return 0.0f;

    // 统计有效点用于线性拟合
    std::vector<float> xs, ys;

    for (const auto &pt : lane)
    {
        // 左车道排除x==1，右车道排除x==159
        if (pt.position.x == 1.0f || pt.position.x == 159.0f)
            continue;
        xs.push_back(pt.position.x);
        ys.push_back(pt.position.y);
    }

    int n = xs.size();
    if (n < 2)
        return 0.0f; // 点数不足以拟合斜率

    // 最小二乘法拟合直线 y = kx + b，求斜率 k
    float sum_x = 0, sum_y = 0, sum_xy = 0, sum_xx = 0;
    for (int i = 0; i < n; ++i)
    {
        sum_x += xs[i];
        sum_y += ys[i];
        sum_xy += xs[i] * ys[i];
        sum_xx += xs[i] * xs[i];
    }

    float denominator = n * sum_xx - sum_x * sum_x;
    if (denominator == 0.0f)
        return 0.0f; // 避免除以零

    float k = (n * sum_xy - sum_x * sum_y) / denominator;
    return std::fabs(k); // 返回斜率绝对值
}
bool LaneProcessor::isStraightLane(const std::vector<TrackPoint> &lane, int trend)
{
    // 行数不足
    if (lane.size() <= 10)
        return false;

    bool trendSet = (trend != 0); // 外部已提供初始趋势？
    int missedPoints = 0;
    int temp = roiHeight-30-startline;
    for (int i = startline + 10; i <= roiHeight-20; ++i)
    {
        int prevX = lane[i - 1].position.x;
        int currX = lane[i].position.x;

        // 忽略无效点
        if (prevX < 0 || currX < 0)
            continue;
        if(currX >=158 || currX <=1)
        {
            missedPoints++;
            continue; // 跳过无效点
        }
        int dx = currX - prevX;

        // 跳变过大（即使方向一致也不能跳太多）
        if (std::abs(dx) >5)
            return false;

        // 设定初始趋势
        if (!trendSet && dx != 0)
        {
            trend = (dx > 0) ? 1 : -1;
            trendSet = true;
        }

        // 趋势不一致
        if (trend != 0 && ((dx > 0 && trend < 0) || (dx < 0 && trend > 0)))
            return false;

    }
    //cerr<< "missedPoints: " << missedPoints << ", temp: " << temp << endl;
    if(missedPoints/temp >0.7)
    {
        return false; // 跳变点过多，认为不是直道
    }

    return trendSet; // 至少有趋势才能认为是直道
}

map<int, float> LaneProcessor::calculateTrackWidthsByY(
    const std::vector<TrackPoint> &leftLane,
    const std::vector<TrackPoint> &rightLane)
{
    std::map<int, float> widths;
    std::map<int, float> leftMap;
    std::map<int, float> rightMap;

    // 将左车道的点按 y 坐标映射
    for (const auto &p : leftLane)
    {
        int y = static_cast<int>(p.position.y + 0.5f); // 四舍五入到 int
        leftMap[y] = p.position.x;
    }

    // 将右车道的点按 y 坐标映射
    for (const auto &p : rightLane)
    {
        int y = static_cast<int>(p.position.y + 0.5f);
        rightMap[y] = p.position.x;
    }

    // 遍历所有出现过的 y 坐标
    for (const auto &[y, leftX] : leftMap)
    {
        if (rightMap.count(y))
        {
            float rightX = rightMap[y];
            widths[y] = rightX - leftX; // 赛道宽度
        }
        // 若缺失右边界，也可以选择记录为 -1 或跳过，这里选择跳过
    }

    for (const auto &[y, width] : widths)
    {
        cerr << "y: " << y << ", 宽度: " << width << endl;
    }
    return widths;
}

float LaneProcessor::getTrackWIdthFormY(int y)
{
    if (y <= 80)
    {
        // 线性部分（y∈[12,80]）：宽度 = 2.092*y - 11.62
        return 2.092f * y - 11.62f;
    }
    else
    {
        // 三次函数部分（y>80）：限制宽度不超过159
        const float a = -0.000267f;
        const float b = 0.0468f;
        const float c = -0.817f;
        const float d = 22.93f;
        float width = a * y * y * y + b * y * y + c * y + d;
        return (width > 159.0f) ? 159.0f : width; // 强制上限
    }
}
int LaneProcessor::checkLaneTrend(const std::vector<TrackPoint> &lane)
{
    if (lane.empty())
        return 0;

    bool allIncreasing = true;
    bool allDecreasing = true;

    for (size_t i = startline + 1; i < roiHeight - 10; ++i)
    {
        int prevX = lane[i - 1].position.x;
        int currX = lane[i].position.x;

        if (currX < prevX)
            // cerr << "当前点: " << currX << ", 前一个点: " << prevX << endl;
            allIncreasing = false;
        if (currX > prevX)
        {
            // cerr << "当前点: " << currX << ", 前一个点: " << prevX << endl;
            allDecreasing = false;
        }
    }

    if (allIncreasing)
        return -1;
    if (allDecreasing)
        return 1;
    return 0;
}
