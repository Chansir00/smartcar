#include <Pers.h>
#include <cameratest.h>
/**
 * @brief 对输入图像进行逆透视变换
 * @param imagePath 输入图像的路径
 * @return 逆透视变换后的图像
 */
Mat applyInversePerspectiveTransform(const Mat &image) {
    // 读取图像

    // 定义源图像中的四个点（例如：图像中的四边形区域）
    std::vector<cv::Point2f> srcPoints;
    srcPoints.push_back(cv::Point2f(82,25));       // 左上角
    srcPoints.push_back(cv::Point2f(289,19));      // 右上角
    srcPoints.push_back(cv::Point2f(317,153));     // 右下角
    srcPoints.push_back(cv::Point2f(4,145));    // 左下角

    // 定义目标平面中的四个点（逆透视变换后的矩形区域）
    std::vector<cv::Point2f> dstPoints;
    dstPoints.push_back(cv::Point2f(0, 0));         // 左上角
    dstPoints.push_back(cv::Point2f(320, 0));       // 右上角
    dstPoints.push_back(cv::Point2f(320, 240));     // 右下角
    dstPoints.push_back(cv::Point2f(0, 240));       // 左下角

    // 计算透视变换矩阵
    cv::Mat M = cv::getPerspectiveTransform(srcPoints, dstPoints);

    // 进行逆透视变换
    cv::Mat warpedImage;
    cv::warpPerspective(image, warpedImage, M, cv::Size(320, 240));

    return warpedImage;
}




