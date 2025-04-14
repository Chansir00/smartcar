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


Mat ApplyInversePerspective(const cv::Mat &inputImage)
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
