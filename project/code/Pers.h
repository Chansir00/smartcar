#ifndef PERS_H
#define PERS_H

#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>


using namespace cv;
using namespace std;

Mat applyInversePerspectiveTransform(const Mat &image); // 逆透视变换

void ImagePerspective_Init(Mat &SimBinImage,Mat &PerImg_ip);
#endif // PERS_H