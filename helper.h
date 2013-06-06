/* 
 * File:   helper.h
 * Author: macbook
 *
 * Created on 26 Апрель 2013 г., 15:18
 */

#ifndef HELPER_H
#define	HELPER_H
#include <opencv2/opencv.hpp>

void drawBox(cv::Mat& image, CvRect box, cv::Scalar color = cvScalarAll(255), int thick=1); 

float median(std::vector<float> v);

std::vector<int> index_shuffle(int begin,int end);

#endif	/* HELPER_H */

