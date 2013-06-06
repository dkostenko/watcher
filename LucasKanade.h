/* 
 * File:   LucasKanade.h
 * Author: macbook
 *
 * Created on 14 Апрель 2013 г., 12:19
 */

#ifndef LUCASKANADE_H
#define	LUCASKANADE_H

#include "helper.h"
#include <opencv2/opencv.hpp>

class LucasKanade {
public:
    LucasKanade();
    virtual ~LucasKanade();
    
    bool trackf2f(const cv::Mat& img1, const cv::Mat& img2, std::vector<cv::Point2f> &points1, std::vector<cv::Point2f> &points2);
    float getFB(){return fbmed;}

private:
    std::vector<cv::Point2f> pointsFB;
    cv::Size window_size;
    int level;
    std::vector<uchar> status;
    std::vector<uchar> FB_status;
    std::vector<float> similarity;
    std::vector<float> FB_error;
    float simmed;
    float fbmed;
    cv::TermCriteria term_criteria;
    float lambda;
    void normCrossCorrelation(const cv::Mat& img1,const cv::Mat& img2, std::vector<cv::Point2f>& points1, std::vector<cv::Point2f>& points2);
    bool filterPts(std::vector<cv::Point2f>& points1,std::vector<cv::Point2f>& points2);
};

#endif	/* LUCASKANADE_H */

