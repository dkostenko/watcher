/* 
 * File:   LucasKanade.cpp
 * Author: macbook
 * 
 * Created on 14 Апрель 2013 г., 12:19
 */

#include "LucasKanade.h"
using namespace cv;

LucasKanade::LucasKanade() {
    term_criteria = TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 20, 0.03);
    window_size = Size(4,4);
    level = 5;
    lambda = 0.5;
}

LucasKanade::~LucasKanade() {
}

bool LucasKanade::trackf2f(const Mat& img1, const Mat& img2,vector<Point2f> &points1, vector<cv::Point2f> &points2){
    calcOpticalFlowPyrLK(img1,img2, points1, points2, status,similarity, window_size, level, term_criteria, lambda, 0);
    calcOpticalFlowPyrLK(img2,img1, points2, pointsFB, FB_status,FB_error, window_size, level, term_criteria, lambda, 0);
    
    for(int i= 0; i<points1.size(); ++i){
        FB_error[i] = norm(pointsFB[i]-points1[i]);
    }
    
    normCrossCorrelation(img1, img2, points1, points2);
    return filterPts(points1,points2);
}

void LucasKanade::normCrossCorrelation(const Mat& img1,const Mat& img2, vector<Point2f>& points1, vector<Point2f>& points2) {
    Mat rec0(10,10,CV_8U);
    Mat rec1(10,10,CV_8U);
    Mat res(1,1,CV_32F);

    for (int i = 0; i < points1.size(); i++) {
        if (status[i] == 1) {
            getRectSubPix( img1, Size(10,10), points1[i],rec0 );
            getRectSubPix( img2, Size(10,10), points2[i],rec1);
            matchTemplate( rec0,rec1, res, CV_TM_CCOEFF_NORMED);
            similarity[i] = ((float *)(res.data))[0];
        } else {
            similarity[i] = 0.0;
        }
    }
    rec0.release();
    rec1.release();
    res.release();
}


bool LucasKanade::filterPts(vector<Point2f>& points1,vector<Point2f>& points2){
    simmed = median(similarity);
    size_t i, k;
    
    for(i = k = 0; i<points2.size(); ++i){
        if(!status[i]){
            continue;
        }
        
        if(similarity[i] > simmed){
            points1[k] = points1[i];
            points2[k] = points2[i];
            FB_error[k] = FB_error[i];
            ++k;
        }
    }
    
    if(k==0){
        return false;
    }
    
    points1.resize(k);
    points2.resize(k);
    FB_error.resize(k);

    fbmed = median(FB_error);
    for(i=k=0; i<points2.size(); ++i){
        if(!status[i]){
            continue;
        }
        if(FB_error[i] <= fbmed){
            points1[k] = points1[i];
            points2[k] = points2[i];
            k++;
        }
    }
    
    points1.resize(k);
    points2.resize(k);
    
    if(k==0){
        return false;
    }else{
        return true;
    }
}