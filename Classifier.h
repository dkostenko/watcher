/* 
 * File:   Classifier.h
 * Author: macbook
 *
 * Created on 15 Апрель 2013 г., 15:03
 */

#ifndef CLASSIFIER_H
#define	CLASSIFIER_H

#include <opencv2/opencv.hpp>
#include <stdio.h>

class Classifier {
public:
    Classifier();
    virtual ~Classifier();

    float thr_nn_valid;
    
    void prepare(const std::vector<cv::Size>& scales);
    void getFeatures(const cv::Mat& image,const int& scale_idx,std::vector<int>& fern);
    void update(const std::vector<int>& fern, int C, int N);
    float measure_forest(std::vector<int> fern);
    void trainF(const std::vector<std::pair<std::vector<int>,int> >& ferns,int resample);
    void trainNN(const std::vector<cv::Mat>& nn_examples);
    void NNConf(const cv::Mat& example,std::vector<int>& isin,float& rsconf,float& csconf);
    void evaluateTh(const std::vector<std::pair<std::vector<int>,int> >& nXT,const std::vector<cv::Mat>& nExT);

    int getNumStructs(){return nstructs;}
    float getFernTh(){return thr_fern;}
    float getNNTh(){return thr_nn;}
    struct Feature
        {
            uchar x1, y1, x2, y2;
            Feature() : x1(0), y1(0), x2(0), y2(0) {}
            Feature(int _x1, int _y1, int _x2, int _y2)
            : x1((uchar)_x1), y1((uchar)_y1), x2((uchar)_x2), y2((uchar)_y2)
            {}
            bool operator ()(const cv::Mat& patch) const
            { return patch.at<uchar>(y1,x1) > patch.at<uchar>(y2, x2); }
        };
    std::vector<std::vector<Feature> > features;
    std::vector< std::vector<int> > nCounter;
    std::vector< std::vector<int> > pCounter;
    std::vector< std::vector<float> > posteriors;
    float thrN; // меньший порог
    float thrP;  // больший порог

    std::vector<cv::Mat> pEx;
    std::vector<cv::Mat> nEx;
    
private:
    float thr_fern;
    int structSize;
    int nstructs;
    float valid;
    float ncc_thesame;
    float thr_nn;
    int acum;
};

#endif	/* CLASSIFIER_H */

