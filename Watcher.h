/* 
 * File:   Watcher.h
 * Author: macbook
 *
 * Created on 10 Апрель 2013 г., 11:49
 */

#ifndef WATCHER_H
#define	WATCHER_H

#include <opencv2/opencv.hpp>
#include <opencv2/legacy/legacy.hpp>
#include "helper.h"
#include "LucasKanade.h"
#include "Classifier.h"
#include <fstream>



struct BoundingBox : public cv::Rect {
    BoundingBox(){}
    BoundingBox(cv::Rect r): cv::Rect(r){}
public:
    float overlap;
    int sidx;
};


struct DetStruct{
    std::vector<int> bb;
    std::vector<std::vector<int> > patt;
    std::vector<float> conf1;
    std::vector<float> conf2;
    std::vector<std::vector<int> > isin;
    std::vector<cv::Mat> patch;
};


struct TempStruct {
    std::vector<std::vector<int> > patt;
    std::vector<float> conf;
};

struct OComparator{
    OComparator(const std::vector<BoundingBox>& _grid):grid(_grid){}
    std::vector<BoundingBox> grid;
    bool operator()(int idx1,int idx2){
        return grid[idx1].overlap > grid[idx2].overlap;
    }
};

struct CComparator{
    CComparator(const std::vector<float>& _conf):conf(_conf){}
    std::vector<float> conf;
    bool operator()(int idx1,int idx2){
        return conf[idx1]> conf[idx2];
    }
};



class Watcher {
public:
    Watcher();
    Watcher(const cv::Mat& frame1,const cv::Rect &box);
    virtual ~Watcher();
    
    void generatePositiveData(const cv::Mat& frame, int num_warps);
    void generateNegativeData(const cv::Mat& frame);
    void processFrame(const cv::Mat& img1,const cv::Mat& img2,std::vector<cv::Point2f>& points1,std::vector<cv::Point2f>& points2, BoundingBox& bbnext,bool& lastboxfound, bool tl);
    void track(const cv::Mat& img1, const cv::Mat& img2,std::vector<cv::Point2f>& points1,std::vector<cv::Point2f>& points2);
    void detect(const cv::Mat& frame);
    void clusterConf(const std::vector<BoundingBox>& dbb,const std::vector<float>& dconf,std::vector<BoundingBox>& cbb,std::vector<float>& cconf);
    void learn(const cv::Mat& img);

    void buildGrid(const cv::Mat& img, const cv::Rect& box);
    float bbOverlap(const BoundingBox& box1,const BoundingBox& box2);
    void getOverlappingBoxes(int num_closest);
    void getBBHull();
    void getPattern(const cv::Mat& img, cv::Mat& pattern,cv::Scalar& mean,cv::Scalar& stdev);
    void bbPoints(std::vector<cv::Point2f>& points, const BoundingBox& bb);
    void bbPredict(const std::vector<cv::Point2f>& points1,const std::vector<cv::Point2f>& points2,
        const BoundingBox& bb1,BoundingBox& bb2);
    double getVar(const BoundingBox& box,const cv::Mat& sum,const cv::Mat& sqsum);
    bool bbComp(const BoundingBox& bb1,const BoundingBox& bb2);
    int clusterBB(const std::vector<BoundingBox>& dbb,std::vector<int>& indexes);
  
private:
    cv::PatchGenerator generator;
    Classifier classifier;
    LucasKanade tracker;

    int bbox_step;
    int min_win;
    int patch_size;

    int num_closest_init;
    int num_warps_init;
    int noise_init;
    float angle_init;
    float shift_init;
    float scale_init;

    int num_closest_update;
    int num_warps_update;
    int noise_update;
    float angle_update;
    float shift_update;
    float scale_update;

    float bad_overlap;
    float bad_patches;

    cv::Mat iisum;
    cv::Mat iisqsum;
    float var;

    std::vector<std::pair<std::vector<int>,int> > pX;
    std::vector<std::pair<std::vector<int>,int> > nX;
    cv::Mat pEx;
    std::vector<cv::Mat> nEx;

    
    std::vector<std::pair<std::vector<int>,int> > nXT;
    std::vector<cv::Mat> nExT;

    
    BoundingBox lastbox;
    bool lastvalid;
    float lastconf;

    
    bool tracked;
    BoundingBox tbb;
    bool tvalid;
    float tconf;


    TempStruct tmp;
    DetStruct dt;
    std::vector<BoundingBox> dbb;
    std::vector<bool> dvalid;
    std::vector<float> dconf;
    bool detected;
    
    std::vector<BoundingBox> grid;
    std::vector<cv::Size> scales;
    std::vector<int> good_boxes;
    std::vector<int> bad_boxes;
    BoundingBox bbhull;
    BoundingBox best_box;
};

#endif	/* WATCHER_H */

