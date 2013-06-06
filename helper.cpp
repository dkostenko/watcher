/* 
 * File:   helper.cpp
 * Author: macbook
 * 
 * Created on 20 Апрель 2013 г., 14:17
 */

#include "helper.h"

using namespace cv;
using namespace std;

void drawBox(Mat& image, CvRect box, Scalar color, int thick){
    rectangle( image, cvPoint(box.x, box.y), cvPoint(box.x+box.width,box.y+box.height),color, thick);
} 

float median(vector<float> v)
{
    int n = floor(v.size() / 2);
    nth_element(v.begin(), v.begin()+n, v.end());
    return v[n];
}

vector<int> index_shuffle(int begin,int end){
    vector<int> indexes(end-begin);
    for(int i=begin; i<end; ++i){
        indexes[i]=i;
    }
    random_shuffle(indexes.begin(),indexes.end());
    return indexes;
}


