/* 
 * File:   Classifier.cpp
 * Author: macbook
 * 
 * Created on 15 Апрель 2013 г., 15:03
 */

#include "Classifier.h"
#include "Constants.h"

using namespace cv;
using namespace std;

Classifier::Classifier() {
    valid = Constants::valid;
    ncc_thesame = Constants::ncc_thesame;
    nstructs = Constants::num_trees;
    structSize = Constants::num_features;
    thr_fern = Constants::thr_fern;
    thr_nn = Constants::thr_nn;
    thr_nn_valid = Constants::thr_nn_valid;
}

Classifier::~Classifier() {
}

void Classifier::prepare(const vector<Size>& scales){
  acum = 0;
  
  int totalFeatures = nstructs*structSize;
  features = vector<vector<Feature> >(scales.size(),vector<Feature> (totalFeatures));
  RNG& rng = theRNG();
  float x1f,x2f,y1f,y2f;
  int x1, x2, y1, y2;
  
  for(int i=0; i<totalFeatures; ++i){
      x1f = (float)rng;
      y1f = (float)rng;
      x2f = (float)rng;
      y2f = (float)rng;
      for(int s=0; s<scales.size(); ++s){
          x1 = x1f * scales[s].width;
          y1 = y1f * scales[s].height;
          x2 = x2f * scales[s].width;
          y2 = y2f * scales[s].height;
          features[s][i] = Feature(x1, y1, x2, y2);
      }

  }
  // Порог
  thrN = 0.5*nstructs;

  for(int i=0; i<nstructs; ++i){
      posteriors.push_back(vector<float>(pow(2.0,structSize), 0));
      pCounter.push_back(vector<int>(pow(2.0,structSize), 0));
      nCounter.push_back(vector<int>(pow(2.0,structSize), 0));
  }
}

void Classifier::getFeatures(const cv::Mat& image,const int& scale_idx, vector<int>& fern){
  int leaf;
  for(int i=0; i<nstructs; ++i){
      leaf=0;
      for(int j=0; j<structSize; ++j){
          leaf = (leaf << 1) + features[scale_idx][i * nstructs + j](image);
      }
      fern[i]=leaf;
  }
}

float Classifier::measure_forest(vector<int> fern) {
  float votes = 0;
  
  for(int i = 0; i < nstructs; ++i) {
      votes += posteriors[i][fern[i]];
  }
  
  return votes;
}

void Classifier::update(const vector<int>& fern, int C, int N) {
  int idx;
  
  for(int i = 0; i < nstructs; ++i) {
      idx = fern[i];
      
      (C==1) ? pCounter[i][idx] += N : nCounter[i][idx] += N;
      
      if(pCounter[i][idx]==0){
          posteriors[i][idx] = 0;
      }else{
          posteriors[i][idx] = ((float)(pCounter[i][idx]))/(pCounter[i][idx] + nCounter[i][idx]);
      }
  }
}

void Classifier::trainF(const vector<std::pair<vector<int>,int> >& ferns,int resample){ 
    thrP = thr_fern*nstructs;
    for(int i=0; i<ferns.size(); ++i){
        if(ferns[i].second==1){                          
            if(measure_forest(ferns[i].first)<=thrP){
                update(ferns[i].first,1,1);
            }
        }else{
            if(measure_forest(ferns[i].first) >= thrN){
                update(ferns[i].first,0,1);
            }
        }
    }
}

void Classifier::trainNN(const vector<cv::Mat>& nn_examples){
    float conf,dummy;
    vector<int> y(nn_examples.size(),0);
    y[0]=1;
    vector<int> isin;
    for(int i=0; i<nn_examples.size(); ++i){
        NNConf(nn_examples[i],isin,conf,dummy);
        if (y[i]==1 && conf <= thr_nn){
            if (isin[1] < 0){
                pEx = vector<Mat>(1,nn_examples[i]);
                continue; 
            }

            pEx.push_back(nn_examples[i]);
        }
        if(y[i]==0 && conf>0.5){
            nEx.push_back(nn_examples[i]);
        }

    }
    ++acum;
}                                                                 


void Classifier::NNConf(const Mat& example, vector<int>& isin,float& rsconf,float& csconf){
    isin=vector<int>(3,-1);
    
    if (pEx.empty()){
        rsconf = 0;
        csconf=0;
        return;
    }
    
    if (nEx.empty()){
        rsconf = 1;
        csconf=1;
        return;
    }
    
    Mat ncc(1,1,CV_32F);
    float nccP,csmaxP,maxP=0;
    bool anyP=false;
    int maxPidx,validatedPart = ceil(pEx.size()*valid);
    float nccN, maxN=0;
    bool anyN=false;
    for (int i=0; i<pEx.size(); ++i){
        matchTemplate(pEx[i],example,ncc,CV_TM_CCORR_NORMED);
        nccP=(((float*)ncc.data)[0]+1)*0.5;
        
        if (nccP>ncc_thesame){
          anyP=true;
        }
        
        if(nccP > maxP){
            maxP=nccP;
            maxPidx = i;
            if(i<validatedPart)
              csmaxP=maxP;
        }
    }
    for(int i=0; i<nEx.size(); ++i){
        matchTemplate(nEx[i],example,ncc,CV_TM_CCORR_NORMED);
        nccN=(((float*)ncc.data)[0]+1)*0.5;
        if (nccN>ncc_thesame)
          anyN=true;
        if(nccN > maxN)
          maxN=nccN;
    }

    if(anyP){
        isin[0]=1;
    }
    
    isin[1]=maxPidx;
    
    if(anyN){
        isin[2]=1;
    }

    float dN=1-maxN;
    float dP=1-maxP;
    rsconf = (float)dN/(dN+dP);

    dP = 1 - csmaxP;
    csconf =(float)dN / (dN + dP);
}

void Classifier::evaluateTh(const vector<pair<vector<int>,int> >& nXT,const vector<cv::Mat>& nExT){
    float fconf;
    for (int i=0; i<nXT.size(); ++i){
        fconf = (float) measure_forest(nXT[i].first)/nstructs;
        if (fconf>thr_fern){
            thr_fern=fconf;
        }
    }
    
    vector <int> isin;
    float conf,dummy;
    for(int i=0; i<nExT.size(); ++i){
        NNConf(nExT[i],isin,conf,dummy);
        if(conf>thr_nn){
            thr_nn=conf;
        }
    }
    
    if(thr_nn>thr_nn_valid){
        thr_nn_valid = thr_nn;
    }
}