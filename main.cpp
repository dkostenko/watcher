/* 
 * File:   main.cpp
 * Author: macbook
 *
 * Created on 10 Апрель 2013 г., 10:29
 */

#include <opencv2/opencv.hpp>
#include <sstream>
#include <iostream>
#include <stdio.h>
#include "Constants.h"
#include "helper.h"
#include "Watcher.h"


using namespace cv;
using namespace std;

Rect rect;
bool isClicked = false;
bool noRect = true;
bool flag = false;


void mouseClickHandler(int event, int x, int y, int flags, void *param){
    switch(event){
        case CV_EVENT_MOUSEMOVE:
            if(isClicked){
                rect.width = x - rect.x;
                rect.height = y - rect.y;
            }
            break;
            
        case CV_EVENT_LBUTTONDOWN:
            isClicked = true;
            rect = Rect(x, y, 0, 0);
            break;
            
        case CV_EVENT_LBUTTONUP:
            isClicked = false;
            if(rect.width < 0){
                rect.x += rect.width;
                rect.width *= -1;
            }
            if(rect.height < 0){
                rect.y += rect.height;
                rect.height *= -1;
            }
            noRect = false;
            break;
    }
}


int main(int argc, char** argv) {
    VideoCapture capture;
    Mat currentFrame;
    Mat lastGrayFrame;
    Mat currentGrayFrame;
    BoundingBox pbox;
    vector<Point2f> points1;
    vector<Point2f> points2;
    bool isDetected = true;
    
    capture.open(0);
    
    if(!capture.isOpened()){
        // вебка недоступна, завершаем программу
        return 1;
    }

    cvNamedWindow("VKR Kostenko 472SE", CV_WINDOW_AUTOSIZE);
    cvSetMouseCallback("VKR Kostenko 472SE", mouseClickHandler, NULL);

    capture.set(CV_CAP_PROP_FRAME_WIDTH, 320);
    capture.set(CV_CAP_PROP_FRAME_HEIGHT, 240);

    
    while(true){
        while(noRect)
        {
            capture >> currentFrame;
            cvtColor(currentFrame, lastGrayFrame, CV_RGB2GRAY);

            drawBox(currentFrame, rect);

            // показывваем новый кадр
            imshow("VKR Kostenko 472SE", currentFrame);

            if(cvWaitKey(33)=='q'){
                return 0;
            }
        }

        if(rect.width > Constants::min_win && rect.height > Constants::min_win){
            // обвел квадрат подходящего размера
            // удаляем обработчик мышки
            cvSetMouseCallback("VKR Kostenko 472SE", NULL, NULL );
            break;
        }else{
            // необходимо выделить новый квадрат
            noRect = true;
        }
    }
    

    Watcher watcher(lastGrayFrame, rect);

    while(capture.read(currentFrame)){
        cvtColor(currentFrame, currentGrayFrame, CV_RGB2GRAY);
        
        // ищем совпадение
        watcher.processFrame(lastGrayFrame, currentGrayFrame, points1,points2, pbox, isDetected, flag);
        
        // рисуем квадрат, если нашли совпадение
        if(isDetected){
            drawBox(currentFrame, pbox);
        }
        
        // показывваем новый кадр
        imshow("VKR Kostenko 472SE", currentFrame);
        
        swap(lastGrayFrame, currentGrayFrame);
        points1.clear();
        points2.clear();
        
        if(cvWaitKey(33)=='q'){
            break;
        }
    }
    
    return 0;
}