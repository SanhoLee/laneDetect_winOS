#include <iostream>
#include "common.hpp"
#include "CV_header.hpp"
#include "struct_drawData.h"
#include "preprocImg.hpp"
#include "calcLaneImg.hpp"
#include "drawOnWarpImg.hpp"


/* 
    only runnable in Release mode.
    ref for videocapture setting : https://junstar92.tistory.com/401
*/

int main(int argc, char **argv)
{

    Mat img, imgBinary, imgShow;
    Mat Matx, invMatx;
    drawDataInfo drawDataSet;    
    String videoName = "img/project_video.mp4";
    //String videoName = "img/challenge_video.mp4";

    VideoCapture cap(videoName);
    if (!cap.isOpened()) {
        cerr << "Video open FAILED... \n";
        return -1;
    }

    // get FPS numbers of the video
    // calculate delay(ms) considering fps number.
    double fps = cap.get(CAP_PROP_FPS);
    int delay = cvRound(1000 / fps);

    cout << "Frame counts : " << fps << endl;
    cout << "delay time : " << delay << endl;

    int rtn = 0;
    Mat frame;

    while (true) {
        cap >> frame;
        if (frame.empty())
            break;

        imgBinary = preprocImg(frame, &Matx, &invMatx);
        drawDataSet = calcImg(imgBinary);
        imgShow = drawAll(frame, imgBinary, invMatx, drawDataSet);


        imshow("MOVIE FRAME", imgShow);
        rtn = waitKey(delay);
        if (rtn == 27) {
            break;
            frame.release();
            cap.release();
        }
    }

    destroyAllWindows();
    return 0;
}