#include <iostream>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

using namespace std;
using namespace cv;

int main()
{
    int rtn = 0;
    Mat src;

    src = imread("img/road.jpg");
    if (src.empty()) {
        cout << "IMG LOAD FAILED ! \n";
        return -1;
    }

    while (true) {
        imshow("img", src);
        rtn = waitKey(0);
        if (rtn == 27) return 0;
    }
}