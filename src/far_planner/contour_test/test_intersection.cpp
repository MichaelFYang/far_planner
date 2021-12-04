#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include "intersection.h"

using namespace std;
using namespace cv;


int main() {
    Point2f offset(10.0, 10.0);
    Point2f p1(3.36, -1.96);
    Point2f p2(7.96, 7.96);
    Point2f p3(7.58, -1.652);
    Point2f p4(4.54, 1.452);
    p1 = p1 + offset;
    p2 = p2 + offset;
    p3 = p3 + offset;
    p4 = p4 + offset;

    if (doIntersect(p1, p2, p3, p4)) {
        cout<<"line segment do intersected"<<endl;
    }
}