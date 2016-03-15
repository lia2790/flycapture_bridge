
#include "cvdrawingutils.h"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace cv;
namespace mid {
/****
 *
 *
 *
 ****/
void CvDrawingUtils::draw3dAxis(cv::Mat &Image, Marker &m, const CameraParameters &CP) {

    float size = m.ssize * 3;
    Mat objectPoints(4, 3, CV_32FC1);
    objectPoints.at< float >(0, 0) = 0;
    objectPoints.at< float >(0, 1) = 0;
    objectPoints.at< float >(0, 2) = 0;
    objectPoints.at< float >(1, 0) = size;
    objectPoints.at< float >(1, 1) = 0;
    objectPoints.at< float >(1, 2) = 0;
    objectPoints.at< float >(2, 0) = 0;
    objectPoints.at< float >(2, 1) = size;
    objectPoints.at< float >(2, 2) = 0;
    objectPoints.at< float >(3, 0) = 0;
    objectPoints.at< float >(3, 1) = 0;
    objectPoints.at< float >(3, 2) = size;

    vector< Point2f > imagePoints;
    cv::projectPoints(objectPoints, m.Rvec, m.Tvec, CP.CameraMatrix, CP.Distorsion, imagePoints);
    // draw lines of different colours
    cv::line(Image, imagePoints[0], imagePoints[1], Scalar(0, 0, 255, 255), 1, CV_AA);
    cv::line(Image, imagePoints[0], imagePoints[2], Scalar(0, 255, 0, 255), 1, CV_AA);
    cv::line(Image, imagePoints[0], imagePoints[3], Scalar(255, 0, 0, 255), 1, CV_AA);
    putText(Image, "x", imagePoints[1], FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 0, 255, 255), 2);
    putText(Image, "y", imagePoints[2], FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 255, 0, 255), 2);
    putText(Image, "z", imagePoints[3], FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255, 0, 0, 255), 2);
}

/****
 *
 *
 *
 ****/
void CvDrawingUtils::draw3dCube(cv::Mat &Image, Marker &m, const CameraParameters &CP, bool setYperpendicular) {
    Mat objectPoints(8, 3, CV_32FC1);
    double halfSize = m.ssize / 2;

    if (setYperpendicular) {
        objectPoints.at< float >(0, 0) = -halfSize;
        objectPoints.at< float >(0, 1) = 0;
        objectPoints.at< float >(0, 2) = -halfSize;
        objectPoints.at< float >(1, 0) = halfSize;
        objectPoints.at< float >(1, 1) = 0;
        objectPoints.at< float >(1, 2) = -halfSize;
        objectPoints.at< float >(2, 0) = halfSize;
        objectPoints.at< float >(2, 1) = 0;
        objectPoints.at< float >(2, 2) = halfSize;
        objectPoints.at< float >(3, 0) = -halfSize;
        objectPoints.at< float >(3, 1) = 0;
        objectPoints.at< float >(3, 2) = halfSize;

        objectPoints.at< float >(4, 0) = -halfSize;
        objectPoints.at< float >(4, 1) = m.ssize;
        objectPoints.at< float >(4, 2) = -halfSize;
        objectPoints.at< float >(5, 0) = halfSize;
        objectPoints.at< float >(5, 1) = m.ssize;
        objectPoints.at< float >(5, 2) = -halfSize;
        objectPoints.at< float >(6, 0) = halfSize;
        objectPoints.at< float >(6, 1) = m.ssize;
        objectPoints.at< float >(6, 2) = halfSize;
        objectPoints.at< float >(7, 0) = -halfSize;
        objectPoints.at< float >(7, 1) = m.ssize;
        objectPoints.at< float >(7, 2) = halfSize;
    } else {
        objectPoints.at< float >(0, 0) = -halfSize;
        objectPoints.at< float >(0, 1) = -halfSize;
        objectPoints.at< float >(0, 2) = 0;
        objectPoints.at< float >(1, 0) = halfSize;
        objectPoints.at< float >(1, 1) = -halfSize;
        objectPoints.at< float >(1, 2) = 0;
        objectPoints.at< float >(2, 0) = halfSize;
        objectPoints.at< float >(2, 1) = halfSize;
        objectPoints.at< float >(2, 2) = 0;
        objectPoints.at< float >(3, 0) = -halfSize;
        objectPoints.at< float >(3, 1) = halfSize;
        objectPoints.at< float >(3, 2) = 0;

        objectPoints.at< float >(4, 0) = -halfSize;
        objectPoints.at< float >(4, 1) = -halfSize;
        objectPoints.at< float >(4, 2) = m.ssize;
        objectPoints.at< float >(5, 0) = halfSize;
        objectPoints.at< float >(5, 1) = -halfSize;
        objectPoints.at< float >(5, 2) = m.ssize;
        objectPoints.at< float >(6, 0) = halfSize;
        objectPoints.at< float >(6, 1) = halfSize;
        objectPoints.at< float >(6, 2) = m.ssize;
        objectPoints.at< float >(7, 0) = -halfSize;
        objectPoints.at< float >(7, 1) = halfSize;
        objectPoints.at< float >(7, 2) = m.ssize;
    }

    vector< Point2f > imagePoints;
    projectPoints(objectPoints, m.Rvec, m.Tvec, CP.CameraMatrix, CP.Distorsion, imagePoints);
    // draw lines of different colours
    for (int i = 0; i < 4; i++)
        cv::line(Image, imagePoints[i], imagePoints[(i + 1) % 4], Scalar(0, 0, 255, 255), 1, CV_AA);

    for (int i = 0; i < 4; i++)
        cv::line(Image, imagePoints[i + 4], imagePoints[4 + (i + 1) % 4], Scalar(0, 0, 255, 255), 1, CV_AA);

    for (int i = 0; i < 4; i++)
        cv::line(Image, imagePoints[i], imagePoints[i + 4], Scalar(0, 0, 255, 255), 1, CV_AA);
}




}
