
#include "cameraparameters.h"
#include <fstream>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
using namespace std;
namespace mid {


CameraParameters::CameraParameters() {
    CameraMatrix = cv::Mat();
    Distorsion = cv::Mat();
    CamSize = cv::Size(-1, -1);
}
/**Creates the object from the info passed
 * @param cameraMatrix 3x3 matrix (fx 0 cx, 0 fy cy, 0 0 1)
 * @param distorsionCoeff 4x1 matrix (k1,k2,p1,p2)
 * @param size image size
 */
CameraParameters::CameraParameters(cv::Mat cameraMatrix, cv::Mat distorsionCoeff, cv::Size size) throw(cv::Exception) {
    setParams(cameraMatrix, distorsionCoeff, size);
}
/**
 */
CameraParameters::CameraParameters(const CameraParameters &CI) {
    CI.CameraMatrix.copyTo(CameraMatrix);
    CI.Distorsion.copyTo(Distorsion);
    CamSize = CI.CamSize;
}

/**
*/
CameraParameters &CameraParameters::operator=(const CameraParameters &CI) {
    CI.CameraMatrix.copyTo(CameraMatrix);
    CI.Distorsion.copyTo(Distorsion);
    CamSize = CI.CamSize;
    return *this;
}
/**
 */
void CameraParameters::setParams(cv::Mat cameraMatrix, cv::Mat distorsionCoeff, cv::Size size) throw(cv::Exception) {
    if (cameraMatrix.rows != 3 || cameraMatrix.cols != 3)
        throw cv::Exception(9000, "invalid input cameraMatrix", "CameraParameters::setParams", __FILE__, __LINE__);
    cameraMatrix.convertTo(CameraMatrix, CV_32FC1);
    if (distorsionCoeff.total() < 4 || distorsionCoeff.total() >= 7)
        throw cv::Exception(9000, "invalid input distorsionCoeff", "CameraParameters::setParams", __FILE__, __LINE__);
    cv::Mat auxD;

    distorsionCoeff.convertTo(Distorsion, CV_32FC1);

    //     Distorsion.create(1,4,CV_32FC1);
    //     for (int i=0;i<4;i++)
    //         Distorsion.ptr<float>(0)[i]=auxD.ptr<float>(0)[i];

    CamSize = size;
}

/**
*/
cv::Point3f CameraParameters::getCameraLocation(cv::Mat Rvec, cv::Mat Tvec) {
    cv::Mat m33(3, 3, CV_32FC1);
    cv::Rodrigues(Rvec, m33);

    cv::Mat m44 = cv::Mat::eye(4, 4, CV_32FC1);
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            m44.at< float >(i, j) = m33.at< float >(i, j);

    // now, add translation information
    for (int i = 0; i < 3; i++)
        m44.at< float >(i, 3) = Tvec.at< float >(0, i);
    // invert the matrix
    m44.inv();
    return cv::Point3f(m44.at< float >(0, 0), m44.at< float >(0, 1), m44.at< float >(0, 2));
}

/**Reads the camera parameters from file
 */
void CameraParameters::readFromFile(string path) throw(cv::Exception) {

    ifstream file(path.c_str());
    if (!file)
        throw cv::Exception(9005, "could not open file:" + path, "CameraParameters::readFromFile", __FILE__, __LINE__);
    // Create the matrices
    Distorsion = cv::Mat::zeros(4, 1, CV_32FC1);
    CameraMatrix = cv::Mat::eye(3, 3, CV_32FC1);
    char line[1024];
    while (!file.eof()) {
        file.getline(line, 1024);
        char cmd[20];
        float fval;
        if (sscanf(line, "%s = %f", cmd, &fval) == 2) {
            string scmd(cmd);
            if (scmd == "fx")
                CameraMatrix.at< float >(0, 0) = fval;
            else if (scmd == "cx")
                CameraMatrix.at< float >(0, 2) = fval;
            else if (scmd == "fy")
                CameraMatrix.at< float >(1, 1) = fval;
            else if (scmd == "cy")
                CameraMatrix.at< float >(1, 2) = fval;
            else if (scmd == "k1")
                Distorsion.at< float >(0, 0) = fval;
            else if (scmd == "k2")
                Distorsion.at< float >(1, 0) = fval;
            else if (scmd == "p1")
                Distorsion.at< float >(2, 0) = fval;
            else if (scmd == "p2")
                Distorsion.at< float >(3, 0) = fval;
            else if (scmd == "width")
                CamSize.width = fval;
            else if (scmd == "height")
                CamSize.height = fval;
        }
    }
}
/**Saves this to a file
  */
void CameraParameters::saveToFile(string path, bool inXML) throw(cv::Exception) {
    if (!isValid())
        throw cv::Exception(9006, "invalid object", "CameraParameters::saveToFile", __FILE__, __LINE__);
    if (!inXML) {
        ofstream file(path.c_str());
        if (!file)
            throw cv::Exception(9006, "could not open file:" + path, "CameraParameters::saveToFile", __FILE__, __LINE__);
        file << "# Aruco 1.0 CameraParameters" << endl;
        file << "fx = " << CameraMatrix.at< float >(0, 0) << endl;
        file << "cx = " << CameraMatrix.at< float >(0, 2) << endl;
        file << "fy = " << CameraMatrix.at< float >(1, 1) << endl;
        file << "cy = " << CameraMatrix.at< float >(1, 2) << endl;
        file << "k1 = " << Distorsion.at< float >(0, 0) << endl;
        file << "k2 = " << Distorsion.at< float >(1, 0) << endl;
        file << "p1 = " << Distorsion.at< float >(2, 0) << endl;
        file << "p2 = " << Distorsion.at< float >(3, 0) << endl;
        file << "width = " << CamSize.width << endl;
        file << "height = " << CamSize.height << endl;
    } else {
        cv::FileStorage fs(path, cv::FileStorage::WRITE);
        fs << "image_width" << CamSize.width;
        fs << "image_height" << CamSize.height;
        fs << "camera_matrix" << CameraMatrix;
        fs << "distortion_coefficients" << Distorsion;
    }
}

/**Adjust the parameters to the size of the image indicated
 */
void CameraParameters::resize(cv::Size size) throw(cv::Exception) {
    if (!isValid())
        throw cv::Exception(9007, "invalid object", "CameraParameters::resize", __FILE__, __LINE__);
    if (size == CamSize)
        return;
    // now, read the camera size
    // resize the camera parameters to fit this image size
    float AxFactor = float(size.width) / float(CamSize.width);
    float AyFactor = float(size.height) / float(CamSize.height);
    CameraMatrix.at< float >(0, 0) *= AxFactor;
    CameraMatrix.at< float >(0, 2) *= AxFactor;
    CameraMatrix.at< float >(1, 1) *= AyFactor;
    CameraMatrix.at< float >(1, 2) *= AyFactor;
}

/****
 *
 *
 *
 *
 */
void CameraParameters::readFromXMLFile(string filePath) throw(cv::Exception) {
    cv::FileStorage fs(filePath, cv::FileStorage::READ);
    int w = -1, h = -1;
    cv::Mat MCamera, MDist;

    fs["image_width"] >> w;
    fs["image_height"] >> h;
    fs["distortion_coefficients"] >> MDist;
    fs["camera_matrix"] >> MCamera;

    if (MCamera.cols == 0 || MCamera.rows == 0)
        throw cv::Exception(9007, "File :" + filePath + " does not contains valid camera matrix", "CameraParameters::readFromXML", __FILE__, __LINE__);
    if (w == -1 || h == 0)
        throw cv::Exception(9007, "File :" + filePath + " does not contains valid camera dimensions", "CameraParameters::readFromXML", __FILE__, __LINE__);

    if (MCamera.type() != CV_32FC1)
        MCamera.convertTo(CameraMatrix, CV_32FC1);
    else
        CameraMatrix = MCamera;

    if (MDist.total() < 4)
        throw cv::Exception(9007, "File :" + filePath + " does not contains valid distortion_coefficients", "CameraParameters::readFromXML", __FILE__,
                            __LINE__);
    // convert to 32 and get the 4 first elements only
    cv::Mat mdist32;
    MDist.convertTo(mdist32, CV_32FC1);
    //     Distorsion.create(1,4,CV_32FC1);
    //     for (int i=0;i<4;i++)
    //         Distorsion.ptr<float>(0)[i]=mdist32.ptr<float>(0)[i];

    Distorsion.create(1, 5, CV_32FC1);
    for (int i = 0; i < 5; i++)
        Distorsion.ptr< float >(0)[i] = mdist32.ptr< float >(0)[i];
    CamSize.width = w;
    CamSize.height = h;
}


cv::Mat CameraParameters::getRTMatrix(const cv::Mat &R_, const cv::Mat &T_, int forceType) {
    cv::Mat M;
    cv::Mat R, T;
    R_.copyTo(R);
    T_.copyTo(T);
    if (R.type() == CV_64F) {
        assert(T.type() == CV_64F);
        cv::Mat Matrix = cv::Mat::eye(4, 4, CV_64FC1);

        cv::Mat R33 = cv::Mat(Matrix, cv::Rect(0, 0, 3, 3));
        if (R.total() == 3) {
            cv::Rodrigues(R, R33);
        } else if (R.total() == 9) {
            cv::Mat R64;
            R.convertTo(R64, CV_64F);
            R.copyTo(R33);
        }
        for (int i = 0; i < 3; i++)
            Matrix.at< double >(i, 3) = T.ptr< double >(0)[i];
        M = Matrix;
    } else if (R.depth() == CV_32F) {
        cv::Mat Matrix = cv::Mat::eye(4, 4, CV_32FC1);
        cv::Mat R33 = cv::Mat(Matrix, cv::Rect(0, 0, 3, 3));
        if (R.total() == 3) {
            cv::Rodrigues(R, R33);
        } else if (R.total() == 9) {
            cv::Mat R32;
            R.convertTo(R32, CV_32F);
            R.copyTo(R33);
        }

        for (int i = 0; i < 3; i++)
            Matrix.at< float >(i, 3) = T.ptr< float >(0)[i];
        M = Matrix;
    }

    if (forceType == -1)
        return M;
    else {
        cv::Mat MTyped;
        M.convertTo(MTyped, forceType);
        return MTyped;
    }
}
};
