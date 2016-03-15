
#ifndef _CameraParameters_H
#define _CameraParameters_H
#include <opencv2/core/core.hpp>
#include <string>
using namespace std;
namespace mid {
/**\brief Parameters of the camera
 */

class CameraParameters {
  public:
    // 3x3 matrix (fx 0 cx, 0 fy cy, 0 0 1)
    cv::Mat CameraMatrix;
    // 4x1 matrix (k1,k2,p1,p2)
    cv::Mat Distorsion;
    // size of the image
    cv::Size CamSize;

    /**Empty constructor
     */
    CameraParameters();
    /**Creates the object from the info passed
     * @param cameraMatrix 3x3 matrix (fx 0 cx, 0 fy cy, 0 0 1)
     * @param distorsionCoeff 4x1 matrix (k1,k2,p1,p2)
     * @param size image size
     */
    CameraParameters(cv::Mat cameraMatrix, cv::Mat distorsionCoeff, cv::Size size) throw(cv::Exception);
    /**Sets the parameters
     * @param cameraMatrix 3x3 matrix (fx 0 cx, 0 fy cy, 0 0 1)
     * @param distorsionCoeff 4x1 matrix (k1,k2,p1,p2)
     * @param size image size
     */
    void setParams(cv::Mat cameraMatrix, cv::Mat distorsionCoeff, cv::Size size) throw(cv::Exception);
    /**Copy constructor
     */
    CameraParameters(const CameraParameters &CI);

    /**Indicates whether this object is valid
     */
    bool isValid() const {
        return CameraMatrix.rows != 0 && CameraMatrix.cols != 0 && Distorsion.rows != 0 && Distorsion.cols != 0 && CamSize.width != -1 && CamSize.height != -1;
    }
    /**Assign operator
    */
    CameraParameters &operator=(const CameraParameters &CI);
    /**Reads the camera parameters from a file generated using saveToFile.
     */
    void readFromFile(string path) throw(cv::Exception);
    /**Saves this to a file
     */
    void saveToFile(string path, bool inXML = true) throw(cv::Exception);

    /**Reads from a YAML file generated with the opencv2.2 calibration utility
     */
    void readFromXMLFile(string filePath) throw(cv::Exception);

    /**Adjust the parameters to the size of the image indicated
     */
    void resize(cv::Size size) throw(cv::Exception);

    /**Returns the location of the camera in the reference system given by the rotation and translation vectors passed
     * NOT TESTED
    */
    static cv::Point3f getCameraLocation(cv::Mat Rvec, cv::Mat Tvec);

   

    /**Returns the 4x4 homogeneous transform matrix from the R and T vectors computed
     */
    static cv::Mat getRTMatrix(const cv::Mat &R_, const cv::Mat &T_, int forceType);


};
}
#endif
