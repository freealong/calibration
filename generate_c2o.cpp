//
// Created by yongqi on 17-5-3.
//

#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco/charuco.hpp>

using namespace std;
using namespace cv;

static bool readCameraParameters(string filename, Mat &camMatrix, Mat &distCoeffs) {
  FileStorage fs(filename, FileStorage::READ);
  if(!fs.isOpened())
    return false;
  fs["camera_matrix"] >> camMatrix;
  fs["distortion_coefficients"] >> distCoeffs;
  return true;
}

int main(int argc, char *argv[]) {
  int squaresX = 5;
  int squaresY = 7;
  float squareLength = 0.0337;
  float markerLength = 0.025;

  Mat pose_tf(4, 4, CV_64F);
  Mat camMatrix, distCoeffs;
  bool readOk = readCameraParameters("calibrationPara.yml", camMatrix, distCoeffs);
  if(!readOk) {
    cerr << "Invalid camera file" << endl;
    return 0;
  }

  Ptr<aruco::DetectorParameters> detectorParams = aruco::DetectorParameters::create();

  Ptr<aruco::Dictionary> dictionary =
      aruco::getPredefinedDictionary(aruco::DICT_6X6_1000);


  float axisLength = 0.5f * ((float)min(squaresX, squaresY) * (squareLength));

  // create charuco board object
  Ptr<aruco::CharucoBoard> charucoboard =
      aruco::CharucoBoard::create(squaresX, squaresY, squareLength, markerLength, dictionary);
  Ptr<aruco::Board> board = charucoboard.staticCast<aruco::Board>();

  size_t N = 27;
  ofstream cMo_file("cMo_save.txt");
  for (int i = 0; i < N; ++i) {
    Mat image, imageCopy;
    image = imread("pic/color" + to_string(i) + ".bmp");

    vector< int > markerIds, charucoIds;
    vector< vector< Point2f > > markerCorners, rejectedMarkers;
    vector< Point2f > charucoCorners;
    Vec3d rvec, tvec;

    // detect markers
    aruco::detectMarkers(image, dictionary, markerCorners, markerIds, detectorParams,
                         rejectedMarkers);

    // interpolate charuco corners
    int interpolatedCorners = 0;
    if(markerIds.size() > 0)
      interpolatedCorners =
          aruco::interpolateCornersCharuco(markerCorners, markerIds, image, charucoboard,
                                           charucoCorners, charucoIds, camMatrix, distCoeffs);

    // estimate charuco board pose
    bool validPose = false;
    if(camMatrix.total() != 0)
      validPose = aruco::estimatePoseCharucoBoard(charucoCorners, charucoIds, charucoboard,
                                                  camMatrix, distCoeffs, rvec, tvec);

    // draw results
    image.copyTo(imageCopy);
    if(markerIds.size() > 0) {
      aruco::drawDetectedMarkers(imageCopy, markerCorners);
    }

    if(rejectedMarkers.size() > 0)
      aruco::drawDetectedMarkers(imageCopy, rejectedMarkers, noArray(), Scalar(100, 0, 255));

    if(interpolatedCorners > 0) {
      Scalar color;
      color = Scalar(255, 0, 0);
      aruco::drawDetectedCornersCharuco(imageCopy, charucoCorners, charucoIds, color);
    }

    if(validPose)
      aruco::drawAxis(imageCopy, camMatrix, distCoeffs, rvec, tvec, axisLength);

    // save results
    if(validPose) {
      Mat rmat;
      Rodrigues(rvec, rmat);
      cMo_file << rmat.at<double>(0, 0) << " " << rmat.at<double>(0, 1) << " " << rmat.at<double>(0, 2) << " " << tvec(0) << endl;
      cMo_file << rmat.at<double>(1, 0) << " " << rmat.at<double>(1, 1) << " " << rmat.at<double>(1, 2) << " " << tvec(1) << endl;
      cMo_file << rmat.at<double>(2, 0) << " " << rmat.at<double>(2, 1) << " " << rmat.at<double>(2, 2) << " " << tvec(2) << endl;
      cMo_file << 0 << " " << 0 << " " << 0 << " " << 1 << endl;
    }
    else cout << "can not detect valid pose at image " << i << endl;

    // show results
    imshow("out", imageCopy);
    char key = (char)waitKey(0);
    if(key == 27) break;
  }
  cMo_file.close();

  return 0;
}
