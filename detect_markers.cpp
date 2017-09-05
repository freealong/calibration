/*
By downloading, copying, installing or using the software you agree to this
license. If you do not agree to this license, do not download, install,
copy or use the software.

                          License Agreement
               For Open Source Computer Vision Library
                       (3-clause BSD License)

Copyright (C) 2013, OpenCV Foundation, all rights reserved.
Third party copyrights are property of their respective owners.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

  * Redistributions of source code must retain the above copyright notice,
    this list of conditions and the following disclaimer.

  * Redistributions in binary form must reproduce the above copyright notice,
    this list of conditions and the following disclaimer in the documentation
    and/or other materials provided with the distribution.

  * Neither the names of the copyright holders nor the names of the contributors
    may be used to endorse or promote products derived from this software
    without specific prior written permission.

This software is provided by the copyright holders and contributors "as is" and
any express or implied warranties, including, but not limited to, the implied
warranties of merchantability and fitness for a particular purpose are
disclaimed. In no event shall copyright holders or contributors be liable for
any direct, indirect, incidental, special, exemplary, or consequential damages
(including, but not limited to, procurement of substitute goods or services;
loss of use, data, or profits; or business interruption) however caused
and on any theory of liability, whether in contract, strict liability,
or tort (including negligence or otherwise) arising in any way out of
the use of this software, even if advised of the possibility of such damage.
*/


#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <iostream>
#include <eigen3/Eigen/Eigen>
#include <fstream>
#include <opencv/cv.hpp>

using namespace std;
using namespace cv;

namespace {
const char *about = "Basic marker detection";
const char *keys =
    "{d        |       | dictionary: DICT_4X4_50=0, DICT_4X4_100=1, DICT_4X4_250=2,"
        "DICT_4X4_1000=3, DICT_5X5_50=4, DICT_5X5_100=5, DICT_5X5_250=6, DICT_5X5_1000=7, "
        "DICT_6X6_50=8, DICT_6X6_100=9, DICT_6X6_250=10, DICT_6X6_1000=11, DICT_7X7_50=12,"
        "DICT_7X7_100=13, DICT_7X7_250=14, DICT_7X7_1000=15, DICT_ARUCO_ORIGINAL = 16}"
        "{v        |       | Input from video file, if ommited, input comes from camera }"
        "{ci       | 0     | Camera id if input doesnt come from video (-v) }"
        "{c        |       | Camera intrinsic parameters. Needed for camera pose }"
        "{l        | 0.1   | Marker side lenght (in meters). Needed for correct scale in camera pose }"
        "{dp       |       | File of marker detector parameters }"
        "{r        |       | show rejected candidates too }";
}

// rpy in ZYX
void getEulerAngles(const Eigen::Matrix4f t, float &x, float &y, float &z, float &roll, float &pitch, float &yaw) {
  x = t(0, 3);
  y = t(1, 3);
  z = t(2, 3);
  yaw = atan2(t(2, 1), t(2, 2));
  pitch = asin(-t(2, 0));
  roll = atan2(t(1, 0), t(0, 0));
}

/**
 */
static bool readCameraParameters(string filename, Mat &camMatrix, Mat &distCoeffs) {
  FileStorage fs(filename, FileStorage::READ);
  if (!fs.isOpened())
    return false;
  fs["r_intrin"] >> camMatrix;
  fs["r_coeffs"] >> distCoeffs;
  return true;
}

/**
 */
static bool readDetectorParameters(string filename, Ptr<aruco::DetectorParameters> &params) {
  FileStorage fs(filename, FileStorage::READ);
  if (!fs.isOpened())
    return false;
  fs["adaptiveThreshWinSizeMin"] >> params->adaptiveThreshWinSizeMin;
  fs["adaptiveThreshWinSizeMax"] >> params->adaptiveThreshWinSizeMax;
  fs["adaptiveThreshWinSizeStep"] >> params->adaptiveThreshWinSizeStep;
  fs["adaptiveThreshConstant"] >> params->adaptiveThreshConstant;
  fs["minMarkerPerimeterRate"] >> params->minMarkerPerimeterRate;
  fs["maxMarkerPerimeterRate"] >> params->maxMarkerPerimeterRate;
  fs["polygonalApproxAccuracyRate"] >> params->polygonalApproxAccuracyRate;
  fs["minCornerDistanceRate"] >> params->minCornerDistanceRate;
  fs["minDistanceToBorder"] >> params->minDistanceToBorder;
  fs["minMarkerDistanceRate"] >> params->minMarkerDistanceRate;
  fs["doCornerRefinement"] >> params->doCornerRefinement;
  fs["cornerRefinementWinSize"] >> params->cornerRefinementWinSize;
  fs["cornerRefinementMaxIterations"] >> params->cornerRefinementMaxIterations;
  fs["cornerRefinementMinAccuracy"] >> params->cornerRefinementMinAccuracy;
  fs["markerBorderBits"] >> params->markerBorderBits;
  fs["perspectiveRemovePixelPerCell"] >> params->perspectiveRemovePixelPerCell;
  fs["perspectiveRemoveIgnoredMarginPerCell"] >> params->perspectiveRemoveIgnoredMarginPerCell;
  fs["maxErroneousBitsInBorderRate"] >> params->maxErroneousBitsInBorderRate;
  fs["minOtsuStdDev"] >> params->minOtsuStdDev;
  fs["errorCorrectionRate"] >> params->errorCorrectionRate;
  return true;
}

/**
 */
int main(int argc, char *argv[]) {
  ofstream cMo_file("cMo_save.txt");
  if (!cMo_file.is_open()) {
    cout << "open save file failed" << endl;
    return -1;
  }
  ifstream wMc_file("wMc.txt");
  if (!wMc_file.is_open()) {
    cout << "open wMc file failed" << endl;
    return -1;
  }
  Eigen::Matrix4f wMc;
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      float val;
      wMc_file >> val;
      wMc(i, j) = val;
    }
  }
  wMc_file.close();
  // mm to m if necessary
  if (abs(wMc(0, 3)) > 1 || abs(wMc(1,3)) > 1 || abs(wMc(2, 3)) > 1) {
    wMc(0, 3) /= 1000;
    wMc(1, 3) /= 1000;
    wMc(2, 3) /= 1000;
  }
  cout << "wMc:\n" << wMc << endl;

  CommandLineParser parser(argc, argv, keys);
  parser.about(about);

  if (argc < 2) {
    parser.printMessage();
    return 0;
  }

  int dictionaryId = parser.get<int>("d");
  bool showRejected = parser.has("r");
  bool estimatePose = parser.has("c");
  float markerLength = parser.get<float>("l");

  Ptr<aruco::DetectorParameters> detectorParams = aruco::DetectorParameters::create();
  if (parser.has("dp")) {
    bool readOk = readDetectorParameters(parser.get<string>("dp"), detectorParams);
    if (!readOk) {
      cerr << "Invalid detector parameters file" << endl;
      return 0;
    }
  }
  detectorParams->doCornerRefinement = true; // do corner refinement in markers

  int camId = parser.get<int>("ci");

  String video;
  if (parser.has("v")) {
    video = parser.get<String>("v");
  }

  if (!parser.check()) {
    parser.printErrors();
    return 0;
  }

  Ptr<aruco::Dictionary> dictionary =
      aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME(dictionaryId));

  Mat camMatrix, distCoeffs;
  if (estimatePose) {
    bool readOk = readCameraParameters(parser.get<string>("c"), camMatrix, distCoeffs);
    if (!readOk) {
      cerr << "Invalid camera file" << endl;
      return 0;
    }
  }

  VideoCapture inputVideo;
  int waitTime;
  if (!video.empty()) {
    inputVideo.open(video);
    waitTime = 0;
  } else {
    inputVideo.open(camId);
    waitTime = 10;
    for (int i = 0; i < 30; ++i) {
      inputVideo.grab();
    }
  }

  double totalTime = 0;
  int totalIterations = 0;

  cout << "begin to loop" << endl;
  while (inputVideo.grab()) {
    Mat image, imageCopy;
    inputVideo.retrieve(image);

    double tick = (double) getTickCount();

    vector<int> ids;
    vector<vector<Point2f> > corners, rejected;
    vector<Vec3d> rvecs, tvecs;

    // detect markers and estimate pose
    aruco::detectMarkers(image, dictionary, corners, ids, detectorParams, rejected);
    if (estimatePose && ids.size() > 0)
      aruco::estimatePoseSingleMarkers(corners, markerLength, camMatrix, distCoeffs, rvecs,
                                       tvecs);

//    double currentTime = ((double) getTickCount() - tick) / getTickFrequency();
//    totalTime += currentTime;
//    totalIterations++;
//    if (totalIterations % 30 == 0) {
//      cout << "Detection Time = " << currentTime * 1000 << " ms "
//           << "(Mean = " << 1000 * totalTime / double(totalIterations) << " ms)" << endl;
//    }

    // draw results
    image.copyTo(imageCopy);
    if (ids.size() > 0) {
      aruco::drawDetectedMarkers(imageCopy, corners, ids);

      if (estimatePose) {
        for (unsigned int i = 0; i < ids.size(); i++) {
          aruco::drawAxis(imageCopy, camMatrix, distCoeffs, rvecs[i], tvecs[i],
                          markerLength * 0.5f);
	}
      }
    }

    if (showRejected && rejected.size() > 0)
      aruco::drawDetectedMarkers(imageCopy, rejected, noArray(), Scalar(100, 0, 255));

    imshow("out", imageCopy);
    char key = (char) waitKey(waitTime);
    if (key == 27) break;

    // save tf to file
    bool validPose;
    Vec3d rvec, tvec;
    if (ids.size() == 1) {
      rvec = rvecs[0];
      tvec = tvecs[0];
      validPose = true;
    }
    else {
      validPose = false;
    }
    Mat rmat;
    Rodrigues(rvec, rmat);
    if (validPose && key == 115) {
      cMo_file << rmat.at<double>(0, 0) << " " << rmat.at<double>(0, 1) << " " << rmat.at<double>(0, 2) << " "
               << tvec(0) * 1000 << endl;
      cMo_file << rmat.at<double>(1, 0) << " " << rmat.at<double>(1, 1) << " " << rmat.at<double>(1, 2) << " "
               << tvec(1) * 1000 << endl;
      cMo_file << rmat.at<double>(2, 0) << " " << rmat.at<double>(2, 1) << " " << rmat.at<double>(2, 2) << " "
               << tvec(2) * 1000 << endl;
      cMo_file << 0 << " " << 0 << " " << 0 << " " << 1 << endl;
      cout << rmat << endl;
      cout << tvec << endl;
      cout << "save tf ok" << endl;
    }

    // calculate pose based on wMc
    if (validPose && key == 116) {
      Eigen::Matrix4f cMo = Eigen::Matrix4f::Identity();
      for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
          cMo(i, j) = rmat.at<double>(i, j);
        }
      }
      cMo(0, 3) = tvec(0);
      cMo(1, 3) = tvec(1);
      cMo(2, 3) = tvec(2);
      cout << "cMo:" << cMo << endl;
      Eigen::Matrix4f wMo = wMc * cMo;
      cout << "wMo:" << wMo << endl;
      vector<float> pose(6, 0);
      getEulerAngles(wMo, pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]);
      cout << "target pose: X: " << pose[0] * 1000 << "mm Y: " << pose[1] * 1000 << "mm Z: " << pose[2] * 1000 << "mm r: "
           << pose[3] * 180 / M_PI << "° p: " << pose[4] * 180 / M_PI << "° y: " << pose[5] * 180 / M_PI << "°" << endl;
    }
  }

  return 0;
}
