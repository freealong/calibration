//
// Created by yongqi on 17-5-3.
//

#ifndef CALIBRATION_COMMON_H
#define CALIBRATION_COMMON_H

#include <iostream>
#include <fstream>

using namespace std;
//#include <eigen3/Eigen/Eigen>
//using namespace Eigen;

//void EigenMatrix2VPMatrix(const Matrix4d &tf1, vpHomogeneousMatrix &tf2) {
//  for (int i = 0; i < 4; ++i) {
//    for (int j = 0; j < 4; ++j) {
//      tf2[i][j] = tf1(i, j);
//    }
//  }
//}
//
//void VPMatrix2EigenMatrix(const vpHomogeneousMatrix &tf1, Matrix4d &tf2) {
//  for (int i = 0; i < 4; ++i) {
//    for (int j = 0; j < 4; ++j) {
//      tf2(i, j) = tf1[i][j];
//    }
//  }
//}

vpRotationMatrix RPY2Rotation(double r, double p, double y) {
  vpRotationMatrix R, P, Y;
  R.eye();
  R[0][0] = cos(r);
  R[0][1] = -sin(r);
  R[1][0] = sin(r);
  R[1][1] = cos(r);
  P.eye();
  P[0][0] = cos(p);
  P[0][2] = sin(p);
  P[2][0] = -sin(p);
  P[2][2] = cos(p);
  Y.eye();
  Y[1][1] = cos(y);
  Y[1][2] = -sin(y);
  Y[2][1] = sin(y);
  Y[2][2] = cos(y);
  return R * P * Y;
}

inline void print_matrix(const string &msg, const vpHomogeneousMatrix &m) {
  cout << msg << endl;
  cout << m;
  cout << endl;
}

bool read_matrix(const string &filename, vector<vpHomogeneousMatrix> &vec_m) {
  ifstream fd(filename);
  if (!fd.is_open()) {
    cout << "open " << filename << " failed!!!" << endl;
    return  false;
  }
  vec_m.clear();
  int type, num;
  fd >> type >> num;
  if (type == 0) { // 4*4 matirx
    for (int i = 0; i < num; ++i) {
      vector<double> buffer(16);
      for (int j = 0; j < 16; ++j) {
        fd >> buffer[j];
      }
      vpHomogeneousMatrix vp_m;
      vp_m.buildFrom(buffer);
      vec_m.push_back(vp_m);
    }
  }
  else if (type == 1) { // 1*6 x y z r p y
    for (int i = 0; i < num; ++i) {
      vector<double> buffer(6);
      for (int j = 0; j < 6; ++j) {
        fd >> buffer[j];
      }
      vpHomogeneousMatrix vp_m;
      vp_m.buildFrom({buffer[0], buffer[1], buffer[2]}, RPY2Rotation(buffer[3]/180*M_PI, buffer[4]/180*M_PI, buffer[5]/180*M_PI));
      vec_m.push_back(vp_m);
    }
  }
  fd.close();
  return true;
}

#endif //CALIBRATION_COMMON_H
