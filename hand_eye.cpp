//
// Created by yongqi on 17-5-2.
//

#include <stdio.h>
#include <sstream>
#include <iomanip>
#include <vector>

#include <visp3/core/vpDebug.h>
#include <visp3/io/vpParseArgv.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/vision/vpCalibration.h>
#include <visp3/core/vpExponentialMap.h>

#include "common.h"

using namespace std;

bool ReadTransformationFromFile(string c2o_file, string w2e_file, vector<vpHomogeneousMatrix> &cMo, vector<vpHomogeneousMatrix> &wMe) {
  ifstream f_c2o, f_w2e;
  f_c2o.open(c2o_file);
  f_w2e.open(w2e_file);
  if (!f_c2o.is_open()) {
    cout << "can't open cMo file" << endl;
    return false;
  }
  if (!f_w2e.is_open()) {
    cout << "can't open wMe file" << endl;
    return false;
  }
  long N = std::count(std::istreambuf_iterator<char>(f_c2o),
             std::istreambuf_iterator<char>(), '\n');
  N = N / 4;
  long M = std::count(std::istreambuf_iterator<char>(f_w2e),
             std::istreambuf_iterator<char>(), '\n');
  M = M / 4;
  if (M != N) {
    cout << "c2o file and w2e file have different number of matrixs" << endl;
    return false;
  }
  else {
    cout << "read " << N << " matrixs from file" << endl;
  }
  f_c2o.seekg(f_c2o.beg);
  f_w2e.seekg(f_w2e.beg);
  cMo.clear();
  wMe.clear();

  vpHomogeneousMatrix vp_tf;
  vector<double> vec(16);
  for (int i = 0; i < N; ++i) {
    for (int j = 0; j < vec.size(); ++j) {
      f_c2o >> vec[j];
    }
    vp_tf.buildFrom(vec);
    cMo.push_back(vp_tf);
    print_matrix("cMo " + to_string(i+1) + ": ", vp_tf);

    for (int j = 0; j < vec.size(); ++j) {
      f_w2e >> vec[j];
    }
    vp_tf.buildFrom(vec);
    wMe.push_back(vp_tf);
    print_matrix("wMe " + to_string(i+1) + ": ", vp_tf);
  }
  f_c2o.close();
  f_w2e.close();

  return true;
}

int main(int argc, char **argv)
{
  if (argc != 2) {
    cout << "usage: hand_eye type_id (type_id = 1, eye in hand; type_id = 2, eye to hand" << endl;
    return -1;
  }
  int type = stoi(argv[1]);
  string out_filename;
  try {
    vpHomogeneousMatrix eMc;
    // Reset the eMc matrix to eye
    eMc.eye();
    // Read cMo, wMe from file
    vector<vpHomogeneousMatrix> cMo, wMe;
    read_matrix("cMo.txt", cMo);
    read_matrix("wMe.txt", wMe);
    for (auto &m : cMo) {
      print_matrix("cMo: " , m);
    }
    for (auto &m : wMe) {
      print_matrix("wMe: " , m);
    }
    // Compute the eMc hand to eye transformation from six poses
    // - cMo[6]: camera to object poses as six homogeneous transformations
    // - wMe[6]: world to hand (end-effector) poses as six homogeneous transformations
    if (type == 1) {
      vpCalibration::calibrationTsai(cMo, wMe, eMc) ;
      std::cout << std::endl << "Output: hand in eye calibration result: eMc estimated " << std::endl ;
      std::cout << eMc << std::endl ;
    }
    else if (type == 2) {
      for (int i = 0; i < wMe.size(); ++i)
        wMe[i] = wMe[i].inverse();
      vpCalibration::calibrationTsai(cMo, wMe, eMc);
      std::cout << std::endl << "Output: hand to eye calibration result: wMc estimated " << std::endl ;
      std::cout << eMc << std::endl ;
    }
    else
      cout << "wrong type id" << endl;

    // save result to file
    out_filename = type == 1 ? "eMc.txt" : "wMc.txt";
    ofstream eMc_file(out_filename);
    if (eMc_file.is_open()) {
      eMc.save(eMc_file);
    }
    else {
      cout << "save hand eye calibration result to file failed" << endl;
    }
    eMc_file.close();

//    eMc.extract(erc);
//    std::cout << "Theta U rotation: " << vpMath::deg(erc[0]) << " " << vpMath::deg(erc[1]) << " " << vpMath::deg(erc[2]) << std::endl;
    return 0 ;
  }
  catch(vpException &e) {
    std::cout << "Catch an exception: " << e << std::endl;
    return 1 ;
  }
}