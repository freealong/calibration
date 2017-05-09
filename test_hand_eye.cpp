//
// Created by yongqi on 17-5-3.
//

#include <iostream>
#include <visp3/core/vpHomogeneousMatrix.h>
#include "common.h"

using namespace std;

int main() {
  vpHomogeneousMatrix eMc;
  ifstream eMc_file("eMc.txt");
  eMc.load(eMc_file);
  print_matrix("end effector to camera Matrix: ", eMc);

  vpHomogeneousMatrix wMe, cMo;
  ifstream wMe_file("wMe.txt"), cMo_file("cMo.txt");
  wMe.load(wMe_file);
  print_matrix("world to end effector Matrix: ", wMe);
  cMo.load(cMo_file);
  print_matrix("camera to object Matrix: ", cMo);

  vpHomogeneousMatrix wMo;
  wMo = wMe * eMc * cMo;
  print_matrix("world to object Matrix: ", wMo);

  eMc_file.close();
  wMe_file.close();
  cMo_file.close();

  return 0;
}