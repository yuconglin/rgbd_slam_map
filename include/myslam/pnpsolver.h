#ifndef PNPSOLVER_H
#define PNPSOLVER_H

#include "common_include.h"
#include "camera.h"

namespace myslam
{

class PnPSolver 
{
public:
  typedef shared_ptr<PnPSolver> Ptr;
  PnPSolver();
  bool solvePnP(const vector<cv::Point2f>& img, const vector<cv::Point3f>& obj, const Camera::Ptr camera, vector<int>& inliersIndex,
                      SE3& transform);
private:
  int min_inlier_;
  int min_match_;
};

} //namespace

#endif