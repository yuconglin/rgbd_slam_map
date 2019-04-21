#ifndef SYSTEM_H
#define SYSTEM_H

#include "tracking.h"
#include "localmapping.h"

namespace myslam
{

class System
{
public:
  System();
  ~System();
  bool Process(const cv::Mat &color, const cv::Mat &depth, const double timestamp, Camera::Ptr camera);
  void SaveTrajectoryTUM();
protected:
  Map::Ptr map_;
  Tracking::Ptr tracking_;
  LocalMapping::Ptr localmapping_;

  std::thread* tracking_thread_;
  std::thread* localmapping_thread_;

};

} //namespace

#endif