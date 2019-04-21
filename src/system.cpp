#include "myslam/system.h"

namespace myslam
{

System::System()
{
  cout << "xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx Xiang Gao's rgbd odometry code was built in to a full slam system xxxxxxxxxxxxxxxxxxxxxx\n";

  //Config::setParameterFile ( filename );

  //map
  map_ = Map::Ptr(new Map);
  //tracking
  //using the main thread
  tracking_ = Tracking::Ptr(new Tracking(map_));
  //local mapping
  localmapping_ = LocalMapping::Ptr(new LocalMapping(map_));
  localmapping_thread_ = new thread(&LocalMapping::Run, localmapping_);

  //set pointers between thread
  tracking_->setLocalMapping(localmapping_.get());
}

System::~System()
{

}

bool System::Process(const cv::Mat &color, const cv::Mat &depth, const double timestamp, Camera::Ptr camera)
{
  return tracking_->addFrame(color, depth, timestamp, camera);
}

} //namespace