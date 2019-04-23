#include "myslam/localmapping.h"
#include "myslam/config.h"
#include <algorithm>

namespace myslam
{

LocalMapping::LocalMapping(Map::Ptr map) : abort_ba_(false), map_(map) 
{
  min_shared_mappoints_ = Config::get<int> ( "min_shared_mappoints" );
}

void LocalMapping::Run()
{
  finished_ = false;
  while (true)
  {
    SetAcceptKeyFrame(false);
    if (CheckNewKeyframes()) 
    {
       ProcessKeyFrame();
    }
  }
}

bool LocalMapping::AcceptKeyFrame()
{
  unique_lock<mutex> lock(mutex_accept_keyframe_);
  return accept_keyframe_;
}

void LocalMapping::SetAcceptKeyFrame(bool flag)
{
  unique_lock<mutex> lock(mutex_accept_keyframe_);
  accept_keyframe_ = flag;
}

void LocalMapping::InsertKeyFrame(Frame::Ptr frame)
{
  unique_lock<mutex> lock(mutex_new_frames_);
  new_keyframes_.push_back(frame);
  abort_ba_ = true;
}

bool LocalMapping::CheckNewKeyframes()
{
  unique_lock<mutex> lock(mutex_new_frames_);
  return (!new_keyframes_.empty());
}

void LocalMapping::ProcessKeyFrame()
{

  {
    unique_lock<mutex> lock(mutex_new_frames_);
    new_kf_ = new_keyframes_.front();
    new_keyframes_.pop_front();
  }

   vector<MapPoint*> mappoints = new_kf_->map_points_;
   for (large_interator it = graph_.begin(); it != graph_.end(); ++ it)
   {
     Frame::Ptr frame = it->first;
     vector<MapPoint*> mppts = frame->map_points_;
     int count = 0;
     for (MapPoint* pt : mappoints) {
       if (find(mppts.begin(), mppts.end(), pt) != mppts.end()) {
          count ++;
       }
     }
     if (count > min_shared_mappoints_) {
       graph_[new_kf_].emplace_back(count, frame);
       graph_[frame].emplace_back(count, new_kf_);
     }
   }

}

void LocalMapping::RequestStop()
{
  unique_lock<mutex> lock(mutex_stop_);
  stop_requested_ = true;
  unique_lock<mutex> lock2(mutex_new_frames_);
  abort_ba_ = true;
}

bool LocalMapping::Stop()
{
  unique_lock<mutex> lock(mutex_stop_);
  if (stop_requested_ && !not_stop_)
  {
    stopped_ = true;
    cout << "local mapping stopped\n";
    return true;
  }
}

bool LocalMapping::isStopped()
{
  unique_lock<mutex> lock(mutex_stop_);
  return stopped_;
}

bool LocalMapping::stopRequrested()
{
  unique_lock<mutex> lock(mutex_stop_);
  return stop_requested_;
}

} // namespace