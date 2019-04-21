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

void LocalMapping::ProcessKeyFrame(Frame::Ptr fm)
{
   vector<MapPoint*> mappoints = fm->map_points_;
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
       graph_[fm].emplace_back(count, frame);
       graph_[frame].emplace_back(count, fm);
     }
   }
}

} // namespace