#ifndef LOCALMAPPING_H
#define LOCALMAPPING_H

#include "myslam/map.h"
#include <utility>

namespace myslam
{

class LocalMapping 
{
public: 
  typedef shared_ptr<LocalMapping> Ptr;
  typedef map< Frame::Ptr, vector< pair<int, Frame::Ptr> > >::iterator large_interator;

  LocalMapping(Map::Ptr map);  
  void Run();
  void InsertKeyFrame(Frame::Ptr frame);
  bool CheckNewKeyframes();

  //thread synch
  bool AcceptKeyFrame();
  void SetAcceptKeyFrame(bool flag);
  void RequestStop();
  bool Stop();
  bool isStopped();
  bool stopRequested();

protected:
  void ProcessKeyFrame();
  //map
  Map::Ptr map_;
  //local connected graph between key frames.
  //frame_id, <frame_id, frame>
  map< Frame::Ptr, vector< pair<int, Frame::Ptr> > > graph_; 
  
  //mappoint_id, count
  //unordered_map< long, long> mappoints_counts_;
  
  //shared mappoints number threshold
  int min_shared_mappoints_;

  //potential keyframes to add
  list<Frame::Ptr> new_keyframes_;
  Frame::Ptr new_kf_;
  //corresponding mutex
  mutex mutex_new_frames_;

  bool accept_keyframe_;
  mutex mutex_accept_keyframe_;

  bool stopped_;
  bool stop_requested_;
  bool not_stop_;
  mutex mutex_stop_;

  bool abort_ba_;
  bool finished_;
};

} //namespace end

#endif