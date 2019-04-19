/*
 * <one line to give the program's name and a brief idea of what it does.>
 * Copyright (C) 2016  <copyright holder> <email>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#ifndef TRACKING_H
#define TRACKING_H

#include "myslam/common_include.h"
#include "myslam/map.h"
#include "myslam/ORBextractor.h"
#include "myslam/pnpsolver.h"

#include "myslam/localmapping.h"

#include <mutex>
#include <thread>
#include <fstream>

#include <opencv2/features2d/features2d.hpp>

namespace myslam 
{

class Tracking
{
public:
    typedef shared_ptr<Tracking> Ptr;
    enum TrackState {
        INITIALIZING=-1,
        OK=0,
        LOST
    };
    
    LocalMapping* local_mapping_;

    TrackState     state_;     // current VO status
    Map::Ptr    map_;       // map with all frames and map points
    PnPSolver::Ptr pnpsolver_;
    
    Frame::Ptr  ref_;       // reference key-frame 
    Frame::Ptr  curr_;      // current frame 
    
    ORB_SLAM2::ORBextractor* orb_;  // orb detector and computer
    
    vector<cv::KeyPoint>    keypoints_curr_;    // keypoints in current frame
    Mat                     descriptors_curr_;  // descriptor in current frame 
    
    cv::FlannBasedMatcher   matcher_flann_;     // flann matcher
    vector<MapPoint::Ptr>   match_3dpts_;       // matched 3d points 
    vector<int>             match_2dkp_index_;  // matched 2d pixels (index of kp_curr)
   
    SE3 T_c_w_estimated_;    // the estimated pose of current frame 
    int num_inliers_;        // number of inlier features in icp
    int num_lost_;           // number of lost times
    
    // parameters 
    int num_of_features_;   // number of features
    double scale_factor_;   // scale in image pyramid
    int level_pyramid_;     // number of pyramid levels
    int ini_fast_thres_;    // initial FAST feature threshold
    int min_fast_thres_;    // minimal FAST feature threshold

    float match_ratio_;     // ratio for selecting  good matches
    int max_num_lost_;      // max number of continuous lost times
    int min_inliers_;       // minimum inliers
    double key_frame_min_rot;   // minimal rotation of two key-frames
    double key_frame_min_trans; // minimal translation of two key-frames
    double  map_point_erase_ratio_; // remove map point ratio
    
public: // functions 
    Tracking(Map::Ptr map);
    Tracking();
    ~Tracking();
    
    bool addFrame( Frame::Ptr frame );      // add a new frame 
    inline void setLocalMapping( LocalMapping* local_mapping) {
      local_mapping_ = local_mapping;
    };
    
protected:  
    // inner operation 
    void extractKeyPoints();
    void featureMatching();
    void poseEstimationPnP(); 
    void optimizeMap();
    
    void addKeyFrame();
    void addMapPoints();
    bool checkEstimatedPose(); 
    bool checkKeyFrame();
    
    double getViewAngle( Frame::Ptr frame, MapPoint::Ptr point );
    
    mutex map_mutex;

    //writing
    ofstream output_file;
};
}

#endif // VISUALODOMETRY_H
