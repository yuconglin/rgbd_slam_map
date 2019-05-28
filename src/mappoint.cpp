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

#include "myslam/common_include.h"
#include "myslam/mappoint.h"
#include "myslam/frame.h"
#include "myslam/utility.h"

namespace myslam
{

MapPoint::MapPoint()
: id_(-1), pos_(Vector3d(0,0,0)), norm_(Vector3d(0,0,0)), good_(true), visible_times_(0), matched_times_(0)
{

}

MapPoint::MapPoint ( long unsigned int id, const Vector3d& position, const Vector3d& norm, Frame* frame, const Mat& descriptor )
: id_(id), pos_(position), norm_(norm), good_(true), visible_times_(1), matched_times_(1), descriptor_(descriptor), ref_frame_(frame), n_obs_(0)
{
    
}

MapPoint::Ptr MapPoint::createMapPoint()
{
    return MapPoint::Ptr( 
        new MapPoint( factory_id_++, Vector3d(0,0,0), Vector3d(0,0,0) )
    );
}

MapPoint::Ptr MapPoint::createMapPoint ( 
    const Vector3d& pos_world, 
    const Vector3d& norm, 
    const Mat& descriptor, 
    Frame* frame )
{
    return MapPoint::Ptr( 
        new MapPoint( factory_id_++, pos_world, norm, frame, descriptor )
    );
}

void MapPoint::UpdateNormal()
{
    map<Frame*, size_t> observations;
    Frame* ref_frame;
    Vector3d pos;
    {
        unique_lock<mutex> lock1(mutex_frames_);
        unique_lock<mutex> lock2(mutex_pos_);
        if (!good_) {return;}
        observations = observed_frames_;
        ref_frame = ref_frame_;
        pos = pos_;
    }

    if (observations.empty()) {return;}

    Vector3d normal;
    normal.setZero();
    int n = 0;
    for (map<Frame*,size_t>::iterator it = observations.begin(); it != observations.end(); it ++)
    {
        Frame* KF = it -> first;
        Vector3d Owi = KF->getCamCenter();
        Vector3d normali = pos_ - Owi;
        normal = normal + normali / normali.norm();
        n ++;
    }

    Vector3d PC = pos_ - ref_frame->getCamCenter();
    {
        unique_lock<mutex> lock3(mutex_pos_);
        norm_ = normal / n;
    }
}

void MapPoint::UpdateDescriptors()
{
    // retrieve all observed descriptors
    vector<Mat> vDescriptors;
    map<Frame*, size_t> observations;
    {
        unique_lock<mutex> lock1(mutex_frames_);
        if (!good_) {return;}
        observations = observed_frames_;
    }
    if (observations.empty()) {return;}
    vDescriptors.reserve(observations.size());

    for (map<Frame*, size_t>::iterator mit = observations.begin(); mit != observations.end(); mit ++) 
    {
        Frame* pF = mit->first;
        if (pF->isGood()) {
            vDescriptors.push_back(pF->descriptors_.row(mit->second));
        }
    }
    if (vDescriptors.empty()) {
        return;
    }
    // compute distances between them
    const size_t N = vDescriptors.size();
    float Distances[N][N];
    for (size_t i = 0; i < N; ++ i) 
    {
        Distances[i][i] = 0;
        for (size_t j = i + 1; j < N; ++ j)
        {
            int dist_ij = cv::norm(vDescriptors[i], vDescriptors[j],cv::NORM_HAMMING);
            Distances[i][j] = dist_ij;
            Distances[j][i] = dist_ij;
        }
    }
    // Take the descriptor with least median distance to the rest
    float BestMedian = FLT_MAX;
    int BestIdx = 0;
    for (size_t i = 0; i < N; ++ i)
    {
        vector<float> vDists(Distances[i], Distances[i] + N);
        float median = utility::GetMedian(vDists);
        if (median < BestMedian)
        {
            BestMedian = median;
            BestIdx = i;
        }
    }
    
    {
        unique_lock<mutex> lock(mutex_features_);
        descriptor_ = vDescriptors[BestIdx].clone();
    }
}

Mat MapPoint::GetDescriptor()
{
    unique_lock<mutex> lock(mutex_features_);
    return descriptor_.clone();
}

void MapPoint::AddObservationFrame(Frame* frame, size_t idx)
{
    unique_lock<mutex> lock(mutex_frames_);
    if (observed_frames_.count(frame)) {return;}
    observed_frames_[frame] = idx;
    n_obs_ ++;
}

unsigned long MapPoint::factory_id_ = 0;

}
