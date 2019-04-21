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

#include "myslam/map.h"

namespace myslam
{

void Map::insertKeyFrame ( Frame::Ptr frame )
{
    //cout<<"Key frame size = "<<keyframes_.size()<<endl;
    //unique_lock<mutex> lock(mutex_map_);
    if ( keyframes_.find(frame->id_) == keyframes_.end() )
    {
        keyframes_.insert( make_pair(frame->id_, frame) );
    }
    else
    {
        keyframes_[ frame->id_ ] = frame;
    }
}

void Map::insertMapPoint ( MapPoint::Ptr map_point )
{
    //unique_lock<mutex> lock(mutex_map_);
    if ( map_points_.find(map_point->id_) == map_points_.end() )
    {
        map_points_.insert( make_pair(map_point->id_, map_point) );
        //reference_map_points_.insert( make_pair(map_point->id_, map_point) );
    }
    else 
    {
        map_points_[map_point->id_] = map_point;
        //reference_map_points_[map_point->id_] = map_point;
    }
}

void Map::insertReferenceMapPoint(MapPoint::Ptr map_point)
{
    if (reference_map_points_.find(map_point->id_) == map_points_.end())
    {
        reference_map_points_.insert(make_pair(map_point->id_, map_point));
    }
    else
    {
        reference_map_points_[map_point->id_] = map_point;
    }
}

void Map::UpdateReferenceMap(Frame::Ptr curr_, double map_point_erase_ratio_)
{
    //unique_lock<mutex> lock(mutex_map_);
    for (auto iter = reference_map_points_.begin(); iter != reference_map_points_.end();)
    {
        if ( !curr_->isInFrame(iter->second->pos_) )
        {
            iter = reference_map_points_.erase(iter);
            continue;
        }
        float match_ratio = float(iter->second->matched_times_)/ iter->second->visible_times_;
        if ( match_ratio < map_point_erase_ratio_ )
        {
            iter = reference_map_points_.erase(iter);
            continue;
        }
        
        Vector3d n = iter->second->pos_ - curr_->getCamCenter();
        n.normalize();
        double angle = acos( n.transpose()* iter->second->norm_ );
        if ( angle > M_PI/6. )
        {
            iter = reference_map_points_.erase(iter);
            // need to also remove from keyframe's sets
            continue;
        }
        if ( iter->second->good_ == false )
        {
	        std::cout << "bad mappoint" << "\n";
            // TODO try triangulate this map point 
        }
        iter++;
    }
}

}
