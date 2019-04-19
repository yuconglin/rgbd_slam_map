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

#ifndef MAP_H
#define MAP_H

#include "myslam/common_include.h"
#include "myslam/frame.h"
#include "myslam/mappoint.h"

namespace myslam
{
class Map
{
public:
    typedef shared_ptr<Map> Ptr;
    // all landmarks
    unordered_map<unsigned long, MapPoint::Ptr >  map_points_;       // all key-frames 
    unordered_map<unsigned long, Frame::Ptr >     keyframes_;   
    // reference_map_points_     
    unordered_map<unsigned long, MapPoint::Ptr > reference_map_points_;

    Map() {}
    
    void insertKeyFrame( Frame::Ptr frame );
    void insertMapPoint( MapPoint::Ptr map_point );
    void UpdateReferenceMap(Frame::Ptr frame, double ratio_);

    mutex mutex_map_;
protected:
    //mutex mutex_map_update_;
    
};
}

#endif // MAP_H
