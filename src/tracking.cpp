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

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <algorithm>
#include <iomanip>
#include <boost/timer.hpp>

#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_impl.h>

#include "myslam/config.h"
#include "myslam/tracking.h"

namespace myslam
{

Tracking::Tracking(Map::Ptr map) :
    state_ ( INITIALIZING ), ref_ ( nullptr ), curr_ ( nullptr ), local_mapping_ (nullptr), map_ ( map ), pnpsolver_ (new PnPSolver), num_lost_ ( 0 ), num_inliers_ ( 0 ), matcher_flann_ ( new cv::flann::LshIndexParams ( 5,10,2 ) )
{
    num_of_features_    = Config::get<int> ( "number_of_features" );
    scale_factor_       = Config::get<double> ( "scale_factor" );
    level_pyramid_      = Config::get<int> ( "level_pyramid" );
    ini_fast_thres_     = Config::get<int> ( "ini_fast_thres");
    min_fast_thres_     = Config::get<int> ( "min_fast_thres");

    match_ratio_        = Config::get<float> ( "match_ratio" );
    max_num_lost_       = Config::get<float> ( "max_num_lost" );
    min_inliers_        = Config::get<int> ( "min_inliers" );
    key_frame_min_rot   = Config::get<double> ( "keyframe_rotation" );
    key_frame_min_trans = Config::get<double> ( "keyframe_translation" );
    map_point_erase_ratio_ = Config::get<double> ( "map_point_erase_ratio" );

    orb_ = new ORB_SLAM2::ORBextractor(num_of_features_, scale_factor_, level_pyramid_, ini_fast_thres_, min_fast_thres_);
    output_file.open(Config::get<string>("trajectory_file"));
}

Tracking::Tracking() :
    state_ ( INITIALIZING ), ref_ ( nullptr ), curr_ ( nullptr ), local_mapping_ (nullptr), map_ (new Map), pnpsolver_ (new PnPSolver), num_lost_ ( 0 ), num_inliers_ ( 0 ), matcher_flann_ ( new cv::flann::LshIndexParams ( 5,10,2 ) )
{
    num_of_features_    = Config::get<int> ( "number_of_features" );
    scale_factor_       = Config::get<double> ( "scale_factor" );
    level_pyramid_      = Config::get<int> ( "level_pyramid" );
    ini_fast_thres_     = Config::get<int> ( "ini_fast_thres");
    min_fast_thres_     = Config::get<int> ( "min_fast_thres");

    match_ratio_        = Config::get<float> ( "match_ratio" );
    max_num_lost_       = Config::get<float> ( "max_num_lost" );
    min_inliers_        = Config::get<int> ( "min_inliers" );
    key_frame_min_rot   = Config::get<double> ( "keyframe_rotation" );
    key_frame_min_trans = Config::get<double> ( "keyframe_translation" );
    map_point_erase_ratio_ = Config::get<double> ( "map_point_erase_ratio" );

    orb_ = new ORB_SLAM2::ORBextractor(num_of_features_, scale_factor_, level_pyramid_, ini_fast_thres_, min_fast_thres_);
    output_file.open(Config::get<string>("trajectory_file"));
}

Tracking::~Tracking()
{
    delete orb_;
    output_file.close();
}

bool Tracking::addFrame ( Frame::Ptr frame )
{
    switch ( state_ )
    {
    case INITIALIZING:
    {
        state_ = OK;
        curr_ = ref_ = frame;
        // extract features from first frame and add them into map
        extractKeyPoints();
        addKeyFrame();      // the first frame is a key-frame
        break;
    }
    case OK:
    {
        curr_ = frame;
        curr_->T_c_w_ = ref_->T_c_w_;
        extractKeyPoints();
        featureMatching();
        poseEstimationPnP();
        if ( checkEstimatedPose() == true ) // a good estimation
        {
            curr_->T_c_w_ = T_c_w_estimated_;
            optimizeMap();
            num_lost_ = 0;
            if ( checkKeyFrame() == true ) // is a key-frame
            {
                addKeyFrame();
            }
        }
        else // bad estimation due to various reasons
        {
            num_lost_++;
            if ( num_lost_ > max_num_lost_ )
            {
                state_ = LOST;
            }
            return false;
        }
        break;
    }
    case LOST:
    {
        cout<<"tracking has lost."<<endl;
        break;
    }
    }

    return true;
}

bool Tracking::addFrame(const cv::Mat &color, const cv::Mat &depth, const double timestamp, Camera::Ptr camera)
{
    myslam::Frame::Ptr pFrame = myslam::Frame::createFrame();
    pFrame->camera_ = camera;
    pFrame->color_ = color;
    pFrame->depth_ = depth;
    pFrame->time_stamp_ = timestamp;

    boost::timer timer;
    addFrame(pFrame);
    cout << "Tracking costs time: " << timer.elapsed() << endl;

    if (state_ == myslam::Tracking::LOST)
        return false;
    SE3 Twc = pFrame->T_c_w_.inverse();

    // show the map and the camera pose

    Mat img_show = color.clone();
    for (auto &p : map_->reference_map_points_)
    {
        Vector2d pixel = pFrame->camera_->world2pixel(p.second->pos_, pFrame->T_c_w_);
        cv::circle(img_show, cv::Point2f(pixel(0, 0), pixel(1, 0)), 5, cv::Scalar(0, 255, 0), 2);
    }

    cv::imshow("image", img_show);
    cv::waitKey(1);

    cout << endl;
}

void Tracking::extractKeyPoints()
{
    boost::timer timer;
    
    cv::Mat gray = curr_->color_;
    cv::cvtColor(gray, gray, CV_RGB2GRAY);
    (*orb_)( gray, cv::Mat(), keypoints_curr_, descriptors_curr_ );
    
    cout<<"extract keypoints and descriptor cost time: "<<timer.elapsed() <<endl;
}

void Tracking::featureMatching()
{
    boost::timer timer;
    vector<cv::DMatch> matches;
    // select the candidates in map 
    Mat desp_map;
    vector<MapPoint::Ptr> candidate;
    for ( auto &p : map_->reference_map_points_)
    {
        // check if p in curr frame image 
        if ( curr_->isInFrame(p.second->pos_) )
        {
            // add to candidate 
            p.second->visible_times_++;
            candidate.push_back( p.second );
            desp_map.push_back( p.second->descriptor_ );
        }
    }
    
    matcher_flann_.match ( desp_map, descriptors_curr_, matches );
    // select the best matches
    float min_dis = std::min_element (
                        matches.begin(), matches.end(),
                        [] ( const cv::DMatch& m1, const cv::DMatch& m2 )
    {
        return m1.distance < m2.distance;
    } )->distance;

    match_3dpts_.clear();
    match_2dkp_index_.clear();
    for ( cv::DMatch& m : matches )
    {
        if ( m.distance < max<float> ( min_dis*match_ratio_, 30.0 ) )
        {
            match_3dpts_.push_back( candidate[m.queryIdx] );
            match_2dkp_index_.push_back( m.trainIdx );
        }
    }
    cout<<"good matches: "<<match_3dpts_.size() <<endl;
    cout<<"match cost time: "<<timer.elapsed() <<endl;
}

void Tracking::poseEstimationPnP()
{
    boost::timer timer;
    // construct the 3d 2d observations
    vector<cv::Point3f> pts3d;
    vector<cv::Point2f> pts2d;
    vector<long> mappoint_ids;

    for ( int index:match_2dkp_index_ )
    {
        pts2d.push_back ( keypoints_curr_[index].pt );
    }
    for ( MapPoint::Ptr pt:match_3dpts_ )
    {
        pts3d.push_back( pt->getPositionCV() );
        mappoint_ids.push_back( pt->id_ );
    }
   
    vector<int> inliersIndex;
    pnpsolver_->solvePnP(pts2d, pts3d, curr_->camera_, inliersIndex, T_c_w_estimated_);
    num_inliers_ = inliersIndex.size();

    // using bundle adjustment to optimize the pose
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<6,3>> Block;
    Block::LinearSolverType* linearSolver = new g2o::LinearSolverDense<Block::PoseMatrixType>();
    Block* solver_ptr = new Block ( linearSolver );
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg ( solver_ptr );
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm ( solver );

    // parameter: camera intrinsics
    g2o::CameraParameters* camera = new g2o::CameraParameters (
       curr_->camera_->fx_, Eigen::Vector2d ( curr_->camera_->cx_, curr_->camera_->cy_ ), 0
    );
    camera->setId ( 0 );
    optimizer.addParameter ( camera );

    // vertex for pose
    g2o::VertexSE3Expmap* pose = new g2o::VertexSE3Expmap();
    pose->setId ( 0 );
    pose->setEstimate ( g2o::SE3Quat (
        T_c_w_estimated_.rotation_matrix(), T_c_w_estimated_.translation()
    )); 
    optimizer.addVertex ( pose );

    // vertex for points
    for (int i=0; i<inliersIndex.size(); i++)
    {
        g2o::VertexSBAPointXYZ* point = new g2o::VertexSBAPointXYZ();
        point->setId(i+1);
        int index = inliersIndex[i];
        cv::Point3f p = pts3d[index];
        point->setEstimate(Eigen::Vector3d(p.x, p.y, p.z));
        point->setMarginalized(true);
        optimizer.addVertex(point);
    }

    // edges
    for ( int i=0; i<inliersIndex.size(); i++ )
    {
        int index = inliersIndex[i];
        // 3D -> 2D projection
        g2o::EdgeProjectXYZ2UV* edge = new g2o::EdgeProjectXYZ2UV();
        edge->setId ( i );
        edge->setVertex ( 0, dynamic_cast<g2o::VertexSBAPointXYZ*> ( optimizer.vertex ( i+1 ) ) );
        edge->setVertex ( 1, pose );
        edge->setMeasurement ( Vector2d ( pts2d[index].x, pts2d[index].y ) );
        edge->setParameterId ( 0,0 );
        edge->setInformation ( Eigen::Matrix2d::Identity() );
        optimizer.addEdge ( edge );
        // set the inlier map points 
        match_3dpts_[index]->matched_times_++;
    }

    optimizer.initializeOptimization();
    optimizer.optimize ( 10 );
    
    // recover results
    // pose
    T_c_w_estimated_ = SE3 (
        pose->estimate().rotation(),
        pose->estimate().translation()
    );
    //cout<<"T_c_w_estimated_: "<<endl<<T_c_w_estimated_.matrix()<<endl;

    Vector3d trans = T_c_w_estimated_.translation();
    Quaterniond quat = T_c_w_estimated_.unit_quaternion();
    output_file << fixed << std::setprecision(4) 
    << curr_->time_stamp_
    << ' ' << trans(0) << ' ' << trans(1) << ' ' << trans(2)
    << ' ' << quat.x() << ' ' << quat.y() << ' ' << quat.z()
    << ' ' << quat.w() << '\n';
    // points
    {
        unique_lock<mutex> lck(map_->mutex_map_);
        for (int i = 0; i < inliersIndex.size(); i++)
        {
            g2o::VertexSBAPointXYZ *point = static_cast<g2o::VertexSBAPointXYZ *>(optimizer.vertex(i + 1));
            Vector3d pt = point->estimate();
            long id = mappoint_ids[ inliersIndex[i] ];
            map_->map_points_[id]->pos_ = pt;
        }
    }
    cout << inliersIndex.size() << " map_points updated\n";
    cout<<"pnp estimation cost time: "<<timer.elapsed() <<endl;
}

bool Tracking::checkEstimatedPose()
{
    // check if the estimated pose is good
    if ( num_inliers_ < min_inliers_ )
    {
        cout<<"reject because inlier is too small: "<<num_inliers_<<endl;
        return false;
    }
    // if the motion is too large, it is probably wrong
    SE3 T_r_c = ref_->T_c_w_ * T_c_w_estimated_.inverse();
    Sophus::Vector6d d = T_r_c.log();
    if ( d.norm() > 5.0 )
    {
        cout<<"reject because motion is too large: "<<d.norm() <<endl;
        return false;
    }
    return true;
}

bool Tracking::checkKeyFrame()
{
    SE3 T_r_c = ref_->T_c_w_ * T_c_w_estimated_.inverse();
    Sophus::Vector6d d = T_r_c.log();
    Vector3d trans = d.head<3>();
    Vector3d rot = d.tail<3>();
    if ( rot.norm() >key_frame_min_rot || trans.norm() >key_frame_min_trans )
        return true;
    return false;
}

void Tracking::addKeyFrame()
{
    {
        unique_lock<mutex> lck(map_->mutex_map_);
        if (map_->keyframes_.empty())
        {
            // first key-frame, add all 3d points into map
            for (size_t i = 0; i < keypoints_curr_.size(); i++)
            {
                double d = curr_->findDepth(keypoints_curr_[i]);
                if (d < 0)
                    continue;
                Vector3d p_world = ref_->camera_->pixel2world(
                    Vector2d(keypoints_curr_[i].pt.x, keypoints_curr_[i].pt.y), curr_->T_c_w_, d);
                Vector3d n = p_world - ref_->getCamCenter();
                n.normalize();
                MapPoint::Ptr map_point = MapPoint::createMapPoint(
                    p_world, n, descriptors_curr_.row(i).clone(), curr_.get());
                map_->insertMapPoint(map_point);
                map_->insertReferenceMapPoint(map_point);
                curr_->AddMapPoint(map_point.get());
            }
        }
        map_->insertKeyFrame(curr_);
    }
    ref_ = curr_;
    if (local_mapping_->AcceptKeyFrame()) 
    {
      local_mapping_->InsertKeyFrame(curr_);
    }
}  

void Tracking::addMapPoints()
{
    // add the new map points into map
    //vector<bool> matched(keypoints_curr_.size(), false); 
    unordered_map<int, int> record;
    /*
    for ( int index:match_2dkp_index_ )
        matched[index] = true;
    */
    for (int i = 0; i < match_2dkp_index_.size(); ++ i)
    {
        record[ match_2dkp_index_[i] ] = i;
    }
    bool add_reference_map = (match_2dkp_index_.size() < 100);

    {
        unique_lock<mutex> lck(map_->mutex_map_);
        for (int i = 0; i < keypoints_curr_.size(); ++ i)
        {
            if (record.count(i)) {
                curr_->AddMapPoint(match_3dpts_[record[i]].get());
                match_3dpts_[record[i]]->AddObservationFrame(curr_.get());
                continue;
            }
            double d = ref_->findDepth(keypoints_curr_[i]);
            if (d < 0) {
                continue;
            }
            Vector3d p_world = ref_->camera_->pixel2world(
                Vector2d(keypoints_curr_[i].pt.x, keypoints_curr_[i].pt.y),
                curr_->T_c_w_, d);
            Vector3d n = p_world - ref_->getCamCenter();
            n.normalize();
            MapPoint::Ptr map_point = MapPoint::createMapPoint(
                p_world, n, descriptors_curr_.row(i).clone(), curr_.get());
          
            map_->insertMapPoint(map_point);
            if (add_reference_map) map_->insertReferenceMapPoint(map_point);
            curr_->AddMapPoint(map_point.get());
        }
    }
}

void Tracking::optimizeMap()
{
    //update reference map points according to current keyframe
    map_->UpdateReferenceMap(curr_, map_point_erase_ratio_);
    
    //if ( match_2dkp_index_.size()<100 )
    addMapPoints();
    if ( map_->reference_map_points_.size() > 1000 )  
    {
        // TODO map is too large, remove some one 
        map_point_erase_ratio_ += 0.05;
    }
    else 
        map_point_erase_ratio_ = 0.1;
    cout<<"local map points: "<< map_->reference_map_points_.size()<<endl;
    cout << "global map points: " << map_->map_points_.size() << '\n';
}

double Tracking::getViewAngle ( Frame::Ptr frame, MapPoint::Ptr point )
{
    Vector3d n = point->pos_ - frame->getCamCenter();
    n.normalize();
    return acos( n.transpose()*point->norm_ );
}


}
