// -------------- test tracking and localmapping -------------
#include <fstream>
#include <boost/timer.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/viz.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "myslam/config.h"
#include "myslam/tracking.h"
#include "myslam/localmapping.h"

int main ( int argc, char** argv )
{
    if ( argc != 2 )
    {
        cout<<"usage: run_tracking_localmapping config_file"<<endl;
        return 1;
    }

    myslam::Config::setParameterFile ( argv[1] );

    myslam::Map::Ptr map(new myslam::Map);
   
    myslam::Tracking::Ptr tracking ( new myslam::Tracking(map) );
    myslam::LocalMapping::Ptr localmapping(new myslam::LocalMapping(map));

    tracking->setLocalMapping(localmapping.get());
    //tracking->setLocalMapping(new myslam::LocalMapping);

    string dataset_dir = myslam::Config::get<string> ( "dataset_dir" );
    cout<<"dataset: "<<dataset_dir<<endl;
    ifstream fin ( dataset_dir+"/associate.txt" );
    if ( !fin )
    {
        cout<<"please generate the associate file called associate.txt!"<<endl;
        return 1;
    }

    vector<string> rgb_files, depth_files;
    vector<double> rgb_times, depth_times;
    while ( !fin.eof() )
    {
        string rgb_time, rgb_file, depth_time, depth_file;
        fin>>rgb_time>>rgb_file>>depth_time>>depth_file;
        rgb_times.push_back ( atof ( rgb_time.c_str() ) );
        depth_times.push_back ( atof ( depth_time.c_str() ) );
        rgb_files.push_back ( dataset_dir+"/"+rgb_file );
        depth_files.push_back ( dataset_dir+"/"+depth_file );

        if ( fin.good() == false )
            break;
    }

    myslam::Camera::Ptr camera ( new myslam::Camera );

    cout<<"read total "<<rgb_files.size() <<" entries"<<endl;
    for ( int i=0; i<rgb_files.size(); i++ )
    {
        cout<<"****** loop "<<i<<" ******"<<endl;
        Mat color = cv::imread ( rgb_files[i] );
        Mat depth = cv::imread ( depth_files[i], -1 );
        if ( color.data==nullptr || depth.data==nullptr ) {
            cout << "xxxxxxxxxxxxxx no more data xxxxxxxxxxx \n";
            break;
        }
        myslam::Frame::Ptr pFrame = myslam::Frame::createFrame();
        pFrame->camera_ = camera;
        pFrame->color_ = color;
        pFrame->depth_ = depth;
        pFrame->time_stamp_ = rgb_times[i];
        
        boost::timer timer;
        tracking->addFrame ( pFrame );
        cout<<"Tracking costs time: "<<timer.elapsed() <<endl;

        if ( tracking->state_ == myslam::Tracking::LOST )
            break;
        SE3 Twc = pFrame->T_c_w_.inverse();

        // show the map and the camera pose

        Mat img_show = color.clone();
        for ( auto& p : tracking->map_->reference_map_points_ )
        {
            Vector2d pixel = pFrame->camera_->world2pixel ( p.second->pos_, pFrame->T_c_w_ );
            cv::circle ( img_show, cv::Point2f ( pixel ( 0,0 ),pixel ( 1,0 ) ), 5, cv::Scalar ( 0,255,0 ), 2 );
        }

        cv::imshow ( "image", img_show );
        cv::waitKey ( 1 );
        
        cout<<endl;
    }

    return 0;
}
