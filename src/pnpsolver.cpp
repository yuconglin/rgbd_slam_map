#include "myslam/pnpsolver.h"
#include "myslam/g2o_types.h"
#include "myslam/converter.h"
#include "myslam/config.h"

namespace myslam
{ 
  PnPSolver::PnPSolver()
  {
    min_inlier_ = Config::get<int>("min_inlier");
    min_match_  = Config::get<int>("min_match");
  }

  bool PnPSolver::solvePnP(const vector<cv::Point2f>& img, const vector<cv::Point3f>& obj, const Camera::Ptr camera, vector<int>& inliersIndex,
                      SE3& transform)
  {
    assert(img.size() == obj.size());
    // g2o初始化
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<6,2>> Block;
    Block::LinearSolverType* linearSolver = new g2o::LinearSolverDense<Block::PoseMatrixType>();
    Block* solver_ptr = new Block ( linearSolver );
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg ( solver_ptr );
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm ( solver );

    // 加入待估计的位姿：一个se3 vertex
    g2o::VertexSE3Expmap*   vSE3 = new g2o::VertexSE3Expmap();
    vSE3->setEstimate( Converter::toSE3Quat( Eigen::Isometry3d::Identity()) );
    vSE3->setFixed( false );
    // id为零
    vSE3->setId(0);
    optimizer.addVertex( vSE3 );

    // 接下来就是一堆边，边类型为se3 project xyz only pose
    // 这种边只有一个端点，就是se3本身
    // 先用一个vector装起来，之后要用
    vector<EdgeProjectXYZ2UVPoseOnly*> edges;

    // 谜之delta
    const float delta = sqrt(5.991);
    // 每条边是否为inliers
    vector<bool>    inliers( img.size(), true );
    int good = 0;
    for ( size_t i=0; i<obj.size(); i++ )
    {
        if (obj[i] == cv::Point3f(0,0,0))
        {
            // 该点值不存在
            inliers[i] = false;
            continue;
        }
        good++;

        EdgeProjectXYZ2UVPoseOnly * edge = new EdgeProjectXYZ2UVPoseOnly();
        edge->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
        // 2D点通过setmeasurement来设
        edge->setMeasurement( Eigen::Vector2d(img[i].x, img[i].y) );
        // 这种edge比较特殊，3D点和相机参数直接为成员变量
        edge->camera_ = camera.get();
        edge->point_ = Vector3d(obj[i].x, obj[i].y, obj[i].z);
        // information其实没多大意义，但为了能求解还是要设一个
        edge->setInformation( Eigen::Matrix2d::Identity()*1 );
        // 由于误匹配的存在，要设置robust kernel
        g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber();
        edge->setRobustKernel( rk );
        rk->setDelta( delta );
        optimizer.addEdge( edge );
        edge->setId( i );
        edges.push_back(edge);
    }

    // 使用g2o来判断inliers
    // 一共进行四轮迭代，每轮迭代十次
    for(size_t it=0; it<4; it++)
    {
        vSE3->setEstimate( Converter::SE3toSE3Quat( transform ) );
        optimizer.initializeOptimization(0);
        optimizer.optimize( 10 );

        for ( size_t i=0; i<edges.size(); i++ )
        {
            EdgeProjectXYZ2UVPoseOnly* e = edges[i];
            if ( inliers[ e->id() ])
            {
                e->computeError();
            }
            //如果某条边的均方误差太大，说明它是一条outlier
            if ( e->chi2() > 5.991 )
            {
                inliers[ e->id() ] = false;
                e->setLevel(1);
                good -- ;
            }
            else
            {
                // 否则就是inlier
                inliers[i] = true;
                e->setLevel(0);
            }

            // 去掉较大误差的边后，就不必再用robust kernel了
            if (it==2)
                e->setRobustKernel( nullptr );
        }

        // 如果inlier太少，就中断
        if (good < 5)
            break;
    }

    for ( size_t i=0; i<inliers.size(); i++ )
    {
        if ( inliers[i] )
        {
            inliersIndex.push_back(i);
        }
    }

    g2o::VertexSE3Expmap* vSE_recov = dynamic_cast<g2o::VertexSE3Expmap*> (optimizer.vertex(0));
    g2o::SE3Quat    se3_recov = vSE_recov->estimate();

    transform = SE3 (se3_recov.rotation(), se3_recov.translation());

    if (inliers.size() > min_inlier_)
        return true;
    return false;
  }
  
} // namespace