#ifndef CONVERTER_H
#define CONVERTER_H

#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/types/sba/types_six_dof_expmap.h>

#include "common_include.h"

namespace myslam
{

class Converter
{
public:
    static  g2o::SE3Quat    toSE3Quat(const Eigen::Isometry3d& T)
    {
        Eigen::Matrix3d R = T.rotation();
        Eigen::Vector3d t( T(0,3), T(1,3), T(2,3) );
        return g2o::SE3Quat( R, t );
    }

    static g2o::SE3Quat SE3toSE3Quat(const SE3& T)
    {
        return g2o::SE3Quat (T.rotation_matrix(), T.translation());
    }

    static SE3 SE3Quat2SE3(const g2o::SE3Quat& T)
    {
        return SE3(T.rotation(), T.translation());
    }
};

}

#endif