#ifndef __GRASP_PCL_TYPES__
#define __GRASP_PCL_TYPES__
#pragma once

#include <pcl/point_cloud.h>  
#include <pcl/point_types.h>  
#include <pcl_ros/point_cloud.h>

namespace aml_grasp{



    namespace types {

        typedef pcl::PointXYZRGB PointType; 
        typedef pcl::Normal NormalType; 
        typedef pcl::PointXYZRGBNormal PointNormalType; 
        typedef pcl::PrincipalCurvatures FeatureType;

        typedef pcl::PointCloud<PointType> PointCloud;
        typedef pcl::PointCloud<NormalType> NormalCloud;
        typedef pcl::PointCloud<PointNormalType> PointNormalCloud;
        typedef pcl::PointCloud<FeatureType> FeatureCloud;

        typedef pcl::PCLPointCloud2 PointCloud2;



    }


}
   


#endif // __GRASP_PCL_TYPES__