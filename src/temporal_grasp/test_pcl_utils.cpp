#include <cstdlib>
#include <cstdio>
#include "temporal_grasp/types.h"
#include "temporal_grasp/pcl_utils.h"

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <ros/ros.h>

namespace types = aml_grasp::types;
namespace utils = aml_grasp::utils;

ros::Publisher pub;

void publish(types::PointNormalCloud::Ptr& cloud, types::FeatureCloud::Ptr& features){

    pcl::PCLPointCloud2 cloud2, features2, cloud_out;

    // pcl::fromPCLPointCloud2( point_cloud2, point_cloud);
    // for( auto& p : cloud->points){
    //     p.rgb = 255;
    //     p.curvature = 123;
    // }
    pcl::toPCLPointCloud2(*cloud, cloud2);
    pcl::toPCLPointCloud2(*features, features2);

    pcl::concatenateFields (cloud2, features2, cloud_out);

    cloud_out.header.frame_id = "world";
    pub.publish(cloud_out);
}

int main(int argc, char** argv){

    ros::init (argc, argv, "my_pcl_tutorial");
    ros::NodeHandle nh;

    const char* cloud_path = std::getenv("CLOUD_PATH");
    if( !cloud_path ){
      printf("Set CLOUD_PATH environment variable to point to a .pcd file!\n");
      exit(1);
      return 1;
    }

    printf("Loading cloud %s\n", cloud_path);


    types::PointCloud::Ptr cloud (new types::PointCloud);

    if (pcl::io::loadPCDFile<types::PointType> (cloud_path, *cloud) == -1) //* load the file
    {
      PCL_ERROR ("Couldn't read file");
      return (-1);
    }

    std::cout << "Loaded " << cloud->points.size() << " points." << std::endl;

    types::FeatureCloud::Ptr feature_cloud (new types::FeatureCloud);
    types::PointNormalCloud::Ptr point_normal_cloud (new types::PointNormalCloud);
    utils::compute_features(cloud, point_normal_cloud, feature_cloud);


    
    pub = nh.advertise<pcl::PCLPointCloud2> ("output", 1);

    
//     pcl::PointCloud<type::PointXYZ> cloud;
//   pcl::fromROSMsg (*input, cloud);


    ros::Rate loop_rate(2);
    while( ros::ok() ){

        publish(point_normal_cloud, feature_cloud);

        ros::spinOnce();

        loop_rate.sleep();

    }


    return 0;
}


    