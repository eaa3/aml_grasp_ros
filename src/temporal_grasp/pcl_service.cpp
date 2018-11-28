#include "ros/ros.h"
#include <string>
#include <sstream>
#include "temporal_grasp_ros/PCLUtility.h"
#include "temporal_grasp/types.h"
#include "temporal_grasp/pcl_utils.h"
#include <pcl_ros/transforms.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>

namespace types = aml_grasp::types;
namespace utils = aml_grasp::utils;


tf::TransformListener* listener;


void clear(){

    delete listener;
}

// ----- starts the ros service
void initiliseServer();

// ----- processes the service request and calls the required method from pcl_processing
bool processRequest_(temporal_grasp_ros::PCLUtility::Request  &req,
         temporal_grasp_ros::PCLUtility::Response &res);


bool processRequest_(temporal_grasp_ros::PCLUtility::Request  &req,
         temporal_grasp_ros::PCLUtility::Response &res)
{
    ROS_INFO("Received a call!\n");
    if (req.function_call_id == "compute_features")
    {
        types::FeatureCloud::Ptr feature_cloud (new types::FeatureCloud);
        types::PointNormalCloud::Ptr point_normal_cloud (new types::PointNormalCloud);

        types::PointCloud::Ptr cloud(new types::PointCloud());

        // Transform point cloud
        sensor_msgs::PointCloud2 transformed_msg_cloud;
        pcl_ros::transformPointCloud("world", req.cloud_input, transformed_msg_cloud, *listener);

        pcl::PCLPointCloud2 cloud_tmp;
        pcl_conversions::toPCL(transformed_msg_cloud, cloud_tmp);
        pcl::fromPCLPointCloud2(cloud_tmp,*cloud);

        // printf("Cloud should have colors here!\n");
        // for( auto e : cloud->points){
        //     printf("Color %u\n", e.rgb);
        // }
        // int dummy;
        // printf("Is there color?\n");
        // scanf("%d",&dummy);

        std::cout << "Received cloud with " << cloud->points.size() << " points." << std::endl;


        // Crop point cloud before hand
        // x=0.4, y=-0.35, z=1.05
        // Do any other pre-processing
        Eigen::Vector4f min_pt (req.crop_min_pt[0], req.crop_min_pt[1], req.crop_min_pt[2], 1.0f);//(0.2, -0.40, -0.01, 1.0f);
        Eigen::Vector4f max_pt (req.crop_max_pt[0], req.crop_max_pt[1], req.crop_max_pt[2], 1.0f);//(0.6, 0.0, 0.2, 1.0f);
        types::PointCloud::Ptr cloud_cropped(new types::PointCloud());
        utils::crop_cloud(cloud, cloud_cropped, min_pt, max_pt);

        // for( auto e : cloud_cropped->points){
        //     printf("Color %u\n", e.rgb);
        // }

        // Extract the eigenvalues and eigenvectors
        Eigen::Vector3f eigen_values;
        Eigen::Matrix3f eigen_vectors;


        utils::compute_principal_axis(cloud_cropped, eigen_vectors, eigen_values);

        res.eigen_values.resize(3);
        res.eigen1.resize(3); res.eigen2.resize(3); res.eigen3.resize(3);
        for( int i = 0; i < 3; i++){
            res.eigen_values[i] = eigen_values(i);
            res.eigen1[i] = eigen_vectors.col(0)(i);
            res.eigen2[i] = eigen_vectors.col(1)(i);
            res.eigen3[i] = eigen_vectors.col(2)(i);
        }


        // Compute features
        utils::compute_features(cloud_cropped, point_normal_cloud, feature_cloud, req.view_point[0], req.view_point[1], req.view_point[2], 0.01, 0.01);
        

        // for( auto e : point_normal_cloud->points){
        //     printf("Color %d\n", e.rgb);
        // }

        pcl::PCLPointCloud2 cloud2, features2, cloud_out;

        pcl::toPCLPointCloud2(*point_normal_cloud, cloud2);
        pcl::toPCLPointCloud2(*feature_cloud, features2);

        pcl::concatenateFields (cloud2, features2, cloud_out);

        pcl_conversions::fromPCL(cloud_out, res.cloud_output);
        std::cout << "Output cloud with " << feature_cloud->points.size() << " points." << std::endl;

        
        res.info = "Successfully computed features!";
        res.success = true;
        return true;

    }




}


void initiliseServer()
{
    ros::NodeHandle nh_;
    ros::ServiceServer service_ = nh_.advertiseService("aml_pcl_service", processRequest_);
    listener = new tf::TransformListener();

    atexit(clear);

    ROS_INFO("Kinova AML Grasp PCL Service Started!");
    ros::spin();
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "pcl_utility_server");

    initiliseServer();


    // PCLUtilityServer server;

    return 0;
}