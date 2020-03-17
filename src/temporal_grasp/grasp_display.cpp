
#include <map>
#include <string>

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "robot_markers/builder.h"
#include "ros/ros.h"
#include "urdf/model.h"
#include "visualization_msgs/MarkerArray.h"
#include <urdf_model/types.h>
#include <memory>


const std::string HAND_FILE = "/home/earruda/Projects/temporal_grasp/aml_data/pisa_iit_hand/pisa_hand_right.urdf";
const std::string GRASP_TOPIC = "/grasp_pose";
class GraspDisplay {

    public:

    urdf::Model model;
    std::shared_ptr<robot_markers::Builder> builder;
    visualization_msgs::MarkerArray robot_hand;
    ros::Publisher marker_arr_pub;
    ros::Subscriber sub;
    geometry_msgs::Pose pose;



    GraspDisplay(ros::NodeHandle& nh) {
        
        marker_arr_pub = nh.advertise<visualization_msgs::MarkerArray>("grasp_markers", 10);


        model.initFile(HAND_FILE);

        builder = std::shared_ptr<robot_markers::Builder>(new robot_markers::Builder(model));

        builder->Init();

        builder->SetNamespace("pisa_iit");
        builder->SetFrameId("world");
        builder->SetColor(0.33, 0.17, 0.45, 1);


        sub = nh.subscribe<geometry_msgs::PoseStamped> (GRASP_TOPIC, 5, &GraspDisplay::on_grasp_pose, this);
    }


    void on_grasp_pose(geometry_msgs::PoseStamped msg){

        ROS_INFO("Received pose");

        pose = msg.pose;
        builder->SetPose(msg.pose);
        builder->Build(&robot_hand);
        marker_arr_pub.publish(robot_hand);

    }

    void spin(){

        ros::spin();

    }


}; // GraspDisplay


int main(int argc, char** argv) {
  ros::init(argc, argv, "robot_markers_demo");
  ros::NodeHandle nh;
  
  GraspDisplay gdisplay(nh);

  gdisplay.spin();
  

  return 0;
}


// int main(int argc, char** argv) {
//   ros::init(argc, argv, "robot_markers_demo");
//   ros::NodeHandle nh;
//   ros::Publisher marker_arr_pub =
//       nh.advertise<visualization_msgs::MarkerArray>("grasp_markers", 10);
//   ros::Duration(0.5).sleep();


//   //ros::Subscriber sub = n.subscribe("chatter", 1, chatterCallback);

//   urdf::Model model;
//   model.initFile("/home/earruda/Projects/temporal_grasp/aml_data/pisa_iit_hand/pisa_hand_right.urdf");
//   //model.initParam("robot_description");

//   robot_markers::Builder builder(model);
//   builder.Init();

//   // Robot 1: Default configuration, purple.
//   visualization_msgs::MarkerArray robot1;
//   builder.SetNamespace("pisa_iit");
//   builder.SetFrameId("world");
//   builder.SetColor(0.33, 0.17, 0.45, 1);
//   geometry_msgs::Pose pose;
//   pose.position.y = 1;
//   pose.orientation.w = 0.92387953;
//   pose.orientation.z = -0.38268343;
//   builder.SetPose(pose);

  
//   builder.Build(&robot1);
//   marker_arr_pub.publish(robot1);

//   for(std::map<std::string, urdf::JointSharedPtr>::iterator joint = model.joints_.begin(); joint != model.joints_.end(); joint++)
//   {
//     ROS_INFO("%s",joint->second->name.c_str());
//   }

//   // Robot 2: Different pose, joints changed.
//   visualization_msgs::MarkerArray robot2;
//   builder.SetNamespace("robot2");
//   builder.SetColor(0, 0, 0, 0);

//   std::map<std::string, double> joint_positions;
//   joint_positions["soft_hand_index_inner_joint"] = 1.0;
//   joint_positions["soft_hand_thumb_abd_joint"] = 1.0;
//   builder.SetJointPositions(joint_positions);

//   geometry_msgs::Pose pose2;
//   pose2.position.y = 1;
//   pose2.orientation.w = 0.92387953;
//   pose2.orientation.z = -0.38268343;
//   builder.SetPose(pose2);
//   builder.Build(&robot2);
//   marker_arr_pub.publish(robot2);

//   ros::Duration(0.5).sleep();

//   return 0;
// }
