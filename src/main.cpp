#include <ros/ros.h>
#include <drive_ros_msgs/BoundingBoxArray.h>
#include <drive_ros_msgs/Homography.h>

static bool homog_arrived = false;


void bbCallback(drive_ros_msgs::BoundingBoxArrayConstPtr msg)
{
  ROS_INFO_STREAM("bb size: " << msg->boxes.size());

  if(!homog_arrived)
  {
    ROS_INFO_STREAM("homog still missing. Aborting.");
    return;
  }
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "object_fusion");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  ros::Subscriber sub = pnh.subscribe("bb_topic", 1, bbCallback);

  while(ros::ok()){
    ros::spin();
  }
  return 0;
}
