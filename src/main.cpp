#include <ros/ros.h>
#include <drive_ros_msgs/BoundingBoxArray.h>
#include <drive_ros_msgs/Homography.h>
#include <drive_ros_image_recognition/common_image_operations.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/Image.h>

drive_ros_image_recognition::ImageOperator* image_operator;
ros::Publisher pub_marker;
ros::Publisher pub_image;

std::string output_frame;


void bbCallback(const drive_ros_msgs::BoundingBoxArrayConstPtr& msg)
{
    visualization_msgs::MarkerArray array;





    for(auto i=msg->boxes.begin(); i != msg->boxes.end(); i++)
    {
        // set frame
        std::string frame = output_frame.empty() ? i->header.frame_id : output_frame;

        std::vector<cv::Point2f> imagePoints, worldPoints;
        // bottom left
        imagePoints.push_back(cv::Point2f(i->x1, i->y2));
        // bottom right
        imagePoints.push_back(cv::Point2f(i->x2, i->y2));

        if(image_operator->imageToWorld(imagePoints, worldPoints))
        {

        }
    }

    pub_marker.publish(array);

}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "object_fusion");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  image_operator = new drive_ros_image_recognition::ImageOperator();

  // common image operations
  if(!image_operator->init()) {
    ROS_WARN_STREAM("Failed to init image_operator");
    return 1;
  }

  pnh.param<std::string>("output_frame", output_frame, "");

  pub_marker = pnh.advertise<visualization_msgs::MarkerArray>("output_obj", 100);
  ros::Subscriber sub_bb = pnh.subscribe("bb_topic", 1, bbCallback);

  while(ros::ok()){
    ros::spin();
  }
  return 0;
}
