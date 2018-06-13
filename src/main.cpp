#include <ros/ros.h>
#include <cmath>
#include <sensor_msgs/Image.h>
#include <drive_ros_msgs/BoundingBoxArray.h>
#include <drive_ros_msgs/Homography.h>
#include <drive_ros_msgs/ObstacleArray.h>
#include <drive_ros_image_recognition/common_image_operations.h>

drive_ros_image_recognition::ImageOperator* image_operator;
ros::Publisher pub_obs;
std::string output_frame;


void bbCallback(const drive_ros_msgs::BoundingBoxArrayConstPtr& msg)
{
    drive_ros_msgs::ObstacleArray obs;

    for(auto i=msg->boxes.begin(); i != msg->boxes.end(); i++)
    {

        std::vector<cv::Point2f> imagePoints, worldPoints;
        // bottom left
        imagePoints.push_back(cv::Point2f(i->x1, i->y2));
        // bottom right
        imagePoints.push_back(cv::Point2f(i->x2, i->y2));

        if(image_operator->imageToWorld(imagePoints, worldPoints))
        {
            ROS_ASSERT(2 == worldPoints.size());

            drive_ros_msgs::Obstacle ob;
            ob.header.stamp = i->header.stamp;
            ob.header.frame_id = output_frame.empty() ? i->header.frame_id : output_frame;
            ob.obstacle_type = drive_ros_msgs::Obstacle::TYPE_CAMERA;
            ob.trust = i->confidence;
            ob.width = cv::norm(worldPoints.at(0) - worldPoints.at(1));

            geometry_msgs::Point centroid;
            centroid.x = ((worldPoints.at(0) + worldPoints.at(1))/2).x;
            centroid.y = ((worldPoints.at(0) + worldPoints.at(1))/2).y;
            centroid.z = 0;
            ob.centroid = centroid;

            for(auto it = worldPoints.begin(); it != worldPoints.end(); it++)
            {
               geometry_msgs::Point32 pt;
               pt.x = it->x;
               pt.y = it->y;
               pt.z = 0;
               ob.polygon.points.push_back(pt);
            }

            obs.obstacles.push_back(ob);
        }
    }

    pub_obs.publish(obs);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "object_fusion");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  image_operator = new drive_ros_image_recognition::ImageOperator();

  // common image operations
  if(!image_operator->init()) {
    ROS_ERROR_STREAM("Failed to init image_operator");
    return 1;
  }

  pnh.param<std::string>("output_frame", output_frame, "");

  pub_obs = pnh.advertise<drive_ros_msgs::ObstacleArray>("obs_topic", 100);
  ros::Subscriber sub_bb = pnh.subscribe("bb_topic", 1, bbCallback);

  while(ros::ok()){
    ros::spin();
  }
  return 0;
}
