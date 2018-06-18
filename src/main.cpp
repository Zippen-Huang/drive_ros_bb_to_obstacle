#include <ros/ros.h>
#include <cmath>
#include <sensor_msgs/Image.h>
#include <drive_ros_msgs/BoundingBoxArray.h>
#include <drive_ros_msgs/Homography.h>
#include <drive_ros_msgs/ObstacleArray.h>
#include <drive_ros_image_recognition/common_image_operations.h>

// image operator
drive_ros_image_recognition::ImageOperator* image_operator;

// publisher
ros::Publisher pub_obs;

// output frame (if empty we use from message)
std::string output_frame;

// covariances
boost::array<float, 36> pose_covariance;
boost::array<float, 36> twist_covariance;


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

            // set dimensions and orientation
            ob.width = cv::norm(worldPoints.at(0) - worldPoints.at(1));
            ob.height = 0;
            ob.length = 0;

            geometry_msgs::Quaternion q;
            q.x=0; q.y=0; q.z=0; q.w=0;
            ob.centroid_pose.pose.orientation = q;

            geometry_msgs::Point centroid;
            centroid.x = ((worldPoints.at(0) + worldPoints.at(1))/2).x;
            centroid.y = ((worldPoints.at(0) + worldPoints.at(1))/2).y;
            centroid.z = 0;
            ob.centroid_pose.pose.position = centroid;

            ob.centroid_pose.covariance = pose_covariance;
            ob.centroid_twist.covariance = twist_covariance;

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

  bool ret = true;

  // load covariances
  std::vector<float> pose_cov_vec, twist_cov_vec;
  ret &= pnh.getParam("pose_covariance",   pose_cov_vec);
  ret &= pnh.getParam("twist_covariance",  twist_cov_vec);

  ROS_ASSERT(36 == pose_cov_vec.size());
  ROS_ASSERT(36 == twist_cov_vec.size());

  std::copy(pose_cov_vec.begin(), pose_cov_vec.begin() + 36, pose_covariance.begin());
  std::copy(twist_cov_vec.begin(), twist_cov_vec.begin() + 36, twist_covariance.begin());

  // load output frame
  ret &= pnh.param<std::string>("output_frame", output_frame, "");

  if(!ret)
  {
      ROS_ERROR("Failed to load parameters!");
      return 1;
  }

  pub_obs = pnh.advertise<drive_ros_msgs::ObstacleArray>("obs_topic", 100);
  ros::Subscriber sub_bb = pnh.subscribe("bb_topic", 1, bbCallback);

  while(ros::ok()){
    ros::spin();
  }
  return 0;
}
