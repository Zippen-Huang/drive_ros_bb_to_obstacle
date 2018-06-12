#include <ros/ros.h>
#include <drive_ros_msgs/BoundingBoxArray.h>
#include <drive_ros_msgs/Homography.h>
#include <drive_ros_image_recognition/common_image_operations.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/Image.h>

drive_ros_image_recognition::ImageOperator* image_operator;
ros::Publisher pub_marker;
ros::Publisher pub_image;


void publishMarker(const std::vector<cv::Point2f>& worldPoints,
                   visualization_msgs::MarkerArray& array)
{
    // left
    visualization_msgs::Marker marker;
    marker.header.frame_id = "left_camera";
    marker.header.stamp = ros::Time();
    marker.ns = "objects";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = worldPoints.at(0).x;
    marker.pose.position.y = worldPoints.at(0).y;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    array.markers.push_back( marker );

    // right
    marker.header.frame_id = "left_camera";
    marker.header.stamp = ros::Time();
    marker.ns = "objects";
    marker.id = 1;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = worldPoints.at(1).x;
    marker.pose.position.y = worldPoints.at(1).y;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    array.markers.push_back( marker );
}


void bbCallback(const drive_ros_msgs::BoundingBoxArrayConstPtr& msg)
{
    visualization_msgs::MarkerArray array;

    for(auto i=msg->boxes.begin(); i != msg->boxes.end(); i++)
    {
        std::vector<cv::Point2f> imagePoints, worldPoints;
        // bottom left
        imagePoints.push_back(cv::Point2f(i->x1, i->y2));
        // bottom right
        imagePoints.push_back(cv::Point2f(i->x2, i->y2));

        if(image_operator->imageToWorld(imagePoints, worldPoints))
        {
            publishMarker(worldPoints, array);
        }
    }

    pub_marker.publish(array);

}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{

    auto img = convertImageMessage(msg);

    cv::Mat dest;
    if(image_operator->homographImage(*img, dest))
    {
        pub_image.publish(cv_bridge::CvImage(std_msgs::Header(),
                          sensor_msgs::image_encodings::RGB8, dest).toImageMsg());
    }

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

  pub_marker = pnh.advertise<visualization_msgs::MarkerArray>("output_obj", 100);
  ros::Subscriber sub_bb = pnh.subscribe("bb_topic", 1, bbCallback);

  //pub_image = pnh.advertise<sensor_msgs::Image>("output_image", 1);
  //ros::Subscriber sub_image = nh.subscribe("/left/image_rect_color", 1, imageCallback);


  while(ros::ok()){
    ros::spin();
  }
  return 0;
}
