# Bounding Box to Obstacle
Converts [BoundingBoxArray.msg](https://github.com/tum-phoenix/drive_ros_msgs/blob/master/msg/BoundingBoxArray.msg) to [ObstacleArray.msg](https://github.com/tum-phoenix/drive_ros_msgs/blob/master/msg/ObstacleArray.msg) using the [camera homography](https://github.com/tum-phoenix/drive_ros_camera_homography). Uses the lower points as width of the object and center of left and right as centroid.

Dependencies:
* [image recognition](https://github.com/tum-phoenix/drive_ros_image_recognition)
