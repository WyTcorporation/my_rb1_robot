#include <cmath>
#include <geometry_msgs/Twist.h>
#include <my_rb1_ros/Rotate.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

ros::Publisher pub;
double currentYaw = 0;

void odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
  double s = msg->pose.pose.orientation.w;
  double x = msg->pose.pose.orientation.x;
  double y = msg->pose.pose.orientation.y;
  double z = msg->pose.pose.orientation.z;
  currentYaw = atan2(2.0 * (y * x + s * z), s * s + x * x - y * y - z * z); // Calculate angle
}

bool rotateCallback(my_rb1_ros::Rotate::Request &req,
                    my_rb1_ros::Rotate::Response &res) {
  double targetYaw = currentYaw + (req.degrees * M_PI / 180.0);
  if (targetYaw > M_PI)
    targetYaw -= 2 * M_PI; 
  const double kP = 0.5; 
  ros::Rate rate(10);
  while (ros::ok()) {
    double errorYaw = targetYaw - currentYaw;
    if (fabs(errorYaw) < 0.01)
        break;
    geometry_msgs::Twist twist;
    twist.angular.z = kP * errorYaw; 
    pub.publish(twist);
    ros::spinOnce();
    rate.sleep();
  }
  geometry_msgs::Twist stopTwist;
  pub.publish(stopTwist);
  res.result = "Rotation completed!";
  return true;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "rotate_service_server");
  ros::NodeHandle nh;
  pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
  ros::Subscriber odom_sub = nh.subscribe("odom", 10, odomCallback);
  ros::ServiceServer service =
      nh.advertiseService("rotate_robot", rotateCallback);
  ROS_INFO("Server started!");
  ros::spin();
  return 0;
}