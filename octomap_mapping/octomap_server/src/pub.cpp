#include <iostream>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

using namespace std;

ros::Publisher _cloud_pub;
ros::Publisher _pose_pub;
ros::Subscriber _camera_pointcloud_sub;
ros::Subscriber _odom_sub;

pcl::PointCloud<pcl::PointXYZ> cameraCloud; // The live point cloud from the camera
tf::TransformListener* tf_listener;

void cameraPointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& input) {
    // Transform the camera point cloud to the world frame
    sensor_msgs::PointCloud2 transformed_cloud;
    try {
        tf_listener->waitForTransform("world", input->header.frame_id, ros::Time(0), ros::Duration(3.0));
        pcl_ros::transformPointCloud("world", *input, transformed_cloud, *tf_listener);
        ROS_INFO("Transformed point cloud from camera frame to world frame.");
    } catch (tf::TransformException &ex) {
        ROS_ERROR("Transform error: %s", ex.what());
        return;
    }

    // Convert the transformed PointCloud2 message to a pcl::PointCloud
    pcl::fromROSMsg(transformed_cloud, cameraCloud);
    
    ROS_INFO("Received and transformed live point cloud from camera with %lu points", cameraCloud.points.size());

    // Publish the transformed point cloud
    _cloud_pub.publish(transformed_cloud);
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg) {
    // Create a PoseStamped message from the odometry data
    geometry_msgs::PoseStamped pose;
    pose.header = odom_msg->header;  // Use the same header as the odometry message
    pose.pose = odom_msg->pose.pose;  // Get the pose from the odometry message

    // Publish the camera pose
    _pose_pub.publish(pose);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "camera_pointcloud_transformer");
    ros::NodeHandle n("~");

    _cloud_pub = n.advertise<sensor_msgs::PointCloud2>("transformed_point_cloud", 1);
    _pose_pub = n.advertise<geometry_msgs::PoseStamped>("camera_pose", 1);

    // Initialize the tf listener
    tf_listener = new tf::TransformListener();

    // Subscribe to the camera/depth/points topic
    _camera_pointcloud_sub = n.subscribe("/camera/depth/points", 1, cameraPointCloudCallback);
    
    // Subscribe to the odometry topic
    _odom_sub = n.subscribe("/odom", 1, odomCallback);  // Adjust the topic name as needed

    ros::Rate loop_rate(1);  // Adjust this rate as needed

    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }

    delete tf_listener;
    return 0;
}
