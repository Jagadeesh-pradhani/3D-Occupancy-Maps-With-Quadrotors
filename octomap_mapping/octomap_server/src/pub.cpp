#include <iostream>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>

#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/PointCloud2.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/conversions.h>
#include <octomap/octomap_types.h>

using namespace std;
using namespace octomap;

ros::Publisher _local_map_pub;
ros::Publisher _all_map_pub;
ros::Subscriber _odom_sub;

bool _map_ok = false;
bool _has_odom = false;

sensor_msgs::PointCloud2 localMap_pcd;
sensor_msgs::PointCloud2 globalMap_pcd;
pcl::PointCloud<pcl::PointXYZ> cloudMap;

void loadSavedMap(const std::string& map_file) {
    // Load the saved OctoMap from the file
    AbstractOcTree* tree = AbstractOcTree::read(map_file);
    OcTree* octree = dynamic_cast<OcTree*>(tree);

    if (!octree) {
        ROS_ERROR("Failed to load OctoMap from %s", map_file.c_str());
        return;
    }

    ROS_INFO("Loaded OctoMap from %s", map_file.c_str());

    // Convert the OctoMap to a PointCloud
    for (OcTree::leaf_iterator it = octree->begin_leafs(); it != octree->end_leafs(); ++it) {
        if (octree->isNodeOccupied(*it)) {
            pcl::PointXYZ pt;
            pt.x = it.getX();
            pt.y = it.getY();
            pt.z = it.getZ();
            cloudMap.points.push_back(pt);
        }
    }

    cloudMap.width = cloudMap.points.size();
    cloudMap.height = 1;
    cloudMap.is_dense = true;

    ROS_WARN("Finished loading saved map.");
    
    _map_ok = true;

    delete octree;
}

void pubSensedPoints() {
    if (!_map_ok)
        return;

    // Publish the loaded map as a ROS PointCloud2 message
    pcl::toROSMsg(cloudMap, globalMap_pcd);
    globalMap_pcd.header.frame_id = "world";
    _all_map_pub.publish(globalMap_pcd);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "load_and_publish_map");
    ros::NodeHandle n("~");

    _all_map_pub = n.advertise<sensor_msgs::PointCloud2>("all_map", 1);
    
    // Load map file from parameter or argument
    std::string map_file;
    n.param("map_file", map_file, std::string("/home/ros/btraj/my_octomap.ot"));

    // Load the saved map
    loadSavedMap(map_file);

    ros::Rate loop_rate(1);  // Adjust this rate as needed

    while (ros::ok()) {
        pubSensedPoints();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
