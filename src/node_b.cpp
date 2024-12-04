#include <ros/ros.h>
#include <tiago_iaslab_simulation/Objs.h>
#include <assignment1/AprilTagIDs.h>

void aprilTagIDsCallback(const assignment1::AprilTagIDs::ConstPtr& msg) {
    // TODO: Handle the april tags ids published by node a
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "node_b");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("april_tags_ids", 10, aprilTagIDsCallback);
    ROS_INFO("Subscribed to april_tags_ids");

    ros::spin();
    return 0;
}
