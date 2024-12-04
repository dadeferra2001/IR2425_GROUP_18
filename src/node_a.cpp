#include <ros/ros.h>
#include <std_msgs/String.h>
#include <tiago_iaslab_simulation/Objs.h>
#include <assignment1/AprilTagIDs.h>

void feedbackCallback(const std_msgs::String::ConstPtr& msg) {
    // TODO: Handle feedback from node_b
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "node_a");
    ros::NodeHandle nh;

    ros::ServiceClient id_client = nh.serviceClient<tiago_iaslab_simulation::Objs>("apriltag_ids_srv");
    ros::Publisher id_publisher = nh.advertise<assignment1::AprilTagIDs>("april_tags_ids", 10, true); // Latch, so late connection receive the latest msg
    ros::Subscriber feedback_sub = nh.subscribe("node_b_feedback", 10, feedbackCallback);

    // Creating the service request to send on 'apriltag_ids_srv'
    tiago_iaslab_simulation::Objs srv;
    srv.request.ready = true;

    // Calling the service and getting the response
    if (!id_client.call(srv)) {
        ROS_ERROR("Failed to call service /apriltags_ids_srv");
        return -1;
    }

    ROS_INFO("Service call succeeded.");

    // Prepare the custom message to publish the IDs
    assignment1::AprilTagIDs msg;
    msg.ids = srv.response.ids;

    // Publishing
    id_publisher.publish(msg);   
    ROS_INFO("Publishing IDs: %d", msg.ids.size());

    ros::spin();

    return 0;
}
