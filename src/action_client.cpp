#include <ros/ros.h>

#include <iostream>
#include <iterator>
#include <sstream>
#include <string>
#include <vector>

#include <assignment1/SearchIdsAction.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tiago_iaslab_simulation/Objs.h>

using namespace std;


class ActionClient {
    
    private:
        actionlib::SimpleActionClient<assignment1::SearchIdsAction> ac;
        std::vector<int> poses;

    public:

        /**
        * Initialize the action client and wait for the connection to the server.
        *
        * @param targets April tag IDs to find,
        * @return New instance of running action client.
        */
        ActionClient(const std::vector<int>& targets) : ac("SearchIds", true) {
            ROS_INFO("Waiting for action server to start.");
            ac.waitForServer();

            ROS_INFO("Action server started, sending goal.");

            poses = targets;

            // Initialize the action goal
            assignment1::SearchIdsGoal goal;
            goal.ids = targets;

            // Send the goal to the action server with callbacks for done, active, and feedback
            ac.sendGoal(goal, boost::bind(&ActionClient::doneCallback, this, _1, _2), boost::bind(&ActionClient::activeCallback, this), boost::bind(&ActionClient::feedbackCallback, this, _1));
        }

        /**
        * Action client done callback. Gets called when the action server is done.
        *
        * @param state Actionlib goal state.
        * @param result Action result message, containing the goal result and the list of april tag poses.
        */
        void doneCallback(const actionlib::SimpleClientGoalState &state, const assignment1::SearchIdsResultConstPtr &result) {
            if(result->completed) {
                ROS_INFO("The robot found ALL the april tags!");
            } else {
                ROS_INFO("The robot found only %ld out of %ld april tags", result->poses.size(),  poses.size());
            }

            for(auto const& pose : result->poses) {
                ROS_INFO("Pose in world frame: Position(x=%f, y=%f, z=%f), Orientation(x=%f, y=%f, z=%f, w=%f)",
                    pose.position.x,
                    pose.position.y,
                    pose.position.z,
                    pose.orientation.x,
                    pose.orientation.y,
                    pose.orientation.z,
                    pose.orientation.w);
            }

            ros::shutdown();
        }

        // This is node used since the action server will feedback when the robot receive the goal.
        void activeCallback() {}

        /**
        * Action client feedback callback. Gets called when the action server publish a feedback.
        *
        * @param feedback Action feedback message, containing the status of the robot (string).
        */
        void feedbackCallback(const assignment1::SearchIdsFeedbackConstPtr &feedback) {
            ROS_INFO("Feedback from the action server: %s", feedback->status.c_str());
        }
};



int main(int argc, char **argv) {
    // Initialize node
    ros::init(argc, argv, "action_client");

    ros::NodeHandle nh;
    ros::ServiceClient id_client = nh.serviceClient<tiago_iaslab_simulation::Objs>("apriltag_ids_srv");

    tiago_iaslab_simulation::Objs srv;
    srv.request.ready = true;

    // Calling the service and getting the response
    if (!id_client.call(srv)) {
        ROS_ERROR("Failed to call service /apriltags_ids_srv");
        return -1;
    }

    ROS_INFO("Service call succeeded.");

    ActionClient client(srv.response.ids);

    ros::spin();
    return 0;
}
