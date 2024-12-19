#include <ros/ros.h>
#include <vector>
#include <map>
#include <algorithm>
#include <geometry_msgs/Point.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <assignment1/SearchIdsAction.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <assignment1/functions.h>
#include <assignment1/robot_status.h>


class ActionServer {
    protected:

        ROBOT_STATUS robot_status_;

        ros::NodeHandle nh_;
        std::string action_name_;

        actionlib::SimpleActionServer<assignment1::SearchIdsAction> as_;
        actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> movebase_client_;

        std::vector<int> target_ids;
        std::map<int, geometry_msgs::Point> found_ids;

        ros::Subscriber tag_detections;
        ros::Publisher vel_pub;

    public:

        ActionServer(std::string name) : action_name_(name), as_(nh_, name, boost::bind(&ActionServer::executeCB, this, _1), false), movebase_client_("/move_base", true) {
            
            tiltHeadDown(-0.6);

            ROS_INFO("Starting the 'SearchIds' action server!");
            as_.start();
        }

        void updateStatus(ROBOT_STATUS status) {
            robot_status_ = status;
            
            assignment1::SearchIdsFeedback feedback;
            feedback.status = statusToString(status);
            as_.publishFeedback(feedback);
        }


        std::vector<geometry_msgs::Point> generateWaypoints() {
            std::vector<geometry_msgs::Point> waypoints;

            geometry_msgs::Point p1;
            p1.x = 2.6516246795654297;
            p1.y = 0.08407711982727051;
            waypoints.push_back(p1);

            geometry_msgs::Point p2;
            p2.x = 5.743946075439453;
            p2.y = 0.14989829063415527;
            waypoints.push_back(p2);

            geometry_msgs::Point p3;
            p3.x = 9.689477920532227;
            p3.y = 0.7225534915924072;
            waypoints.push_back(p3);

            geometry_msgs::Point p4;
            p4.x = 12.680120468139648;
            p4.y = 1.0551815032958984;
            waypoints.push_back(p4);

            geometry_msgs::Point p5;
            p5.x = 10.68838119506836;
            p5.y = -0.6969592571258545;
            waypoints.push_back(p5);

            geometry_msgs::Point p6;
            p6.x = 12.999018669128418;
            p6.y = -1.910409688949585;
            waypoints.push_back(p6);

            geometry_msgs::Point p7;
            p7.x = 12.88753890991211;
            p7.y = -3.583296537399292;
            waypoints.push_back(p7);

            geometry_msgs::Point p8;
            p8.x = 10.769646644592285;
            p8.y = -3.632005453109741;
            waypoints.push_back(p8);

            geometry_msgs::Point p9;
            p9.x = 8.05963134765625;
            p9.y = -3.894465208053589;
            waypoints.push_back(p9);

            geometry_msgs::Point p10;
            p10.x = 8.133699417114258;
            p10.y = -1.7906239032745361;
            waypoints.push_back(p10);

            geometry_msgs::Point p11;
            p11.x = -0.2616691589355469;
            p11.y = -0.7500753402709961;
            waypoints.push_back(p11);

            geometry_msgs::Point p12;
            p12.x = 0;
            p12.y = 0;
            waypoints.push_back(p12);

            return waypoints;
        }

        void navigateWaypoints(const std::vector<geometry_msgs::Point>& waypoints) {

            while (!movebase_client_.waitForServer(ros::Duration(5.0))) {
                ROS_INFO("Waiting for the move_base action server to come up");
            }

            for (size_t i = 0; i < waypoints.size(); ++i) {
                // Prepare a goal message
                move_base_msgs::MoveBaseGoal goal;
                goal.target_pose.header.frame_id = "map"; // Use the map frame
                goal.target_pose.header.stamp = ros::Time::now();
                goal.target_pose.pose.position = waypoints[i];
                goal.target_pose.pose.orientation.w = 1.0;

                // Send the goal to move_base
                ROS_INFO("Navigating to waypoint %lu: [x=%.2f, y=%.2f, z=%.2f]", i + 1, waypoints[i].x, waypoints[i].y, waypoints[i].z);
                movebase_client_.sendGoal(goal);

                // Wait for the robot to reach the goal
                bool finished_before_timeout = movebase_client_.waitForResult(ros::Duration(30.0)); // Adjust timeout as needed

                if (finished_before_timeout) {
                    if (movebase_client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
                        ROS_INFO("Reached waypoint %lu.", i + 1);
                    } else {
                        ROS_WARN("Failed to reach waypoint %lu. State: %s", i + 1, movebase_client_.getState().toString().c_str());
                    }
                } else {
                    ROS_ERROR("Timed out while trying to reach waypoint %lu.", i + 1);
                }
            }

            ROS_INFO("Finished navigating all waypoints.");
        }

        void executeCB(const assignment1::SearchIdsGoalConstPtr &goal) {
            
            target_ids = goal->ids;
            ROS_INFO("Received target IDs: [%s]", vectorToCSV(target_ids).c_str());

            tag_detections = nh_.subscribe("/tag_detections", 1000, &ActionServer::tagDetectionsCallback, this);

            ROS_INFO("Generating waypoints...");
            std::vector<geometry_msgs::Point> waypoints = generateWaypoints();

            navigateWaypoints(waypoints);

            ROS_INFO("Target IDs: %s", vectorToCSV(target_ids).c_str());
            ROS_INFO("Found IDs: %s", vectorToCSV(getKeys(found_ids)).c_str());
        
            assignment1::SearchIdsResult result;
            if(isJobCompleted()){
                result.completed = true;
                result.positions = getValues(found_ids);
                ROS_INFO("Succeeded");
            } else {
                result.completed = false;
                result.positions = {};
                ROS_INFO("Succeeded");
            }

            as_.setSucceeded(result);
        }

        void tagDetectionsCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr &detections) {
                for(const auto& detection : detections->detections){
                    int id = detection.id[0];

                    // Check if the tag was already found and if the tag is target
                    if(!found_ids.count(id) && std::find(target_ids.begin(), target_ids.end(), id) != target_ids.end()){
                        ROS_INFO("Found april tag with id %d", id);
                        found_ids.insert(std::pair<int, geometry_msgs::Point>(id, detection.pose.pose.pose.position));
                    } 
                }
        }

        bool isJobCompleted() {
            for (int target_id : target_ids) {
                if (found_ids.find(target_id) == found_ids.end()) {
                    return false;
                }
            }

            return true;
        }

        void tiltHeadDown(double tilt_angle) {
            ROS_INFO("Tilting the head");

            ros::Publisher head_pub = nh_.advertise<trajectory_msgs::JointTrajectory>("/head_controller/command", 10);
            ros::Duration(0.5).sleep();

            trajectory_msgs::JointTrajectory head_cmd;

            head_cmd.header.stamp = ros::Time::now();
            head_cmd.joint_names.push_back("head_1_joint");
            head_cmd.joint_names.push_back("head_2_joint");

            trajectory_msgs::JointTrajectoryPoint head_cmd_point;
            head_cmd_point.positions = {0.0, tilt_angle};
            head_cmd_point.time_from_start = ros::Duration(1.0);

            head_cmd.points.push_back(head_cmd_point);

            head_pub.publish(head_cmd);
            ros::Duration(1).sleep();
        }
};


int main(int argc, char** argv) {
  ros::init(argc, argv, "action_server");

  ActionServer searchIds("SearchIds");
  ros::spin();

  return 0;
}
