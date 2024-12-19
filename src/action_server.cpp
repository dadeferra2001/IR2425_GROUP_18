#include <ros/ros.h>
#include <vector>
#include <map>
#include <algorithm>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <assignment1/SearchIdsAction.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <assignment1/functions.h>
#include <assignment1/robot_status.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class ActionServer {
    protected:

        ROBOT_STATUS robot_status_;

        ros::NodeHandle nh_;
        std::string action_name_;

        actionlib::SimpleActionServer<assignment1::SearchIdsAction> as_;
        actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> movebase_client_;

        std::vector<int> target_ids;
        std::map<int, geometry_msgs::Pose> found_ids;

        ros::Subscriber tag_detections;
        ros::Publisher vel_pub;
        ros::Publisher cmd_vel_pub;

    public:

        ActionServer(std::string name) : action_name_(name), as_(nh_, name, boost::bind(&ActionServer::executeCB, this, _1), false), movebase_client_("/move_base", true) {
            
            tiltHeadDown(-0.6);

            ROS_INFO("Starting the 'SearchIds' action server!");
            as_.start();

            cmd_vel_pub = nh_.advertise<geometry_msgs::Twist>("/mobile_base_controller/cmd_vel", 10);
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

        bool moveToPoint(const geometry_msgs::Point& point) {
            move_base_msgs::MoveBaseGoal goal;
            goal.target_pose.header.frame_id = "map";
            goal.target_pose.header.stamp = ros::Time::now();

            goal.target_pose.pose.position = point;
            goal.target_pose.pose.orientation.w = 1.0;

            ROS_INFO("Navigating to: [x=%.2f, y=%.2f, z=%.2f]", point.x, point.y, point.z);

            movebase_client_.sendGoal(goal);

            bool finished_before_timeout = movebase_client_.waitForResult(ros::Duration(30.0)); // Adjust timeout as needed

            if (finished_before_timeout) {
                if (movebase_client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
                    ROS_INFO("Reached waypoint, performing a full 360-degree spin...");

                    geometry_msgs::Twist spin_msg;
                    spin_msg.linear.x = 0.0;
                    spin_msg.linear.y = 0.0;
                    spin_msg.linear.z = 0.0;

                    spin_msg.angular.x = 0.0;
                    spin_msg.angular.y = 0.0;
                    spin_msg.angular.z = 1.0;

                    ros::Rate rate(10);
                    double duration = 2 * M_PI / fabs(spin_msg.angular.z);

                    ros::Time start_time = ros::Time::now();
                    while (ros::Time::now() - start_time < ros::Duration(duration)) {
                        cmd_vel_pub.publish(spin_msg);
                        rate.sleep();
                    }

                    spin_msg.angular.z = 0.0;
                    cmd_vel_pub.publish(spin_msg);

                    ROS_INFO("Completed full spin.");
                    return true;
                } else {
                    ROS_WARN("Failed to reach waypoint.");
                }
            } else {
                ROS_WARN("Timed out while trying to reach waypoint.");
            }

            return false;
        }


        void navigateWaypoints(const std::vector<geometry_msgs::Point>& waypoints) {

            while (!movebase_client_.waitForServer(ros::Duration(5.0))) {
                ROS_INFO("Waiting for the move_base action server to come up");
            }

            for (size_t i = 0; i < waypoints.size() && !isJobCompleted(); ++i) {
                moveToPoint(waypoints[i]);
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
            result.poses = getValues(found_ids);
            if(isJobCompleted()){
                result.completed = true;
                ROS_INFO("Succeeded");
            } else {
                result.completed = false;
                ROS_INFO("Failed");
            }

            as_.setSucceeded(result);
        }

        void tagDetectionsCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr &detections) {
                for(const auto& detection : detections->detections){
                    int id = detection.id[0];

                    // Check if the tag was already found and if the tag is target
                    if(!found_ids.count(id) && std::find(target_ids.begin(), target_ids.end(), id) != target_ids.end()){
                        ROS_INFO("Found april tag with id %d", id);

                        tf2_ros::Buffer tf(ros::Duration(1.0));
                        tf2_ros::TransformListener tfListener(tf);

                        geometry_msgs::PoseStamped pose_camera_frame;

                        pose_camera_frame.header.frame_id = "xtion_rgb_optical_frame";
                        pose_camera_frame.header.stamp = ros::Time(0);
                        pose_camera_frame.pose = detection.pose.pose.pose;

                        geometry_msgs::PoseStamped pose_base_frame;
                        tf.transform(pose_camera_frame, pose_base_frame, "base_link", ros::Duration(1.0));
                        
                        pose_base_frame.header.stamp = ros::Time(0);
                        geometry_msgs::PoseStamped pose_map_frame;
                        tf.transform(pose_base_frame, pose_map_frame, "map", ros::Duration(1.0));
                        
                        ROS_INFO("Transformed the pose to map frame");

                        found_ids.insert(std::pair<int, geometry_msgs::Pose>(id, pose_map_frame.pose));
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
