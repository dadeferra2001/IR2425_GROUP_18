#include <assignment1/robot_status.h>

const char* statusToString(ROBOT_STATUS status)
{
    switch (status) {
        case REQUEST_CANCELED:
            return "Cancel request has been received.";
        case GOAL_UNREACHABLE:
            return "Goal cannot be reached, robot has aborted the operations.";
        case GOAL_RECEIVED:
            return "Goal has been received, transmitting to the robot.";
        case INSIDE_NARROW:
            return "Moving through the narrow passage";
        case OUTSIDE_NARROW:
            return "Robot has left the narrow passage";
        case ROBOT_MOVING:
            return "Robot is moving to reach the desired pose.";
        case ROBOT_REACHED:
            return "Robot has reached the desired pose.";
        case ROBOT_SCANNING:
            return "Robot is scanning the environment to detect the movable obstacles.";
        case ROBOT_COMPLETED_DETECTION:
            return "Obstacle detection has been completed, sending the results back.";
        case ROBOT_MOVING_NARROW:
            return "Going through narrow passage.";
        default:
            return "Unknown status.";
    }
}
