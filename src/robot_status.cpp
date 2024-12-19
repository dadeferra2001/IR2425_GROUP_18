#include <assignment1/robot_status.h>

const char* statusToString(ROBOT_STATUS status)
{
    switch (status) {
        case GOAL_RECEIVED:
            return "Robot received the goal.";
        case GENERATING_WAYPOINTS:
            return "Robot is generating the waypoints.";
        case MOVING:
            return "Robot is moving towards a goal waypoint.";
        case SCANNING:
            return "Robot has reached the goal waypoint and is performing a 360 degrees scan.";
        case SCAN_COMPLETE:
            return "Robot finished scanning.";
        case MOVE_FAILED:
            return "Robot was not able to reach the current goal waypoint.";
        case NAVIGATION_COMPLETED:
            return "Robot finished navigating all the waypoints.";
        case GOAL_REACHED:
            return "Robot completed succesfully its goal";
        case GOAL_FAILED:
            return "Robot could not find all the tags.";
        case TAG_FOUND:
            return "Robot found a target april tag.";
        case TILT_HEAD:
            return "Robot is tilting the head.";
        default:
            return "Unknown status.";
    }
}
