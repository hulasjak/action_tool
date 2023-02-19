#ifndef ACTION_MONITOR_HPP_
#define ACTION_MONITOR_HPP_

#include <ros/ros.h>
#include "std_msgs/String.h"
#include <actionlib_msgs/GoalStatusArray.h>
#include <action_tool/ActionInfoArray.h>
#include <chrono>

class ActionMonitor
{
public:
    std::string name_;

    action_tool::ActionInfo get_msg() const;
    void callback(const actionlib_msgs::GoalStatusArray &status);

private:
    double average_duration_{0.0};
    double max_duration_{0.0};
    double min_duration_{0.0};

    bool active_{false};
    int number_of_calls_{0};
    int number_of_errors_{0};
    std::chrono::steady_clock::time_point time_of_start_;

    void update_durations();
};

#endif
