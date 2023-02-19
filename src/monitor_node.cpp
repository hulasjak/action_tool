#include <ros/ros.h>
#include "std_msgs/String.h"
#include <actionlib_msgs/GoalStatusArray.h>
#include <action_tool/ActionInfoArray.h>
#include <sstream>
#include <algorithm>
#include <action_tool/action_monitor.hpp>

std::vector<std::string> get_active_actions()
{
    static std::string status_string{"/status"};
    std::vector<std::string> actions;
    ros::master::V_TopicInfo topics;
    ros::master::getTopics(topics);

    for (ros::master::V_TopicInfo::iterator it = topics.begin(); it != topics.end(); it++)
    { /* code */

        const ros::master::TopicInfo &info = *it;
        if (std::string::npos != info.name.find(status_string))
        {
            actions.push_back(info.name);
            // std::cout << "action " << info.name << "\n";
        }
    }
    return actions;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "action_tool");
    ros::NodeHandle nh;
    std::map<std::string, ActionMonitor> monitors;

    std::vector<std::string> started_actions, not_stared_actions;

    std::vector<ros::Subscriber> subs;
    ros::Rate loop_rate(10);
    ros::Publisher pub = nh.advertise<action_tool::ActionInfoArray>("/action_monitor", 1000);
    action_tool::ActionInfoArray msg;

    while (ros::ok())
    {
        auto actions = get_active_actions();

        std::set_difference(actions.begin(), actions.end(), started_actions.begin(), started_actions.end(),
                            std::back_inserter(not_stared_actions));

        if (not_stared_actions.size() > 0)
        {
            std::cout << "not started action " << not_stared_actions[0] << "\n";
            std::cout << "action1 " << actions[0] << "\n";
            std::cout << "true? " << std::to_string(actions[0] == not_stared_actions[0]) << "\n";
        }
        for (auto &&action : not_stared_actions)
        {
            ActionMonitor new_monitor;
            new_monitor.name_ = action;
            new_monitor.name_.erase(new_monitor.name_.find("/status"));
            monitors.insert({new_monitor.name_, new_monitor});
            subs.push_back(nh.subscribe(action, 1, &ActionMonitor::callback, &monitors[new_monitor.name_]));
            started_actions.push_back(action);
            std::cout << "added action " << action << "\n";
        }

        for (auto &&action : monitors)
        {
            msg.action_info_list.push_back(action.second.get_msg());
        }
        pub.publish(msg);

        msg.action_info_list.clear();
        not_stared_actions.clear();

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
