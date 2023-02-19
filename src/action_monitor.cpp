#include <algorithm>

#include <action_tool/action_monitor.hpp>

void ActionMonitor::update_durations()
{
    auto duration = static_cast<double>(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - time_of_start_).count()) / 1000;
    average_duration_ = (average_duration_ * (number_of_calls_ - 1) + duration) / number_of_calls_;
    max_duration_ = std::max(max_duration_, duration);
    if (min_duration_ < 0.001)
    {
        min_duration_ = duration;
    }
    min_duration_ = std::min(min_duration_, duration);
}

action_tool::ActionInfo ActionMonitor::get_msg() const
{
    action_tool::ActionInfo msg;
    msg.name = name_;
    msg.average_duration = average_duration_;
    msg.max_duration = max_duration_;
    msg.min_duration = min_duration_;

    msg.num_of_calls = number_of_calls_;
    msg.num_of_errors = number_of_errors_;
    msg.active = active_;
    return msg;
};

void ActionMonitor::callback(const actionlib_msgs::GoalStatusArray &status)
{
    if (status.status_list.size() > 0)
    {
        switch (status.status_list.back().status)
        {
        case actionlib_msgs::GoalStatus::ACTIVE:
            if (!active_)
            {
                active_ = true;
                time_of_start_ = std::chrono::steady_clock::now();
            }
            break;

        case actionlib_msgs::GoalStatus::SUCCEEDED:
            if (active_)
            {
                active_ = false;
                ++number_of_calls_;
                update_durations();
                std::cout << "num of calls" << number_of_calls_ << "\n";
            }
            break;

        case actionlib_msgs::GoalStatus::ABORTED:
            if (active_)
            {
                active_ = false;
                ++number_of_errors_;
                std::cout << "num of errors" << number_of_errors_ << "\n";
            }
            break;

        default:
            break;
        }
    }
};