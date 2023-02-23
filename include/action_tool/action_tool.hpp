#ifndef ACTION_TOOL_HPP_
#define ACTION_TOOL_HPP_

#include <ros/ros.h>
#include <rqt_gui_cpp/plugin.h>
#include <action_tool/ui_action_tool.h>
#include <action_tool/ActionInfoArray.h>
#include <QWidget>
#include <QTableView>
#include <QItemDelegate>
#include <QStandardItemModel>
#include <thread>
#include <actionlib_msgs/GoalStatusArray.h>
#include <std_msgs/String.h>
#include <mutex>
#include <std_msgs/Bool.h>

namespace action_tool
{
    class ActionTool : public rqt_gui_cpp::Plugin
    {
        Q_OBJECT
    public:
        ActionTool();
        virtual void initPlugin(qt_gui_cpp::PluginContext &context);
        virtual void shutdownPlugin();
        virtual void saveSettings(
            qt_gui_cpp::Settings &plugin_settings,
            qt_gui_cpp::Settings &instance_settings) const;
        virtual void restoreSettings(
            const qt_gui_cpp::Settings &plugin_settings,
            const qt_gui_cpp::Settings &instance_settings);

        // Comment in to signal that the plugin has a way to configure it
        // bool hasConfiguration() const;
        // void triggerConfiguration();

    private:
        Ui::action_tool ui_;
        std::shared_ptr<QWidget> widget_;
        // std::shared_ptr<QStandardItemModel> model_;

    protected:
        ros::Subscriber _subscriber;
        void callback(const action_tool::ActionInfoArray &status);
    };
} // action_tool

#endif