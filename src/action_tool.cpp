#include "action_tool/action_tool.hpp"
#include <pluginlib/class_list_macros.h>

#include <QStringList>
#include <ros/master.h>

PLUGINLIB_EXPORT_CLASS(action_tool::ActionTool, rqt_gui_cpp::Plugin);

namespace action_tool
{
    ActionTool::ActionTool()
        : rqt_gui_cpp::Plugin(),
          widget_(0)
    {
        // Constructor is called first before initPlugin function, needless to say.
        // give QObjects reasonable names
        setObjectName("action_tool");
    }

    void ActionTool::initPlugin(qt_gui_cpp::PluginContext &context)
    {
        // Access standalone command line arguments
        QStringList argv = context.argv();

        // Create QWidget
        widget_ = std::make_shared<QWidget>();
        ui_.setupUi(widget_.get());

        // add widget to the user interface
        context.addWidget(widget_.get());

        // Define ROS publishers
        // buttton_1_pub_ = getNodeHandle().advertise<std_msgs::Bool>("button_1_topic", 1);
        // buttton_2_pub_ = getNodeHandle().advertise<std_msgs::Bool>("button_2_topic", 1);

        // // Declare ROS msg_
        // msg_.data = true;
        _subscriber = getNodeHandle().subscribe("/action_monitor", 1, &ActionTool::callback, this);
        // Connect Qt Widgets
        // connect(ui_.helloButton, SIGNAL(pressed()), this, SLOT(button_callback_()));
        // connect(ui_.pushButton_2, SIGNAL(pressed()), this, SLOT(button_2_callback_()));

        // QStandardItemModel(int rows, int columns, QObject * parent = 0)
        model_ = std::make_shared<QStandardItemModel>(1, 4, this);

        // Attach the model to the view

        ui_.tableView->verticalHeader()->setVisible(false);
        ui_.tableView->setSortingEnabled(true);
        ui_.tableView->setShowGrid(false);
        QStringList horizontalHeader;
        horizontalHeader.append({"Actions", "Active", "Num. succeced", "Num. of errors", "Average duration", "Min duration", "Max duration"});
        model_->setHorizontalHeaderLabels(horizontalHeader);
        ui_.tableView->resizeColumnsToContents();
        ui_.tableView->setEditTriggers(QAbstractItemView::NoEditTriggers);
        ui_.tableView->setModel(model_.get());
    }

    void ActionTool::shutdownPlugin()
    {
    }

    void ActionTool::saveSettings(
        qt_gui_cpp::Settings &plugin_settings,
        qt_gui_cpp::Settings &instance_settings) const
    {
        ;
    }

    void ActionTool::restoreSettings(
        const qt_gui_cpp::Settings &plugin_settings,
        const qt_gui_cpp::Settings &instance_settings)
    {
        ;
    }

    void ActionTool::callback(const action_tool::ActionInfoArray &info)
    {
        // const std::lock_guard<std::mutex> lock(mutex);
        // name = info.action_info_list[0].name;
        // QModelIndex index = model_->index(0, 0, QModelIndex());
        // model_->setData(index, name.c_str());
        // auto model = std::make_shared<QStandardItemModel>(1, 4, this);
        // ui_.tableView->setModel(model.get());
        // QString name;
        // name.resize(info.action_info_list[0].name.size());
        // std::copy(info.action_info_list[0].name.begin(), info.action_info_list[0].name.end(), name.begin());

        if (model_->rowCount() < info.action_info_list.size())
        {
            model_->insertRow(model_->rowCount(QModelIndex()));
        }
        int row = 0;
        for (auto &&action_info : info.action_info_list)
        {
            int col = 0;
            QModelIndex index = model_->index(row, col++, QModelIndex());
            auto name = action_info.name.c_str();
            model_->setData(index, name);
            index = model_->index(row, col++, QModelIndex());
            model_->setData(index, action_info.active);
            index = model_->index(row, col++, QModelIndex());
            model_->setData(index, action_info.num_of_calls);
            index = model_->index(row, col++, QModelIndex());
            model_->setData(index, action_info.num_of_errors);
            index = model_->index(row, col++, QModelIndex());
            model_->setData(index, action_info.average_duration);
            index = model_->index(row, col++, QModelIndex());
            model_->setData(index, action_info.min_duration);
            index = model_->index(row, col++, QModelIndex());
            model_->setData(index, action_info.max_duration);
            ++row;
        }

        // QModelIndex index = model_->index(0, 0, QModelIndex());
        // 0 for all data
    }

} // namespace action_tool
