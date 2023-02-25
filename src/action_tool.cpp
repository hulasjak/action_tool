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

        _subscriber = getNodeHandle().subscribe("/action_monitor", 1, &ActionTool::callback, this);

        ui_.table->verticalHeader()->setVisible(false);
        ui_.table->setSortingEnabled(false);
        ui_.table->setShowGrid(false);
        ui_.table->setColumnCount(7);
        ui_.table->setRowCount(1);
        QStringList horizontalHeader;
        horizontalHeader.append({"Actions", "Active", "Num. succeced", "Num. of errors", "Average duration", "Min duration", "Max duration"});
        ui_.table->setHorizontalHeaderLabels(horizontalHeader);
        ui_.table->resizeColumnsToContents();
        ui_.table->setEditTriggers(QAbstractItemView::NoEditTriggers);
        ui_.table->setSelectionBehavior(QAbstractItemView::SelectRows);
    }

    void ActionTool::shutdownPlugin()
    {
    }

    void ActionTool::saveSettings(
        qt_gui_cpp::Settings &plugin_settings,
        qt_gui_cpp::Settings &instance_settings) const
    {
    }

    void ActionTool::restoreSettings(
        const qt_gui_cpp::Settings &plugin_settings,
        const qt_gui_cpp::Settings &instance_settings)
    {
    }

    void ActionTool::callback(const action_tool::ActionInfoArray &info)
    {

        if (ui_.table->rowCount() < info.action_info_list.size())
        {
            ui_.table->insertRow(ui_.table->rowCount());
        }
        int row = 0;
        for (auto &&action_info : info.action_info_list)
        {
            int col = 0;

            ui_.table->setItem(row, col++, new QTableWidgetItem(action_info.name.c_str()));
            // Create an element, which will serve as a checkbox
            QTableWidgetItem *item = new QTableWidgetItem();
            item->data(Qt::CheckStateRole);
            item->setFlags(item->flags() & ~Qt::ItemIsEditable);
            if (action_info.active)
            {
                item->setCheckState(Qt::Checked);
            }
            else
            {
                item->setCheckState(Qt::Unchecked);
            }
            ui_.table->setItem(row, col++, item);
            ui_.table->setItem(row, col++, new QTableWidgetItem(std::to_string(action_info.num_of_calls).c_str()));
            ui_.table->setItem(row, col++, new QTableWidgetItem(std::to_string(action_info.num_of_errors).c_str()));
            ui_.table->setItem(row, col++, new QTableWidgetItem(std::to_string(action_info.average_duration).c_str()));
            ui_.table->setItem(row, col++, new QTableWidgetItem(std::to_string(action_info.min_duration).c_str()));
            ui_.table->setItem(row, col++, new QTableWidgetItem(std::to_string(action_info.max_duration).c_str()));
            ++row;
        }
    }

} // namespace action_tool
