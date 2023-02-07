
#ifndef FRANKA_RESET_RESET_BUTTON_PLUGIN_H
#define FRANKA_RESET_RESET_BUTTON_PLUGIN_H

#include "ros/ros.h"
#include <rqt_gui_cpp/plugin.h>
#include <franka_reset/ui_reset_button_plugin.h>
#include <QWidget>
#include<std_msgs/String.h>
#include<qtimer.h>

#include<franka_msgs/FrankaState.h>
#include<franka_msgs/ErrorRecoveryActionGoal.h>

namespace franka_reset
{

class ResetButtonPlugin
  : public rqt_gui_cpp::Plugin
{
  Q_OBJECT
public:
  ResetButtonPlugin();
  virtual void initPlugin(qt_gui_cpp::PluginContext& context);
  virtual void shutdownPlugin();
  virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings,
      qt_gui_cpp::Settings& instance_settings) const;
  virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings,
      const qt_gui_cpp::Settings& instance_settings);

public slots:
  void spinOnce();

  // Comment in to signal that the plugin has a way to configure it
  // bool hasConfiguration() const;
  // void triggerConfiguration();
private slots:
  void on_pushButton_clicked();

private:
  void chatterCallback(const std_msgs::String::ConstPtr &msg);
  void frankaErrorStateCallback(const franka_msgs::FrankaState::ConstPtr &msg);
  void recoverFromErrorState();

  void change_pushButton_color();
  Ui::ResetButtonPluginWidget ui_;
  QWidget* widget_;
  QPalette pal;
  franka_msgs::FrankaState::ConstPtr current_franka_state_constptr_;

  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  ros::Publisher pub_;
  QTimer *rostimer;

  bool set_text_;
  bool error_;
  bool user_stop_reset;
  bool user_stop_error;

};

}  // namespace franka_reset
#endif  // FRANKA_RESET_BUTTON_PLUGIN_H
