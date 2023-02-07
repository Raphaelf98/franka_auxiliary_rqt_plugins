#include "franka_reset/reset_button_plugin.h"
#include <pluginlib/class_list_macros.h>
#include <QStringList>
#include <string.h>
namespace franka_reset
{

ResetButtonPlugin::ResetButtonPlugin()
  : rqt_gui_cpp::Plugin()
  , widget_(0)
{
  // Constructor is called first before initPlugin function, needless to say.
  setObjectName("ResetButtonPlugin");
}

void ResetButtonPlugin::initPlugin(qt_gui_cpp::PluginContext& context)
{
  // access standalone command line arguments
  QStringList argv = context.argv();
  // create QWidget
  widget_ = new QWidget();
  // extend the widget with all attributes and children from UI file
  ui_.setupUi(widget_);
  nh_ = rqt_gui_cpp::Plugin::getNodeHandle();
  // add widget to the user interface
  context.addWidget(widget_);

  rostimer = new QTimer(this);
  connect(rostimer, SIGNAL(timeout()), this ,SLOT(spinOnce()));

  rostimer->start(100);

  connect(ui_.pushButton, SIGNAL(clicked()), this, SLOT(on_pushButton_clicked()));


  //subscribe to /franka_state_controller/franka_states
  sub_ = nh_.subscribe<franka_msgs::FrankaState>("/franka_state_controller/franka_states", 1, &ResetButtonPlugin::frankaErrorStateCallback, this);

  //initialize Error recovery publisher
  std::string errorRecoveryTopic = "/franka_control/error_recovery/goal";
  pub_ = nh_.advertise<franka_msgs::ErrorRecoveryActionGoal>(errorRecoveryTopic,1);
  error_ = false;
  set_text_ = false;
  pal = ui_.pushButton->palette();
  user_stop_reset = false;
  user_stop_error = false;
}

void ResetButtonPlugin::shutdownPlugin()
{
  // unregister all publishers here
  sub_.shutdown();
  pub_.shutdown();
}

void ResetButtonPlugin::saveSettings(qt_gui_cpp::Settings& plugin_settings,
    qt_gui_cpp::Settings& instance_settings) const
{
  // instance_settings.setValue(k, v)
}

void ResetButtonPlugin::restoreSettings(const qt_gui_cpp::Settings& plugin_settings,
    const qt_gui_cpp::Settings& instance_settings)
{
  // v = instance_settings.value(k)
}

/*bool hasConfiguration() const
{
  return true;
}

void triggerConfiguration()
{
  // Usually used to open a dialog to offer the user a set of configuration
}*/
void ResetButtonPlugin::spinOnce(){
  if (ros::ok()){
    ros::spinOnce();
  }
  else{
   QApplication::quit();
  }
}
void ResetButtonPlugin::chatterCallback(const std_msgs::String::ConstPtr &msg)
{
  auto qstring_msg = QString::fromStdString(msg->data.c_str());
  ui_.status->setText(qstring_msg);
}
void ResetButtonPlugin::frankaErrorStateCallback(const franka_msgs::FrankaState::ConstPtr &msg)
{
  //auto qstring_msg = "Error received";
  //ui_.status->setText(qstring_msg);
  current_franka_state_constptr_ = msg;
  uint8_t counter = 0;
  uint8_t err_count = 0;

  for (auto it = &current_franka_state_constptr_->current_errors.joint_position_limits_violation;
       it != &current_franka_state_constptr_->current_errors.base_acceleration_invalid_reading;
       ++it, ++counter)
  {

      if (*it)
      {
          //TODO: retrieve specific error message from class last_motion_errors
          //std::string err = std::to_string(current_franka_state_constptr_->last_motion_errors);

          ROS_INFO("ERROR DETECTED IN RQT PLUGIN");
          err_count++;
          error_ = true;
      }
  }
  //robot mode 5 corresponds to user stop.

  if(std::to_string(current_franka_state_constptr_->robot_mode) == "5")
  {
      ROS_WARN("USER STOP PRESSED");
      if (!set_text_)
      {
          ui_.pushButton->setText("RELEASE USER STOP");

          set_text_=true;
          pal.setColor(QPalette::Button, QColor(Qt::red));
          ui_.pushButton->setAutoFillBackground(true);
          ui_.pushButton->setPalette(pal);
          ui_.pushButton->update();
          ui_.status->setText("User stop error received");
      }
      user_stop_reset = false;
      user_stop_error = true;
  }
  if (std::to_string(current_franka_state_constptr_->robot_mode) != "5" && !user_stop_reset)
  {

        ui_.pushButton->setText("RESET");
        set_text_=true;
        pal.setColor(QPalette::Button, QColor(Qt::red));
        ui_.pushButton->setAutoFillBackground(true);
        ui_.pushButton->setPalette(pal);
        ui_.pushButton->update();


    user_stop_reset = true;
  }

  else if (error_ && !set_text_)
  {
      ui_.pushButton->setText("RESET");
      set_text_=true;
      pal.setColor(QPalette::Button, QColor(Qt::red));
      ui_.pushButton->setAutoFillBackground(true);
      ui_.pushButton->setPalette(pal);
      ui_.pushButton->update();
      ui_.status->setText("Franka error state receieved");
  }



}
void ResetButtonPlugin::on_pushButton_clicked()
{
    if(error_)
    {
        recoverFromErrorState();
        error_ = false;
        ROS_INFO("reset");
        change_pushButton_color();
    }

    if (user_stop_error)
    {
        recoverFromErrorState();
        ROS_INFO("reset");
        user_stop_error = false;
        change_pushButton_color();
    }

}

void ResetButtonPlugin::change_pushButton_color()
{

    pal.setColor(QPalette::Button, QColor(Qt::green));
    ui_.pushButton->setAutoFillBackground(true);
    ui_.pushButton->setPalette(pal);
    ui_.pushButton->update();
}
void ResetButtonPlugin::recoverFromErrorState()
{
    //TODO: publish
   franka_msgs::ErrorRecoveryActionGoal goal;
   pub_.publish(goal);

   ui_.pushButton->setText("OK");

   ui_.status->setText("OK");
   set_text_ = false;
}
}  // namespace rqt_example_cpp
PLUGINLIB_EXPORT_CLASS(franka_reset::ResetButtonPlugin, rqt_gui_cpp::Plugin)

