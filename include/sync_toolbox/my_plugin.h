#ifndef SYNC_TOOLBOX_MY_PLUGIN_H
#define SYNC_TOOLBOX_MY_PLUGIN_H

#include <rqt_gui_cpp/plugin.h>
#include <sync_toolbox/ui_my_plugin.h>
#include <QWidget>
#include <QSerialPort>
#include <QSerialPortInfo>
#include <QDebug>
#include <string>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <thread>

namespace sync_toolbox
{
#define PORT_BUFFER_SIZE 1000
  enum Channel
  {
    channel_1,
    channel_2,
    channel_3,
    channel_4,
    clock_channel,
    start_indicator
  };
  class MyPlugin
      : public rqt_gui_cpp::Plugin
  {
    Q_OBJECT
  public:
    MyPlugin();
    ~MyPlugin()
    {
      serialPort->close();
      start_record_ = false;
      if (t_ != nullptr)
        t_->join();
    }
    virtual void initPlugin(qt_gui_cpp::PluginContext &context);
    virtual void shutdownPlugin();
    virtual void saveSettings(qt_gui_cpp::Settings &plugin_settings,
                              qt_gui_cpp::Settings &instance_settings) const;
    virtual void restoreSettings(const qt_gui_cpp::Settings &plugin_settings,
                                 const qt_gui_cpp::Settings &instance_settings);

    // Comment in to signal that the plugin has a way to configure it
    // bool hasConfiguration() const;
    // void triggerConfiguration();
  private slots:
    void on_StartButton_clicked();
    void on_SearchButton_clicked();
    void on_ConfigButton_clicked();

    void readyReadSlot();

    void pulse_width_closed_1(int i);
    void pulse_width_closed_2(int i);
    void pulse_width_closed_3(int i);
    void pulse_width_closed_4(int i);
    void pulse_width_closed_c(int i);
    void pulse_width_closed_s(int i);

    void on_Stopfreq_1_enabled(bool);
    void on_Stopfreq_2_enabled(bool);
    void on_Stopfreq_3_enabled(bool);
    void on_Stopfreq_4_enabled(bool);

    void on_1_enabled(bool);
    void on_2_enabled(bool);
    void on_3_enabled(bool);
    void on_4_enabled(bool);
    void on_cl_enabled(bool);
    void on_si_enabled(bool);
    void on_bag_export_enabled(bool);
    void on_online_refactor_enabled(bool);

  private:
    Ui::MyPluginWidget ui_;
    QWidget *widget_;
    QSerialPort *serialPort;
    boost::shared_ptr<std::thread> t_;

    bool config_microcontroller = true;
    bool start_record_ = false;

    std::string config_;

  private:
    void EncoderSetting(void);
    void test(int);
    void PrintProtocal(std::stringstream &, int, int, int, int, int);
    void RefactorThread(void);
  };

  class DataSubscriber
  {
  public:
    enum ChannelType
    {
      Image = 0,
      Imu,
      PointCloud,
      StampedPose
    };
    struct ChannelStruct
    {
      ChannelType channel_type_;
      ros::Subscriber channel_1_sub_;
      ros::Subscriber channel_2_sub_;
      ros::Publisher channel_1_pub_;
      ros::Publisher channel_2_pub_;
      uint32_t seq_1_ = 0;
      uint32_t seq_2_ = 0;
      uint64_t time_1_ = 0;
      uint64_t time_2_ = 0;
      uint64_t time_diff_ = 0;
      uint64_t time_start_;
      void CallbackImage_1(const sensor_msgs::Image::ConstPtr &msg);
      void CallbackImu_1(const sensor_msgs::Imu::ConstPtr &msg);
      void CallbackPointcloud_1(const sensor_msgs::PointCloud2::ConstPtr &msg);
      void CallbackStampedpose_1(const geometry_msgs::PoseStamped::ConstPtr &msg);
      void CallbackImage_2(const sensor_msgs::Image::ConstPtr &msg);
      void CallbackImu_2(const sensor_msgs::Imu::ConstPtr &msg);
      void CallbackPointcloud_2(const sensor_msgs::PointCloud2::ConstPtr &msg);
      void CallbackStampedpose_2(const geometry_msgs::PoseStamped::ConstPtr &msg);
    };

    ros::NodeHandle nh_;
    ChannelStruct channel_1_;
    ChannelStruct channel_2_;
    ChannelStruct channel_3_;
    ChannelStruct channel_4_;

    uint64_t time_start_;

  public:
    void SetPublisher(std::vector<std::vector<std::string>> &);
  };

} // namespace sync_toolbox
#endif // SYNC_TOOLBOX_MY_PLUGIN_H