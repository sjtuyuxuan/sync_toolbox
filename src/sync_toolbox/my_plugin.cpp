#include "sync_toolbox/my_plugin.h"
#include <pluginlib/class_list_macros.h>
#include <QStringList>
#include <stdio.h>

namespace sync_toolbox
{

  MyPlugin::MyPlugin()
      : rqt_gui_cpp::Plugin(), widget_(0)
  {
    // Constructor is called first before initPlugin function, needless to say.

    // give QObjects reasonable names
    setObjectName("MyPlugin");
  }

  void MyPlugin::initPlugin(qt_gui_cpp::PluginContext &context)
  {
    // access standalone command line arguments
    QStringList argv = context.argv();
    // create QWidget
    widget_ = new QWidget();
    // extend the widget with all attributes and children from UI file
    ui_.setupUi(widget_);
    // add widget to the user interface
    context.addWidget(widget_);

    // add serial port
    serialPort = new QSerialPort();

    // set unvisibale for stop freq
    ui_.channel_1_stop_ratio->setVisible(false);
    ui_.channel_2_stop_ratio->setVisible(false);
    ui_.channel_3_stop_ratio->setVisible(false);
    ui_.channel_4_stop_ratio->setVisible(false);
    ui_.start_indicator_width_input->setVisible(false);

    // connect the slot
    connect(ui_.pushButton, SIGNAL(clicked()), this, SLOT(on_StartButton_clicked()));
    connect(ui_.serial_port_search_button, SIGNAL(clicked()), this, SLOT(on_SearchButton_clicked()));
    connect(ui_.serial_port_congfig_button, SIGNAL(clicked()), this, SLOT(on_ConfigButton_clicked()));

    connect(ui_.channel_1_stop_freq_enable, SIGNAL(clicked(bool)), this, SLOT(on_Stopfreq_1_enabled(bool)));
    connect(ui_.channel_2_stop_freq_enable, SIGNAL(clicked(bool)), this, SLOT(on_Stopfreq_2_enabled(bool)));
    connect(ui_.channel_3_stop_freq_enable, SIGNAL(clicked(bool)), this, SLOT(on_Stopfreq_3_enabled(bool)));
    connect(ui_.channel_4_stop_freq_enable, SIGNAL(clicked(bool)), this, SLOT(on_Stopfreq_4_enabled(bool)));

    connect(ui_.channel_1_enable, SIGNAL(clicked(bool)), this, SLOT(on_1_enabled(bool)));
    connect(ui_.channel_2_enable, SIGNAL(clicked(bool)), this, SLOT(on_2_enabled(bool)));
    connect(ui_.channel_3_enable, SIGNAL(clicked(bool)), this, SLOT(on_3_enabled(bool)));
    connect(ui_.channel_4_enable, SIGNAL(clicked(bool)), this, SLOT(on_4_enabled(bool)));
    connect(ui_.channel_clock_enable, SIGNAL(clicked(bool)), this, SLOT(on_cl_enabled(bool)));
    connect(ui_.start_indicator_enable, SIGNAL(clicked(bool)), this, SLOT(on_si_enabled(bool)));
    connect(ui_.bag_export_enable, SIGNAL(clicked(bool)), this, SLOT(on_bag_export_enabled(bool)));
    connect(ui_.online_export_enable, SIGNAL(clicked(bool)), this, SLOT(on_online_refactor_enabled(bool)));

    connect(ui_.channel_1_type, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged), this, &MyPlugin::pulse_width_closed_1);
    connect(ui_.channel_2_type, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged), this, &MyPlugin::pulse_width_closed_2);
    connect(ui_.channel_3_type, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged), this, &MyPlugin::pulse_width_closed_3);
    connect(ui_.channel_4_type, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged), this, &MyPlugin::pulse_width_closed_4);
    connect(ui_.channel_clock_type, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged), this, &MyPlugin::pulse_width_closed_c);
    connect(ui_.start_indicator_type, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged), this, &MyPlugin::pulse_width_closed_s);
    connect(serialPort, &QSerialPort::readyRead, this, &MyPlugin::readyReadSlot);
  }

  void MyPlugin::shutdownPlugin()
  {
    // unregister all publishers here
  }

  void MyPlugin::saveSettings(qt_gui_cpp::Settings &plugin_settings,
                              qt_gui_cpp::Settings &instance_settings) const
  {
    // instance_settings.setValue(k, v)
  }

  void MyPlugin::restoreSettings(const qt_gui_cpp::Settings &plugin_settings,
                                 const qt_gui_cpp::Settings &instance_settings)
  {
    // v = instance_settings.value(k)
  }

  void MyPlugin::on_StartButton_clicked()
  {
    if (config_microcontroller && !start_record_)
    {
      ui_.pushButton->setText(tr("Close"));
      start_record_ = true;
      if (ui_.online_export_enable->isChecked())
      {
        t_ = boost::make_shared<std::thread>(std::mem_fn(&MyPlugin::RefactorThread), this);
        t_->detach();
      }
      if (ui_.bag_export_enable->isChecked())
      {
      }
      QString str = tr("sss\n");
      QByteArray sendData = str.toLatin1();
      serialPort->write(sendData);
    }
    else if (start_record_)
    {
      ui_.pushButton->setText(tr("Start"));
      QString str = tr("eee\n");
      QByteArray sendData = str.toLatin1();
      serialPort->write(sendData);
      start_record_ = false;
    }
  }

  void MyPlugin::on_SearchButton_clicked()
  {
    ui_.serial_port->clear();
    // get aviliable serialport
    foreach (const QSerialPortInfo &info, QSerialPortInfo::availablePorts())
      ui_.serial_port->addItem(info.portName());
  }

  void MyPlugin::on_Stopfreq_1_enabled(bool checked)
  {
    ui_.channel_1_stop_ratio->setVisible(checked);
  }
  void MyPlugin::on_Stopfreq_2_enabled(bool checked)
  {
    ui_.channel_2_stop_ratio->setVisible(checked);
  }
  void MyPlugin::on_Stopfreq_3_enabled(bool checked)
  {
    ui_.channel_3_stop_ratio->setVisible(checked);
  }
  void MyPlugin::on_Stopfreq_4_enabled(bool checked)
  {
    ui_.channel_4_stop_ratio->setVisible(checked);
  }

  void MyPlugin::on_1_enabled(bool checked)
  {
    if (checked)
      ui_.channel_1_enable->setText(tr("enable"));
    else
      ui_.channel_1_enable->setText(tr("disable"));
  }
  void MyPlugin::on_2_enabled(bool checked)
  {
    if (checked)
      ui_.channel_2_enable->setText(tr("enable"));
    else
      ui_.channel_2_enable->setText(tr("disable"));
  }
  void MyPlugin::on_3_enabled(bool checked)
  {
    if (checked)
      ui_.channel_3_enable->setText(tr("enable"));
    else
      ui_.channel_3_enable->setText(tr("disable"));
  }
  void MyPlugin::on_4_enabled(bool checked)
  {
    if (checked)
      ui_.channel_4_enable->setText(tr("enable"));
    else
      ui_.channel_4_enable->setText(tr("disable"));
  }
  void MyPlugin::on_cl_enabled(bool checked)
  {
    if (checked)
      ui_.channel_clock_enable->setText(tr("enable"));
    else
      ui_.channel_clock_enable->setText(tr("disable"));
  }
  void MyPlugin::on_si_enabled(bool checked)
  {
    if (checked)
      ui_.start_indicator_enable->setText(tr("enable"));
    else
      ui_.start_indicator_enable->setText(tr("disable"));
  }
  void MyPlugin::on_bag_export_enabled(bool checked)
  {
    if (checked)
      ui_.bag_export_enable->setText(tr("enable"));
    else
      ui_.bag_export_enable->setText(tr("disable"));
  }
  void MyPlugin::on_online_refactor_enabled(bool checked)
  {
    if (checked)
      ui_.online_export_enable->setText(tr("enable"));
    else
      ui_.online_export_enable->setText(tr("disable"));
  }

  void MyPlugin::pulse_width_closed_1(int i)
  {
    if (i == 2)
      ui_.channel_1_width_input->setVisible(false);
    else
      ui_.channel_1_width_input->setVisible(true);
  }
  void MyPlugin::pulse_width_closed_2(int i)
  {
    if (i == 2)
      ui_.channel_2_width_input->setVisible(false);
    else
      ui_.channel_2_width_input->setVisible(true);
  }
  void MyPlugin::pulse_width_closed_3(int i)
  {
    if (i == 2)
      ui_.channel_3_width_input->setVisible(false);
    else
      ui_.channel_3_width_input->setVisible(true);
  }
  void MyPlugin::pulse_width_closed_4(int i)
  {
    if (i == 2)
      ui_.channel_4_width_input->setVisible(false);
    else
      ui_.channel_4_width_input->setVisible(true);
  }
  void MyPlugin::pulse_width_closed_c(int i)
  {
    if (i == 2)
      ui_.channel_clock_width_input->setVisible(false);
    else
      ui_.channel_clock_width_input->setVisible(true);
  }
  void MyPlugin::pulse_width_closed_s(int i)
  {
    if (i == 2)
      ui_.start_indicator_width_input->setVisible(true);
    else
      ui_.start_indicator_width_input->setVisible(false);
  }

  void MyPlugin::on_ConfigButton_clicked()
  {
    serialPort->setPortName(ui_.serial_port->currentText());
    if (!serialPort->isOpen())
    {
      if (!serialPort->open(QIODevice::ReadWrite))
      {
        ui_.setting_show_browser->clear();
        ui_.setting_show_browser->setTextColor(Qt::red);
        ui_.setting_show_browser->setText(tr("Unable to open the Serial Port"));
        return;
      }
      else
      {
        serialPort->setBaudRate(QSerialPort::Baud115200);
        serialPort->setDataBits(QSerialPort::Data8);
        serialPort->setFlowControl(QSerialPort::NoFlowControl);
        serialPort->setStopBits(QSerialPort::OneStop);
        serialPort->setParity(QSerialPort::NoParity);
        serialPort->setReadBufferSize(PORT_BUFFER_SIZE);
      }
    }
    EncoderSetting();
    QString str = tr(config_.c_str());

    if (serialPort->write(str.toLatin1()))
    {
      ui_.setting_show_browser->clear();
      ui_.setting_show_browser->setTextColor(Qt::red);
      ui_.setting_show_browser->setText(tr("Fail to send"));
    };
    ui_.setting_show_browser->setTextColor(Qt::green);
    std::string output("The encode result is sent:\n");
    output += config_;
    ui_.setting_show_browser->setText(QString::fromStdString(output));
    serialPort->waitForBytesWritten();
    serialPort->flush();
  }

  void MyPlugin::EncoderSetting()
  {
    config_.clear();
    std::stringstream buf;

    PrintProtocal(buf,
                  atoi(ui_.channel_1_freq_input->text().toStdString().c_str()),
                  atoi(ui_.channel_1_offset_input->text().toStdString().c_str()),
                  atoi(ui_.channel_1_width_input->text().toStdString().c_str()),
                  ui_.channel_1_enable->isChecked(),
                  ui_.channel_1_type->currentIndex());
    PrintProtocal(buf,
                  atoi(ui_.channel_2_freq_input->text().toStdString().c_str()),
                  atoi(ui_.channel_2_offset_input->text().toStdString().c_str()),
                  atoi(ui_.channel_2_width_input->text().toStdString().c_str()),
                  ui_.channel_2_enable->isChecked(),
                  ui_.channel_2_type->currentIndex());
    PrintProtocal(buf,
                  atoi(ui_.channel_3_freq_input->text().toStdString().c_str()),
                  atoi(ui_.channel_3_offset_input->text().toStdString().c_str()),
                  atoi(ui_.channel_3_width_input->text().toStdString().c_str()),
                  ui_.channel_3_enable->isChecked(),
                  ui_.channel_3_type->currentIndex());
    PrintProtocal(buf,
                  atoi(ui_.channel_4_freq_input->text().toStdString().c_str()),
                  atoi(ui_.channel_4_offset_input->text().toStdString().c_str()),
                  atoi(ui_.channel_4_width_input->text().toStdString().c_str()),
                  ui_.channel_4_enable->isChecked(),
                  ui_.channel_4_type->currentIndex());
    PrintProtocal(buf,
                  atoi(ui_.channel_clock_freq_input->text().toStdString().c_str()),
                  atoi(ui_.channel_clock_offset_input->text().toStdString().c_str()),
                  atoi(ui_.channel_clock_width_input->text().toStdString().c_str()),
                  ui_.channel_clock_enable->isChecked(),
                  ui_.channel_clock_type->currentIndex());
    PrintProtocal(buf,
                  30,
                  atoi(ui_.start_indicator_offset_input->text().toStdString().c_str()),
                  atoi(ui_.start_indicator_width_input->text().toStdString().c_str()),
                  ui_.start_indicator_enable->isChecked(),
                  ui_.start_indicator_type->currentIndex());
    config_ = buf.str();
    config_ += '\n';
  }

  void MyPlugin::PrintProtocal(std::stringstream &buf, int freq, int offset, int width, int enable, int mode)
  {
    buf << enable << " ";                                    // enable
    buf << mode << " ";                                      //mode
    buf << std::setw(3) << std::setfill('0') << freq << " "; // freq
    if (offset >= 0)
      buf << "+";
    else
      buf << "-";
    buf << std::setw(5) << std::setfill('0') << offset << " "; // offset
    buf << std::setw(5) << std::setfill('0') << width << " ";  //width
  }

  void MyPlugin::readyReadSlot()
  {
    ui_.setting_show_browser->clear();
    QByteArray arr = serialPort->readAll();
    ui_.setting_show_browser->setText(arr);
  }

  void MyPlugin::RefactorThread(void)
  {
    DataSubscriber data_sub;
    data_sub.channel_1_.channel_type_ = static_cast<DataSubscriber::ChannelType>(ui_.channel_1_type_box->currentIndex());
    data_sub.channel_2_.channel_type_ = static_cast<DataSubscriber::ChannelType>(ui_.channel_2_type_box->currentIndex());
    data_sub.channel_3_.channel_type_ = static_cast<DataSubscriber::ChannelType>(ui_.channel_3_type_box->currentIndex());
    data_sub.channel_4_.channel_type_ = static_cast<DataSubscriber::ChannelType>(ui_.channel_4_type_box->currentIndex());
    data_sub.channel_1_.time_diff_ = 1000000000 / atoi(ui_.channel_1_freq_input->text().toStdString().c_str());
    data_sub.channel_2_.time_diff_ = 1000000000 / atoi(ui_.channel_2_freq_input->text().toStdString().c_str());
    data_sub.channel_3_.time_diff_ = 1000000000 / atoi(ui_.channel_3_freq_input->text().toStdString().c_str());
    data_sub.channel_4_.time_diff_ = 1000000000 / atoi(ui_.channel_4_freq_input->text().toStdString().c_str());
    data_sub.time_start_ = ros::Time::now().toNSec();
    data_sub.channel_1_.time_start_ = data_sub.time_start_;
    data_sub.channel_2_.time_start_ = data_sub.time_start_;
    data_sub.channel_3_.time_start_ = data_sub.time_start_;
    data_sub.channel_4_.time_start_ = data_sub.time_start_;
    std::vector<std::vector<std::string>> topics = {{"", "", "", ""}, {"", "", "", ""}, {"", "", "", ""}, {"", "", "", ""}};

    if (ui_.channel_1_topic_1_enable->isChecked())
    {
      topics[0][0] = ui_.channel_1_topic_1_input->text().toStdString();
      topics[0][2] = ui_.channel_1_topic_1_output->text().toStdString();
    }
    if (ui_.channel_1_topic_2_enable->isChecked())
    {
      topics[0][1] = ui_.channel_1_topic_2_input->text().toStdString();
      topics[0][3] = ui_.channel_1_topic_2_output->text().toStdString();
    }
    if (ui_.channel_2_topic_1_enable->isChecked())
    {
      topics[1][0] = ui_.channel_2_topic_1_input->text().toStdString();
      topics[1][2] = ui_.channel_2_topic_1_output->text().toStdString();
    }
    if (ui_.channel_2_topic_2_enable->isChecked())
    {
      topics[1][1] = ui_.channel_2_topic_2_input->text().toStdString();
      topics[1][3] = ui_.channel_2_topic_2_output->text().toStdString();
    }
    if (ui_.channel_3_topic_1_enable->isChecked())
    {
      topics[2][0] = ui_.channel_3_topic_1_input->text().toStdString();
      topics[2][2] = ui_.channel_3_topic_1_output->text().toStdString();
    }
    if (ui_.channel_3_topic_2_enable->isChecked())
    {
      topics[2][1] = ui_.channel_3_topic_2_input->text().toStdString();
      topics[2][3] = ui_.channel_3_topic_2_output->text().toStdString();
    }
    if (ui_.channel_4_topic_1_enable->isChecked())
    {
      topics[3][0] = ui_.channel_4_topic_1_input->text().toStdString();
      topics[3][2] = ui_.channel_4_topic_1_output->text().toStdString();
    }
    if (ui_.channel_4_topic_2_enable->isChecked())
    {
      topics[3][1] = ui_.channel_4_topic_2_input->text().toStdString();
      topics[3][3] = ui_.channel_4_topic_2_output->text().toStdString();
    }
    data_sub.SetPublisher(topics);
    while (ros::ok() && start_record_)
      ros::spinOnce();
    return;
  }

  void DataSubscriber::SetPublisher(std::vector<std::vector<std::string>> &topics)
  {
    std::vector<DataSubscriber::ChannelStruct *> channle_ = {&(this->channel_1_), &(this->channel_2_), &(this->channel_3_), &(this->channel_4_)};
    for (size_t i = 0; i < channle_.size(); ++i)
    {
      switch (channle_[i]->channel_type_)
      {
      case DataSubscriber::ChannelType::Image:
        if (topics[i][2] != "")
        {
          channle_[i]->channel_1_pub_ = nh_.advertise<sensor_msgs::Image>(topics[i][2], 100);
          channle_[i]->channel_1_sub_ = nh_.subscribe<sensor_msgs::Image>(
              topics[i][0], 10, &DataSubscriber::ChannelStruct::CallbackImage_1,
              channle_[i]);
        }
        if (topics[i][3] != "")
        {
          channle_[i]->channel_2_pub_ = nh_.advertise<sensor_msgs::Image>(topics[i][3], 100);
          channle_[i]->channel_2_sub_ = nh_.subscribe<sensor_msgs::Image>(
              topics[i][0], 10, &DataSubscriber::ChannelStruct::CallbackImage_2,
              channle_[i]);
        }
        break;

      case DataSubscriber::ChannelType::Imu:
        if (topics[i][2] != "")
        {
          channle_[i]->channel_1_pub_ = nh_.advertise<sensor_msgs::Imu>(topics[i][2], 100);
          channle_[i]->channel_1_sub_ = nh_.subscribe<sensor_msgs::Imu>(
              topics[i][0], 10, &DataSubscriber::ChannelStruct::CallbackImu_1,
              channle_[i]);
        }
        if (topics[i][3] != "")
        {
          channle_[i]->channel_2_pub_ = nh_.advertise<sensor_msgs::Imu>(topics[i][3], 100);
          channle_[i]->channel_2_sub_ = nh_.subscribe<sensor_msgs::Imu>(
              topics[i][0], 10, &DataSubscriber::ChannelStruct::CallbackImu_2,
              channle_[i]);
        }
        break;

      case DataSubscriber::ChannelType::PointCloud:
        if (topics[i][2] != "")
        {
          channle_[i]->channel_1_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(topics[i][2], 100);
          channle_[i]->channel_1_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>(
              topics[i][0], 10, &DataSubscriber::ChannelStruct::CallbackPointcloud_1,
              channle_[i]);
        }
        if (topics[i][3] != "")
        {
          channle_[i]->channel_2_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(topics[i][3], 100);
          channle_[i]->channel_2_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>(
              topics[i][0], 10, &DataSubscriber::ChannelStruct::CallbackPointcloud_2,
              channle_[i]);
        }
        break;

      case DataSubscriber::ChannelType::StampedPose:
        if (topics[i][2] != "")
        {
          channle_[i]->channel_1_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(topics[i][2], 100);
          channle_[i]->channel_1_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>(
              topics[i][0], 10, &DataSubscriber::ChannelStruct::CallbackStampedpose_1,
              channle_[i]);
        }
        if (topics[i][3] != "")
        {
          channle_[i]->channel_2_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(topics[i][3], 100);
          channle_[i]->channel_2_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>(
              topics[i][1], 10, &DataSubscriber::ChannelStruct::CallbackStampedpose_2,
              channle_[i]);
        }
        break;
      default:
        break;
      }
    }
  }

  void DataSubscriber::ChannelStruct::CallbackImage_1(const sensor_msgs::Image::ConstPtr &msg)
  {
    auto ptr = boost::const_pointer_cast<sensor_msgs::Image>(msg);
    ptr->header.stamp.fromNSec(time_start_ + time_1_);
    ptr->header.seq = ++seq_1_;
    time_1_ += time_diff_;
    channel_1_pub_.publish(msg);
  }
  void DataSubscriber::ChannelStruct::CallbackImage_2(const sensor_msgs::Image::ConstPtr &msg)
  {
    auto ptr = boost::const_pointer_cast<sensor_msgs::Image>(msg);
    ptr->header.stamp.fromNSec(time_start_ + time_2_);
    ptr->header.seq = ++seq_2_;
    time_2_ += time_diff_;
    channel_2_pub_.publish(msg);
  }

  void DataSubscriber::ChannelStruct::CallbackImu_1(const sensor_msgs::Imu::ConstPtr &msg)
  {
    auto ptr = boost::const_pointer_cast<sensor_msgs::Imu>(msg);
    ptr->header.stamp.fromNSec(time_start_ + time_1_);
    ptr->header.seq = ++seq_1_;
    time_1_ += time_diff_;
    channel_1_pub_.publish(msg);
  }
  void DataSubscriber::ChannelStruct::CallbackImu_2(const sensor_msgs::Imu::ConstPtr &msg)
  {
    auto ptr = boost::const_pointer_cast<sensor_msgs::Imu>(msg);
    ptr->header.stamp.fromNSec(time_start_ + time_2_);
    ptr->header.seq = ++seq_2_;
    time_2_ += time_diff_;
    channel_2_pub_.publish(msg);
  }

  void DataSubscriber::ChannelStruct::CallbackPointcloud_1(const sensor_msgs::PointCloud2::ConstPtr &msg)
  {
    auto ptr = boost::const_pointer_cast<sensor_msgs::PointCloud2>(msg);
    ptr->header.stamp.fromNSec(time_start_ + time_1_);
    ptr->header.seq = ++seq_1_;
    time_1_ += time_diff_;
    channel_1_pub_.publish(msg);
  }
  void DataSubscriber::ChannelStruct::CallbackPointcloud_2(const sensor_msgs::PointCloud2::ConstPtr &msg)
  {
    auto ptr = boost::const_pointer_cast<sensor_msgs::PointCloud2>(msg);
    ptr->header.stamp.fromNSec(time_start_ + time_2_);
    ptr->header.seq = ++seq_2_;
    time_2_ += time_diff_;
    channel_2_pub_.publish(msg);
  }

  void DataSubscriber::ChannelStruct::CallbackStampedpose_1(const geometry_msgs::PoseStamped::ConstPtr &msg)
  {
    auto ptr = boost::const_pointer_cast<geometry_msgs::PoseStamped>(msg);
    ptr->header.stamp.fromNSec(time_start_ + time_1_);
    ptr->header.seq = ++seq_1_;
    time_1_ += time_diff_;
    channel_1_pub_.publish(msg);
  }
  void DataSubscriber::ChannelStruct::CallbackStampedpose_2(const geometry_msgs::PoseStamped::ConstPtr &msg)
  {
    auto ptr = boost::const_pointer_cast<geometry_msgs::PoseStamped>(msg);
    ptr->header.stamp.fromNSec(time_start_ + time_2_);
    ptr->header.seq = ++seq_2_;
    time_2_ += time_diff_;
    channel_2_pub_.publish(msg);
  }
} // namespace sync_toolbox
//PLUGINLIB_DECLARE_CLASS(sync_toolbox, MyPlugin, sync_toolbox::MyPlugin, rqt_gui_cpp::Plugin)
PLUGINLIB_EXPORT_CLASS(sync_toolbox::MyPlugin, rqt_gui_cpp::Plugin)