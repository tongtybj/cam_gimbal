#ifndef SIMPLE_BGC_DRIVER_H
#define SIMPLE_BGC_DRIVER_H

#include <ros/ros.h>
#include <serial/serial.h>
#include <SBGC.h>
#include <geometry_msgs/Vector3Stamped.h>

#include <dynamic_reconfigure/server.h>
#include <simple_bgc/ControlConfig.h>

#include <string>
#include <iostream>


class SimpleBgc  : public SBGC_ComObj
{
public:
  SimpleBgc(ros::NodeHandle nh, ros::NodeHandle nhp);
  ~SimpleBgc();


  virtual uint16_t getBytesAvailable() {
    return serial_->available();
  }
  virtual uint8_t readByte() {
    uint8_t val =  (serial_->read().c_str())[0];
    return val;
  }
  virtual void writeByte(uint8_t b) {
    std::vector<uint8_t> buffer(1);
    buffer[0] = b;
    serial_->write(buffer);
  }
  // Arduino com port is not buffered, so empty space is unknown.
  virtual uint16_t getOutEmptySpace() {
    return 0xFFFF;
  }

private:
  ros::NodeHandle nh_;
  ros::NodeHandle nhp_;
  ros::Timer  rx_timer_;
  ros::Timer  tx_timer_;

  dynamic_reconfigure::Server<simple_bgc::ControlConfig>* config_server_;
  dynamic_reconfigure::Server<simple_bgc::ControlConfig>::CallbackType config_func_;

  serial::Serial *serial_;
  SBGC_Parser sbgc_parser_;

  ros::Publisher rpy_pub_;
  ros::Subscriber angles_cmd_sub_;
  int baud_;
  std::string port_;
  int profile_id_;

  int rx_loop_rate_;
  int tx_loop_rate_;

  //basic control param for gimbal
  

  void rxFunc(const ros::TimerEvent & e);
  void txFunc(const ros::TimerEvent & e);

  void anglesCmdCallback(const geometry_msgs::Vector3ConstPtr & msg);
  void configCallback(simple_bgc::ControlConfig &config, uint32_t level);

  SBGC_cmd_param_t param_info_;

  bool set_param_;
  bool configure_;
};


#endif  // SIMPLE_BGC_DRIVER_H
