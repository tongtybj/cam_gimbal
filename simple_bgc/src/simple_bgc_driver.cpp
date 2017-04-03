#include <simple_bgc/simple_bgc_driver.h>

SimpleBgc::SimpleBgc(ros::NodeHandle nh, ros::NodeHandle nhp): nh_(nh), nhp_(nhp)
{
  nhp_.param("port", port_, std::string("/dev/ttyUSB0"));
  nhp_.param("baud", baud_, 115200);
  nhp_.param("profile_id", profile_id_, 0);
  nhp_.param("rx_loop_rate", rx_loop_rate_, 100);
  nhp_.param("tx_loop_rate", tx_loop_rate_, 20);

  set_param_ = false;
  configure_ = false;

  rpy_pub_ = nh_.advertise<geometry_msgs::Vector3Stamped>("/gimbal_rpy", 10);
  angles_cmd_sub_ = nh_.subscribe<geometry_msgs::Vector3>("angles_cmd", 1, &SimpleBgc::anglesCmdCallback, this, ros::TransportHints().tcpNoDelay());

  serial_ = new serial::Serial(port_, baud_, serial::Timeout::simpleTimeout(1000));

  if(!serial_->isOpen())
    {
      ROS_FATAL("can't open serial port of %s", port_.c_str());
      ros::shutdown();
    }
  else
    {
      ROS_INFO("open the serial port");
    }

  sbgc_parser_.init(this);

  rx_timer_ = nhp_.createTimer(ros::Duration(1.0 / rx_loop_rate_), &SimpleBgc::rxFunc,this);
  tx_timer_ = nhp_.createTimer(ros::Duration(1.0 / tx_loop_rate_), &SimpleBgc::txFunc,this);

}
SimpleBgc::~SimpleBgc()
{
  delete serial_;
}


void SimpleBgc::txFunc(const ros::TimerEvent & e)
{
  if(set_param_)
    {
      /* get orientation of stabilized board */
      //sbgc_parser_.send_cmd(SBGC_CMD_GET_ANGLES);

      sbgc_parser_.send_cmd(SBGC_CMD_REALTIME_DATA);
    }
  else
    {
      sbgc_parser_.send_param_read_cmd(profile_id_);
    }
}

void SimpleBgc::anglesCmdCallback(const geometry_msgs::Vector3ConstPtr & msg)
{
  SBGC_cmd_control_t c = { 0, 0, 0, 0, 0, 0, 0 };
  c.mode = SBGC_CONTROL_MODE_ANGLE;
  c.angleROLL = SBGC_DEGREE_TO_ANGLE(msg->x * 180.0 / M_PI);
  c.anglePITCH = SBGC_DEGREE_TO_ANGLE(msg->y * 180.0 / M_PI);
  c.speedROLL = c.speedPITCH = c.speedYAW = 30 * SBGC_SPEED_SCALE;
  SBGC_cmd_control_send(c, sbgc_parser_);
}

void SimpleBgc::rxFunc(const ros::TimerEvent & e)
{

  if(sbgc_parser_.read_cmd())
    {

      switch(sbgc_parser_.in_cmd.id)
        {
        case SBGC_CMD_READ_PARAMS:
          {
            set_param_ = true;

            if(SBGC_cmd_param_unpack(param_info_, sbgc_parser_.in_cmd) != 0)
              ROS_WARN("bad unpacking");

            // for(int i = 0; i < sbgc_parser_.in_cmd.len; i ++)
            //   ROS_INFO("No.%d: %d", i + 1, sbgc_parser_.in_cmd.data[i]);

            int roll_p_ = param_info_.base_param[0].p;
            int roll_i_ = param_info_.base_param[0].i;
            int roll_d_ = param_info_.base_param[0].d;
            int roll_power_ = param_info_.base_param[0].power;
            bool roll_invert_ = param_info_.base_param[0].invert;
            int roll_poles_ = param_info_.base_param[0].poles;
            int pitch_p_ = param_info_.base_param[1].p;
            int pitch_i_ = param_info_.base_param[1].i;
            int pitch_d_ = param_info_.base_param[1].d;
            int pitch_power_ = param_info_.base_param[1].power;
            bool pitch_invert_ = param_info_.base_param[1].invert;
            int pitch_poles_ = param_info_.base_param[1].poles;

            nhp_.setParam("roll_p", roll_p_);
            nhp_.setParam("roll_i", roll_i_);
            nhp_.setParam("roll_d", roll_d_);
            nhp_.setParam("roll_power", roll_power_);
            nhp_.setParam("roll_invert", roll_invert_);
            nhp_.setParam("roll_poles", roll_poles_);
            nhp_.setParam("pitch_p", pitch_p_);
            nhp_.setParam("pitch_i", pitch_i_);
            nhp_.setParam("pitch_d", pitch_d_);
            nhp_.setParam("pitch_power", pitch_power_);
            nhp_.setParam("pitch_invert", pitch_invert_);
            nhp_.setParam("pitch_poles", pitch_poles_);

            /* dnyamic reconfigure just read the rosparam in the initial step */
            config_server_ = new dynamic_reconfigure::Server<simple_bgc::ControlConfig>(nhp_);
            config_func_ = boost::bind(&SimpleBgc::configCallback, this, _1, _2);
            config_server_->setCallback(config_func_);

            break;
          }
        case SBGC_CMD_BOARD_INFO:
          {
            SBGC_cmd_board_info_t board_info;
            SBGC_cmd_board_info_unpack(board_info, sbgc_parser_.in_cmd);
            ROS_INFO("Board Info: firmaware is %d.%db%d", 
                     board_info.firmware_ver / 1000, 
                     (board_info.firmware_ver - board_info.firmware_ver/ 1000) / 100,
                     board_info.firmware_ver % 10);
            break;
          }
        case SBGC_CMD_GET_ANGLES:
          {
            SBGC_cmd_get_angles_t angle_info;
            SBGC_cmd_get_angle_unpack(angle_info, sbgc_parser_.in_cmd);
            geometry_msgs::Vector3Stamped rpy_msg;
            rpy_msg.vector.x = angle_info.angle_data[0].angle * ANGLE_SCALE;
            rpy_msg.vector.y = angle_info.angle_data[1].angle * ANGLE_SCALE;
            rpy_msg.header.stamp = ros::Time::now();
            rpy_pub_.publish(rpy_msg);
            break;
          }
        case SBGC_CMD_REALTIME_DATA:
          {
            SBGC_cmd_realtime_data_v1_t realtime_data;
            // for(int i = 0; i < sbgc_parser_.in_cmd.len; i ++)
            //   ROS_INFO("No.%d: %d", i + 1, sbgc_parser_.in_cmd.data[i]);
            SBGC_cmd_realtime_data_v1_unpack(realtime_data, sbgc_parser_.in_cmd);
            geometry_msgs::Vector3Stamped rpy_msg;
            rpy_msg.vector.x = realtime_data.imu_angle[0] * ANGLE_SCALE;
            rpy_msg.vector.y = realtime_data.imu_angle[1] * ANGLE_SCALE;
            rpy_msg.header.stamp = ros::Time::now();
            rpy_pub_.publish(rpy_msg);
            // ROS_INFO("roll is %f, pitch is %f", rpy_msg.vector.x, rpy_msg.vector.y);
            ROS_INFO("battery level is %d", realtime_data.battery_voltage);
            break;
          }

        default:
          break;
        }
    }
}

void SimpleBgc::configCallback(simple_bgc::ControlConfig &config, uint32_t level)
{
  if(config.configure_flag)
    {
      configure_ = true;

      ROS_INFO("change param");

      param_info_.base_param[0].p = config.roll_p;
      param_info_.base_param[0].i = config.roll_i;
      param_info_.base_param[0].d = config.roll_d;
      param_info_.base_param[0].power = config.roll_power;
      param_info_.base_param[0].invert = config.roll_invert?1:0;
      param_info_.base_param[0].poles = config.roll_poles;

      param_info_.base_param[1].p = config.pitch_p;
      param_info_.base_param[1].i = config.pitch_i;
      param_info_.base_param[1].d = config.pitch_d;
      param_info_.base_param[1].power = config.pitch_power;
      param_info_.base_param[1].invert = config.pitch_invert?1:0;
      param_info_.base_param[1].poles = config.pitch_poles;

      SBGC_cmd_param_send(param_info_, sbgc_parser_);

      if(config.motor) sbgc_parser_.send_cmd(SBGC_CMD_MOTORS_ON);
      else sbgc_parser_.send_cmd(SBGC_CMD_MOTORS_OFF);
    }
}
