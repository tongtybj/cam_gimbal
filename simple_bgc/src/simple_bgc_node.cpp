#include <simple_bgc/simple_bgc_driver.h>

int main (int argc, char **argv)
{
  ros::init (argc, argv, "simple_brushless_gimbal_controller");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  boost::shared_ptr<SimpleBgc> simpleBgcNode(new SimpleBgc(nh, nh_private));
  ros::spin ();
  return 0;
}
