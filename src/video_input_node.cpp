#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>

#include <blackmagic_camera_driver/decklink_interface.hpp>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

namespace blackmagic_camera_driver
{
namespace
{
ros::console::levels::Level ConvertLogLevels(const LogLevel level)
{
  if (level == LogLevel::DEBUG)
  {
    return ros::console::levels::Debug;
  }
  else if (level == LogLevel::INFO)
  {
    return ros::console::levels::Info;
  }
  else if (level == LogLevel::WARN)
  {
    return ros::console::levels::Warn;
  }
  else if (level == LogLevel::ERROR)
  {
    return ros::console::levels::Error;
  }
  else
  {
    throw std::runtime_error("Invalid LogLevel value");
  }
}

void RosLoggingFunction(
    const LogLevel level, const std::string& message, const bool throttle)
{
  const auto ros_level = ConvertLogLevels(level);
  if (throttle)
  {
    ROS_LOG_THROTTLE(10, ros_level, ROSCONSOLE_DEFAULT_NAME, message.c_str());
  }
  else
  {
    ROS_LOG(ros_level, ROSCONSOLE_DEFAULT_NAME, message.c_str());
  }
}

int DoMain()
{
  // Get node handles
  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");

  // Load parameters
  const int32_t decklink_device_index
      = nhp.param(std::string("decklink_device_index"), 0);
  const std::string image_topic
      = nhp.param(std::string("image_topic"), std::string("video_input"));
  const std::string image_frame
      = nhp.param(std::string("image_frame"), std::string("video_input"));

  // Discover DeckLink devices
  std::vector<DeckLinkHandle> decklink_devices = GetDeckLinkHardwareDevices();
  if (decklink_devices.size() > 0)
  {
    ROS_INFO("Found [%zu] DeckLink device(s)", decklink_devices.size());
  }
  else
  {
    throw std::runtime_error("No DeckLink device(s) found");
  }

  // Get the selected device
  ROS_INFO("Selecting DeckLink device [%d]", decklink_device_index);

  // Setup ROS interface
  image_transport::ImageTransport it(nh);
  image_transport::Publisher image_pub = it.advertise(image_topic, 1, false);
  sensor_msgs::Image ros_image;

  // Create callback functions
  const auto frame_size_change_fn = [&](
      const int64_t width, const int64_t height, const int64_t step)
  {
    // Resize the ROS image for publishing
    ros_image.header.frame_id = image_frame;
    ros_image.width = static_cast<uint32_t>(width);
    ros_image.height = static_cast<uint32_t>(height);
    ros_image.encoding = "bgra8";
    ros_image.is_bigendian = false;
    ros_image.step = static_cast<uint32_t>(step);
    ros_image.data.clear();
    ros_image.data.resize(ros_image.step * ros_image.height, 0x00);
  };

  const auto frame_received_fn = [&](const BMDCompatibleVideoFrame& video_frame)
  {
    if (video_frame.DataSize() != static_cast<int64_t>(ros_image.data.size()))
    {
      throw std::runtime_error("Video frame and ROS image are different sizes");
    }

    ros_image.header.stamp = ros::Time::now();
    std::memcpy(
        ros_image.data.data(), video_frame.Data(), ros_image.data.size());
    image_pub.publish(ros_image);
  };

  DeckLinkInputDevice input_device(
      RosLoggingFunction, frame_size_change_fn, frame_received_fn,
      std::move(decklink_devices.at(decklink_device_index)));

  // Start video input
  ROS_INFO("Starting video input...");
  input_device.Start();

  // Spin while video callbacks run
  ros::Rate spin_rate(30.0);
  while (ros::ok())
  {
    ros::spinOnce();
    spin_rate.sleep();
  }

  // Stop video input
  ROS_INFO("...stopping video input");
  input_device.Stop();

  return 0;
}
}  // namespace
}  // namespace blackmagic_camera_driver

int main(int argc, char** argv)
{
  ros::init(argc, argv, "video_input_node");
  return blackmagic_camera_driver::DoMain();
}
