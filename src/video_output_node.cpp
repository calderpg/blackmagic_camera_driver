#include <cstring>
#include <memory>
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
std::unique_ptr<DeckLinkOutputDevice> g_output_device;

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

void ImageCallback(const sensor_msgs::ImageConstPtr& image)
{
  if (!g_output_device)
  {
    throw std::runtime_error("Output device not available");
  }

  auto output_frame = g_output_device->CreateBGRA8OutputVideoFrame();

  bool image_matches_output = true;
  if (image->encoding != "bgra8")
  {
    ROS_ERROR("Only images with bgra8 encoding are supported");
    image_matches_output = false;
  }

  if (static_cast<int64_t>(image->width) != output_frame->Width())
  {
    ROS_ERROR(
        "Received image with width [%ud] does not match output width [%ld]",
        image->width, output_frame->Width());
    image_matches_output = false;
  }

  if (static_cast<int64_t>(image->height) != output_frame->Height())
  {
    ROS_ERROR(
        "Received image with height [%ud] does not match output height [%ld]",
        image->height, output_frame->Height());
    image_matches_output = false;
  }

  if (static_cast<int64_t>(image->data.size()) != output_frame->DataSize())
  {
    ROS_ERROR("Received image data and output frame data are different sizes");
    image_matches_output = false;
  }

  if (image_matches_output)
  {
    std::memcpy(output_frame->Data(), image->data.data(), image->data.size());
    g_output_device->EnqueueOutputFrame(std::move(output_frame));
  }
  else
  {
    ROS_ERROR("Received image cannot be output");
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

  // Output display mode, 1920x1080 pixels @ 30 frames/sec
  const BMDDisplayMode output_mode = bmdModeHD1080p30;

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
  image_transport::Subscriber image_sub =
      it.subscribe(image_topic, 1, ImageCallback);

  g_output_device = std::unique_ptr<DeckLinkOutputDevice>(
      new DeckLinkOutputDevice(
          RosLoggingFunction, output_mode,
          std::move(decklink_devices.at(decklink_device_index))));

  // Start video output
  ROS_INFO("Starting video output...");
  g_output_device->Start();

  // Spin while video callbacks run
  ros::Rate spin_rate(30.0);
  while (ros::ok())
  {
    ros::spinOnce();
    spin_rate.sleep();
  }

  // Stop video output
  ROS_INFO("...stopping video output");
  g_output_device->Stop();

  return 0;
}
}  // namespace
}  // namespace blackmagic_camera_driver

int main(int argc, char** argv)
{
  ros::init(argc, argv, "video_output_node");
  return blackmagic_camera_driver::DoMain();
}
