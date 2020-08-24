#include <atomic>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <vector>

#include <blackmagic_camera_driver/decklink_interface.hpp>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

namespace blackmagic_camera_driver
{
namespace
{
inline ros::console::levels::Level ConvertLogLevels(const LogLevel level)
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

class RosDeckLinkDeviceWrapper
{
public:
  RosDeckLinkDeviceWrapper(
      const ros::NodeHandle& nh, const std::string& camera_topic,
      const std::string& camera_frame, DeckLinkHandle device)
      : nh_(nh), it_(nh_), camera_frame_(camera_frame)
  {
    // Make the image publisher
    camera_pub_ = it_.advertise(camera_topic, 1, false);

    VideoFrameSizeChangedCallbackFunction video_frame_size_changed_callback_fn =
        [&] (const int32_t image_width, const int32_t image_height,
             const int32_t image_step)
    {
      VideoFrameSizeChangedCallback(image_width, image_height, image_step);
    };

    ConvertedVideoFrameCallbackFunction converted_video_frame_callback_fn =
        [&] (IDeckLinkVideoFrame& video_frame)
    {
      ConvertedVideoFrameCallback(video_frame);
    };

    // Make the device
    decklink_device_ = std::unique_ptr<DeckLinkDevice>(new DeckLinkDevice(
        LoggingFunction(RosLoggingFunction),
        video_frame_size_changed_callback_fn,
        converted_video_frame_callback_fn,
        std::move(device)));
  }

  void StartVideoCapture() { decklink_device_->StartVideoCapture(); }

  void StopVideoCapture() { decklink_device_->StopVideoCapture(); }

  void EnqueueCameraCommand(const BlackmagicSDICameraControlMessage& command)
  {
    decklink_device_->EnqueueCameraCommand(command);
  }

private:
  void VideoFrameSizeChangedCallback(
      const int32_t image_width, const int32_t image_height,
      const int32_t image_step)
  {
    // Make the ROS image for publishing
    ros_image_.header.frame_id = camera_frame_;
    ros_image_.width = static_cast<uint32_t>(image_width);
    ros_image_.height = static_cast<uint32_t>(image_height);
    ros_image_.encoding = "bgra8";
    ros_image_.is_bigendian = false;
    ros_image_.step = static_cast<uint32_t>(image_step);
    ros_image_.data.clear();
    ros_image_.data.resize(ros_image_.step * ros_image_.height, 0x00);
  }

  void ConvertedVideoFrameCallback(IDeckLinkVideoFrame& video_frame)
  {
    uint8_t* frame_buffer = nullptr;
    const auto get_bytes_result
        = video_frame.GetBytes(reinterpret_cast<void**>(&frame_buffer));
    if (get_bytes_result == S_OK && frame_buffer != nullptr)
    {
      ros_image_.header.stamp = ros::Time::now();
      std::memcpy(ros_image_.data.data(), frame_buffer, ros_image_.data.size());
      camera_pub_.publish(ros_image_);
    }
    else
    {
      throw std::runtime_error("Failed to get frame bytes");
    }
  }

  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Publisher camera_pub_;
  std::string camera_frame_;
  sensor_msgs::Image ros_image_;

  std::unique_ptr<DeckLinkDevice> decklink_device_;
};

int DoMain()
{
  // Get node handles
  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");

  // Load parameters
  const int32_t decklink_device_index
      = nhp.param(std::string("decklink_device_index"), 0);
  const int32_t camera_number
      = nhp.param(std::string("camera_number"), 1);
  const std::string camera_topic
      = nhp.param(std::string("camera_topic"), std::string("bmd_camera"));
  const std::string camera_frame
      = nhp.param(std::string("camera_frame"), std::string("bmd_camera"));

  if (camera_number < 0)
  {
    throw std::runtime_error("camera_number < 1");
  }
  else if (camera_number > 255)
  {
    throw std::runtime_error("camera_number > 255");
  }

  const uint8_t camera_id = static_cast<uint8_t>(camera_number);

  // Discover DeckLink devices
  std::vector<DeckLinkHandle> decklink_devices = GetDeckLinkDevices();
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
  RosDeckLinkDeviceWrapper capture_device(
      nh, camera_topic, camera_frame,
      std::move(decklink_devices.at(decklink_device_index)));

  // Start capture
  ROS_INFO("Starting capture...");
  capture_device.StartVideoCapture();

  // const uint16_t set_focus = ConvertToFixed16(0.5);
  // ROS_INFO("mid focus command %hx", set_focus);
  // const uint8_t set_focus_bottom_byte
  //     = static_cast<uint8_t>(set_focus & 0x00ff);
  // const uint8_t set_focus_top_byte
  //     = static_cast<uint8_t>((set_focus & 0xff00) >> 8);
  // // Focus to the near limit
  // const BlackmagicSDICameraControlMessage set_focus_command(
  //     camera_id,  // Destination camera
  //     0x00,  // "Change configuration"
  //     {0x00,  // "Lens"
  //      0x00,  // "Focus"
  //      0x00, 0x00,  // Empty
  //      set_focus_bottom_byte, set_focus_top_byte});  // Mid focus

  // capture_device.EnqueueCameraCommand(set_focus_command);

  const uint8_t lens_category = 0x00;

  const uint8_t ois_enable = 0x06;
  const uint8_t ordinal_aperture = 0x04;
  //const uint8_t normalized_aperture = 0x03;
  const uint8_t instantaneous_autofocus = 0x01;

  const uint8_t assign_value = 0x00;

  // Turn on OIS (if available)
  const BlackmagicSDICameraControlMessage enable_ois_command
      = BlackmagicSDICameraControlMessage::MakeCommandBool(
          camera_id, lens_category, ois_enable, assign_value, true);

  capture_device.EnqueueCameraCommand(enable_ois_command);

  std::this_thread::sleep_for(std::chrono::seconds(1));

  // Open the aperture all the way
  // const BlackmagicSDICameraControlMessage open_aperture_command
  //     = BlackmagicSDICameraControlMessage::MakeCommandFixed16(
  //         camera_id, lens_category, normalized_aperture, assign_value, 0.0);

  const BlackmagicSDICameraControlMessage open_aperture_command
      = BlackmagicSDICameraControlMessage::MakeCommandInt16(
          camera_id, lens_category, ordinal_aperture, assign_value, 0);

  capture_device.EnqueueCameraCommand(open_aperture_command);

  std::this_thread::sleep_for(std::chrono::seconds(1));

  // Run an instantaneous autofocus
  const BlackmagicSDICameraControlMessage autofocus_command
      = BlackmagicSDICameraControlMessage::MakeCommandVoid(
          camera_id, lens_category, instantaneous_autofocus);

  capture_device.EnqueueCameraCommand(autofocus_command);

  std::this_thread::sleep_for(std::chrono::seconds(1));

  // Spin while video callbacks run
  ros::Rate spin_rate(30.0);
  while (ros::ok())
  {
    ros::spinOnce();
    spin_rate.sleep();
  }

  const BlackmagicSDICameraControlMessage disable_ois_command
      = BlackmagicSDICameraControlMessage::MakeCommandBool(
          camera_id, lens_category, ois_enable, assign_value, false);

  capture_device.EnqueueCameraCommand(disable_ois_command);

  std::this_thread::sleep_for(std::chrono::seconds(1));

  // Stop capture
  ROS_INFO("Stopping capture...");
  capture_device.StopVideoCapture();

  // Let RAII clean everything else
  ROS_INFO("...capture complete");

  return 0;
}
}  // namespace
}  // namespace blackmagic_camera_driver

int main(int argc, char** argv)
{
  ros::init(argc, argv, "blackmagic_camera_driver_node");
  return blackmagic_camera_driver::DoMain();
}
