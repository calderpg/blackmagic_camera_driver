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

  // Setup ROS interface
  image_transport::ImageTransport it(nh);
  image_transport::Publisher camera_pub = it.advertise(camera_topic, 1, false);
  sensor_msgs::Image ros_image;

  // Create callback functions
  const auto frame_size_change_fn = [&](
      const int64_t width, const int64_t height, const int64_t step)
  {
    // Resize the ROS image for publishing
    ros_image.header.frame_id = camera_frame;
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
    camera_pub.publish(ros_image);
  };

  // Get the selected device
  ROS_INFO("Selecting DeckLink device [%d]", decklink_device_index);
  DeckLinkInputOutputDevice capture_device(
      RosLoggingFunction, frame_size_change_fn, frame_received_fn, output_mode,
      std::move(decklink_devices.at(decklink_device_index)));

  // Start capture
  ROS_INFO("Starting capture...");
  capture_device.Start();

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

  // Make primary color image data blocks
  const size_t num_image_pixels = 1920 * 1080;
  const size_t num_image_bytes = num_image_pixels * 4;
  const std::vector<uint32_t> red_color(num_image_pixels, 0xffff0000);
  const std::vector<uint32_t> green_color(num_image_pixels, 0xff00ff00);
  const std::vector<uint32_t> blue_color(num_image_pixels, 0xff0000ff);

  int32_t tick = 0;
  // Spin while video callbacks run
  ros::Rate spin_rate(30.0);
  while (ros::ok())
  {
    tick++;

    const void* image_data_ptr = nullptr;
    if (tick == 30)
    {
      image_data_ptr = red_color.data();
    }
    else if (tick == 60)
    {
      image_data_ptr = green_color.data();
    }
    else if (tick == 90)
    {
      image_data_ptr = blue_color.data();
      tick = 0;
    }

    if (image_data_ptr != nullptr)
    {
      auto output_frame = capture_device.CreateBGRA8OutputVideoFrame();
      if (output_frame->DataSize() != static_cast<int64_t>(num_image_bytes))
      {
        throw std::runtime_error(
            "Image data and frame data are different sizes");
      }

      std::memcpy(output_frame->Data(), image_data_ptr, num_image_bytes);

      capture_device.EnqueueOutputFrame(std::move(output_frame));
    }

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
  capture_device.Stop();

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
