#include <blackmagic_camera_driver/decklink_interface.hpp>

#include <atomic>
#include <cstdint>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <string>

#include <blackmagic_camera_driver/bmd_handle.hpp>
#include <image_transport/image_transport.h>
#include <ros/ros.h>

#include "DeckLinkAPI_v10_11.h"

namespace blackmagic_camera_driver
{
// Default format, overriden by the automatic video input format detection.
const int32_t kDefaultImageWidth = 1920;
const int32_t kDefaultImageHeight = 1080;
const BMDDisplayMode kDefaultDisplayMode = bmdModeHD1080p30;
const BMDPixelFormat kDefaultInputPixelFormat = bmdFormat8BitYUV;
const BMDPixelFormat kDefaultOutputPixelFormat = bmdFormat8BitBGRA;

HRESULT FrameReceivedCallback::VideoInputFormatChanged(
    BMDVideoInputFormatChangedEvents notification_events,
    IDeckLinkDisplayMode* new_display_mode,
    BMDDetectedVideoInputFormatFlags detected_signal_flags)
{
  if (new_display_mode != nullptr)
  {
    return parent_device_->InputFormatChangedCallback(
        notification_events, *new_display_mode, detected_signal_flags);
  }
  else
  {
    ROS_WARN_NAMED(
        ros::this_node::getName(),
        "Null new display mode received, discarding");
    return S_OK;
  }
}

HRESULT FrameReceivedCallback::VideoInputFrameArrived(
    IDeckLinkVideoInputFrame* video_frame,
    IDeckLinkAudioInputPacket* audio_packet)
{
  if (video_frame != nullptr
      && (video_frame->GetFlags() & bmdFrameHasNoInputSource) == 0)
  {
    return parent_device_->FrameCallback(*video_frame, *audio_packet);
  }
  else if (video_frame == nullptr)
  {
    ROS_WARN_NAMED(
        ros::this_node::getName(), "Null frame received, discarding");
  }
  else
  {
    ROS_WARN_THROTTLE_NAMED(
        10, ros::this_node::getName(), "Invalid frame received with flags: %lu",
        video_frame->GetFlags());
  }
  return S_OK;
}

HRESULT FrameOutputCallback::ScheduledFrameCompleted(
    IDeckLinkVideoFrame* completed_frame, BMDOutputFrameCompletionResult result)
{
  if (completed_frame != nullptr)
  {
    return parent_device_->ScheduledFrameCallback(*completed_frame, result);
  }
  else
  {
    ROS_WARN_NAMED(
        ros::this_node::getName(), "Null frame completed, discarding");
  }
  return S_OK;
}

HRESULT FrameOutputCallback::ScheduledPlaybackHasStopped()
{
  return S_OK;
}

DeckLinkDevice::DeckLinkDevice(
    const ros::NodeHandle& nh, const std::string& camera_topic,
    const std::string& camera_frame, DeckLinkHandle device)
    : nh_(nh), it_(nh_), camera_frame_(camera_frame), device_(std::move(device))
{
  // Make the image publisher
  camera_pub_ = it_.advertise(camera_topic, 1, false);

  // Get the attributes interface
  IDeckLinkProfileAttributes* attributes_interface_ptr = nullptr;
  const auto get_attributes_result = device_->QueryInterface(
      IID_IDeckLinkProfileAttributes,
      reinterpret_cast<void**>(&attributes_interface_ptr));
  if (get_attributes_result == S_OK)
  {
    attributes_interface_
        = DeckLinkProfileAttributesHandle(attributes_interface_ptr);
  }
  else
  {
    throw std::runtime_error("Failed to get attributes interface");
  }

  // Check if device supports input format detection
  bool supports_input_format_detection = false;
  const auto input_format_detection_result = attributes_interface_->GetFlag(
      BMDDeckLinkSupportsInputFormatDetection,
      &supports_input_format_detection);
  if (input_format_detection_result == S_OK)
  {
    if (supports_input_format_detection)
    {
      ROS_INFO_NAMED(
          ros::this_node::getName(), "Input format detection is supported");
    }
    else
    {
      ROS_WARN_NAMED(
          ros::this_node::getName(), "Input format detection in NOT supported");
    }
  }
  else
  {
    throw std::runtime_error(
        "Failed to check if input format detection is supported");
  }

  // Get the input interface
  IDeckLinkInput* input_device_ptr = nullptr;
  const auto get_input_result = device_->QueryInterface(
      IID_IDeckLinkInput, reinterpret_cast<void**>(&input_device_ptr));
  if (get_input_result == S_OK)
  {
    input_device_ = DeckLinkInputHandle(input_device_ptr);
  }
  else
  {
    throw std::runtime_error("Failed to get input interface");
  }

  // Get the output interface
  IDeckLinkOutput* output_device_ptr = nullptr;
  const auto get_output_result = device_->QueryInterface(
      IID_IDeckLinkOutput, reinterpret_cast<void**>(&output_device_ptr));
  if (get_output_result == S_OK)
  {
    output_device_ = DeckLinkOutputHandle(output_device_ptr);
  }
  else
  {
    throw std::runtime_error("Failed to get output interface");
  }

  // Make sure the command output format is supported
  bool command_output_format_supported = false;
  const auto get_command_output_format_supported_result
      = output_device_->DoesSupportVideoMode(
          bmdVideoConnectionUnspecified, kDefaultDisplayMode,
          kDefaultOutputPixelFormat, bmdSupportedVideoModeDefault, nullptr,
          &command_output_format_supported);
  if (get_command_output_format_supported_result == S_OK)
  {
    if (!command_output_format_supported)
    {
      throw std::runtime_error("Command output format not supported");
    }
  }
  else
  {
    throw std::runtime_error(
        "Failed to check if command output format is supported");
  }

  // Get output display format information
  IDeckLinkDisplayMode* output_display_mode_ptr = nullptr;
  const auto get_display_mode_result = output_device_->GetDisplayMode(
      kDefaultDisplayMode, &output_display_mode_ptr);
  if (get_display_mode_result != S_OK)
  {
    throw std::runtime_error("Failed to get output display mode");
  }
  DeckLinkDisplayModeHandle output_display_mode(output_display_mode_ptr);

  const auto get_framerate_result = output_display_mode->GetFrameRate(
      &output_frame_duration_, &output_frame_timescale_);
  if (get_framerate_result == S_OK)
  {
    ROS_INFO_NAMED(
        ros::this_node::getName(),
        "Output framerate: %ld (frame duration) %ld (timescale)",
        output_frame_duration_, output_frame_timescale_);
  }
  else
  {
    throw std::runtime_error("Failed to get output framerate");
  }

  // Create a frame for output
  IDeckLinkMutableVideoFrame* blue_reference_output_frame_ptr = nullptr;
  const auto create_reference_frame_result = output_device_->CreateVideoFrame(
      static_cast<int32_t>(output_display_mode->GetWidth()),
      static_cast<int32_t>(output_display_mode->GetHeight()),
      static_cast<int32_t>(output_display_mode->GetWidth() * 4),
      bmdFormat8BitBGRA, bmdFrameFlagDefault, &blue_reference_output_frame_ptr);
  if (create_reference_frame_result == S_OK
      && blue_reference_output_frame_ptr != nullptr)
  {
    blue_reference_output_frame_
        = DeckLinkMutableVideoFrameHandle(blue_reference_output_frame_ptr);
  }
  else
  {
    throw std::runtime_error(
        "Failed to create video frame for reference output");
  }

  // Create a frame for output commands
  IDeckLinkMutableVideoFrame* red_command_output_frame_ptr = nullptr;
  const auto create_command_frame_result = output_device_->CreateVideoFrame(
      static_cast<int32_t>(output_display_mode->GetWidth()),
      static_cast<int32_t>(output_display_mode->GetHeight()),
      static_cast<int32_t>(output_display_mode->GetWidth() * 4),
      bmdFormat8BitBGRA, bmdFrameFlagDefault, &red_command_output_frame_ptr);
  if (create_command_frame_result == S_OK
      && red_command_output_frame_ptr != nullptr)
  {
    red_command_output_frame_
        = DeckLinkMutableVideoFrameHandle(red_command_output_frame_ptr);
  }
  else
  {
    throw std::runtime_error(
        "Failed to create video frame for command output");
  }

  // Fill the output frame with blue pixels
  const uint32_t blue_pixel = 0xff0000ffu;
  const std::vector<uint32_t> blue_fill(
      (output_display_mode->GetWidth() * output_display_mode->GetHeight()),
      blue_pixel);

  uint8_t* reference_frame_buffer = nullptr;
  const auto get_reference_bytes_result
      = blue_reference_output_frame_->GetBytes(
          reinterpret_cast<void**>(&reference_frame_buffer));
  if (get_reference_bytes_result == S_OK && reference_frame_buffer != nullptr)
  {
    std::memcpy(reference_frame_buffer, blue_fill.data(), blue_fill.size() * 4);
  }
  else
  {
    throw std::runtime_error("Failed to get blue reference output frame bytes");
  }

  // Fill the command frame with red pixels
  const uint32_t red_pixel = 0xffff0000u;
  const std::vector<uint32_t> red_fill(
      (output_display_mode->GetWidth() * output_display_mode->GetHeight()),
      red_pixel);

  uint8_t* command_frame_buffer = nullptr;
  const auto get_command_bytes_result
      = red_command_output_frame_->GetBytes(
          reinterpret_cast<void**>(&command_frame_buffer));
  if (get_command_bytes_result == S_OK && command_frame_buffer != nullptr)
  {
    std::memcpy(command_frame_buffer, red_fill.data(), red_fill.size() * 4);
  }
  else
  {
    throw std::runtime_error("Failed to get red command output frame bytes");
  }

  // Create the input callback
  input_callback_
      = DeckLinkInputCallbackHandle(new FrameReceivedCallback(this));

  // Bind the callback
  const auto set_input_callback_result
      = input_device_->SetCallback(input_callback_.get());
  if (set_input_callback_result != S_OK)
  {
    throw std::runtime_error("Failed to set input callback");
  }

  // Create the output callback
  output_callback_
      = DeckLinkOutputCallbackHandle(new FrameOutputCallback(this));

  // Bind the output callback
  const auto set_output_callback_result
      = output_device_->SetScheduledFrameCompletionCallback(
          output_callback_.get());
  if (set_output_callback_result != S_OK)
  {
    throw std::runtime_error("Failed to set output callback");
  }

  // Make the video converter
  video_converter_
      = DeckLinkVideoConversionHandle(CreateVideoConversionInstance());
  if (!video_converter_)
  {
    throw std::runtime_error("Failed to create video converter");
  }

  // We have to setup the conversion frames with defaults, since we will not get
  // an input format changed notification if the real input matches the
  // defaults.
  SetupConversionAndPublishingFrames(kDefaultImageWidth, kDefaultImageHeight);
}

void DeckLinkDevice::StartVideoCapture()
{
  EnableVideoInput(kDefaultDisplayMode, kDefaultInputPixelFormat);
  EnableVideoOutput(kDefaultDisplayMode);
  // Blackmagic's examples all schedule 3 frames before starting scheduled
  // playback.
  for (int32_t i = 0; i < 3; i++)
  {
    if (ScheduleNextFrame(*blue_reference_output_frame_) != S_OK)
    {
      throw std::runtime_error("Failed to schedule starting output frame");
    }
  }
  StartStreams();
  StartScheduledPlayback();
}

void DeckLinkDevice::StopVideoCapture()
{
  StopStreams();
  StopScheduledPlayback();
  DisableVideoInput();
  DisableVideoOutput();
}

void DeckLinkDevice::EnqueueCameraCommand(
    const BlackmagicSDICameraControlMessage& command)
{
  std::lock_guard<std::mutex> lock(camera_command_queue_lock_);
  camera_command_queue_.push_back(command);
}

void DeckLinkDevice::RestartCapture(
    const BMDDisplayMode display_mode, const BMDPixelFormat pixel_format)
{
  EnableVideoInput(display_mode, pixel_format);
  FlushStreams();
  StartStreams();
}

void DeckLinkDevice::EnableVideoInput(
    const BMDDisplayMode display_mode, const BMDPixelFormat pixel_format)
{
  const auto result = input_device_->EnableVideoInput(
      display_mode, pixel_format, bmdVideoInputEnableFormatDetection);
  if (result != S_OK)
  {
    throw std::runtime_error("Failed to enable video input");
  }
}

void DeckLinkDevice::EnableVideoOutput(const BMDDisplayMode display_mode)
{
  const auto result
      = output_device_->EnableVideoOutput(display_mode, bmdVideoOutputVANC);
  if (result != S_OK)
  {
    throw std::runtime_error("Failed to enable video output");
  }
}

void DeckLinkDevice::DisableVideoInput()
{
  const auto result = input_device_->DisableVideoInput();
  if (result != S_OK)
  {
    throw std::runtime_error("Failed to disable video input");
  }
}

void DeckLinkDevice::DisableVideoOutput()
{
  const auto result = output_device_->DisableVideoOutput();
  if (result != S_OK)
  {
    throw std::runtime_error("Failed to disable video output");
  }
}

void DeckLinkDevice::StartStreams()
{
  const auto result = input_device_->StartStreams();
  if (result != S_OK)
  {
    throw std::runtime_error("Failed to start streams");
  }
}

void DeckLinkDevice::FlushStreams()
{
  const auto result = input_device_->FlushStreams();
  if (result != S_OK)
  {
    throw std::runtime_error("Failed to flush streams");
  }
}

void DeckLinkDevice::PauseStreams()
{
  const auto result = input_device_->PauseStreams();
  if (result != S_OK)
  {
    throw std::runtime_error("Failed to pause streams");
  }
}

void DeckLinkDevice::StopStreams()
{
  const auto result = input_device_->StopStreams();
  if (result != S_OK)
  {
    throw std::runtime_error("Failed to stop streams");
  }
}

void DeckLinkDevice::StartScheduledPlayback()
{
  const auto result =
      output_device_->StartScheduledPlayback(0, output_frame_timescale_, 1.0);
  if (result != S_OK)
  {
    throw std::runtime_error("Failed to start scheduled playback");
  }
}

void DeckLinkDevice::StopScheduledPlayback()
{
  const auto result = output_device_->StopScheduledPlayback(0, nullptr, 0);
  if (result != S_OK)
  {
    throw std::runtime_error("Failed to stop scheduled playback");
  }
}

void DeckLinkDevice::SetupConversionAndPublishingFrames(
    const int32_t image_width, const int32_t image_height)
{
  // bgra8 is 4 bytes/pixel
  const int32_t image_step = image_width * 4;

  // Make the video frame for conversion
  IDeckLinkMutableVideoFrame* conversion_video_frame_ptr = nullptr;
  const auto create_video_frame_result = output_device_->CreateVideoFrame(
      image_width, image_height, image_step, bmdFormat8BitBGRA,
      bmdFrameFlagDefault, &conversion_video_frame_ptr);
  if (create_video_frame_result == S_OK
      && conversion_video_frame_ptr != nullptr)
  {
    conversion_frame_
        = DeckLinkMutableVideoFrameHandle(conversion_video_frame_ptr);
  }
  else
  {
    throw std::runtime_error(
        "Failed to create video frame for pixel format conversion");
  }

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

HRESULT DeckLinkDevice::InputFormatChangedCallback(
    const BMDVideoInputFormatChangedEvents notification_events,
    const IDeckLinkDisplayMode& new_display_mode,
    const BMDDetectedVideoInputFormatFlags detected_signal_flags)
{
  // What kind of notification is it?
  if (notification_events & bmdVideoInputDisplayModeChanged)
  {
    ROS_INFO_NAMED(
        ros::this_node::getName(),
        "InputFormatChangedCallback -> bmdVideoInputDisplayModeChanged");
  }
  if (notification_events & bmdVideoInputFieldDominanceChanged)
  {
    ROS_INFO_NAMED(
        ros::this_node::getName(),
        "InputFormatChangedCallback -> bmdVideoInputFieldDominanceChanged");
  }
  if (notification_events & bmdVideoInputColorspaceChanged)
  {
    ROS_INFO_NAMED(
        ros::this_node::getName(),
        "InputFormatChangedCallback -> bmdVideoInputColorspaceChanged");
  }

  // What pixel format does it have?
  BMDPixelFormat pixel_format = bmdFormatUnspecified;
  if (detected_signal_flags == bmdDetectedVideoInputYCbCr422)
  {
    ROS_INFO_NAMED(ros::this_node::getName(), "Detected signal is YCbCr422");
    pixel_format = bmdFormat10BitYUV;
  }
  else if (detected_signal_flags == bmdDetectedVideoInputRGB444)
  {
    ROS_INFO_NAMED(ros::this_node::getName(), "Detected signal is RGB444");
    pixel_format = bmdFormat10BitRGB;
  }
  else if (detected_signal_flags == bmdDetectedVideoInputDualStream3D)
  {
    throw std::runtime_error("Detected signal pixel format not recognized");
  }

  // How big is the image format?
  const int32_t width = static_cast<int32_t>(
      const_cast<IDeckLinkDisplayMode&>(new_display_mode).GetWidth());
  const int32_t height = static_cast<int32_t>(
      const_cast<IDeckLinkDisplayMode&>(new_display_mode).GetHeight());
  ROS_INFO_NAMED(
      ros::this_node::getName(), "New display mode is %d (width) x %d (height)",
      width, height);

  // What is the display mode (resolution + framerate)?
  const BMDDisplayMode display_mode
      = const_cast<IDeckLinkDisplayMode&>(new_display_mode).GetDisplayMode();

  // Reset with new format
  PauseStreams();
  SetupConversionAndPublishingFrames(width, height);
  RestartCapture(display_mode, pixel_format);

  return S_OK;
}

HRESULT DeckLinkDevice::ScheduleNextFrame(IDeckLinkVideoFrame& video_frame)
{
  const auto scheduled_result = output_device_->ScheduleVideoFrame(
      &video_frame, output_frame_counter_ * output_frame_duration_,
      output_frame_duration_, output_frame_timescale_);
  if (scheduled_result == S_OK)
  {
    output_frame_counter_++;
  }
  else
  {
    ROS_ERROR_NAMED(
        ros::this_node::getName(),
        "Failed to schedule video frame: %d", scheduled_result);
  }
  return scheduled_result;
}

HRESULT DeckLinkDevice::FrameCallback(
    const IDeckLinkVideoInputFrame& video_frame,
    const IDeckLinkAudioInputPacket& audio_packet)
{
  (void)(audio_packet);
  const auto convert_frame_result = video_converter_->ConvertFrame(
      const_cast<IDeckLinkVideoInputFrame*>(&video_frame),
      conversion_frame_.get());
  if (convert_frame_result == S_OK)
  {
    uint8_t* frame_buffer = nullptr;
    const auto get_bytes_result
        = conversion_frame_->GetBytes(reinterpret_cast<void**>(&frame_buffer));
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

    // Get ancillary packets
    IDeckLinkVideoFrameAncillaryPackets* ancillary_packets_ptr = nullptr;
    const auto get_ancillary_packets_result
        = const_cast<IDeckLinkVideoInputFrame*>(&video_frame)->QueryInterface(
            IID_IDeckLinkVideoFrameAncillaryPackets,
            reinterpret_cast<void**>(&ancillary_packets_ptr));
    DeckLinkVideoFrameAncillaryPacketsHandle ancillary_packets;
    if (get_ancillary_packets_result == S_OK
        && ancillary_packets_ptr != nullptr)
    {
      ancillary_packets
          = DeckLinkVideoFrameAncillaryPacketsHandle(ancillary_packets_ptr);
    }
    else
    {
      throw std::runtime_error("Failed to retrieve ancillary packets");
    }

    // Get the packet iterator
    IDeckLinkAncillaryPacketIterator* ancillary_packet_iterator_ptr = nullptr;
    const auto get_ancillary_packet_iterator_result
        = ancillary_packets->GetPacketIterator(&ancillary_packet_iterator_ptr);
    DeckLinkAncillaryPacketIteratorHandle ancillary_packet_iterator;
    if (get_ancillary_packet_iterator_result == S_OK
        && ancillary_packet_iterator_ptr != nullptr)
    {
      ancillary_packet_iterator = DeckLinkAncillaryPacketIteratorHandle(
          ancillary_packet_iterator_ptr);
    }
    else
    {
      throw std::runtime_error("Failed to retrieve ancillary packet iterator");
    }

    // Go through the packets
    std::vector<DeckLinkAncillaryPacketHandle> received_packets;

    IDeckLinkAncillaryPacket* ancillary_packet_ptr = nullptr;
    while (ancillary_packet_iterator->Next(&ancillary_packet_ptr) == S_OK)
    {
      received_packets.emplace_back(ancillary_packet_ptr);
    }

    for (size_t idx = 0; idx < received_packets.size(); idx++)
    {
      std::cout << "Received ancillary packet [" << idx << "] with DID"
                << static_cast<uint32_t>(received_packets.at(idx)->GetDID())
                << " and SDID "
                << static_cast<uint32_t>(received_packets.at(idx)->GetSDID())
                << std::endl;
    }
  }
  else
  {
    throw std::runtime_error("Failed to convert video frame");
  }
  return S_OK;
}

HRESULT DeckLinkDevice::ScheduledFrameCallback(
    IDeckLinkVideoFrame& completed_frame,
    const BMDOutputFrameCompletionResult result)
{
  (void)(completed_frame);
  (void)(result);

  // Make a SDI control packet
  BlackmagicSDICameraControlPacketHandle control_packet;
  {
    std::lock_guard<std::mutex> lock(camera_command_queue_lock_);
    const size_t pending_commands = camera_command_queue_.size();
    if (pending_commands > 0)
    {
      ROS_INFO_NAMED(
          ros::this_node::getName(),
          "%zu SDI command(s) pending, sending command frame",
          pending_commands);

      control_packet = BlackmagicSDICameraControlPacketHandle(
          new BlackmagicSDICameraControlPacket());

      // Pack as many camera commands as fit into the ancillary packet
      bool packet_space_remaining = true;
      while ((camera_command_queue_.size() > 0) && packet_space_remaining)
      {
        if (control_packet->AddCameraControlMessage(
                camera_command_queue_.front()))
        {
          camera_command_queue_.pop_front();
        }
        else
        {
          packet_space_remaining = false;
        }
      }

      ROS_INFO_NAMED(
          ros::this_node::getName(),
          "Assembled SDI command packet with %zu of %zu pending command(s)",
          pending_commands - camera_command_queue_.size(), pending_commands);
    }
  }

  if (control_packet)
  {
    // Get ancillary packets
    IDeckLinkVideoFrameAncillaryPackets* ancillary_packets_ptr = nullptr;
    const auto get_ancillary_packets_result
        = red_command_output_frame_->QueryInterface(
            IID_IDeckLinkVideoFrameAncillaryPackets,
            reinterpret_cast<void**>(&ancillary_packets_ptr));
    DeckLinkVideoFrameAncillaryPacketsHandle ancillary_packets;
    if (get_ancillary_packets_result == S_OK
        && ancillary_packets_ptr != nullptr)
    {
      ancillary_packets
          = DeckLinkVideoFrameAncillaryPacketsHandle(ancillary_packets_ptr);
    }
    else
    {
      throw std::runtime_error("Failed to retrieve ancillary packets");
    }

    // Get rid of any existing packets
    const auto detach_packets_result = ancillary_packets->DetachAllPackets();
    if (detach_packets_result != S_OK)
    {
      throw std::runtime_error("Failed to detach existing packets");
    }

    // Add ancillary packet to the frame
    const auto add_packet_result
        = ancillary_packets->AttachPacket(control_packet.get());
    if (add_packet_result != S_OK)
    {
      throw std::runtime_error("Failed to attach control packet");
    }
    // Not sure why this is necessary - Blackmagic's examples suggest that
    // AttachPacket doesn't bump refcount, so we don't want the unique_ptr
    // destructor to get rid of the control packet.
    else
    {
      control_packet.release();
    }
    return ScheduleNextFrame(*red_command_output_frame_);
  }
  else
  {
    return ScheduleNextFrame(*blue_reference_output_frame_);
  }
}
}  // namespace blackmagic_camera_driver
