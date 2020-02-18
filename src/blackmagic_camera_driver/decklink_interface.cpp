#include <blackmagic_camera_driver/decklink_interface.hpp>

#include <atomic>
#include <cstdint>
#include <iostream>
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
const BMDPixelFormat kDefaultPixelFormat = bmdFormat8BitYUV;

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
    std::cout << "Null new display mode received, discarding" << std::endl;
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
    std::cout << "Null frame received, discarding" << std::endl;
  }
  else
  {
    std::cout << "Invalid frame received:" << std::endl;
    std::cout << "Frame width: " << video_frame->GetWidth() << std::endl;
    std::cout << "Frame height: " << video_frame->GetHeight() << std::endl;
    std::cout << "Frame bytes/row: " << video_frame->GetRowBytes() << std::endl;
    std::cout << "Pixel format: " << video_frame->GetPixelFormat() << std::endl;
    std::cout << "Frame flags: " << video_frame->GetFlags() << std::endl;
  }
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
      std::cout << "Input format detection is supported" << std::endl;
    }
    else
    {
      std::cout << "Input format detection in NOT supported" << std::endl;
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

  // Create the input callback
  input_callback_
      = DeckLinkInputCallbackHandle(new FrameReceivedCallback(this));

  // Bind the callback
  const auto set_callback_result
      = input_device_->SetCallback(input_callback_.get());
  if (set_callback_result != S_OK)
  {
    throw std::runtime_error("Failed to set input callback");
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
  EnableVideoInput(kDefaultDisplayMode, kDefaultPixelFormat);
  StartStreams();
}

void DeckLinkDevice::StopVideoCapture()
{
  StopStreams();
  DisableVideoInput();
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

void DeckLinkDevice::DisableVideoInput()
{
  const auto result = input_device_->DisableVideoInput();
  if (result != S_OK)
  {
    throw std::runtime_error("Failed to disable video input");
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

void DeckLinkDevice::SetupConversionAndPublishingFrames(
    const int32_t image_width, const int32_t image_height)
{
  // bgra8 is 4 bytes/pixel
  const int32_t image_step = image_width * 4;

  // Make the video frame for conversion
  IDeckLinkMutableVideoFrame* conversion_video_frame_ptr = nullptr;
  const auto create_video_frame_result = output_device_->CreateVideoFrame(
      image_width, image_height, image_step, bmdFormat8BitBGRA,
      bmdVideoInputFlagDefault, &conversion_video_frame_ptr);
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
    std::cout << "InputFormatChangedCallback -> bmdVideoInputDisplayModeChanged"
              << std::endl;
  }
  if (notification_events & bmdVideoInputFieldDominanceChanged)
  {
    std::cout
        << "InputFormatChangedCallback -> bmdVideoInputFieldDominanceChanged"
        << std::endl;
  }
  if (notification_events & bmdVideoInputColorspaceChanged)
  {
    std::cout << "InputFormatChangedCallback -> bmdVideoInputColorspaceChanged"
              << std::endl;
  }

  // What pixel format does it have?
  BMDPixelFormat pixel_format = bmdFormatUnspecified;
  if (detected_signal_flags == bmdDetectedVideoInputYCbCr422)
  {
    std::cout << "Detected signal is YCbCr422" << std::endl;
    pixel_format = bmdFormat10BitYUV;
  }
  else if (detected_signal_flags == bmdDetectedVideoInputRGB444)
  {
    std::cout << "Detected signal is RGB444" << std::endl;
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
  std::cout << "New display mode is " << width << " (width) " << height
            << " (height)" << std::endl;

  // What is the display mode (resolution + framerate)?
  const BMDDisplayMode display_mode
      = const_cast<IDeckLinkDisplayMode&>(new_display_mode).GetDisplayMode();

  // Reset with new format
  PauseStreams();
  SetupConversionAndPublishingFrames(width, height);
  RestartCapture(display_mode, pixel_format);

  return S_OK;
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
  }
  else
  {
    throw std::runtime_error("Failed to convert video frame");
  }
  return S_OK;
}
}  // namespace blackmagic_camera_driver

