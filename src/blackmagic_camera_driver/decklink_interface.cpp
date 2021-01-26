#include <blackmagic_camera_driver/decklink_interface.hpp>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>

#include <blackmagic_camera_driver/bmd_handle.hpp>

#include "DeckLinkAPI_v10_11.h"

namespace blackmagic_camera_driver
{
// Default format, overriden by the automatic video input format detection.
const int32_t kDefaultImageWidth = 1920;
const int32_t kDefaultImageHeight = 1080;
const BMDDisplayMode kDefaultInputDisplayMode = bmdModeHD1080p30;
const BMDPixelFormat kDefaultInputPixelFormat = bmdFormat8BitYUV;

// Output video format, used to send VANC commands.
const BMDDisplayMode kOutputDisplayMode = bmdModeHD1080p30;
// Undocumented, but it looks like output must be v210 YUV for VANC data to be
// encoded into frames properly.
const BMDPixelFormat kOutputPixelFormat = bmdFormat10BitYUV;
// TODO(calderpg) Compute this number from frame width instead.
const int32_t kOutputFrameRowSize = 5120;

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
    parent_device_->LogWarn("Null new display mode received, discarding");
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
    parent_device_->LogWarn("Null frame received, discarding");
  }
  else
  {
    parent_device_->LogWarn(
        "Invalid frame received with flags: "
        + std::to_string(video_frame->GetFlags()),
        true);
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
    parent_device_->LogWarn("Null frame completed, discarding");
  }
  return S_OK;
}

HRESULT FrameOutputCallback::ScheduledPlaybackHasStopped()
{
  return S_OK;
}

DeckLinkDevice::DeckLinkDevice(
    const LoggingFunction& logging_fn,
    const VideoFrameSizeChangedCallbackFunction&
        video_frame_size_changed_callback_fn,
    const ConvertedVideoFrameCallbackFunction&
        converted_video_frame_callback_fn,
    DeckLinkHandle device)
    : logging_fn_(logging_fn),
      video_frame_size_changed_callback_fn_(
          video_frame_size_changed_callback_fn),
      converted_video_frame_callback_fn_(converted_video_frame_callback_fn),
      device_(std::move(device))
{
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
      LogInfo("Input format detection is supported");
    }
    else
    {
      LogWarn("Input format detection is NOT supported");
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
          bmdVideoConnectionUnspecified, kOutputDisplayMode,
          kOutputPixelFormat, bmdNoVideoOutputConversion,
          bmdSupportedVideoModeDefault, nullptr,
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
      kOutputDisplayMode, &output_display_mode_ptr);
  if (get_display_mode_result != S_OK)
  {
    throw std::runtime_error("Failed to get output display mode");
  }
  DeckLinkDisplayModeHandle output_display_mode(output_display_mode_ptr);

  const auto get_framerate_result = output_display_mode->GetFrameRate(
      &output_frame_duration_, &output_frame_timescale_);
  if (get_framerate_result == S_OK)
  {
    LogInfo(
        "Output framerate: " + std::to_string(output_frame_duration_)
        + " (frame duration) " + std::to_string(output_frame_timescale_)
        + " (timescale)");
  }
  else
  {
    throw std::runtime_error("Failed to get output framerate");
  }

  // Create a frame for reference output
  IDeckLinkMutableVideoFrame* reference_output_frame_ptr = nullptr;
  const auto create_reference_frame_result = output_device_->CreateVideoFrame(
      static_cast<int32_t>(output_display_mode->GetWidth()),
      static_cast<int32_t>(output_display_mode->GetHeight()),
      kOutputFrameRowSize, kOutputPixelFormat, bmdFrameFlagDefault,
      &reference_output_frame_ptr);
  if (create_reference_frame_result == S_OK
      && reference_output_frame_ptr != nullptr)
  {
    reference_output_frame_
        = DeckLinkMutableVideoFrameHandle(reference_output_frame_ptr);
  }
  else
  {
    throw std::runtime_error(
        "Failed to create video frame for reference output");
  }

  // Create a frame for output commands
  IDeckLinkMutableVideoFrame* command_output_frame_ptr = nullptr;
  const auto create_command_frame_result = output_device_->CreateVideoFrame(
      static_cast<int32_t>(output_display_mode->GetWidth()),
      static_cast<int32_t>(output_display_mode->GetHeight()),
      kOutputFrameRowSize, kOutputPixelFormat, bmdFrameFlagDefault,
      &command_output_frame_ptr);
  if (create_command_frame_result == S_OK
      && command_output_frame_ptr != nullptr)
  {
    command_output_frame_
        = DeckLinkMutableVideoFrameHandle(command_output_frame_ptr);
  }
  else
  {
    throw std::runtime_error(
        "Failed to create video frame for command output");
  }

  // Fill the output frame with cyan pixels
  uint32_t* reference_frame_buffer = nullptr;
  const auto get_reference_bytes_result
      = reference_output_frame_->GetBytes(
          reinterpret_cast<void**>(&reference_frame_buffer));
  if (get_reference_bytes_result == S_OK && reference_frame_buffer != nullptr)
  {
    const std::vector<uint32_t> cyan_yuv_pixels
        = { 0x040aa298, 0x2a8a62a8, 0x298aa040, 0x2a8102a8};
    const size_t num_words
        = (kOutputFrameRowSize * output_display_mode->GetHeight())
            / sizeof(uint32_t);
    for (size_t word = 0; word < num_words; word += 4)
    {
      reference_frame_buffer[word + 0] = cyan_yuv_pixels.at(0);
      reference_frame_buffer[word + 1] = cyan_yuv_pixels.at(1);
      reference_frame_buffer[word + 2] = cyan_yuv_pixels.at(2);
      reference_frame_buffer[word + 3] = cyan_yuv_pixels.at(3);
    }
  }
  else
  {
    throw std::runtime_error("Failed to get reference output frame bytes");
  }

  // Fill the command frame with cyan pixels
  uint32_t* command_frame_buffer = nullptr;
  const auto get_command_bytes_result
      = command_output_frame_->GetBytes(
          reinterpret_cast<void**>(&command_frame_buffer));
  if (get_command_bytes_result == S_OK && command_frame_buffer != nullptr)
  {
    const std::vector<uint32_t> cyan_yuv_pixels
        = { 0x040aa298, 0x2a8a62a8, 0x298aa040, 0x2a8102a8};
    const size_t num_words
        = (kOutputFrameRowSize * output_display_mode->GetHeight())
            / sizeof(uint32_t);
    for (size_t word = 0; word < num_words; word += 4)
    {
      command_frame_buffer[word + 0] = cyan_yuv_pixels.at(0);
      command_frame_buffer[word + 1] = cyan_yuv_pixels.at(1);
      command_frame_buffer[word + 2] = cyan_yuv_pixels.at(2);
      command_frame_buffer[word + 3] = cyan_yuv_pixels.at(3);
    }
  }
  else
  {
    throw std::runtime_error("Failed to get command output frame bytes");
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
  EnableVideoInput(kDefaultInputDisplayMode, kDefaultInputPixelFormat);
  EnableVideoOutput(kOutputDisplayMode);
  // Blackmagic's examples all schedule 3 frames before starting scheduled
  // playback.
  for (int32_t i = 0; i < 3; i++)
  {
    if (ScheduleNextFrame(*reference_output_frame_) != S_OK)
    {
      throw std::runtime_error("Failed to schedule starting output frame");
    }
  }
  StartStreams();
  StartScheduledPlayback();
  TallyOn();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

void DeckLinkDevice::StopVideoCapture()
{
  TallyOff();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
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

  video_frame_size_changed_callback_fn_(image_width, image_height, image_step);
}

HRESULT DeckLinkDevice::InputFormatChangedCallback(
    const BMDVideoInputFormatChangedEvents notification_events,
    const IDeckLinkDisplayMode& new_display_mode,
    const BMDDetectedVideoInputFormatFlags detected_signal_flags)
{
  // What kind of notification is it?
  if (notification_events & bmdVideoInputDisplayModeChanged)
  {
    LogInfo("InputFormatChangedCallback -> bmdVideoInputDisplayModeChanged");
  }
  if (notification_events & bmdVideoInputFieldDominanceChanged)
  {
    LogInfo("InputFormatChangedCallback -> bmdVideoInputFieldDominanceChanged");
  }
  if (notification_events & bmdVideoInputColorspaceChanged)
  {
    LogInfo("InputFormatChangedCallback -> bmdVideoInputColorspaceChanged");
  }

  // What pixel format does it have?
  bool is_ycrcb422 = false;
  bool is_rgb444 = false;

  if (detected_signal_flags & bmdDetectedVideoInputYCbCr422)
  {
    LogInfo("Detected signal is YCbCr422");
    is_ycrcb422 = true;
  }
  if (detected_signal_flags & bmdDetectedVideoInputRGB444)
  {
    LogInfo("Detected signal is RGB444");
    is_rgb444 = true;
  }

  if (is_ycrcb422 == is_rgb444)
  {
    throw std::runtime_error(
        "Detected signal must be EITHER YCbCr422 OR RGB444");
  }

  if (detected_signal_flags & bmdDetectedVideoInputDualStream3D)
  {
    throw std::runtime_error("Dual-stream 3D is not supported");
  }

  bool is_8bit = false;
  bool is_10bit = false;
  bool is_12bit = false;

  if (detected_signal_flags & bmdDetectedVideoInput8BitDepth)
  {
    LogInfo("Detected signal is 8-bit");
    is_8bit = true;
  }
  if (detected_signal_flags & bmdDetectedVideoInput10BitDepth)
  {
    LogInfo("Detected signal is 10-bit");
    is_10bit = true;
  }
  if (detected_signal_flags & bmdDetectedVideoInput12BitDepth)
  {
    LogInfo("Detected signal is 12-bit");
    is_12bit = true;
  }

  if (!((is_8bit && !is_10bit && !is_12bit) ||
        (!is_8bit && is_10bit && !is_12bit) ||
        (!is_8bit && !is_10bit && is_12bit)))
  {
    throw std::runtime_error(
        "Detected bit-depth must be EITHER 8 OR 10 OR 12 bits");
  }


  BMDPixelFormat pixel_format = bmdFormatUnspecified;
  if (is_ycrcb422)
  {
    if (is_8bit)
    {
      LogInfo("Determined pixel format: bmdFormat8BitYUV");
      pixel_format = bmdFormat8BitYUV;
    }
    else if (is_10bit)
    {
      LogInfo("Determined pixel format: bmdFormat10BitYUV");
      pixel_format = bmdFormat10BitYUV;
    }
    else if (is_12bit)
    {
      throw std::runtime_error("Cannot combine YCrCb422 with 12-bit depth");
    }
  }
  else if (is_rgb444)
  {
    if (is_8bit)
    {
      LogInfo("Determined pixel format: bmdFormat8BitBGRA");
      pixel_format = bmdFormat8BitBGRA;
    }
    else if (is_10bit)
    {
      LogInfo("Determined pixel format: bmdFormat10BitRGB");
      pixel_format = bmdFormat10BitRGB;
    }
    else if (is_12bit)
    {
      LogInfo("Determined pixel format: bmdFormat12BitRGB");
      pixel_format = bmdFormat12BitRGB;
    }
  }

  if (pixel_format == bmdFormatUnspecified)
  {
    throw std::runtime_error("Unable to determine pixel format");
  }

  // How big is the image format?
  const int32_t width = static_cast<int32_t>(
      const_cast<IDeckLinkDisplayMode&>(new_display_mode).GetWidth());
  const int32_t height = static_cast<int32_t>(
      const_cast<IDeckLinkDisplayMode&>(new_display_mode).GetHeight());
  LogInfo(
      "New display mode is " + std::to_string(width) + " (width) x "
      + std::to_string(height) + " (height)");

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
  LogVideoFrameAncillaryPackets(video_frame, "ScheduleNextFrame", true);

  const auto scheduled_result = output_device_->ScheduleVideoFrame(
      &video_frame, output_frame_counter_ * output_frame_duration_,
      output_frame_duration_, output_frame_timescale_);
  if (scheduled_result == S_OK)
  {
    output_frame_counter_++;
  }
  else
  {
    LogError(
        "Failed to schedule video frame: " + std::to_string(scheduled_result));
  }
  return scheduled_result;
}

void DeckLinkDevice::LogVideoFrameAncillaryPackets(
    const IDeckLinkVideoFrame& video_frame, const std::string& msg,
    const bool log_debug)
{
  // Get ancillary packets
  IDeckLinkVideoFrameAncillaryPackets* ancillary_packets_ptr = nullptr;
  const auto get_ancillary_packets_result
      = const_cast<IDeckLinkVideoFrame*>(&video_frame)->QueryInterface(
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

  while (true)
  {
    IDeckLinkAncillaryPacket* ancillary_packet_ptr = nullptr;
    if (ancillary_packet_iterator->Next(&ancillary_packet_ptr) == S_OK)
    {
      received_packets.emplace_back(ancillary_packet_ptr);
    }
    else
    {
      break;
    }
  }

  if (received_packets.size() > 0)
  {
    std::string log_string =
        "[" + msg + "] Video frame contains "
        + std::to_string(received_packets.size()) + " SDI ancillary packets:";

    for (size_t idx = 0; idx < received_packets.size(); idx++)
    {
      const auto& packet = received_packets.at(idx);

      const uint8_t* packet_data_ptr = nullptr;
      uint32_t packet_data_size = 0u;
      const auto get_bytes_result = packet->GetBytes(
          bmdAncillaryPacketFormatUInt8,
          reinterpret_cast<const void**>(&packet_data_ptr), &packet_data_size);

      std::string data_str;
      if (get_bytes_result == S_OK)
      {
        for (uint32_t data_idx = 0; data_idx < packet_data_size; data_idx++)
        {
          const uint8_t data_val = packet_data_ptr[data_idx];
          if (data_idx > 0)
          {
            data_str += ", ";
          }
          data_str += std::to_string(static_cast<uint32_t>(data_val));
        }
      }
      else
      {
        data_str = "non-uint8_t packet data";
      }

      log_string +=
          "\nAncillary packet [" + std::to_string(idx) + "] has DID 0x"
          + HexPrint(packet->GetDID()) + ", SDID 0x"
          + HexPrint(packet->GetSDID()) + ", VANC line "
          + std::to_string(packet->GetLineNumber()) + ", data stream index 0x"
          + HexPrint(packet->GetDataStreamIndex()) + ", and data: [" + data_str
          + "]";
    }

    if (log_debug) {
      LogDebug(log_string);
    } else {
      LogInfo(log_string);
    }
  }
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
    converted_video_frame_callback_fn_(*conversion_frame_);
    LogVideoFrameAncillaryPackets(video_frame, "FrameCallback", false);
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
  using BlackmagicSDICameraControlPacketHandle
      = BMDHandle<BlackmagicSDICameraControlPacket>;

  BlackmagicSDICameraControlPacketHandle control_packet;
  {
    std::lock_guard<std::mutex> lock(camera_command_queue_lock_);
    const size_t num_pending_commands = camera_command_queue_.size();
    if (num_pending_commands > 0)
    {
      LogInfo(
          std::to_string(num_pending_commands)
          + " SDI command(s) pending, sending command frame");

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

      LogInfo(
          "Assembled SDI command packet with "
          + std::to_string(num_pending_commands - camera_command_queue_.size())
          + " of " + std::to_string(num_pending_commands)
          + " pending command(s)");
    }
  }

  // Get ancillary packets
  IDeckLinkVideoFrameAncillaryPackets* ancillary_packets_ptr = nullptr;
  const auto get_ancillary_packets_result
      = command_output_frame_->QueryInterface(
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

  // Add a tally packet
  DeckLinkAncillaryPacketHandle tally_packet(
      new BlackmagicSDITallyControlPacket(enable_tally_.load()));

  const auto add_tally_packet_result
      = ancillary_packets->AttachPacket(tally_packet.get());
  if (add_tally_packet_result != S_OK)
  {
    throw std::runtime_error("Failed to attach tally control packet");
  }

  // Add a control packet
  if (control_packet)
  {
    // Add ancillary packet to the frame
    const auto add_control_packet_result
        = ancillary_packets->AttachPacket(control_packet.get());
    if (add_control_packet_result != S_OK)
    {
      throw std::runtime_error("Failed to attach camera control packet");
    }
  }
  return ScheduleNextFrame(*command_output_frame_);
}

std::vector<DeckLinkHandle> GetDeckLinkDevices()
{
  // Find available DeckLink devices
  std::vector<DeckLinkHandle> decklink_devices;

  DeckLinkIteratorHandle device_iterator(CreateDeckLinkIteratorInstance());
  if (device_iterator)
  {
    while (true)
    {
      IDeckLink* decklink_device_ptr = nullptr;
      if (device_iterator->Next(&decklink_device_ptr) == S_OK)
      {
        decklink_devices.emplace_back(decklink_device_ptr);
      }
      else
      {
        break;
      }
    }
  }

  return decklink_devices;
}
}  // namespace blackmagic_camera_driver
