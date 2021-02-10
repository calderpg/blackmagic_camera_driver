#include <blackmagic_camera_driver/decklink_interface.hpp>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstring>
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

// Poorly documented, many BMD devices only output VANC data when run in
// v210 YUV output display mode. To be careful, we always output in v210 YUV.
const BMDPixelFormat kOutputPixelFormat = bmdFormat10BitYUV;

void CopyVideoFrameBytes(
    const IDeckLinkVideoFrame& source_frame,
    IDeckLinkVideoFrame& destination_frame)
{
  // Const-cast the source frame, because all getters are non-const.
  IDeckLinkVideoFrame& raw_source_frame
      = const_cast<IDeckLinkVideoFrame&>(source_frame);

  // Ensure frame resolutions and formats match
  const auto source_width = raw_source_frame.GetWidth();
  const auto source_height = raw_source_frame.GetHeight();
  const auto source_pixel_format = raw_source_frame.GetPixelFormat();

  const auto destination_width = destination_frame.GetWidth();
  const auto destination_height = destination_frame.GetHeight();
  const auto destination_pixel_format = destination_frame.GetPixelFormat();

  if (source_width != destination_width)
  {
    throw std::runtime_error("Source and destination widths do not match");
  }

  if (source_height != destination_height)
  {
    throw std::runtime_error("Source and destination heights do not match");
  }

  if (source_pixel_format != destination_pixel_format)
  {
    throw std::runtime_error(
        "Source and destination pixel formats do not match");
  }

  // Get frame data sizes
  const size_t source_frame_bytes
      = raw_source_frame.GetRowBytes() * raw_source_frame.GetHeight();

  const size_t destination_frame_bytes
      = destination_frame.GetRowBytes() * destination_frame.GetHeight();

  if (source_frame_bytes != destination_frame_bytes)
  {
    throw std::runtime_error(
        "Source and destination frames are different sizes");
  }

  // Get frame data pointers
  void* source_frame_buffer = nullptr;
  const auto get_source_bytes_result
      = raw_source_frame.GetBytes(&source_frame_buffer);
  if (get_source_bytes_result != S_OK || source_frame_buffer == nullptr)
  {
    throw std::runtime_error("Failed to get source frame bytes");
  }

  void* destination_frame_buffer = nullptr;
  const auto get_destination_bytes_result
      = destination_frame.GetBytes(&destination_frame_buffer);
  if (get_destination_bytes_result != S_OK
      || destination_frame_buffer == nullptr)
  {
    throw std::runtime_error("Failed to get destination frame bytes");
  }

  // Copy frame data
  std::memcpy(
      destination_frame_buffer, source_frame_buffer, destination_frame_bytes);
}

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

// Base class
DeckLinkBaseDevice::DeckLinkBaseDevice(
    const LoggingFunction& logging_fn, DeckLinkHandle device)
{
  InitializeBaseDevice(logging_fn, std::move(device));
}

void DeckLinkBaseDevice::InitializeBaseDevice(
      const LoggingFunction& logging_fn, DeckLinkHandle device)
{
  logging_fn_ = logging_fn;
  device_ = std::move(device);

  // Get the name of the device
  const char* display_name = nullptr;
  const auto get_display_name_result = device_->GetDisplayName(&display_name);
  if (get_display_name_result != S_OK || display_name == nullptr)
  {
    throw std::runtime_error("Failed to get device display name");
  }
  LogInfo("Connecting to device [" + std::string(display_name) + "]");
  free(const_cast<char*>(display_name));
  display_name = nullptr;

  // Get the attributes interface
  const auto get_attributes_result = device_->QueryInterface(
      IID_IDeckLinkProfileAttributes,
      reinterpret_cast<void**>(&attributes_interface_));
  if (get_attributes_result != S_OK || attributes_interface_ == nullptr)
  {
    throw std::runtime_error("Failed to get attributes interface");
  }

  // Make the video converter
  video_converter_
      = DeckLinkVideoConversionHandle(CreateVideoConversionInstance());
  if (!video_converter_)
  {
    throw std::runtime_error("Failed to create video converter");
  }
}

void DeckLinkBaseDevice::LogVideoFrameAncillaryPackets(
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

// Input device class
DeckLinkInputDevice::DeckLinkInputDevice(
    const LoggingFunction& logging_fn,
    const VideoFrameSizeChangedCallbackFunction&
        video_frame_size_changed_callback_fn,
    const ConvertedVideoFrameCallbackFunction&
        converted_video_frame_callback_fn,
    DeckLinkHandle device)
{
  InitializeBaseDevice(logging_fn, std::move(device));
  InitializeInputDevice(
      video_frame_size_changed_callback_fn, converted_video_frame_callback_fn);
}

void DeckLinkInputDevice::InitializeInputDevice(
    const VideoFrameSizeChangedCallbackFunction&
        video_frame_size_changed_callback_fn,
    const ConvertedVideoFrameCallbackFunction&
        converted_video_frame_callback_fn)
{
  video_frame_size_changed_callback_fn_ =
      video_frame_size_changed_callback_fn;
  converted_video_frame_callback_fn_ = converted_video_frame_callback_fn;

  // Check if device supports input format detection
  bool supports_input_format_detection = false;
  const auto input_format_detection_result = GetAttributesInterface().GetFlag(
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
  const auto get_input_result = GetDevice().QueryInterface(
      IID_IDeckLinkInput, reinterpret_cast<void**>(&input_device_));
  if (get_input_result != S_OK || input_device_ == nullptr)
  {
    throw std::runtime_error("Failed to get input interface");
  }

  // Create the input callback
  input_callback_ = FrameReceivedCallback::Make(this);

  // Bind the callback
  const auto set_input_callback_result
      = input_device_->SetCallback(input_callback_.get());
  if (set_input_callback_result != S_OK)
  {
    throw std::runtime_error("Failed to set input callback");
  }

  // We have to setup the conversion frames with defaults, since we will not get
  // an input format changed notification if the real input matches the
  // defaults.
  SetupConversionAndPublishingFrames(kDefaultImageWidth, kDefaultImageHeight);
}

void DeckLinkInputDevice::RestartCapture(
    const BMDDisplayMode display_mode, const BMDPixelFormat pixel_format)
{
  EnableVideoInput(display_mode, pixel_format);
  FlushStreams();
  StartStreams();
}

void DeckLinkInputDevice::EnableVideoInput(
    const BMDDisplayMode display_mode, const BMDPixelFormat pixel_format)
{
  const auto result = input_device_->EnableVideoInput(
      display_mode, pixel_format, bmdVideoInputEnableFormatDetection);
  if (result != S_OK)
  {
    throw std::runtime_error("Failed to enable video input");
  }
}

void DeckLinkInputDevice::DisableVideoInput()
{
  // Since this gets called at shutdown, no reason to error check. Nothing we
  // can do to handle errors at this point.
  input_device_->DisableVideoInput();
}

void DeckLinkInputDevice::StartStreams()
{
  const auto result = input_device_->StartStreams();
  if (result != S_OK)
  {
    throw std::runtime_error("Failed to start streams");
  }
}

void DeckLinkInputDevice::FlushStreams()
{
  const auto result = input_device_->FlushStreams();
  if (result != S_OK)
  {
    throw std::runtime_error("Failed to flush streams");
  }
}

void DeckLinkInputDevice::PauseStreams()
{
  const auto result = input_device_->PauseStreams();
  if (result != S_OK)
  {
    throw std::runtime_error("Failed to pause streams");
  }
}

void DeckLinkInputDevice::StopStreams()
{
  // No error checking here, since the only non-S_OK return comes in the case
  // streams are already stopped.
  input_device_->StopStreams();
}

void DeckLinkInputDevice::SetupConversionAndPublishingFrames(
    const int64_t image_width, const int64_t image_height)
{
  // bgra8 is 4 bytes/pixel
  const int64_t row_bytes = image_width * 4;

  // Make the video frame for input conversion
  input_conversion_frame_ = BMDCompatibleVideoFrame::Make(
      image_width, image_height, row_bytes, bmdFormat8BitBGRA,
      bmdFrameFlagDefault);

  video_frame_size_changed_callback_fn_(image_width, image_height, row_bytes);
}

HRESULT DeckLinkInputDevice::InputFormatChangedCallback(
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
  const int64_t width =
      const_cast<IDeckLinkDisplayMode&>(new_display_mode).GetWidth();
  const int64_t height =
      const_cast<IDeckLinkDisplayMode&>(new_display_mode).GetHeight();
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

HRESULT DeckLinkInputDevice::FrameCallback(
    const IDeckLinkVideoInputFrame& video_frame,
    const IDeckLinkAudioInputPacket& audio_packet)
{
  (void)(audio_packet);
  const auto convert_frame_result = GetVideoConverter().ConvertFrame(
      const_cast<IDeckLinkVideoInputFrame*>(&video_frame),
      input_conversion_frame_.get());
  if (convert_frame_result == S_OK)
  {
    converted_video_frame_callback_fn_(*input_conversion_frame_);
    LogVideoFrameAncillaryPackets(video_frame, "FrameCallback", false);
  }
  else
  {
    throw std::runtime_error("Failed to convert video frame");
  }
  return S_OK;
}

// Output device class
DeckLinkOutputDevice::DeckLinkOutputDevice(
    const LoggingFunction& logging_fn, const BMDDisplayMode output_mode,
    DeckLinkHandle device)
{
  InitializeBaseDevice(logging_fn, std::move(device));
  InitializeOutputDevice(output_mode);
}

void DeckLinkOutputDevice::InitializeOutputDevice(
    const BMDDisplayMode output_mode)
{
  output_display_mode_ = output_mode;

  // Get the output interface
  const auto get_output_result = GetDevice().QueryInterface(
      IID_IDeckLinkOutput, reinterpret_cast<void**>(&output_device_));
  if (get_output_result != S_OK || output_device_ == nullptr)
  {
    throw std::runtime_error("Failed to get output interface");
  }

  // Make sure the command output format is supported
  bool command_output_format_supported = false;
  const auto get_command_output_format_supported_result
      = output_device_->DoesSupportVideoMode(
          bmdVideoConnectionUnspecified, output_display_mode_,
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
      output_display_mode_, &output_display_mode_ptr);
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

  const int32_t output_frame_row_bytes = static_cast<int32_t>(
      Calc10BitYUVRowBytes(output_display_mode->GetWidth()));
  LogInfo(
      "Output resolution: "
      + std::to_string(output_display_mode->GetWidth()) + " (width) "
      + std::to_string(output_display_mode->GetHeight()) + " (height) "
      + std::to_string(output_frame_row_bytes) + " (bytes/row)");

  // Create a frame for reference output
  IDeckLinkMutableVideoFrame* reference_output_frame_ptr = nullptr;
  const auto create_reference_frame_result = output_device_->CreateVideoFrame(
      static_cast<int32_t>(output_display_mode->GetWidth()),
      static_cast<int32_t>(output_display_mode->GetHeight()),
      output_frame_row_bytes, kOutputPixelFormat, bmdFrameFlagDefault,
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

  // Create a frame to store the next output
  IDeckLinkMutableVideoFrame* active_output_frame_ptr = nullptr;
  const auto create_active_frame_result = output_device_->CreateVideoFrame(
      static_cast<int32_t>(output_display_mode->GetWidth()),
      static_cast<int32_t>(output_display_mode->GetHeight()),
      output_frame_row_bytes, kOutputPixelFormat, bmdFrameFlagDefault,
      &active_output_frame_ptr);
  if (create_active_frame_result == S_OK
      && active_output_frame_ptr != nullptr)
  {
    active_output_frame_
        = DeckLinkMutableVideoFrameHandle(active_output_frame_ptr);
  }
  else
  {
    throw std::runtime_error(
        "Failed to create video frame for active output");
  }

  // Create a frame for output commands
  IDeckLinkMutableVideoFrame* command_output_frame_ptr = nullptr;
  const auto create_command_frame_result = output_device_->CreateVideoFrame(
      static_cast<int32_t>(output_display_mode->GetWidth()),
      static_cast<int32_t>(output_display_mode->GetHeight()),
      output_frame_row_bytes, kOutputPixelFormat, bmdFrameFlagDefault,
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
        = (output_frame_row_bytes * output_display_mode->GetHeight())
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

  // Fill the active and command frames from the reference frame
  CopyVideoFrameBytes(*reference_output_frame_, *active_output_frame_);
  CopyVideoFrameBytes(*reference_output_frame_, *command_output_frame_);

  // Create the output callback
  output_callback_ = FrameOutputCallback::Make(this);

  // Bind the output callback
  const auto set_output_callback_result
      = output_device_->SetScheduledFrameCompletionCallback(
          output_callback_.get());
  if (set_output_callback_result != S_OK)
  {
    throw std::runtime_error("Failed to set output callback");
  }
}

void DeckLinkOutputDevice::EnqueueCameraCommand(
    const BlackmagicSDICameraControlMessage& command)
{
  std::lock_guard<std::mutex> lock(camera_command_queue_lock_);
  camera_command_queue_.push_back(command);
}

void DeckLinkOutputDevice::ClearOutputQueueAndResetOutputToReferenceFrame()
{
  std::lock_guard<std::mutex> lock(output_frame_queue_lock_);
  output_frame_queue_.clear();
  CopyVideoFrameBytes(*reference_output_frame_, *active_output_frame_);
}

void DeckLinkOutputDevice::EnqueueOutputFrame(
    BMDHandle<BMDCompatibleVideoFrame> output_frame)
{
  const auto reference_width = reference_output_frame_->GetWidth();
  const auto reference_height = reference_output_frame_->GetHeight();
  const auto output_width = output_frame->GetWidth();
  const auto output_height = output_frame->GetHeight();

  if (reference_width != output_width)
  {
    throw std::runtime_error("Output frame width does not match");
  }
  if (reference_height != output_height)
  {
    throw std::runtime_error("Output frame height does not match");
  }

  if (output_frame->GetPixelFormat() == kOutputPixelFormat)
  {
    // If the output format is already v210 YUV, enqueue directly
    std::lock_guard<std::mutex> lock(output_frame_queue_lock_);
    output_frame_queue_.push_back(std::move(output_frame));
  }
  else
  {
    // Convert the provided frame to v210 YUV, then enqueue
    auto conversion_frame = CreateYUV10OutputVideoFrame();
    const auto convert_frame_result = GetVideoConverter().ConvertFrame(
        output_frame.get(), conversion_frame.get());
    if (convert_frame_result != S_OK)
    {
      throw std::runtime_error("Failed to convert output video frame");
    }

    std::lock_guard<std::mutex> lock(output_frame_queue_lock_);
    output_frame_queue_.push_back(std::move(conversion_frame));
  }
}

BMDHandle<BMDCompatibleVideoFrame>
DeckLinkOutputDevice::CreateBGRA8OutputVideoFrame()
{
  const int64_t image_width = reference_output_frame_->GetWidth();
  const int64_t image_height = reference_output_frame_->GetHeight();

  // BGRA8 is 4 bytes/pixel
  const int64_t image_step = image_width * 4;

  return BMDCompatibleVideoFrame::Make(
      image_width, image_height, image_step, bmdFormat8BitBGRA,
      bmdFrameFlagDefault);
}

BMDHandle<BMDCompatibleVideoFrame>
DeckLinkOutputDevice::CreateYUV10OutputVideoFrame()
{
  const int64_t image_width = reference_output_frame_->GetWidth();
  const int64_t image_height = reference_output_frame_->GetHeight();

  const int64_t image_step = Calc10BitYUVRowBytes(image_width);

  return BMDCompatibleVideoFrame::Make(
      image_width, image_height, image_step, kOutputPixelFormat,
      bmdFrameFlagDefault);
}

void DeckLinkOutputDevice::EnableVideoOutput()
{
  const auto result = output_device_->EnableVideoOutput(
      output_display_mode_, bmdVideoOutputVANC);
  if (result != S_OK)
  {
    throw std::runtime_error("Failed to enable video output");
  }
}

void DeckLinkOutputDevice::DisableVideoOutput()
{
  // Since this gets called at shutdown, no reason to error check. Nothing we
  // can do to handle errors at this point.
  output_device_->DisableVideoOutput();
}

void DeckLinkOutputDevice::StartScheduledPlayback()
{
  const auto result =
      output_device_->StartScheduledPlayback(0, output_frame_timescale_, 1.0);
  if (result != S_OK)
  {
    throw std::runtime_error("Failed to start scheduled playback");
  }
}

void DeckLinkOutputDevice::StopScheduledPlayback()
{
  bool scheduled_playback_running = false;
  const auto playback_running_result =
      output_device_->IsScheduledPlaybackRunning(&scheduled_playback_running);
  if (playback_running_result != S_OK)
  {
    throw std::runtime_error(
        "Failed to check if scheduled playback is running");
  }

  if (scheduled_playback_running)
  {
    const auto playback_stopped_result =
        output_device_->StopScheduledPlayback(0, nullptr, 0);
    if (playback_stopped_result != S_OK)
    {
      throw std::runtime_error("Failed to stop scheduled playback");
    }
  }
}

void DeckLinkOutputDevice::PreScheduleReferenceFrame()
{
  // Blackmagic's examples all schedule 3 frames before starting scheduled
  // playback.
  for (int32_t i = 0; i < 3; i++)
  {
    if (ScheduleNextFrame(*reference_output_frame_) != S_OK)
    {
      throw std::runtime_error("Failed to schedule starting output frame");
    }
  }
}

HRESULT
DeckLinkOutputDevice::ScheduleNextFrame(IDeckLinkVideoFrame& video_frame)
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

HRESULT DeckLinkOutputDevice::ScheduledFrameCallback(
    IDeckLinkVideoFrame& completed_frame,
    const BMDOutputFrameCompletionResult result)
{
  (void)(completed_frame);
  (void)(result);

  // Update the output frame
  {
    // Update the active frame from the output queue
    std::lock_guard<std::mutex> lock(output_frame_queue_lock_);
    if (output_frame_queue_.size() > 0)
    {
      CopyVideoFrameBytes(*output_frame_queue_.front(), *active_output_frame_);
      output_frame_queue_.pop_front();
    }

    // Copy the contents of the active frame into the command frame
    CopyVideoFrameBytes(*active_output_frame_, *command_output_frame_);
  }

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

      control_packet = BlackmagicSDICameraControlPacket::Make();

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
  DeckLinkAncillaryPacketHandle tally_packet =
      BlackmagicSDITallyControlPacket::Make(enable_tally_.load());

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

// Input+output device class
DeckLinkInputOutputDevice::DeckLinkInputOutputDevice(
    const LoggingFunction& logging_fn,
    const VideoFrameSizeChangedCallbackFunction&
        video_frame_size_changed_callback_fn,
    const ConvertedVideoFrameCallbackFunction&
        converted_video_frame_callback_fn,
    const BMDDisplayMode output_mode, DeckLinkHandle device)
{
  InitializeBaseDevice(logging_fn, std::move(device));
  InitializeInputDevice(
      video_frame_size_changed_callback_fn, converted_video_frame_callback_fn);
  InitializeOutputDevice(output_mode);
}

// Class-specific implementations of Start() and Stop() methods
void DeckLinkInputDevice::Start()
{
  EnableVideoInput(kDefaultInputDisplayMode, kDefaultInputPixelFormat);
  StartStreams();
}

void DeckLinkInputDevice::Stop()
{
  StopStreams();
  DisableVideoInput();
}

void DeckLinkOutputDevice::Start()
{
  EnableVideoOutput();
  PreScheduleReferenceFrame();
  StartScheduledPlayback();
  TallyOn();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

void DeckLinkOutputDevice::Stop()
{
  TallyOff();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  StopScheduledPlayback();
  DisableVideoOutput();
}

void DeckLinkInputOutputDevice::Start()
{
  EnableVideoInput(kDefaultInputDisplayMode, kDefaultInputPixelFormat);
  EnableVideoOutput();
  PreScheduleReferenceFrame();
  StartStreams();
  StartScheduledPlayback();
  TallyOn();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

void DeckLinkInputOutputDevice::Stop()
{
  TallyOff();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  StopStreams();
  StopScheduledPlayback();
  DisableVideoInput();
  DisableVideoOutput();
}

std::vector<DeckLinkHandle> GetDeckLinkHardwareDevices()
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
