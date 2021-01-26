#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <iostream>
#include <list>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <blackmagic_camera_driver/bmd_handle.hpp>

#include "DeckLinkAPI_v10_11.h"

namespace blackmagic_camera_driver
{
using DeckLinkIteratorHandle = BMDHandle<IDeckLinkIterator>;
using DeckLinkHandle = BMDHandle<IDeckLink>;
using DeckLinkInputHandle = BMDHandle<IDeckLinkInput>;
using DeckLinkOutputHandle = BMDHandle<IDeckLinkOutput>;
using DeckLinkProfileAttributesHandle = BMDHandle<IDeckLinkProfileAttributes>;
using DeckLinkDisplayModeHandle = BMDHandle<IDeckLinkDisplayMode>;
using DeckLinkInputCallbackHandle = BMDHandle<IDeckLinkInputCallback>;
using DeckLinkOutputCallbackHandle = BMDHandle<IDeckLinkVideoOutputCallback>;
using DeckLinkMutableVideoFrameHandle = BMDHandle<IDeckLinkMutableVideoFrame>;
using DeckLinkVideoConversionHandle = BMDHandle<IDeckLinkVideoConversion>;
using DeckLinkAncillaryPacketIteratorHandle
    = BMDHandle<IDeckLinkAncillaryPacketIterator>;
using DeckLinkAncillaryPacketHandle = BMDHandle<IDeckLinkAncillaryPacket>;
using DeckLinkVideoFrameAncillaryPacketsHandle
    = BMDHandle<IDeckLinkVideoFrameAncillaryPackets>;

inline int16_t ConvertToFixed16(const float val)
{
  const float multiplier = 1 << 11;
  float multiplied = val * multiplier;

  if (multiplied > 32767)
  {
    multiplied = 32767;
  }
  else if (multiplied < -32768)
  {
    multiplied = -32768;
  }

  return static_cast<int16_t>(multiplied);
}

inline int32_t Calc10BitYUVRowBytes(const int32_t frame_width)
{
  return ((frame_width + 47) / 48) * 128;
}

// Enforces that video frames are the same size (# of bytes).
void CopyVideoFrameBytes(
    const IDeckLinkVideoFrame& source_frame,
    IDeckLinkMutableVideoFrame& destination_frame);

template<typename T>
std::string HexPrint(const T& val)
{
  std::ostringstream strm;
  strm << std::hex << val;
  return strm.str();
}

enum class LogLevel : uint8_t
{
  DEBUG = 0x01,
  INFO = 0x02,
  WARN = 0x03,
  ERROR = 0x04
};

using LoggingFunction =
    std::function<void(const LogLevel, const std::string&, const bool)>;

// Arguments are image width, height, and step in bytes.
using VideoFrameSizeChangedCallbackFunction =
    std::function<void(const int32_t, const int32_t, const int32_t)>;

// Note that the IDeckLinkVideoFrame is immutable, but it does not provide const
// getters, so passing via mutable reference is the only way to avoid const_cast
// on every access.
using ConvertedVideoFrameCallbackFunction =
    std::function<void(IDeckLinkVideoFrame& video_frame)>;

// Forward declaration
class DeckLinkDevice;

class FrameReceivedCallback : public IDeckLinkInputCallback
{
public:
  FrameReceivedCallback(DeckLinkDevice* parent_device)
      : parent_device_(parent_device)
  {
    if (parent_device_ == nullptr)
    {
      throw std::runtime_error("parent_device_ cannot be null");
    }
  }

  HRESULT VideoInputFormatChanged(
      BMDVideoInputFormatChangedEvents notification_events,
      IDeckLinkDisplayMode* new_display_mode,
      BMDDetectedVideoInputFormatFlags detected_signal_flags) override;

  HRESULT VideoInputFrameArrived(
      IDeckLinkVideoInputFrame* video_frame,
      IDeckLinkAudioInputPacket* audio_packet) override;

  // Dummy implementation
  HRESULT QueryInterface(REFIID iid, LPVOID* ppv) override
  {
    (void)(iid);
    (void)(ppv);
    return E_NOINTERFACE;
  }

  // Reference count handling
  ULONG AddRef() override
  {
    return ++refcount_;
  }

  ULONG Release() override
  {
    const uint64_t refcount = --refcount_;
    if (refcount == 0)
    {
      delete this;
    }
    return refcount;
  }

private:
  DeckLinkDevice* parent_device_ = nullptr;
  std::atomic<uint64_t> refcount_{1};
};

class FrameOutputCallback : public IDeckLinkVideoOutputCallback
{
public:
  FrameOutputCallback(DeckLinkDevice* parent_device)
      : parent_device_(parent_device)
  {
    if (parent_device_ == nullptr)
    {
      throw std::runtime_error("parent_device_ cannot be null");
    }
  }

  HRESULT ScheduledFrameCompleted(
      IDeckLinkVideoFrame* completed_frame,
      BMDOutputFrameCompletionResult result) override;

  HRESULT ScheduledPlaybackHasStopped() override;

  // Dummy implementation
  HRESULT QueryInterface(REFIID iid, LPVOID* ppv) override
  {
    (void)(iid);
    (void)(ppv);
    return E_NOINTERFACE;
  }

  // Reference count handling
  ULONG AddRef() override
  {
    return ++refcount_;
  }

  ULONG Release() override
  {
    const uint64_t refcount = --refcount_;
    if (refcount == 0)
    {
      delete this;
    }
    return refcount;
  }

private:
  DeckLinkDevice* parent_device_ = nullptr;
  std::atomic<uint64_t> refcount_{1};
};

class BlackmagicSDICameraControlMessage
{
public:
  static BlackmagicSDICameraControlMessage MakeCommandVoid(
      const uint8_t destination_device, const uint8_t category,
      const uint8_t parameter)
  {
    std::vector<uint8_t> bytes(12, 0x00);

    // Header
    bytes.at(0) = destination_device;
    bytes.at(1) = 4;
    bytes.at(2) = 0x00;
    bytes.at(3) = 0x00;

    // Payload
    bytes.at(4) = category;
    bytes.at(5) = parameter;
    bytes.at(6) = 0x00;
    bytes.at(7) = 0x00;

    bytes.at(8) = 0x01;
    bytes.at(9) = 0x00;
    bytes.at(10) = 0x00;
    bytes.at(11) = 0x00;

    return BlackmagicSDICameraControlMessage(bytes);
  }

  static BlackmagicSDICameraControlMessage MakeCommandBool(
      const uint8_t destination_device, const uint8_t category,
      const uint8_t parameter, const uint8_t operation, const bool value)
  {
    std::vector<uint8_t> bytes(12, 0x00);

    // Header
    bytes.at(0) = destination_device;
    bytes.at(1) = 5;
    bytes.at(2) = 0x00;
    bytes.at(3) = 0x00;

    // Payload
    bytes.at(4) = category;
    bytes.at(5) = parameter;
    bytes.at(6) = 0x00;
    bytes.at(7) = operation;

    bytes.at(8) = (value) ? 0x01 : 0x00;
    bytes.at(9) = 0x00;
    bytes.at(10) = 0x00;
    bytes.at(11) = 0x00;

    return BlackmagicSDICameraControlMessage(bytes);
  }

  static BlackmagicSDICameraControlMessage MakeCommandInt8(
      const uint8_t destination_device, const uint8_t category,
      const uint8_t parameter, const uint8_t operation, const int8_t value)
  {
    std::vector<uint8_t> bytes(12, 0x00);

    // Header
    bytes.at(0) = destination_device;
    bytes.at(1) = 5;
    bytes.at(2) = 0x00;
    bytes.at(3) = 0x00;

    // Payload
    bytes.at(4) = category;
    bytes.at(5) = parameter;
    bytes.at(6) = 0x01;
    bytes.at(7) = operation;

    bytes.at(8) = value;
    bytes.at(9) = 0x00;
    bytes.at(10) = 0x00;
    bytes.at(11) = 0x00;

    return BlackmagicSDICameraControlMessage(bytes);
  }

  static BlackmagicSDICameraControlMessage MakeCommandInt16(
      const uint8_t destination_device, const uint8_t category,
      const uint8_t parameter, const uint8_t operation, const int16_t value)
  {
    std::vector<uint8_t> bytes(12, 0x00);

    // Header
    bytes.at(0) = destination_device;
    bytes.at(1) = 6;
    bytes.at(2) = 0x00;
    bytes.at(3) = 0x00;

    // Payload
    bytes.at(4) = category;
    bytes.at(5) = parameter;
    bytes.at(6) = 0x02;
    bytes.at(7) = operation;

    bytes.at(8) = static_cast<uint8_t>((value >> 0) & 0xff);
    bytes.at(9) = static_cast<uint8_t>((value >> 8) & 0xff);
    bytes.at(10) = 0x00;
    bytes.at(11) = 0x00;

    return BlackmagicSDICameraControlMessage(bytes);
  }

  static BlackmagicSDICameraControlMessage MakeCommand1nt32(
      const uint8_t destination_device, const uint8_t category,
      const uint8_t parameter, const uint8_t operation, const int32_t value)
  {
    std::vector<uint8_t> bytes(12, 0x00);

    // Header
    bytes.at(0) = destination_device;
    bytes.at(1) = 8;
    bytes.at(2) = 0x00;
    bytes.at(3) = 0x00;

    // Payload
    bytes.at(4) = category;
    bytes.at(5) = parameter;
    bytes.at(6) = 0x03;
    bytes.at(7) = operation;

    bytes.at(8) = static_cast<uint8_t>((value >> 0) & 0xff);
    bytes.at(9) = static_cast<uint8_t>((value >> 8) & 0xff);
    bytes.at(10) = static_cast<uint8_t>((value >> 16) & 0xff);
    bytes.at(11) = static_cast<uint8_t>((value >> 24) & 0xff);

    return BlackmagicSDICameraControlMessage(bytes);
  }

  static BlackmagicSDICameraControlMessage MakeCommand1nt64(
      const uint8_t destination_device, const uint8_t category,
      const uint8_t parameter, const uint8_t operation, const int64_t value)
  {
    std::vector<uint8_t> bytes(16, 0x00);

    // Header
    bytes.at(0) = destination_device;
    bytes.at(1) = 12;
    bytes.at(2) = 0x00;
    bytes.at(3) = 0x00;

    // Payload
    bytes.at(4) = category;
    bytes.at(5) = parameter;
    bytes.at(6) = 0x04;
    bytes.at(7) = operation;

    bytes.at(8) = static_cast<uint8_t>((value >> 0) & 0xff);
    bytes.at(9) = static_cast<uint8_t>((value >> 8) & 0xff);
    bytes.at(10) = static_cast<uint8_t>((value >> 16) & 0xff);
    bytes.at(11) = static_cast<uint8_t>((value >> 24) & 0xff);

    bytes.at(12) = static_cast<uint8_t>((value >> 32) & 0xff);
    bytes.at(13) = static_cast<uint8_t>((value >> 40) & 0xff);
    bytes.at(14) = static_cast<uint8_t>((value >> 48) & 0xff);
    bytes.at(15) = static_cast<uint8_t>((value >> 56) & 0xff);

    return BlackmagicSDICameraControlMessage(bytes);
  }

  static BlackmagicSDICameraControlMessage MakeCommandFixed16(
      const uint8_t destination_device, const uint8_t category,
      const uint8_t parameter, const uint8_t operation, const float value)
  {
    std::vector<uint8_t> bytes(12, 0x00);

    // Header
    bytes.at(0) = destination_device;
    bytes.at(1) = 6;
    bytes.at(2) = 0x00;
    bytes.at(3) = 0x00;

    // Payload
    bytes.at(4) = category;
    bytes.at(5) = parameter;
    bytes.at(6) = 128;
    bytes.at(7) = operation;

    const int16_t fixed16_value = ConvertToFixed16(value);

    bytes.at(8) = static_cast<uint8_t>((fixed16_value >> 0) & 0xff);
    bytes.at(9) = static_cast<uint8_t>((fixed16_value >> 8) & 0xff);
    bytes.at(10) = 0x00;
    bytes.at(11) = 0x00;

    return BlackmagicSDICameraControlMessage(bytes);
  }

  const std::vector<uint8_t>& GetBytes() const { return bytes_; }

private:
  explicit BlackmagicSDICameraControlMessage(const std::vector<uint8_t>& bytes)
      : bytes_(bytes)
  {
    const size_t message_alignment = 4;
    if ((bytes_.size() % message_alignment) != 0)
    {
      throw std::invalid_argument(
          "Camera control message of " + std::to_string(bytes_.size())
          + " is not 4-byte aligned");
    }
  }

  std::vector<uint8_t> bytes_;
};

class BlackmagicSDICameraControlPacket : public IDeckLinkAncillaryPacket
{
public:
  bool AddCameraControlMessage(const BlackmagicSDICameraControlMessage& msg)
  {
    const auto& message_bytes = msg.GetBytes();
    // A packet can only contain up to 255 bytes
    if ((bytes_.size() + message_bytes.size()) <= 255)
    {
      bytes_.insert(bytes_.end(), message_bytes.begin(), message_bytes.end());
      return true;
    }
    else
    {
      return false;
    }
  }

  HRESULT GetBytes(
      BMDAncillaryPacketFormat format, const void** data,
      uint32_t* size) override
  {
    if (format == bmdAncillaryPacketFormatUInt8)
    {
      if (data != nullptr)
      {
        *data = bytes_.data();
      }
      if (size != nullptr)
      {
        *size = static_cast<uint32_t>(bytes_.size());
      }
      return S_OK;
    }
    else
    {
      return E_NOTIMPL;
    }
  }

  uint8_t GetDID() override { return did_; }

  uint8_t GetSDID() override { return sdid_; }

  uint32_t GetLineNumber() override { return line_number_; }

  uint8_t GetDataStreamIndex() override { return data_stream_index_; }

  // Dummy implementation
  HRESULT QueryInterface(REFIID iid, LPVOID* ppv) override
  {
    (void)(iid);
    (void)(ppv);
    return E_NOINTERFACE;
  }

  // Reference count handling
  ULONG AddRef() override
  {
    return ++refcount_;
  }

  ULONG Release() override
  {
    const uint64_t refcount = --refcount_;
    if (refcount == 0)
    {
      delete this;
    }
    return refcount;
  }

private:
  std::vector<uint8_t> bytes_;
  // These are specified in the Blackmagic SDI Camera Control Protocol.
  const uint8_t did_ = 0x51;
  const uint8_t sdid_ = 0x53;
  const uint32_t line_number_ = 16;
  const uint8_t data_stream_index_ = 0x00;
  std::atomic<uint64_t> refcount_{1};
};

class BlackmagicSDITallyControlPacket : public IDeckLinkAncillaryPacket
{
public:
  BlackmagicSDITallyControlPacket(const bool tally_on)
  {
    const uint8_t tally_header = 0b00000011;
    const uint8_t tally_command = (tally_on) ? 0b00110011 : 0b00000000;
    const std::vector<uint8_t> device_tally(50, tally_command);
    bytes_ = {tally_header};
    bytes_.insert(bytes_.end(), device_tally.begin(), device_tally.end());
  }

  HRESULT GetBytes(
      BMDAncillaryPacketFormat format, const void** data,
      uint32_t* size) override
  {
    if (format == bmdAncillaryPacketFormatUInt8)
    {
      if (data != nullptr)
      {
        *data = bytes_.data();
      }
      if (size != nullptr)
      {
        *size = static_cast<uint32_t>(bytes_.size());
      }
      return S_OK;
    }
    else
    {
      return E_NOTIMPL;
    }
  }

  uint8_t GetDID() override { return did_; }

  uint8_t GetSDID() override { return sdid_; }

  uint32_t GetLineNumber() override { return line_number_; }

  uint8_t GetDataStreamIndex() override { return data_stream_index_; }

  // Dummy implementation
  HRESULT QueryInterface(REFIID iid, LPVOID* ppv) override
  {
    (void)(iid);
    (void)(ppv);
    return E_NOINTERFACE;
  }

  // Reference count handling
  ULONG AddRef() override
  {
    return ++refcount_;
  }

  ULONG Release() override
  {
    const uint64_t refcount = --refcount_;
    if (refcount == 0)
    {
      delete this;
    }
    return refcount;
  }

private:
  std::vector<uint8_t> bytes_;
  // These are specified in the Blackmagic SDI Tally Control Protocol.
  const uint8_t did_ = 0x51;
  const uint8_t sdid_ = 0x52;
  const uint32_t line_number_ = 15;
  const uint8_t data_stream_index_ = 0x00;
  std::atomic<uint64_t> refcount_{1};
};

class DeckLinkDevice
{
public:
  DeckLinkDevice(
      const LoggingFunction& logging_fn,
      const VideoFrameSizeChangedCallbackFunction&
          video_frame_size_changed_callback_fn,
      const ConvertedVideoFrameCallbackFunction&
          converted_video_frame_callback_fn,
      const BMDDisplayMode output_mode, DeckLinkHandle device);

  virtual ~DeckLinkDevice() {}

  void StartVideoCapture();

  void StopVideoCapture();

  void EnqueueCameraCommand(const BlackmagicSDICameraControlMessage& command);

  void ClearOutputQueueAndResetOutputToReferenceFrame();

  void EnqueueOutputFrame(DeckLinkMutableVideoFrameHandle output_frame);

  DeckLinkMutableVideoFrameHandle CreateBGRA8OutputVideoFrame();

  DeckLinkMutableVideoFrameHandle CreateYUV10OutputVideoFrame();

  void Log(
      const LogLevel level, const std::string& message,
      const bool throttle = false)
  {
    logging_fn_(level, message, throttle);
  }

  void LogDebug(const std::string& message, const bool throttle = false)
  {
    Log(LogLevel::DEBUG, message, throttle);
  }

  void LogInfo(const std::string& message, const bool throttle = false)
  {
    Log(LogLevel::INFO, message, throttle);
  }

  void LogWarn(const std::string& message, const bool throttle = false)
  {
    Log(LogLevel::WARN, message, throttle);
  }

  void LogError(const std::string& message, const bool throttle = false)
  {
    Log(LogLevel::ERROR, message, throttle);
  }

private:
  void RestartCapture(
      const BMDDisplayMode display_mode, const BMDPixelFormat pixel_format);

  void EnableVideoInput(
      const BMDDisplayMode display_mode, const BMDPixelFormat pixel_format);

  void EnableVideoOutput(const BMDDisplayMode display_mode);

  void DisableVideoInput();

  void DisableVideoOutput();

  void StartStreams();

  void FlushStreams();

  void PauseStreams();

  void StopStreams();

  void StartScheduledPlayback();

  void StopScheduledPlayback();

  void TallyOn() { enable_tally_.store(true); }

  void TallyOff() { enable_tally_.store(false); }

  HRESULT ScheduleNextFrame(IDeckLinkVideoFrame& video_frame);

  void SetupConversionAndPublishingFrames(
      const int32_t image_width, const int32_t image_height);

  friend class FrameReceivedCallback;

  HRESULT InputFormatChangedCallback(
      const BMDVideoInputFormatChangedEvents notification_events,
      const IDeckLinkDisplayMode& new_display_mode,
      const BMDDetectedVideoInputFormatFlags detected_signal_flags);

  HRESULT FrameCallback(
      const IDeckLinkVideoInputFrame& video_frame,
      const IDeckLinkAudioInputPacket& audio_packet);

  friend class FrameOutputCallback;

  HRESULT ScheduledFrameCallback(
      IDeckLinkVideoFrame& completed_frame,
      const BMDOutputFrameCompletionResult result);

  void LogVideoFrameAncillaryPackets(
      const IDeckLinkVideoFrame& video_frame, const std::string& msg,
      const bool log_debug);

  LoggingFunction logging_fn_;
  VideoFrameSizeChangedCallbackFunction video_frame_size_changed_callback_fn_;
  ConvertedVideoFrameCallbackFunction converted_video_frame_callback_fn_;

  std::mutex camera_command_queue_lock_;
  std::list<BlackmagicSDICameraControlMessage> camera_command_queue_;

  std::atomic<bool> enable_tally_{};

  int64_t output_frame_counter_ = 0;
  int64_t output_frame_duration_ = 0;
  int64_t output_frame_timescale_ = 0;

  BMDDisplayMode output_display_mode_ = bmdModeHD1080p30;

  std::mutex output_frame_queue_lock_;
  std::list<DeckLinkMutableVideoFrameHandle> output_frame_queue_;

  DeckLinkHandle device_;
  DeckLinkProfileAttributesHandle attributes_interface_;
  DeckLinkInputHandle input_device_;
  DeckLinkOutputHandle output_device_;
  DeckLinkVideoConversionHandle input_video_converter_;
  DeckLinkVideoConversionHandle output_video_converter_;
  DeckLinkInputCallbackHandle input_callback_;
  DeckLinkOutputCallbackHandle output_callback_;
  DeckLinkMutableVideoFrameHandle input_conversion_frame_;
  DeckLinkMutableVideoFrameHandle reference_output_frame_;
  DeckLinkMutableVideoFrameHandle active_output_frame_;
  DeckLinkMutableVideoFrameHandle command_output_frame_;
};

std::vector<DeckLinkHandle> GetDeckLinkDevices();
}  // namespace blackmagic_camera_driver
