#pragma once

#include <atomic>
#include <cstdint>
#include <iostream>
#include <list>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include <blackmagic_camera_driver/bmd_handle.hpp>
#include <image_transport/image_transport.h>
#include <ros/ros.h>

#include "DeckLinkAPI_v10_11.h"

namespace blackmagic_camera_driver
{
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
  const float multiplied = val * multiplier;
  return static_cast<int16_t>(multiplied);
}

// Forward declaration
class DeckLinkDevice;

class FrameReceivedCallback : public IDeckLinkInputCallback
{
public:
  FrameReceivedCallback(DeckLinkDevice* parent_device)
      : parent_device_(parent_device), refcount_(1)
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
  std::atomic<uint64_t> refcount_{};
};

class FrameOutputCallback : public IDeckLinkVideoOutputCallback
{
public:
  FrameOutputCallback(DeckLinkDevice* parent_device)
      : parent_device_(parent_device), refcount_(1)
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
  std::atomic<uint64_t> refcount_{};
};

class BlackmagicSDICameraControlMessage
{
public:
  BlackmagicSDICameraControlMessage(
      const uint8_t destination_device, const uint8_t command_id,
      const std::vector<uint8_t>& command_data)
  {
    if (command_data.size() > 60)
    {
      throw std::runtime_error("command_data cannot be larger than 60 bytes");
    }
    // Assemble byte representation of message according to the Blackmagic SDI
    // Camera Control Protocol
    bytes_.push_back(destination_device);
    bytes_.push_back(static_cast<uint8_t>(command_data.size()));
    bytes_.push_back(command_id);
    bytes_.push_back(0x00); // Reserved/padding byte.
    bytes_.insert(bytes_.end(), command_data.begin(), command_data.end());
    // Add padding bytes to ensure message is 32-bit aligned
    const size_t padding_alignment = 4;
    const size_t padding_offset = bytes_.size() % padding_alignment;
    if (padding_offset > 0)
    {
      const size_t padding_size = padding_alignment - padding_offset;
      const size_t padded_size = bytes_.size() + padding_size;
      // Resizing larger appends new elements to the vector
      bytes_.resize(padded_size, 0x00);
    }
  }

  const std::vector<uint8_t>& GetBytes() const { return bytes_; }

private:
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
  std::atomic<uint64_t> refcount_{};
};

using BlackmagicSDICameraControlPacketHandle
    = BMDHandle<BlackmagicSDICameraControlPacket>;

class DeckLinkDevice
{
public:
  explicit DeckLinkDevice(
      const ros::NodeHandle& nh, const std::string& camera_topic,
      const std::string& camera_frame, DeckLinkHandle device);

  void StartVideoCapture();

  void StopVideoCapture();

  void EnqueueCameraCommand(const BlackmagicSDICameraControlMessage& command);

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

  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Publisher camera_pub_;
  std::string camera_frame_;
  sensor_msgs::Image ros_image_;

  std::mutex camera_command_queue_lock_;
  std::list<BlackmagicSDICameraControlMessage> camera_command_queue_;

  int64_t output_frame_counter_ = 0;
  int64_t output_frame_duration_ = 0;
  int64_t output_frame_timescale_ = 0;

  DeckLinkHandle device_;
  DeckLinkProfileAttributesHandle attributes_interface_;
  DeckLinkInputHandle input_device_;
  DeckLinkOutputHandle output_device_;
  DeckLinkVideoConversionHandle video_converter_;
  DeckLinkInputCallbackHandle input_callback_;
  DeckLinkOutputCallbackHandle output_callback_;
  DeckLinkMutableVideoFrameHandle conversion_frame_;
  DeckLinkMutableVideoFrameHandle blue_reference_output_frame_;
  DeckLinkMutableVideoFrameHandle red_command_output_frame_;
};
}  // namespace blackmagic_camera_driver
