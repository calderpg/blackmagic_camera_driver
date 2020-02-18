#pragma once

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
using DeckLinkHandle = BMDHandle<IDeckLink>;
using DeckLinkInputHandle = BMDHandle<IDeckLinkInput>;
using DeckLinkOutputHandle = BMDHandle<IDeckLinkOutput>;
using DeckLinkProfileAttributesHandle = BMDHandle<IDeckLinkProfileAttributes>;
using DeckLinkInputCallbackHandle = BMDHandle<IDeckLinkInputCallback>;
using DeckLinkMutableVideoFrameHandle = BMDHandle<IDeckLinkMutableVideoFrame>;
using DeckLinkVideoConversionHandle = BMDHandle<IDeckLinkVideoConversion>;

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

  HRESULT STDMETHODCALLTYPE VideoInputFormatChanged(
      BMDVideoInputFormatChangedEvents notification_events,
      IDeckLinkDisplayMode* new_display_mode,
      BMDDetectedVideoInputFormatFlags detected_signal_flags) override;

  HRESULT VideoInputFrameArrived(
      IDeckLinkVideoInputFrame* video_frame,
      IDeckLinkAudioInputPacket* audio_packet) override;

  // Dummy implementation
  HRESULT STDMETHODCALLTYPE QueryInterface(REFIID iid, LPVOID* ppv) override
  {
    (void)(iid);
    (void)(ppv);
    return E_NOINTERFACE;
  }

  // Reference count handling
  ULONG STDMETHODCALLTYPE AddRef() override
  {
    return ++refcount_;
  }

  ULONG STDMETHODCALLTYPE Release() override
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
  std::atomic<uint64_t> refcount_;
};

class DeckLinkDevice
{
public:
  explicit DeckLinkDevice(
      const ros::NodeHandle& nh, const std::string& camera_topic,
      const std::string& camera_frame, DeckLinkHandle device);

  void StartVideoCapture();

  void StopVideoCapture();

private:
  void RestartCapture(
      const BMDDisplayMode display_mode, const BMDPixelFormat pixel_format);

  void EnableVideoInput(
      const BMDDisplayMode display_mode, const BMDPixelFormat pixel_format);

  void DisableVideoInput();

  void StartStreams();

  void FlushStreams();

  void PauseStreams();

  void StopStreams();

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

  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Publisher camera_pub_;
  std::string camera_frame_;
  sensor_msgs::Image ros_image_;

  DeckLinkHandle device_;
  DeckLinkProfileAttributesHandle attributes_interface_;
  DeckLinkInputHandle input_device_;
  DeckLinkOutputHandle output_device_;
  DeckLinkVideoConversionHandle video_converter_;
  DeckLinkInputCallbackHandle input_callback_;
  DeckLinkMutableVideoFrameHandle conversion_frame_;
};
}  // namespace blackmagic_camera_driver

