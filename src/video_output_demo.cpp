#include <chrono>
#include <cstring>
#include <stdexcept>
#include <thread>
#include <vector>

#include <blackmagic_camera_driver/decklink_interface.hpp>

namespace blackmagic_camera_driver
{
namespace
{
int DoMain(const int32_t decklink_device_index)
{
  // Discover DeckLink devices
  std::vector<DeckLinkHandle> decklink_devices = GetDeckLinkHardwareDevices();
  if (decklink_devices.size() > 0)
  {
    std::cout << "Found [" << decklink_devices.size() << "] DeckLink device(s)"
              << std::endl;
  }
  else
  {
    throw std::runtime_error("No DeckLink device(s) found");
  }

  // Get the selected device
  std::cout << "Selecting DeckLink device [" << decklink_device_index << "]"
            << std::endl;

  const auto logging_fn = [] (
      const LogLevel level, const std::string& msg, const bool)
  {
    if (level != LogLevel::DEBUG)
    {
      std::cout << msg << std::endl;
    }
  };

  DeckLinkOutputDevice output_device(
      logging_fn, bmdModeHD1080p30,
      std::move(decklink_devices.at(decklink_device_index)));

  // Make primary color image data blocks
  const size_t num_image_pixels = 1920 * 1080;
  const size_t num_image_bytes = num_image_pixels * 4;
  const std::vector<uint32_t> red_color(num_image_pixels, 0xffff0000);
  const std::vector<uint32_t> green_color(num_image_pixels, 0xff00ff00);
  const std::vector<uint32_t> blue_color(num_image_pixels, 0xff0000ff);

  // Start video output
  std::cout << "Starting video output..." << std::endl;
  output_device.Start();

  std::this_thread::sleep_for(std::chrono::seconds(1));

  int32_t tick = 0;
  while (true)
  {
    tick++;

    const void* image_data_ptr = nullptr;
    if (tick == 1)
    {
      image_data_ptr = red_color.data();
    }
    else if (tick == 2)
    {
      image_data_ptr = green_color.data();
    }
    else if (tick == 3)
    {
      image_data_ptr = blue_color.data();
      tick = 0;
    }

    if (image_data_ptr != nullptr)
    {
      auto output_frame = output_device.CreateBGRA8OutputVideoFrame();
      uint8_t* output_frame_buffer = nullptr;
      const auto get_output_bytes_result = output_frame->GetBytes(
          reinterpret_cast<void**>(&output_frame_buffer));
      if (get_output_bytes_result != S_OK || output_frame_buffer == nullptr)
      {
        throw std::runtime_error("Failed to get output frame bytes");
      }
      const size_t output_frame_bytes
          = output_frame->GetRowBytes() * output_frame->GetHeight();
      if (output_frame_bytes != num_image_bytes)
      {
        throw std::runtime_error(
            "Image data and frame data are different sizes");
      }

      std::memcpy(output_frame_buffer, image_data_ptr, num_image_bytes);

      output_device.EnqueueOutputFrame(std::move(output_frame));
    }

    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  // Stop video output
  std::cout << "...stopping video output" << std::endl;
  output_device.Stop();

  return 0;
}
}  // namespace
}  // namespace blackmagic_camera_driver

int main(int argc, char** argv)
{
  const int32_t decklink_device_index = (argc >= 2)
      ? std::atoi(argv[1]) : 0;
  return blackmagic_camera_driver::DoMain(decklink_device_index);
}
