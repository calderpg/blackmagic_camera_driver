#include <chrono>
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

  const auto logging_fn = [](
      const LogLevel level, const std::string& msg, const bool)
  {
    if (level != LogLevel::DEBUG)
    {
      std::cout << msg << std::endl;
    }
  };

  const auto frame_size_change_fn = [](
      const int64_t width, const int64_t height, const int64_t step)
  {
    std::cout << "Input resolution changed to " << width << " (width) "
              << height << " (height) " << step << " (bytes/row)" << std::endl;
  };

  const auto frame_received_fn = [](const InputConversionVideoFrame&)
  {
    std::cout << "Video frame received" << std::endl;
  };

  DeckLinkInputDevice input_device(
      logging_fn, frame_size_change_fn, frame_received_fn,
      std::move(decklink_devices.at(decklink_device_index)));

  // Start video input
  std::cout << "Starting video input..." << std::endl;
  input_device.Start();

  while (true)
  {
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  // Stop video input
  std::cout << "...stopping video input" << std::endl;
  input_device.Stop();

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
