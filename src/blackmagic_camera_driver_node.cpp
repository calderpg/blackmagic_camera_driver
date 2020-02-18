#include <atomic>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <vector>

#include <blackmagic_camera_driver/decklink_interface.hpp>
#include <ros/ros.h>

#include "DeckLinkAPI_v10_11.h"

namespace blackmagic_camera_driver
{
namespace
{
using DeckLinkIteratorHandle = BMDHandle<IDeckLinkIterator>;

int DoMain()
{
  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");
  const int32_t decklink_device_index
      = nhp.param(std::string("decklink_device_index"), 0);
  const std::string camera_topic
      = nhp.param(std::string("camera_topic"), std::string("bmd_camera"));
  const std::string camera_frame
      = nhp.param(std::string("camera_frame"), std::string("bmd_camera"));
  // Discover DeckLink devices
  DeckLinkIteratorHandle device_iterator(CreateDeckLinkIteratorInstance());
  if (device_iterator)
  {
    // Find available DeckLink devices
    std::vector<DeckLinkHandle> decklink_devices;
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
    std::cout << "Found [" << decklink_devices.size() << "] DeckLink device(s)"
              << std::endl;

    // Get the selected device
    std::cout << "Selecting DeckLink device [" << decklink_device_index << "]"
              << std::endl;
    DeckLinkDevice capture_device(
        nh, camera_topic, camera_frame,
        std::move(decklink_devices.at(decklink_device_index)));

    // Start capture
    std::cout << "Starting capture..." << std::endl;
    capture_device.StartVideoCapture();

    // Spin while video callbacks run
    ros::Rate spin_rate(10.0);
    while (ros::ok())
    {
      ros::spinOnce();
      spin_rate.sleep();
    }

    // Stop capture
    std::cout << "Stopping capture..." << std::endl;
    capture_device.StopVideoCapture();

    // Let RAII clean everything else
    std::cout << "...capture complete" << std::endl;
  }
  else
  {
    std::cerr << "Failed to create DeckLinkIterator instance" << std::endl;
  }
  return 0;
}
}  // namespace
}  // namespace blackmagic_camera_driver

int main(int argc, char** argv)
{
  ros::init(argc, argv, "blackmagic_camera_driver_node");
  return blackmagic_camera_driver::DoMain();
}

