#pragma once

#include <memory>

namespace blackmagic_camera_driver
{
template<typename BMDType>
class BMDDeleter
{
public:
  void operator()(BMDType* ptr) const
  {
    if (ptr != nullptr)
    {
      ptr->Release();
    }
  }
};

template<typename BMDType>
using BMDHandle = std::unique_ptr<BMDType, BMDDeleter<BMDType>>;
}  // namespace blackmagic_camera_driver

