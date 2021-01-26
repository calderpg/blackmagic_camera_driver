#pragma once

#include <memory>

namespace blackmagic_camera_driver
{
// Managed pointer type like std::unique_ptr, but aware of BMD's AddRef/Release
// reference counting system.
template<typename BMDType>
class BMDHandle
{
public:
  explicit BMDHandle(BMDType* bmd_item)
  {
    if (bmd_item != nullptr)
    {
      reset(bmd_item);
    }
    else
    {
      throw std::invalid_argument("BMDHandle constructed with nullptr");
    }
  }

  BMDHandle(const BMDHandle<BMDType>& other) = delete;

  BMDHandle(BMDHandle<BMDType>&& other)
  {
    reset(nullptr);
    bmd_item_ = other.release();
  }

  BMDHandle() : bmd_item_(nullptr) {}

  ~BMDHandle()
  {
    reset(nullptr);
  }

  void reset(BMDType* bmd_item = nullptr)
  {
    if (bmd_item_ != nullptr)
    {
      const auto new_refcount = bmd_item_->Release();
      std::cout << "[BMDHandle<" << typeid(BMDType).name() << ">] with item " << bmd_item_ << " called Release() with resulting refcount " << new_refcount << std::endl;
    }
    bmd_item_ = bmd_item;
    if (bmd_item_ != nullptr)
    {
      const auto new_refcount = bmd_item_->AddRef();
      if (new_refcount > 1)
      {
        const auto item_refcount = bmd_item_->Release();
        std::cout << "[BMDHandle<" << typeid(BMDType).name() << ">] with item " << bmd_item_ << " created/reset with refcount " << item_refcount << std::endl;
      }
      else
      {
        std::cout << "[BMDHandle<" << typeid(BMDType).name() << ">] with item " << bmd_item_ << " created/reset with zero refcount" << std::endl;
        throw std::runtime_error("Creating a handle with zero refcount is a BUG!");
      }
    }
  }

  BMDType* release()
  {
    BMDType* const item = bmd_item_;
    bmd_item_ = nullptr;
    return item;
  }

  BMDType* get() const { return bmd_item_; }

  BMDType* operator->() const
  {
    if (bmd_item_ != nullptr)
    {
      return get();
    }
    else
    {
      throw std::runtime_error("Contained BMDType is nullptr");
    }
  }

  typename std::add_lvalue_reference<BMDType>::type operator*() const
  {
    if (bmd_item_ != nullptr)
    {
      return *get();
    }
    else
    {
      throw std::runtime_error("Contained BMDType is nullptr");
    }
  }

  BMDHandle<BMDType>& operator=(const BMDHandle<BMDType>& other) = delete;

  BMDHandle<BMDType>& operator=(BMDHandle<BMDType>&& other)
  {
    reset(nullptr);
    bmd_item_ = other.release();
    return *this;
  }

  explicit operator bool() const { return bmd_item_ != nullptr; }

private:
  BMDType* bmd_item_ = nullptr;
};
}  // namespace blackmagic_camera_driver
