#pragma once

#include "view_interface.hpp"

namespace ranges {

template <class T>
requires std::is_object_v<T> class empty_view : public view_interface<empty_view<T>> {
 public:
  static constexpr T* begin() noexcept { return nullptr; }
  static constexpr T* end() noexcept { return nullptr; }
  static constexpr T* data() noexcept { return nullptr; }
  static constexpr std::size_t size() noexcept { return 0; }
  static constexpr bool empty() noexcept { return true; }
};

}  // namespace ranges

namespace views {

template <class T>
inline constexpr ranges::empty_view<T> empty{};

}  // namespace views

template <class T>
inline constexpr bool std::ranges::enable_borrowed_range<ranges::empty_view<T>> = true;
