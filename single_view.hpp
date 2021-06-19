#pragma once

#include "semiregular_box.hpp"
#include "view_interface.hpp"

namespace ranges {

template <std::copy_constructible T>
requires std::is_object_v<T> class single_view : public view_interface<single_view<T>> {
 private:
  [[no_unique_address]] semiregular_box<T> value_;

 public:
  single_view() = default;
  constexpr explicit single_view(const T& t) : value_{t} {}
  constexpr explicit single_view(T&& t) : value_{std::move(t)} {}
  template <class... Args>
  requires std::constructible_from<T, Args...> constexpr explicit single_view(
    std::in_place_t, Args&&... args)
      : value_{std::in_place, std::forward<Args>(args)...} {}

  constexpr T* begin() noexcept { return data(); }
  constexpr const T* begin() const noexcept { return data(); }
  constexpr T* end() noexcept { return data() + 1; }
  constexpr const T* end() const noexcept { return data() + 1; }
  static constexpr std::size_t size() noexcept { return 1; }
  constexpr T* data() noexcept { return value_.operator->(); }
  constexpr const T* data() const noexcept { return value_.operator->(); }
};

}  // namespace ranges

namespace views {

struct Single {
  template <class T>
  constexpr auto operator()(T&& t) const {
    return ranges::single_view{std::forward<T>(t)};
  }
};

inline constexpr Single single{};

}  // namespace views