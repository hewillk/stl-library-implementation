#pragma once

#include <optional>

namespace ranges {

template <typename T>
requires std::copy_constructible<T>&& std::is_object_v<T> struct semiregular_box
    : std::optional<T> {
  using std::optional<T>::optional;

  constexpr semiregular_box() noexcept(
    std::is_nothrow_default_constructible_v<T>) requires std::default_initializable<T>
      : semiregular_box{std::in_place} {}

  semiregular_box(const semiregular_box&) = default;
  semiregular_box(semiregular_box&&) = default;

  using std::optional<T>::operator=;

  semiregular_box& operator=(const semiregular_box& that) noexcept(
    std::is_nothrow_copy_constructible_v<T>) requires(!std::copyable<T>) {
    if (that)
      this->emplace(*that);
    else
      this->reset();
    return *this;
  }

  semiregular_box& operator=(semiregular_box&& that) noexcept(
    std::is_nothrow_move_constructible_v<T>) requires(!std::movable<T>) {
    if (that)
      this->emplace(std::move(*that));
    else
      this->reset();
    return *this;
  }
};

}  // namespace ranges