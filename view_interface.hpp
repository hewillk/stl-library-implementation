#pragma once

#include "ranges_util.hpp"

namespace ranges {

template <class D>
requires std::is_class_v<D>&& std::same_as<D, std::remove_cv_t<D>> class view_interface
  : public std::ranges::view_base {
 private:
  constexpr D& derived() noexcept { return static_cast<D&>(*this); }

  constexpr const D& derived() const noexcept { return static_cast<const D&>(*this); }

 public:
  constexpr bool empty() requires std::ranges::forward_range<D> {
    return std::ranges::begin(derived()) == std::ranges::end(derived());
  }

  constexpr bool empty() const requires std::ranges::forward_range<const D> {
    return std::ranges::begin(derived()) == std::ranges::end(derived());
  }

  constexpr explicit operator bool() requires requires { std::ranges::empty(derived()); }
  { return !std::ranges::empty(derived()); }

  constexpr explicit operator bool() const requires requires { std::ranges::empty(derived()); }
  { return !std::ranges::empty(derived()); }

  constexpr auto data() requires std::contiguous_iterator<std::ranges::iterator_t<D>> {
    return std::to_address(std::ranges::begin(derived()));
  }

  constexpr auto data() const requires std::ranges::range<const D>&& std::contiguous_iterator<
    std::ranges::iterator_t<const D>> {
    return std::to_address(std::ranges::begin(derived()));
  }

  constexpr auto size() requires std::ranges::forward_range<D>&& std::sized_sentinel_for<
    std::ranges::iterator_t<D>, std::ranges::sentinel_t<D>> {
    return std::ranges::end(derived()) - std::ranges::begin(derived());
  }

  constexpr auto size() const
    requires std::ranges::forward_range<const D>&& std::sized_sentinel_for<
      std::ranges::iterator_t<const D>, std::ranges::sentinel_t<const D>> {
    return std::ranges::end(derived()) - std::ranges::begin(derived());
  }

  constexpr decltype(auto) front() requires std::ranges::forward_range<D> {
    assert(!empty());
    return *std::ranges::begin(derived());
  }
  constexpr decltype(auto) front() const requires std::ranges::forward_range<const D> {
    assert(!empty());
    return *std::ranges::begin(derived());
  }

  constexpr decltype(auto)
  back() requires std::ranges::bidirectional_range<D>&& std::ranges::common_range<D> {
    assert(!empty());
    return *std::ranges::prev(std::ranges::end(derived()));
  }

  constexpr decltype(auto) back() const
    requires std::ranges::bidirectional_range<const D>&& std::ranges::common_range<const D> {
    assert(!empty());
    return *std::ranges::prev(std::ranges::end(derived()));
  }

  template <std::ranges::random_access_range R = D>
  constexpr decltype(auto) operator[](std::ranges::range_difference_t<R> n) {
    return std::ranges::begin(derived())[n];
  }

  template <std::ranges::random_access_range R = const D>
  constexpr decltype(auto) operator[](std::ranges::range_difference_t<R> n) const {
    return std::ranges::begin(derived())[n];
  }
};

}  // namespace ranges