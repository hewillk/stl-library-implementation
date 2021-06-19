#pragma once

#include "ref_view.hpp"

namespace ranges {

template <std::ranges::view V>
  requires(!std::ranges::common_range<V>) &&
  std::copyable<std::ranges::iterator_t<V>> class common_view
  : public view_interface<common_view<V>> {
 private:
  V base_ = V();

 public:
  common_view() = default;
  constexpr explicit common_view(V base) : base_(std::move(base)) {}

  constexpr V base() const& requires std::copy_constructible<V> { return base_; }
  constexpr V base() && { return std::move(base_); }

  constexpr auto begin() {
    if constexpr (std::ranges::random_access_range<V> && std::ranges::sized_range<V>)
      return std::ranges::begin(base_);
    else
      return std::common_iterator<std::ranges::iterator_t<V>, std::ranges::sentinel_t<V>>(
        std::ranges::begin(base_));
  }

  constexpr auto begin() const requires std::ranges::range<const V> {
    if constexpr (std::ranges::random_access_range<const V> && std::ranges::sized_range<const V>)
      return std::ranges::begin(base_);
    else
      return std::common_iterator<
        std::ranges::iterator_t<const V>, std::ranges::sentinel_t<const V>>(
        std::ranges::begin(base_));
  }

  constexpr auto end() {
    if constexpr (std::ranges::random_access_range<V> && std::ranges::sized_range<V>)
      return std::ranges::begin(base_) + std::ranges::size(base_);
    else
      return std::common_iterator<std::ranges::iterator_t<V>, std::ranges::sentinel_t<V>>(
        std::ranges::end(base_));
  }

  constexpr auto end() const requires std::ranges::range<const V> {
    if constexpr (std::ranges::random_access_range<const V> && std::ranges::sized_range<const V>)
      return std::ranges::begin(base_) + std::ranges::size(base_);
    else
      return std::common_iterator<
        std::ranges::iterator_t<const V>, std::ranges::sentinel_t<const V>>(
        std::ranges::end(base_));
  }

  constexpr auto size() requires std::ranges::sized_range<V> { return std::ranges::size(base_); }
  constexpr auto size() const requires std::ranges::sized_range<const V> {
    return std::ranges::size(base_);
  }
};

template <class R>
common_view(R&&) -> common_view<views::all_t<R>>;

}  // namespace ranges

namespace views {

template <class R>
concept already_common = std::ranges::common_range<R>&& requires {
  views::all(std::declval<R>());
};

template <class R>
concept can_common_view = requires {
  ranges::common_view{std::declval<R>()};
};

struct Common : ranges::RangeAdaptorClosure {
  template <std::ranges::viewable_range R>
    requires already_common<R> || can_common_view<R> constexpr auto operator()(R&& r) const {
    if constexpr (already_common<R>)
      return views::all(std::forward<R>(r));
    else
      return ranges::common_view{std::forward<R>(r)};
  }
};

inline constexpr Common common{};

}  // namespace views

template <class T>
inline constexpr bool std::ranges::enable_borrowed_range<ranges::common_view<T>> =
  std::ranges::enable_borrowed_range<T>;