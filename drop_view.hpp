#pragma once

#include "ref_view.hpp"

namespace ranges {

template <std::ranges::view V>
class drop_view : public view_interface<drop_view<V>> {
 private:
  V base_ = V();
  std::ranges::range_difference_t<V> count_ = 0;
  static constexpr bool needs_cached_begin =
    !(std::ranges::random_access_range<const V> && std::ranges::sized_range<const V>);
  [[no_unique_address]] maybe_present_t<needs_cached_begin, CachedIter<V>> cached_begin_{};

 public:
  drop_view() = default;
  constexpr drop_view(V base, std::ranges::range_difference_t<V> count)
    : base_(std::move(base)), count_(count) {
    assert(count >= 0);
  }

  constexpr V base() const& requires std::copy_constructible<V> { return base_; }
  constexpr V base() && { return std::move(base_); }

  constexpr auto begin() requires(
    !(simple_view<V> && std::ranges::random_access_range<const V> &&
      std::ranges::sized_range<const V>)) {
    if constexpr (needs_cached_begin) {
      if (cached_begin_.has_value()) return cached_begin_.get(base_);
    }

    auto it = std::ranges::next(std::ranges::begin(base_), count_, std::ranges::end(base_));
    if constexpr (needs_cached_begin) cached_begin_.set(base_, it);
    return it;
  }

  constexpr auto begin() const
    requires std::ranges::random_access_range<const V>&& std::ranges::sized_range<const V> {
    return std::ranges::next(std::ranges::begin(base_), count_, std::ranges::end(base_));
  }

  constexpr auto end() requires(!simple_view<V>) { return std::ranges::end(base_); }
  constexpr auto end() const requires std::ranges::range<const V> {
    return std::ranges::end(base_);
  }

  constexpr auto size() requires std::ranges::sized_range<V> {
    const auto s = std::ranges::size(base_);
    const auto c = static_cast<decltype(s)>(count_);
    return s < c ? 0 : s - c;
  }

  constexpr auto size() const requires std::ranges::sized_range<const V> {
    const auto s = std::ranges::size(base_);
    const auto c = static_cast<decltype(s)>(count_);
    return s < c ? 0 : s - c;
  }
};

template <class R>
drop_view(R&&, std::ranges::range_difference_t<R>) -> drop_view<views::all_t<R>>;

}  // namespace ranges

namespace views {

template <class R, class T>
concept can_drop_view = requires {
  ranges::drop_view(std::declval<R>(), std::declval<T>());
};

struct Drop : ranges::RangeAdaptor<Drop> {
  template <std::ranges::viewable_range R, typename T>
  requires can_drop_view<R, T> constexpr auto operator()(R&& r, T&& n) const {
    return ranges::drop_view(std::forward<R>(r), std::forward<T>(n));
  }

  using RangeAdaptor<Drop>::operator();
  static constexpr int arity = 2;
};

inline constexpr Drop drop{};

}  // namespace views

template <class T>
inline constexpr bool std::ranges::enable_borrowed_range<ranges::drop_view<T>> =
  std::ranges::enable_borrowed_range<T>;