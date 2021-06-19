#pragma once

#include <algorithm>

#include "ref_view.hpp"

namespace ranges {

template <std::ranges::view V>
class take_view : public view_interface<take_view<V>> {
 private:
  V base_ = V();
  std::ranges::range_difference_t<V> count_ = 0;
  template <bool Const>
  struct sentinel {
   private:
    template <bool OtherConst>
    using CI = std::counted_iterator<std::ranges::iterator_t<maybe_const<OtherConst, V>>>;

    using Base = maybe_const<Const, V>;
    std::ranges::sentinel_t<Base> end_ = std::ranges::sentinel_t<Base>();

    friend sentinel<!Const>;

   public:
    sentinel() = default;
    constexpr explicit sentinel(std::ranges::sentinel_t<Base> end) : end_(end) {}
    constexpr sentinel(sentinel<!Const> s) requires Const&& std::convertible_to<
      std::ranges::sentinel_t<V>, std::ranges::sentinel_t<Base>> : end_(std::move(s.end_)) {}

    constexpr std::ranges::sentinel_t<Base> base() const { return end_; }

    friend constexpr bool operator==(const CI<Const>& y, const sentinel& x) {
      return y.count() == 0 || y.base() == x.end_;
    }

    template <bool OtherConst = !Const>
    requires std::sentinel_for<
      std::ranges::sentinel_t<Base>,
      std::ranges::iterator_t<maybe_const<OtherConst, V>>> friend constexpr bool
    operator==(const CI<OtherConst>& y, const sentinel& x) {
      return y.count() == 0 || y.base() == x.end_;
    }
  };

 public:
  take_view() = default;
  constexpr take_view(V base, std::ranges::range_difference_t<V> count)
    : base_(std::move(base)), count_(count) {}

  constexpr V base() const& requires std::copy_constructible<V> { return base_; }
  constexpr V base() && { return std::move(base_); }

  constexpr auto begin() requires(!simple_view<V>) {
    if constexpr (std::ranges::sized_range<V>) {
      if constexpr (std::ranges::random_access_range<V>)
        return std::ranges::begin(base_);
      else {
        auto sz = size();
        return std::counted_iterator(std::ranges::begin(base_), sz);
      }
    } else
      return std::counted_iterator(std::ranges::begin(base_), count_);
  }

  constexpr auto begin() const requires std::ranges::range<const V> {
    if constexpr (std::ranges::sized_range<const V>) {
      if constexpr (std::ranges::random_access_range<const V>)
        return std::ranges::begin(base_);
      else {
        auto sz = size();
        return std::counted_iterator(std::ranges::begin(base_), sz);
      }
    } else
      return std::counted_iterator(std::ranges::begin(base_), count_);
  }

  constexpr auto end() requires(!simple_view<V>) {
    if constexpr (std::ranges::sized_range<V>) {
      if constexpr (std::ranges::random_access_range<V>)
        return std::ranges::begin(base_) + size();
      else
        return std::default_sentinel;
    } else
      return sentinel<false>{std::ranges::end(base_)};
  }

  constexpr auto end() const requires std::ranges::range<const V> {
    if constexpr (std::ranges::sized_range<const V>) {
      if constexpr (std::ranges::random_access_range<const V>)
        return std::ranges::begin(base_) + size();
      else
        return std::default_sentinel;
    } else
      return sentinel<true>{std::ranges::end(base_)};
  }

  constexpr auto size() requires std::ranges::sized_range<V> {
    auto n = std::ranges::size(base_);
    return std::ranges::min(n, static_cast<decltype(n)>(count_));
  }

  constexpr auto size() const requires std::ranges::sized_range<const V> {
    auto n = std::ranges::size(base_);
    return std::ranges::min(n, static_cast<decltype(n)>(count_));
  }
};

template <class R>
take_view(R&&, std::ranges::range_difference_t<R>) -> take_view<views::all_t<R>>;

}  // namespace ranges

namespace views {

template <class R, class T>
concept can_take_view = requires {
  ranges::take_view(std::declval<R>(), std::declval<T>());
};

struct Take : ranges::RangeAdaptor<Take> {
  template <std::ranges::viewable_range R, typename T>
  requires can_take_view<R, T> constexpr auto operator()(R&& r, T&& n) const {
    return ranges::take_view(std::forward<R>(r), std::forward<T>(n));
  }

  using RangeAdaptor<Take>::operator();
  static constexpr int arity = 2;
};

inline constexpr Take take{};

}  // namespace views

template <class T>
inline constexpr bool std::ranges::enable_borrowed_range<ranges::take_view<T>> =
  std::ranges::enable_borrowed_range<T>;