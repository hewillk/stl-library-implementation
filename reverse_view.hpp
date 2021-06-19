#pragma once

#include "ref_view.hpp"
#include "subrange.hpp"

namespace ranges {

template <std::ranges::view V>
requires std::ranges::bidirectional_range<V> class reverse_view
  : public view_interface<reverse_view<V>> {
 private:
  V base_ = V();
  [[no_unique_address]] CachedIter<V> cached_begin_{};

 public:
  reverse_view() = default;
  constexpr explicit reverse_view(V base) : base_(std::move(base)) {}

  constexpr V base() const& requires std::copy_constructible<V> { return base_; }
  constexpr V base() && { return std::move(base_); }

  constexpr std::reverse_iterator<std::ranges::iterator_t<V>> begin() {
    if (cached_begin_.has_value()) return std::make_reverse_iterator(cached_begin_.get(base_));
    auto it = std::ranges::next(std::ranges::begin(base_), std::ranges::end(base_));
    cached_begin_.set(base_, it);
    return std::make_reverse_iterator(std::move(it));
  }

  constexpr std::reverse_iterator<std::ranges::iterator_t<V>>
  begin() requires std::ranges::common_range<V> {
    return std::make_reverse_iterator(std::ranges::end(base_));
  }

  constexpr auto begin() const requires std::ranges::common_range<const V> {
    return std::make_reverse_iterator(std::ranges::end(base_));
  }

  constexpr std::reverse_iterator<std::ranges::iterator_t<V>> end() {
    return std::make_reverse_iterator(std::ranges::begin(base_));
  }

  constexpr auto end() const requires std::ranges::common_range<const V> {
    return std::make_reverse_iterator(std::ranges::begin(base_));
  }

  constexpr auto size() requires std::ranges::sized_range<V> { return std::ranges::size(base_); }
  constexpr auto size() const requires std::ranges::sized_range<const V> {
    return std::ranges::size(base_);
  }
};

template <class R>
reverse_view(R&&) -> reverse_view<views::all_t<R>>;

}  // namespace ranges

namespace views {

template <class>
inline constexpr bool is_reversible_subrange = false;

template <class I, ranges::subrange_kind K>
inline constexpr bool
  is_reversible_subrange<ranges::subrange<std::reverse_iterator<I>, std::reverse_iterator<I>, K>> =
    true;

template <class>
inline constexpr bool is_reverse_view = false;

template <class V>
inline constexpr bool is_reverse_view<ranges::reverse_view<V>> = true;

template <class R>
concept can_reverse_view = requires {
  ranges::reverse_view{std::declval<R>()};
};

struct Reverse : ranges::RangeAdaptorClosure {
  template <std::ranges::viewable_range R>
    requires is_reversible_subrange<std::remove_cvref_t<R>> ||
    is_reverse_view<std::remove_cvref_t<R>> ||
    can_reverse_view<R> constexpr auto operator()(R&& r) const {
    using T = std::remove_cvref_t<R>;
    if constexpr (is_reversible_subrange<T>) {
      using I = decltype(r.begin().base());
      if constexpr (requires { r.size(); })
        return ranges::subrange<I, I, ranges::subrange_kind::sized>(
          r.end().base(), r.begin().base(), r.size());
      else
        return ranges::subrange<I, I, ranges::subrange_kind::unsized>(
          r.end().base(), r.begin().base());

    } else if constexpr (is_reverse_view<T>) {
      return std::forward<R>(r).base();
    } else
      return ranges::reverse_view{std::forward<R>(r)};
  }
};

inline constexpr Reverse reverse{};

}  // namespace views

template <class T>
inline constexpr bool std::ranges::enable_borrowed_range<ranges::reverse_view<T>> =
  std::ranges::enable_borrowed_range<T>;