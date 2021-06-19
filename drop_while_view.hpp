#pragma once

#include <algorithm>

#include "ref_view.hpp"
#include "semiregular_box.hpp"

namespace ranges {

template <std::ranges::view V, class Pred>
requires std::ranges::input_range<V>&& std::is_object_v<Pred>&&
  std::indirect_unary_predicate<const Pred, std::ranges::iterator_t<V>> class drop_while_view
  : public view_interface<drop_while_view<V, Pred>> {
 private:
  V base_ = V();
  semiregular_box<Pred> pred_;
  [[no_unique_address]] CachedIter<V> cached_begin_;

 public:
  drop_while_view() = default;
  constexpr drop_while_view(V base, Pred pred) : base_(std::move(base)), pred_(std::move(pred)) {}

  constexpr V base() const& requires std::copy_constructible<V> { return base_; }
  constexpr V base() && { return std::move(base_); }

  constexpr const Pred& pred() const { return *pred_; }

  constexpr auto begin() {
    assert(pred_.has_value());
    if (cached_begin_.has_value()) return cached_begin_.get(base_);
    auto it = std::ranges::find_if_not(base_, std::cref(*pred_));
    cached_begin_.set(base_, it);
    return it;
  }

  constexpr auto end() { return std::ranges::end(base_); }
};

template <class R, class Pred>
drop_while_view(R&&, Pred) -> drop_while_view<views::all_t<R>, Pred>;

}  // namespace ranges

namespace views {

template <class R, class Pred>
concept can_drop_while_view = requires {
  ranges::drop_while_view(std::declval<R>(), std::declval<Pred>());
};

struct DropWhile : ranges::RangeAdaptor<DropWhile> {
  template <std::ranges::viewable_range R, class Pred>
  requires can_drop_while_view<R, Pred> constexpr auto operator()(R&& r, Pred&& pred) const {
    return ranges::drop_while_view(std::forward<R>(r), std::forward<Pred>(pred));
  }

  using RangeAdaptor<DropWhile>::operator();
  static constexpr int arity = 2;
};

inline constexpr DropWhile drop_while;

}  // namespace views

template <class T, class Pred>
inline constexpr bool std::ranges::enable_borrowed_range<ranges::drop_while_view<T, Pred>> =
  std::ranges::enable_borrowed_range<T>;