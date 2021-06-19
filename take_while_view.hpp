#pragma once

#include <functional>

#include "ref_view.hpp"
#include "semiregular_box.hpp"

namespace ranges {

template <std::ranges::view V, class Pred>
requires std::ranges::input_range<V>&& std::is_object_v<Pred>&&
  std::indirect_unary_predicate<const Pred, std::ranges::iterator_t<V>> class take_while_view
  : public view_interface<take_while_view<V, Pred>> {
 private:
  V base_ = V();
  semiregular_box<Pred> pred_;

  template <bool Const>
  class sentinel {
   private:
    using Base = maybe_const<Const, V>;
    std::ranges::sentinel_t<Base> end_ = std::ranges::sentinel_t<Base>();
    const Pred* pred_ = nullptr;

   public:
    sentinel() = default;
    constexpr explicit sentinel(std::ranges::sentinel_t<Base> end, const Pred* pred)
      : end_(end), pred_(pred) {}
    constexpr sentinel(sentinel<!Const> s) requires Const&& std::convertible_to<
      std::ranges::sentinel_t<V>, std::ranges::sentinel_t<Base>> : end_(s.end_),
                                                                   pred_(s.pred_) {}
    constexpr std::ranges::sentinel_t<Base> base() const { return end_; }

    friend constexpr bool operator==(const std::ranges::iterator_t<Base>& x, const sentinel& y) {
      return y.end_ == x || !std::invoke(*y.pred_, *x);
    }

    template <bool OtherConst = !Const>
    requires std::sentinel_for<
      std::ranges::sentinel_t<Base>,
      std::ranges::iterator_t<maybe_const<OtherConst, V>>> friend constexpr bool
    operator==(const std::ranges::iterator_t<maybe_const<OtherConst, V>>& x, const sentinel& y) {
      return y.end_ == x || !std::invoke(*y.pred_, *x);
    }
  };

 public:
  take_while_view() = default;
  constexpr take_while_view(V base, Pred pred) : base_(std::move(base)), pred_(std::move(pred)) {}

  constexpr V base() const& requires std::copy_constructible<V> { return base_; }
  constexpr V base() && { return std::move(base_); }

  constexpr const Pred& pred() const { return *pred_; }

  constexpr auto begin() requires(!simple_view<V>) { return std::ranges::begin(base_); }
  constexpr auto begin() const requires std::ranges::range<const V>&& std::indirect_unary_predicate<
    const Pred, std::ranges::iterator_t<const V>> {
    return std::ranges::begin(base_);
  }
  constexpr auto end() requires(!simple_view<V>) {
    return sentinel<false>{std::ranges::end(base_), std::addressof(*pred_)};
  }
  constexpr auto end() const requires std::ranges::range<const V>&& std::indirect_unary_predicate<
    const Pred, std::ranges::iterator_t<const V>> {
    return sentinel<true>{std::ranges::end(base_), std::addressof(*pred_)};
  }
};

template <class R, class Pred>
take_while_view(R&&, Pred) -> take_while_view<views::all_t<R>, Pred>;

}  // namespace ranges

namespace views {

template <class R, class Pred>
concept can_take_while_view = requires {
  ranges::take_while_view(std::declval<R>(), std::declval<Pred>());
};

struct TakeWhile : ranges::RangeAdaptor<TakeWhile> {
  template <std::ranges::viewable_range R, class Pred>
  requires can_take_while_view<R, Pred> constexpr auto operator()(R&& r, Pred&& pred) const {
    return ranges::take_while_view(std::forward<R>(r), std::forward<Pred>(pred));
  }

  using RangeAdaptor<TakeWhile>::operator();
  static constexpr int arity = 2;
};

inline constexpr TakeWhile take_while;

}  // namespace views