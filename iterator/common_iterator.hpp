#pragma once

#include <variant>

#include "ranges_util.hpp"

namespace ranges {

template <std::input_or_output_iterator I, std::sentinel_for<I> S>
requires(!std::same_as<I, S> && std::copyable<I>) class common_iterator {
 private:
  std::variant<I, S> v_;
  class proxy {
    std::iter_value_t<I> keep_;
    proxy(std::iter_reference_t<I>&& x) : keep_(std::move(x)) {}
    friend common_iterator;

   public:
    const std::iter_value_t<I>* operator->() const { return std::addressof(keep_); }
  };

  class postfix_proxy {
    std::iter_value_t<I> keep_;
    postfix_proxy(std::iter_reference_t<I>&& x) : keep_(std::move(x)) {}
    friend common_iterator;

   public:
    const std::iter_value_t<I>& operator*() const { return keep_; }
  };

  template <std::input_or_output_iterator I2, std::sentinel_for<I2> S2>
  friend class common_iterator;

 public:
  constexpr common_iterator() = default;
  constexpr common_iterator(I i) : v_{std::in_place_type<I>, std::move(i)} {}
  constexpr common_iterator(S s) : v_{std::in_place_type<S>, std::move(s)} {}
  template <class I2, class S2>
  requires std::convertible_to<const I2&, I>&&
    std::convertible_to<const S2&, S> constexpr common_iterator(const common_iterator<I2, S2>& x)
    : v_{std::in_place_index<x.v_.index()>, std::get<x.v_.index()>(x.v_)} {
    assert(!x.v_.valueless_by_exception());
  }

  template <class I2, class S2>
  requires std::convertible_to<const I2&, I>&& std::convertible_to<const S2&, S>&&
    std::assignable_from<I&, const I2&>&& std::assignable_from<S&, const S2&>
      common_iterator& operator=(const common_iterator<I2, S2>& x) {
    assert(!x.v_.valueless_by_exception());
    constexpr auto i = x.v_.index();
    if (v_.index() == i)
      std::get<i>(v_) = std::get<i>(x.v_);
    else
      v_.emplace<i>(std::get<i>(x.v_));
    return *this;
  }

  decltype(auto) operator*() {
    assert(std::holds_alternative<I>(v_));
    return *std::get<I>(v_);
  }
  decltype(auto) operator*() const requires dereferenceable<const I> {
    assert(std::holds_alternative<I>(v_));
    return *std::get<I>(v_);
  }
  decltype(auto) operator->() const requires std::indirectly_readable<const I> &&
    (requires(const I& i) { i.operator->(); } || std::is_reference_v<std::iter_reference_t<I>> ||
     std::constructible_from<std::iter_value_t<I>, std::iter_reference_t<I>>) {
    assert(std::holds_alternative<I>(v_));
    if constexpr (std::is_pointer_v<I> || requires { std::get<I>(v_).operator->(); })
      return std::get<I>(v_);
    else if (std::is_reference_v<std::iter_reference_t<I>>) {
      auto&& tmp = *std::get<I>(v_);
      return std::addressof(tmp);
    } else
      return proxy(*std::get<I>(v_));
  }

  common_iterator& operator++() {
    assert(std::holds_alternative<I>(v_));
    ++std::get<I>(v_);
    return *this;
  }

  decltype(auto) operator++(int) {
    assert(std::holds_alternative<I>(v_));
    if constexpr (std::forward_iterator<I>) {
      common_iterator tmp = *this;
      ++*this;
      return tmp;
    } else if (
      requires(I & i) {
        { *i++ }
        ->can_reference;
      } ||
      !std::constructible_from<std::iter_value_t<I>, std::iter_reference_t<I>>)
      return get<I>(v_)++;
    else {
      postfix_proxy p(**this);
      ++*this;
      return p;
    }
  }

  template <class I2, std::sentinel_for<I> S2>
  requires std::sentinel_for<S, I2> friend bool operator==(
    const common_iterator& x, const common_iterator<I2, S2>& y) {
    assert(!x.v_.valueless_by_exception() && !y.v_.valueless_by_exception());
    constexpr auto i = x.v_.index();
    constexpr auto j = y.v_.index();
    if (i == j) return true;
    return std::get<i>(x.v_) == std::get<j>(y.v_);
  }

  template <class I2, std::sentinel_for<I> S2>
  requires std::sentinel_for<S, I2>&& std::equality_comparable_with<I, I2> friend bool operator==(
    const common_iterator& x, const common_iterator<I2, S2>& y) {
    assert(!x.v_.valueless_by_exception() && !y.v_.valueless_by_exception());
    constexpr auto i = x.v_.index();
    constexpr auto j = y.v_.index();
    if (i == 1 && j == 1) return true;
    return std::get<i>(x.v_) == std::get<j>(y.v_);
  }

  template <std::sized_sentinel_for<I> I2, std::sized_sentinel_for<I> S2>
  requires std::sized_sentinel_for<S, I2> friend std::iter_difference_t<I2> operator-(
    const common_iterator& x, const common_iterator<I2, S2>& y) {
    assert(!x.v_.valueless_by_exception() && !y.v_.valueless_by_exception());
    constexpr auto i = x.v_.index();
    constexpr auto j = y.v_.index();
    if (i == 1 && j == 1) return 0;
    return std::get<i>(x.v_) - std::get<j>(y.v_);
  }

  friend std::iter_rvalue_reference_t<I> iter_move(const common_iterator& i) noexcept(
    noexcept(std::ranges::iter_move(std::declval<const I&>()))) requires std::input_iterator<I> {
    assert(std::holds_alternative<I>(i.v_));
    return std::ranges::iter_move(std::get<I>(i.v_));
  }
  template <std::indirectly_swappable<I> I2, class S2>
  friend void iter_swap(const common_iterator& x, const common_iterator<I2, S2>& y) noexcept(
    noexcept(std::ranges::iter_swap(std::declval<const I&>(), std::declval<const I2&>()))) {
    assert(std::holds_alternative<I>(x.v_) && std::holds_alternative<I2>(y.v_));
    std::ranges::iter_swap(std::get<I>(x.v_), std::get<I2>(y.v_));
  }
};

}  // namespace ranges