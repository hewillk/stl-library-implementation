#pragma once

#include <cassert>
#include <concepts>
#include <iterator>
#include <ranges>

namespace ranges {

template <class R>
concept simple_view = std::ranges::view<R>&& std::ranges::range<const R>&&
  std::same_as<std::ranges::iterator_t<R>, std::ranges::iterator_t<const R>>&&
    std::same_as<std::ranges::sentinel_t<R>, std::ranges::sentinel_t<const R>>;

template <class I>
concept has_arrow = std::input_iterator<I> &&
                    (std::is_pointer_v<I> || requires(I i) { i.operator->(); });

template <typename T, typename U>
concept not_same_as = !std::same_as<std::remove_cvref_t<T>, std::remove_cvref_t<U>>;

template <class T>
concept boolean_testable_impl = std::convertible_to<T, bool>;

template <class T>
concept boolean_testable = boolean_testable_impl<T>&& requires(T&& t) {
  { !std::forward<T>(t) }
  ->boolean_testable_impl;
};

template <class T, class U>
concept weakly_equality_comparable_with =
  requires(const std::remove_reference_t<T>& t, const std::remove_reference_t<U>& u) {
  { t == u }
  ->boolean_testable;
  { t != u }
  ->boolean_testable;
  { u == t }
  ->boolean_testable;
  { u != t }
  ->boolean_testable;
};

template <class T>
concept is_integer_like = std::integral<T> && !std::same_as<std::remove_cv_t<T>, bool>;

template <class T>
concept is_signed_integer_like = is_integer_like<T>&& static_cast<T>(-1) < static_cast<T>(0);

template <std::integral T>
constexpr auto to_unsigned_like(T t) noexcept {
  return static_cast<std::make_unsigned_t<T>>(t);
}

template <typename T>
using make_unsigned_like_t = std::make_unsigned_t<T>;

template <std::ranges::range R>
struct CachedIter {
  constexpr bool has_value() const { return false; }

  constexpr std::ranges::iterator_t<R> get(const R&) const {
    assert(false);
    return {};
  }

  constexpr void set(const R&, const std::ranges::iterator_t<R>&) const {}
};

template <std::ranges::forward_range R>
struct CachedIter<R> {
 private:
  std::ranges::iterator_t<R> iter_{};

 public:
  constexpr bool has_value() const { return iter_ != std::ranges::iterator_t<R>{}; }

  constexpr std::ranges::iterator_t<R> get(const R&) const {
    assert(has_value());
    return iter_;
  }

  constexpr void set(const R&, const std::ranges::iterator_t<R>& iter) {
    assert(!has_value());
    iter_ = iter;
  }
};

template <std::ranges::random_access_range R>
requires(
  sizeof(std::ranges::range_difference_t<R>) <=
  sizeof(std::ranges::iterator_t<R>)) struct CachedIter<R> {
 private:
  std::ranges::range_difference_t<R> offset_ = -1;

 public:
  constexpr bool has_value() const { return offset_ >= 0; }

  constexpr std::ranges::iterator_t<R> get(const R& r) const {
    assert(has_value());
    return std::ranges::begin(r) + offset_;
  }

  constexpr void set(R& r, const std::ranges::iterator_t<R>& iter) {
    assert(!has_value());
    offset_ = iter - std::ranges::begin(r);
  }
};

struct Empty {};

template <bool Present, class T>
using maybe_present_t = std::conditional_t<Present, T, Empty>;

template <class T>
using with_reference = T&;

template <class T>
concept can_reference = requires {
  typename with_reference<T>;
};

template <class T>
concept dereferenceable = requires(T& t) {
  { *t }
  ->can_reference;
};

template <bool Const, class T>
using maybe_const = std::conditional_t<Const, const T, T>;

}  // namespace ranges