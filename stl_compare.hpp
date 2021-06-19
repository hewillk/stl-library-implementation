#pragma once

#include <utility>

namespace ranges {

enum class ord { equal, equivalent = equal, less = -1, greater = 1 };
enum class ncmp { unordered = -127 };

struct unspecified {
  constexpr unspecified(unspecified*) noexcept {}
};

class partial_ordering {
  int value;
  bool is_ordered;

  constexpr explicit partial_ordering(ord v) noexcept : value(int(v)), is_ordered(true) {}
  constexpr explicit partial_ordering(ncmp v) noexcept : value(int(v)), is_ordered(false) {}

 public:
  static const partial_ordering less;
  static const partial_ordering equivalent;
  static const partial_ordering greater;
  static const partial_ordering unordered;

  friend constexpr bool
  operator==(partial_ordering v, partial_ordering w) noexcept = default;

  friend constexpr bool
  operator==(partial_ordering v, unspecified) noexcept {
    return v.is_ordered && v.value == 0;
  }

  friend constexpr bool
  operator<(partial_ordering v, unspecified) noexcept {
    return v.is_ordered && v.value < 0;
  }

  friend constexpr bool
  operator>(partial_ordering v, unspecified) noexcept {
    return v.is_ordered && v.value > 0;
  }

  friend constexpr bool
  operator<=(partial_ordering v, unspecified) noexcept {
    return v.is_ordered && v.value <= 0;
  }

  friend constexpr bool
  operator>=(partial_ordering v, unspecified) noexcept {
    return v.is_ordered && v.value >= 0;
  }

  friend constexpr bool
  operator<(unspecified, partial_ordering v) noexcept {
    return v.is_ordered && 0 < v.value;
  }

  friend constexpr bool
  operator>(unspecified, partial_ordering v) noexcept {
    return v.is_ordered && 0 > v.value;
  }

  friend constexpr bool
  operator<=(unspecified, partial_ordering v) noexcept {
    return v.is_ordered && 0 <= v.value;
  }

  friend constexpr bool
  operator>=(unspecified, partial_ordering v) noexcept {
    return v.is_ordered && 0 >= v.value;
  }

  friend constexpr partial_ordering
  operator<=>(partial_ordering v, unspecified) noexcept {
    return v;
  }

  friend constexpr partial_ordering
  operator<=>(unspecified, partial_ordering v) noexcept {
    return v < 0 ? partial_ordering::greater : v > 0 ? partial_ordering::less : v;
  }
};

inline constexpr partial_ordering partial_ordering::less(ord::less);
inline constexpr partial_ordering partial_ordering::equivalent(ord::equivalent);
inline constexpr partial_ordering partial_ordering::greater(ord::greater);
inline constexpr partial_ordering partial_ordering::unordered(ncmp::unordered);

class weak_ordering {
  int value;

  constexpr explicit weak_ordering(ord v) noexcept : value(int(v)) {}

 public:
  static const weak_ordering less;
  static const weak_ordering equivalent;
  static const weak_ordering greater;

  constexpr operator partial_ordering() const noexcept {
    return value == 0 ? partial_ordering::equivalent :
           value < 0  ? partial_ordering::less :
                        partial_ordering::greater;
  }

  friend constexpr bool
  operator==(weak_ordering v, weak_ordering w) noexcept = default;

  friend constexpr bool
  operator==(weak_ordering v, unspecified) noexcept {
    return v.value == 0;
  }

  friend constexpr bool
  operator<(weak_ordering v, unspecified) noexcept {
    return v.value < 0;
  }

  friend constexpr bool
  operator>(weak_ordering v, unspecified) noexcept {
    return v.value > 0;
  }

  friend constexpr bool
  operator<=(weak_ordering v, unspecified) noexcept {
    return v.value <= 0;
  }

  friend constexpr bool
  operator>=(weak_ordering v, unspecified) noexcept {
    return v.value >= 0;
  }

  friend constexpr bool
  operator<(unspecified, weak_ordering v) noexcept {
    return 0 < v.value;
  }

  friend constexpr bool
  operator>(unspecified, weak_ordering v) noexcept {
    return 0 > v.value;
  }

  friend constexpr bool
  operator<=(unspecified, weak_ordering v) noexcept {
    return 0 <= v.value;
  }

  friend constexpr bool
  operator>=(unspecified, weak_ordering v) noexcept {
    return 0 >= v.value;
  }

  friend constexpr weak_ordering
  operator<=>(weak_ordering v, unspecified) noexcept {
    return v;
  }

  friend constexpr weak_ordering
  operator<=>(unspecified, weak_ordering v) noexcept {
    return v < 0 ? weak_ordering::greater : v > 0 ? weak_ordering::less : v;
  }
};

inline constexpr weak_ordering weak_ordering::less(ord::less);
inline constexpr weak_ordering weak_ordering::equivalent(ord::equivalent);
inline constexpr weak_ordering weak_ordering::greater(ord::greater);

class strong_ordering {
  int value;

  constexpr explicit strong_ordering(ord v) noexcept : value(int(v)) {}

 public:
  static const strong_ordering less;
  static const strong_ordering equal;
  static const strong_ordering equivalent;
  static const strong_ordering greater;

  constexpr operator partial_ordering() const noexcept {
    return value == 0 ? partial_ordering::equivalent :
           value < 0  ? partial_ordering::less :
                        partial_ordering::greater;
  }

  constexpr operator weak_ordering() const noexcept {
    return value == 0 ? weak_ordering::equivalent :
           value < 0  ? weak_ordering::less :
                        weak_ordering::greater;
  }

  friend constexpr bool
  operator==(strong_ordering v, strong_ordering w) noexcept = default;

  friend constexpr bool
  operator==(strong_ordering v, unspecified) noexcept {
    return v.value == 0;
  }

  friend constexpr bool
  operator<(strong_ordering v, unspecified) noexcept {
    return v.value < 0;
  }

  friend constexpr bool
  operator>(strong_ordering v, unspecified) noexcept {
    return v.value > 0;
  }

  friend constexpr bool
  operator<=(strong_ordering v, unspecified) noexcept {
    return v.value <= 0;
  }

  friend constexpr bool
  operator>=(strong_ordering v, unspecified) noexcept {
    return v.value >= 0;
  }

  friend constexpr bool
  operator<(unspecified, strong_ordering v) noexcept {
    return 0 < v.value;
  }

  friend constexpr bool
  operator>(unspecified, strong_ordering v) noexcept {
    return 0 > v.value;
  }

  friend constexpr bool
  operator<=(unspecified, strong_ordering v) noexcept {
    return 0 <= v.value;
  }

  friend constexpr bool
  operator>=(unspecified, strong_ordering v) noexcept {
    return 0 >= v.value;
  }

  friend constexpr strong_ordering
  operator<=>(strong_ordering v, unspecified) noexcept {
    return v;
  }

  friend constexpr strong_ordering
  operator<=>(unspecified, strong_ordering v) noexcept {
    return v < 0 ? strong_ordering::greater : v > 0 ? strong_ordering::less : v;
  }
};

inline constexpr strong_ordering strong_ordering::less(ord::less);
inline constexpr strong_ordering strong_ordering::equal(ord::equal);
inline constexpr strong_ordering strong_ordering::equivalent(ord::equivalent);
inline constexpr strong_ordering strong_ordering::greater(ord::greater);

constexpr bool
is_eq(partial_ordering cmp) noexcept {
  return cmp == 0;
}
constexpr bool
is_neq(partial_ordering cmp) noexcept {
  return cmp != 0;
}
constexpr bool
is_lt(partial_ordering cmp) noexcept {
  return cmp < 0;
}
constexpr bool
is_lteq(partial_ordering cmp) noexcept {
  return cmp <= 0;
}
constexpr bool
is_gt(partial_ordering cmp) noexcept {
  return cmp > 0;
}
constexpr bool
is_gteq(partial_ordering cmp) noexcept {
  return cmp >= 0;
}

namespace detail {

template<class T>
inline constexpr unsigned cmp_cat_id = 1;
template<>
inline constexpr unsigned cmp_cat_id<partial_ordering> = 2;
template<>
inline constexpr unsigned cmp_cat_id<weak_ordering> = 4;
template<>
inline constexpr unsigned cmp_cat_id<strong_ordering> = 8;

template<class... Ts>
constexpr auto
common_cmp_cat() {
  constexpr unsigned cats = (cmp_cat_id<Ts> || ...);
  if constexpr (cats & 1)
    return;
  else if constexpr (cats & 2)
    return partial_ordering::equivalent;
  else if constexpr (cats & 4)
    return weak_ordering::equivalent;
  else
    return strong_ordering::equivalent;
}

}  // namespace detail

template<class... Ts>
struct common_comparison_category {
  using type = decltype(detail::common_cmp_cat<Ts...>());
};

template<class... Ts>
using common_comparison_category_t = common_comparison_category<Ts...>::type;

namespace detail {

template<class T, class U>
using cmp3way_res_t = decltype(std::declval<const std::remove_reference_t<T>&> <=>
                               std::declval<const std::remove_reference_t<U>&>);

template<class T, class U>
struct cmp3way_res_impl {};

template<class T, class U>
  requires requires { typename cmp3way_res_t<T, U>; }
struct cmp3way_res_impl<T, U> {
  using type = cmp3way_res_t<T, U>;
};

}  // namespace detail

template<class T, class U = T>
struct compare_three_way_result : detail::cmp3way_res_impl<T, U> {};

template<class T, class U = T>
using compare_three_way_result_t = compare_three_way_result<T, U>::type;

template<class T>
concept boolean_testable_impl = std::convertible_to<T, bool>;

template<class T>
concept boolean_testable = boolean_testable_impl<T> && requires(T&& t) {
  { !std::forward<T>(t) } -> boolean_testable_impl;
};

template<class T, class U>
concept weakly_equality_comparable_with = requires(const std::remove_reference_t<T>& t,
                                                   const std::remove_reference_t<U>& u) {
  { t == u } -> boolean_testable;
  { t != u } -> boolean_testable;
  { u == t } -> boolean_testable;
  { u != t } -> boolean_testable;
};

template<class T, class U>
concept partially_ordered_with = requires(const std::remove_reference_t<T>& t,
                                          const std::remove_reference_t<U>& u) {
  { t < u } -> boolean_testable;
  { t > u } -> boolean_testable;
  { t <= u } -> boolean_testable;
  { t >= u } -> boolean_testable;
  { u < t } -> boolean_testable;
  { u > t } -> boolean_testable;
  { u <= t } -> boolean_testable;
  { u >= t } -> boolean_testable;
};

template<class T, class Cat>
concept compares_as = std::same_as<common_comparison_category_t<T, Cat>, Cat>;

template<class T, class Cat = partial_ordering>
concept three_way_comparable = weakly_equality_comparable_with<T, T> &&
  partially_ordered_with<T, T> &&
  requires(const std::remove_reference_t<T>& a, const std::remove_reference_t<T>& b) {
  { a <=> b } -> compares_as<Cat>;
};

template<class T, class U, class Cat = partial_ordering>
concept three_way_comparable_with = three_way_comparable<T, Cat> && three_way_comparable<U, Cat> &&
                                    std::common_reference_with < const std::remove_reference_t<T>
&,
  const std::remove_reference_t<U>& >
    &&three_way_comparable<
      std::common_reference_t<const std::remove_reference_t<T>&, const std::remove_reference_t<U>&>,
      Cat>&& weakly_equality_comparable_with<T, U>&&
      partially_ordered_with<T, U>&& requires(const std::remove_reference_t<T>& t,
                                              const std::remove_reference_t<U>& u) {
  { t <=> u } -> compares_as<Cat>;
  { u <=> t } -> compares_as<Cat>;
};

struct compare_three_way {
  template<class T, class U>
    requires three_way_comparable_with<T, U>
  constexpr auto
  operator()(T&& t, U&& u) const noexcept(noexcept(std::declval<T>() <=> std::declval<U>())) {
    return std::forward<T>(t) <=> std::forward<U>(u);
  }

  using is_transparent = void;
};

template<class T, class U>
concept adl_strong = requires(T&& t, U&& u) {
  strong_ordering(strong_order(std::forward<T>(t), std::forward<U>(u)));
};

template<class Ord, class T, class U>
concept cmp3way = requires(T&& t, U&& u) {
  Ord(compare_three_way()(std::forward<T>(t), std::forward<U>(u)));
};

template<class T, class U>
concept strongly_ordered = adl_strong<T, U> || cmp3way<strong_ordering, T, U>;

template<class T, class U>
concept decayed_same_as = std::same_as<std::decay_t<T>, std::decay_t<U>>;

struct StrongOrder {
 private:
  template<class T, class U>
  static constexpr bool
  _noexcept() {
    if constexpr (std::floating_point<std::decay_t<T>>)
      return true;
    else if constexpr (adl_strong<T, U>)
      return noexcept(strong_order(std::declval<T>(), std::declval<U>()));
    else if constexpr (cmp3way<strong_ordering, T, U>)
      return noexcept(compare_three_way()(std::declval<T>(), std::declval<U>()));
  }

 public:
  template<class T, decayed_same_as<T> U>
    requires strongly_ordered<T, U>
  constexpr strong_ordering
  operator()(T&& t, U&& u) const noexcept(_noexcept<T, U>()) {
    if constexpr (adl_strong<T, U>)
      return strong_order(std::forward<T>(t), std::forward<U>(u));
    else if constexpr (cmp3way<strong_ordering, T, U>)
      return compare_three_way()(std::forward<T>(t), std::forward<U>(u));
  }
};

template<class T, class U>
concept adl_weak = requires(T&& t, U&& u) {
  weak_ordering(weak_order(std::forward<T>(t), std::forward<U>(u)));
};

template<class T, class U>
concept weakly_ordered = std::floating_point<std::remove_reference_t<T>> || adl_weak<T, U> ||
  cmp3way<weak_ordering, T, U> || strongly_ordered<T, U>;

struct WeakOrder {
 private:
  template<class T, class U>
  static constexpr bool
  _noexcept() {
    if constexpr (std::floating_point<std::decay_t<T>>)
      return true;
    else if constexpr (adl_weak<T, U>)
      return noexcept(weak_order(std::declval<T>(), std::declval<U>()));
    else if constexpr (cmp3way<weak_ordering, T, U>)
      return noexcept(compare_three_way()(std::declval<T>(), std::declval<U>()));
    else if constexpr (strongly_ordered<T, U>)
      return noexcept(StrongOrder{}(std::declval<T>(), std::declval<U>()));
  }

 public:
  template<class T, decayed_same_as<T> U>
    requires weakly_ordered<T, U>
  constexpr weak_ordering
  operator()(T&& t, U&& u) const noexcept(_noexcept<T, U>()) {
    if constexpr (adl_weak<T, U>)
      return weak_order(std::forward<T>(t), std::forward<U>(u));
    else if constexpr (cmp3way<weak_ordering, T, U>)
      return compare_three_way()(std::forward<T>(t), std::forward<U>(u));
    else if constexpr (strongly_ordered<T, U>)
      return StrongOrder{}(std::declval<T>(), std::declval<U>());
  }
};

template<class T, class U>
concept adl_partial = requires(T&& t, U&& u) {
  partial_ordering(partial_order(std::forward<T>(t), std::forward<U>(u)));
};

template<class T, class U>
concept partially_ordered =
  adl_partial<T, U> || cmp3way<partial_ordering, T, U> || weakly_ordered<T, U>;

struct PartialOrder {
 private:
  template<class T, class U>
  static constexpr bool
  _noexcept() {
    if constexpr (adl_partial<T, U>)
      return noexcept(partial_order(std::declval<T>(), std::declval<U>()));
    else if constexpr (cmp3way<partial_ordering, T, U>)
      return noexcept(compare_three_way()(std::declval<T>(), std::declval<U>()));
    else if constexpr (weakly_ordered<T, U>)
      return noexcept(WeakOrder{}(std::declval<T>(), std::declval<U>()));
  }

 public:
  template<class T, decayed_same_as<T> U>
    requires partially_ordered<T, U>
  constexpr partial_ordering
  operator()(T&& t, U&& u) const noexcept(_noexcept<T, U>()) {
    if constexpr (adl_partial<T, U>)
      return partial_order(std::forward<T>(t), std::forward<U>(u));
    else if constexpr (cmp3way<partial_ordering, T, U>)
      return compare_three_way()(std::forward<T>(t), std::forward<U>(u));
    else if constexpr (strongly_ordered<T, U>)
      return WeakOrder{}(std::declval<T>(), std::declval<U>());
  }
};

template<class T, class U>
concept op_eq_lt = requires(T&& t, U&& u) {
  { std::forward<T>(t) == std::forward<U>(u) } -> std::convertible_to<bool>;
  { std::forward<T>(t) < std::forward<U>(u) } -> std::convertible_to<bool>;
};

struct StrongOrderFallback {
 private:
  template<class T, class U>
  static constexpr bool
  _noexcept() {
    if constexpr (strongly_ordered<T, U>)
      return noexcept(StrongOrder{}(std::declval<T>(), std::declval<U>()));
    else if constexpr (op_eq_lt<T, U>)
      return noexcept(std::declval<T>() ==
                      std::declval<U>())&& noexcept(std::declval<T>() < std::declval<U>());
  }

 public:
  template<class T, decayed_same_as<T> U>
    requires strongly_ordered<T, U> || op_eq_lt<T, U>
  constexpr strong_ordering
  operator()(T&& t, U&& u) const noexcept(_noexcept<T, U>()) {
    if constexpr (strongly_ordered<T, U>)
      return StrongOrder{}(std::forward<T>(t), std::forward<U>(u));
    else if constexpr (op_eq_lt<T, U>)
      return std::forward<T>(t) == std::forward<U>(u) ? strong_ordering::equal :
             std::forward<T>(t) < std::forward<U>(u)  ? strong_ordering::less :
                                                        strong_ordering::greater;
  }
};

struct WeakOrderFallback {
 private:
  template<class T, class U>
  static constexpr bool
  _noexcept() {
    if constexpr (weakly_ordered<T, U>)
      return noexcept(WeakOrder{}(std::declval<T>(), std::declval<U>()));
    else if constexpr (op_eq_lt<T, U>)
      return noexcept(std::declval<T>() ==
                      std::declval<U>())&& noexcept(std::declval<T>() < std::declval<U>());
  }

 public:
  template<class T, decayed_same_as<T> U>
    requires weakly_ordered<T, U> || op_eq_lt<T, U>
  constexpr weak_ordering
  operator()(T&& t, U&& u) const noexcept(_noexcept<T, U>()) {
    if constexpr (weakly_ordered<T, U>)
      return WeakOrder{}(std::forward<T>(t), std::forward<U>(u));
    else if constexpr (op_eq_lt<T, U>)
      return std::forward<T>(t) == std::forward<U>(u) ? weak_ordering::equivalent :
             std::forward<T>(t) < std::forward<U>(u)  ? weak_ordering::less :
                                                        weak_ordering::greater;
  }
};

template<class T, class U>
concept op_eq_lt_lt = op_eq_lt<T, U> && requires(T&& t, U&& u) {
  { std::forward<U>(u) < std::forward<T>(t) } -> std::convertible_to<bool>;
};

struct PartialOrderFallback {
 private:
  template<class T, class U>
  static constexpr bool
  _noexcept() {
    if constexpr (partially_ordered<T, U>)
      return noexcept(PartialOrder{}(std::declval<T>(), std::declval<U>()));
    else if constexpr (op_eq_lt_lt<T, U>)
      return noexcept(std::declval<T>() == std::declval<U>())&& noexcept(
        std::declval<T>() < std::declval<U>())&& noexcept(std::declval<U>() < std::declval<T>());
  }

 public:
  template<class T, decayed_same_as<T> U>
    requires partially_ordered<T, U> || op_eq_lt_lt<T, U>
  constexpr partial_ordering
  operator()(T&& t, U&& u) const noexcept(_noexcept<T, U>()) {
    if constexpr (partially_ordered<T, U>)
      return PartialOrder{}(std::forward<T>(t), std::forward<U>(u));
    else if constexpr (op_eq_lt_lt<T, U>)
      return std::forward<T>(t) == std::forward<U>(u) ? partial_ordering::equivalent :
             std::forward<T>(t) < std::forward<U>(u)  ? partial_ordering::less :
             std::forward<T>(u) < std::forward<U>(t)  ? partial_ordering::greater :
                                                        partial_ordering::unordered;
  }
};

inline namespace cmp {
inline constexpr StrongOrder strong_order;
inline constexpr WeakOrder weak_order;
inline constexpr PartialOrder partial_order;
inline constexpr StrongOrderFallback compare_strong_order_fallback;
inline constexpr WeakOrderFallback compare_weak_order_fallback;
inline constexpr PartialOrderFallback compare_partial_order_fallback;
}  // namespace cmp

inline constexpr auto synth_three_way =
  []<class T, class U>(const T& t, const U& u) requires requires {
  { t < u } -> boolean_testable;
  { u < t } -> boolean_testable;
}
{
  if constexpr (three_way_comparable_with<T, U>)
    return t <=> u;
  else {
    if (t < u) return weak_ordering::less;
    if (u < t) return weak_ordering::greater;
    return weak_ordering::equivalent;
  }
};

template<class T, class U = T>
using synth_three_way_result = decltype(synth_three_way(std::declval<T&>(), std::declval<U&>()));

}  // namespace ranges

int
main() {}