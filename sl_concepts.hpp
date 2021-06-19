#pragma once

#include <algorithm>
#include <functional>
#include <type_traits>

namespace ranges {

namespace detail {
template<class T, class U>
concept same_as_impl = std::is_same_v<T, U>;
}

template<class T, class U>
concept same_as = detail::same_as_impl<T, U> && detail::same_as_impl<U, T>;

template<class Derived, class Base>
concept derived_from = std::is_base_of_v<Base, Derived> &&
  std::is_convertible_v<const volatile Derived*, const volatile Base*>;

template<class From, class To>
concept convertible_to = std::is_convertible_v<From, To> &&
  requires(std::add_rvalue_reference_t<From> (&f)()) {
  static_cast<To>(f());
};

template<class T, class U>
concept common_reference_with =
  same_as<std::common_reference_t<T, U>, std::common_reference_t<U, T>> &&
  convertible_to<T, std::common_reference_t<T, U>> &&
  convertible_to<U, std::common_reference_t<T, U>>;

template<class T, class U>
concept common_with = same_as<std::common_type_t<T, U>, std::common_type_t<U, T>> && requires {
  static_cast<std::common_type_t<T, U>>(std::declval<T>());
  static_cast<std::common_type_t<T, U>>(std::declval<U>());
} && common_reference_with<std::add_lvalue_reference_t<const T>,
                           std::add_lvalue_reference_t<const U>> &&
  common_reference_with<std::add_lvalue_reference_t<std::common_type_t<T, U>>,
                        std::common_reference_t<std::add_lvalue_reference_t<const T>,
                                                std::add_lvalue_reference_t<const U>>>;

template<class T>
concept integral = std::is_integral_v<T>;

template<class T>
concept signed_integral = integral<T> && std::is_signed_v<T>;

template<class T>
concept unsigned_integral = integral<T> && !signed_integral<T>;

template<class T>
concept floating_point = std::is_floating_point_v<T>;

template<class LHS, class RHS>
concept assignable_from =
  std::is_lvalue_reference_v<LHS> && common_reference_with < const std::remove_reference_t<LHS>
&, const std::remove_reference_t<RHS>& > &&requires(LHS lhs, RHS&& rhs) {
  { lhs = std::forward<RHS>(rhs) } -> same_as<LHS>;
};

template<class T>
concept destructible = std::is_nothrow_destructible_v<T>;

template<class T, class... Args>
concept constructible_from = destructible<T> && std::is_constructible_v<T, Args...>;

template<class T>
concept default_initializable = constructible_from<T> && requires {
  T{};
  (void)::new T;
};

template<class T>
concept move_constructible = constructible_from<T, T> && convertible_to<T, T>;

template<class T>
concept copy_constructible = move_constructible<T> && constructible_from<T, T&> &&
  convertible_to<T&, T> && constructible_from<T, const T&> && convertible_to<const T&, T> &&
  constructible_from<T, const T> && convertible_to<const T, T>;

namespace ranges::detail {
  template<class T>
  concept class_or_enum = std::is_class_v<T> || std::is_enum_v<T> || std::is_union_v<T>;
}

namespace cust {

template<class T>
void
swap(T&, T&) = delete;

template<class T, class U>
concept adl_swap = (ranges::detail::class_or_enum<std::remove_reference_t<T>> ||
                    ranges::detail::class_or_enum<std::remove_reference_t<U>>)&&requires(T&& t,
                                                                                         U&& u) {
  swap(std::forward<T>(t), std::forward<U>(u));
};

struct Swap {
 private:
  template<class T, class U>
  static constexpr bool
  _noexcept() {
    if constexpr (adl_swap<T, U>)
      return noexcept(swap(std::declval<T>(), std::declval<U>()));
    else
      return std::is_nothrow_move_constructible_v<std::remove_reference_t<T>> &&
             std::is_nothrow_move_assignable_v<std::remove_reference_t<T>>;
  }

 public:
  template<class T, class U>
    requires adl_swap<T, U> ||
      (same_as<T, U>&& std::is_lvalue_reference_v<T>&& move_constructible<
        std::remove_reference_t<T>>&& assignable_from<T, std::remove_reference_t<T>>)constexpr void
      operator()(T&& t, U&& u) const noexcept(_noexcept<T, U>()) {
      if constexpr (adl_swap<T, U>)
        swap(std::forward<T>(t), std::forward<U>(u));
      else {
        auto tmp = std::move(t);
        t        = std::move(u);
        u        = std::move(tmp);
      }
    };

    template<class T, class U, std::size_t N>
      requires requires(const Swap& swap, T& t, U& u) { swap(t, u); }
    constexpr void
    operator()(T (&t)[N], U (&u)[N]) const noexcept(noexcept(std::declval<const Swap&>()(*t, *u))) {
      std::ranges::swap_ranges(t, u);
    }
};

}  // namespace cust

inline constexpr cust::Swap swap;

template<class T>
concept swappable = requires(T& a, T& b) {
  swap(a, b);
};

template<class T, class U>
concept swappable_with = common_reference_with<T, U> && requires(T&& t, U&& u) {
  swap(std::forward<T>(t), std::forward<T>(t));
  swap(std::forward<U>(u), std::forward<U>(u));
  swap(std::forward<T>(t), std::forward<U>(u));
  swap(std::forward<U>(u), std::forward<T>(t));
};

namespace detail {

  template<class T>
  concept boolean_testable_impl = convertible_to<T, bool>;

  template<class T>
  concept boolean_testable = boolean_testable_impl<T> && requires(T && t) {
    { !std::forward<T>(t) } -> boolean_testable_impl;
  };

  template<class T, class U>
  concept weakly_equality_comparable_with =
    requires(const std::remove_reference_t<T>& t, const std::remove_reference_t<U>& u) {
    { t == u } -> boolean_testable;
    { t != u } -> boolean_testable;
    { u == t } -> boolean_testable;
    { u != t } -> boolean_testable;
  };

  template<class T, class U>
  concept partially_ordered_with =
    requires(const std::remove_reference_t<T>& t, const std::remove_reference_t<U>& u) {
    { t < u } -> boolean_testable;
    { t > u } -> boolean_testable;
    { t <= u } -> boolean_testable;
    { t >= u } -> boolean_testable;
    { u < t } -> boolean_testable;
    { u > t } -> boolean_testable;
    { u <= t } -> boolean_testable;
    { u >= t } -> boolean_testable;
  };

}  // namespace detail

template<class T>
concept equality_comparable = detail::weakly_equality_comparable_with<T, T>;

template<class T, class U>
concept equality_comparable_with = equality_comparable<T> && equality_comparable<U> &&
                                   common_reference_with < const std::remove_reference_t<T>
&, const std::remove_reference_t<U>& >
     &&equality_comparable<std::common_reference_t<const std::remove_reference_t<T>&,
                                                   const std::remove_reference_t<U>&>>&&
       detail::weakly_equality_comparable_with<T, U>;

template<class T>
concept totally_ordered = equality_comparable<T> && detail::partially_ordered_with<T, T>;

template<class T, class U>
concept totally_ordered_with =
  totally_ordered<T> && totally_ordered<U> && equality_comparable_with<T, U> &&
  totally_ordered < std::common_reference_t < const std::remove_reference_t<T>
&, const std::remove_reference_t<U>& >> &&detail::partially_ordered_with<T, U>;

template<class T>
concept movable =
  std::is_object_v<T> && move_constructible<T> && assignable_from<T&, T> && swappable<T>;

template<class T>
concept copyable = copy_constructible<T> && movable<T> && assignable_from<T&, T&> &&
  assignable_from<T&, const T&> && assignable_from<T&, const T>;

template<class T>
concept semiregular = copyable<T> && default_initializable<T>;

template<class T>
concept regular = semiregular<T> && equality_comparable<T>;

template<class F, class... Args>
concept invocable = requires(F&& f, Args&&... args) {
  std::invoke(std::forward<F>(f), std::forward<Args>(args)...);
};

template<class F, class... Args>
concept regular_invocable = invocable<F, Args...>;

template<class F, class... Args>
concept predicate =
  regular_invocable<F, Args...> && detail::boolean_testable<std::invoke_result_t<F, Args...>>;

template<class R, class T, class U>
concept relation =
  predicate<R, T, T> && predicate<R, U, U> && predicate<R, T, U> && predicate<R, U, T>;

template<class R, class T, class U>
concept equivalence_relation = relation<R, T, U>;

template<class R, class T, class U>
concept strict_weak_order = relation<R, T, U>;

}  // namespace ranges
