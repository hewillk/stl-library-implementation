#pragma once

#include <concepts>
#include <iterator>

namespace ranges {

template<class>
struct incrementable_traits {};

template<class T>
  requires std::is_object_v<T>
struct incrementable_traits<T*> {
  using difference_type = std::ptrdiff_t;
};

template<class I>
struct incrementable_traits<const I> : incrementable_traits<I> {};

template<class T>
  requires requires { typename T::difference_type; }
struct incrementable_traits<T> {
  using difference_type = typename T::difference_type;
};

template<class T>
  requires(
    !requires { typename T::difference_type; } &&
    requires(const T& a, const T& b) {
      { a - b } -> std::integral;
    })
struct incrementable_traits<T> {
  using difference_type = std::make_signed_t<decltype(std::declval<T>() - std::declval<T>())>;
};

namespace detail {
template<class>
struct cond_value_type {};

template<class T>
  requires std::is_object_v<T>
struct cond_value_type<T> {
  using value_type = std::remove_cv_t<T>;
};

template<class T>
concept has_member_value_type = requires {
  typename T::value_type;
};

template<class T>
concept has_member_element_type = requires {
  typename T::element_type;
};

}  // namespace detail

template<class>
struct indirectly_readable_traits {};

template<class T>
struct indirectly_readable_traits<T*> : detail::cond_value_type<T> {};

template<class I>
  requires std::is_array_v<I>
struct indirectly_readable_traits<I> {
  using value_type = std::remove_cv_t<std::remove_extent_t<I>>;
};

template<class I>
struct indirectly_readable_traits<const I> : indirectly_readable_traits<I> {};

template<detail::has_member_value_type T>
struct indirectly_readable_traits<T> : detail::cond_value_type<typename T::value_type> {};

template<detail::has_member_element_type T>
struct indirectly_readable_traits<T> : detail::cond_value_type<typename T::element_type> {};

template<detail::has_member_value_type T>
  requires detail::has_member_element_type<T> &&
    std::same_as<std::remove_cv_t<typename T::element_type>,
                 std::remove_cv_t<typename T::value_type>>
struct indirectly_readable_traits<T> : detail::cond_value_type<typename T::value_type> {
};

template<detail::has_member_value_type T>
  requires detail::has_member_element_type<T>
struct indirectly_readable_traits<T> {
};

namespace detail {

template<class T>
using with_reference = T&;
template<class T>
concept can_reference = requires {
  typename with_reference<T>;
};
template<class T>
concept dereferenceable = requires(T& t) {
  { *t } -> can_reference;
};

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

}  // namespace detail

template<class I>
concept weakly_incrementable = std::default_initializable<I> && std::movable<I> && requires(I i) {
  typename std::iter_difference_t<I>;
  requires std::signed_integral<std::iter_difference_t<I>>;
  { ++i } -> std::same_as<I&>;
  i++;
};

template<class I>
concept incrementable = std::regular<I> && weakly_incrementable<I> && requires(I i) {
  { i++ } -> std::same_as<I>;
};

namespace detail {

  template<class In>
  concept indirectly_readable_impl = requires(const In in) {
    typename std::iter_value_t<In>;
    typename std::iter_reference_t<In>;
    typename std::iter_rvalue_reference_t<In>;
    { *in } -> std::same_as<std::iter_reference_t<In>>;
    { std::ranges::iter_move(in) } -> std::same_as<std::iter_rvalue_reference_t<In>>;
  }
  &&std::common_reference_with<std::iter_reference_t<In>&&, std::iter_value_t<In>&>&&
    std::common_reference_with<std::iter_reference_t<In>&&, std::iter_rvalue_reference_t<In>&&>&&
      std::common_reference_with<std::iter_rvalue_reference_t<In>&&, const std::iter_value_t<In>&>;

}  // namespace detail

template<class In>
concept indirectly_readable = detail::indirectly_readable_impl<std::remove_cvref_t<In>>;

template<class Out, class T>
concept indirectly_writable = requires(Out&& o, T&& t) {
  *o                                                                    = std::forward<T>(t);
  *std::forward<Out>(o)                                                 = std::forward<T>(t);
  const_cast<const std::iter_reference_t<Out>&&>(*o)                    = std::forward<T>(t);
  const_cast<const std::iter_reference_t<Out>&&>(*std::forward<Out>(o)) = std::forward<T>(t);
};

template<class In, class Out>
concept indirectly_movable =
  indirectly_readable<In> && indirectly_writable<Out, std::iter_rvalue_reference_t<In>>;

template<class In, class Out>
concept indirectly_movable_storable =
  indirectly_movable<In, Out> && indirectly_writable<Out, std::iter_value_t<In>> &&
  std::movable<std::iter_value_t<In>> &&
  std::constructible_from<std::iter_value_t<In>, std::iter_rvalue_reference_t<In>> &&
  std::assignable_from < std::iter_value_t<In>
&, std::iter_rvalue_reference_t<In> > ;

template<class In, class Out>
concept indirectly_copyable =
  indirectly_readable<In> && indirectly_writable<Out, std::iter_reference_t<In>>;

template<class In, class Out>
concept indirectly_copyable_storable = indirectly_copyable<In, Out> && indirectly_writable < Out,
        std::iter_value_t<In>
& > &&indirectly_writable<Out, const std::iter_value_t<In>&>&& indirectly_writable<
      Out, std::iter_value_t<In>&&>&& indirectly_writable<Out, const std::iter_value_t<In>&&>&&
      std::copyable<std::iter_value_t<In>>&&
        std::constructible_from<std::iter_value_t<In>, std::iter_reference_t<In>>&&
          std::assignable_from<std::iter_value_t<In>&, std::iter_reference_t<In>>;

template<class I>
concept cpp17_iterator = requires(I i) {
  { *i } -> detail::can_reference;
  { ++i } -> std::same_as<I&>;
  { *i++ } -> detail::can_reference;
}
&&std::copyable<I>;

template<class I>
concept cpp17_input_iterator = cpp17_iterator<I> && std::equality_comparable<I> && requires(I i) {
  typename incrementable_traits<I>::difference_type;
  typename indirectly_readable_traits<I>::value_type;
  typename std::common_reference_t<std::iter_reference_t<I>&&,
                                   typename indirectly_readable_traits<I>::value_type&>;
  typename std::common_reference_t<decltype(*i++)&&,
                                   typename indirectly_readable_traits<I>::value_type&>;
  requires std::signed_integral<typename incrementable_traits<I>::difference_type>;
};

template<class I>
concept cpp17_forward_iterator = cpp17_input_iterator<I> && std::constructible_from<I> &&
                                 std::is_lvalue_reference_v<std::iter_reference_t<I>> &&
                                 std::same_as < std::remove_cvref_t<std::iter_reference_t<I>>,
typename indirectly_readable_traits<I>::value_type > &&requires(I i) {
  { i++ } -> std::convertible_to<const I&>;
  { *i++ } -> std::same_as<std::iter_reference_t<I>>;
};

template<class I>
concept cpp17_bidirectional_iterator = cpp17_forward_iterator<I> && requires(I i) {
  { --i } -> std::same_as<I&>;
  { i-- } -> std::convertible_to<const I&>;
  { *i-- } -> std::same_as<std::iter_reference_t<I>>;
};

template<class I>
concept cpp17_random_access_iterator = cpp17_bidirectional_iterator<I> && std::totally_ordered<I> &&
  requires(I i, typename incrementable_traits<I>::difference_type n) {
  { i += n } -> std::same_as<I&>;
  { i -= n } -> std::same_as<I&>;
  { i + n } -> std::same_as<I>;
  { n + i } -> std::same_as<I>;
  { i - n } -> std::same_as<I>;
  { i - i } -> std::same_as<decltype(n)>;
  { i[n] } -> std::convertible_to<std::iter_reference_t<I>>;
};

template<class I>
concept input_or_output_iterator = requires(I i) {
  { *i } -> detail::can_reference;
}
&&weakly_incrementable<I>;

template<class S, class I>
concept sentinel_for = std::semiregular<S> && input_or_output_iterator<I> &&
  detail::weakly_equality_comparable_with<S, I>;

template<class S, class I>
concept sized_sentinel_for = sentinel_for<S, I> && requires(const I& i, const S& s) {
  { s - i } -> std::same_as<std::iter_difference_t<I>>;
  { i - s } -> std::same_as<std::iter_difference_t<I>>;
};

template<class I>
concept input_iterator = input_or_output_iterator<I> && indirectly_readable<I> && requires {
  typename std::__detail::__iter_concept<I>;
} && std::derived_from<std::__detail::__iter_concept<I>, std::input_iterator_tag>;

template<class I, class T>
concept output_iterator = input_or_output_iterator<I> && indirectly_writable<I, T> &&
  requires(I i, T&& t) {
  *i++ = std::forward<T>(t);
};

template<class I>
concept forward_iterator = input_iterator<I> &&
  std::derived_from<std::__detail::__iter_concept<I>, std::forward_iterator_tag> &&
  incrementable<I> && std::sentinel_for<I, I>;

template<class I>
concept bidirectional_iterator = forward_iterator<I> &&
  std::derived_from<std::__detail::__iter_concept<I>, std::bidirectional_iterator_tag> &&
  requires(I i) {
  { --i } -> std::same_as<I&>;
  { i-- } -> std::same_as<I>;
};

template<class I>
concept random_access_iterator = bidirectional_iterator<I> &&
  std::derived_from<std::__detail::__iter_concept<I>, std::random_access_iterator_tag> &&
  std::totally_ordered<I> && std::sized_sentinel_for<I, I> &&
  requires(I i, const I j, const std::iter_difference_t<I> n) {
  { i += n } -> std::same_as<I&>;
  { j + n } -> std::same_as<I>;
  { n + j } -> std::same_as<I>;
  { i -= n } -> std::same_as<I&>;
  { j - n } -> std::same_as<I>;
  { j[n] } -> std::same_as<std::iter_reference_t<I>>;
};

template<class I>
concept contiguous_iterator = random_access_iterator<I> &&
  std::derived_from<std::__detail::__iter_concept<I>, std::contiguous_iterator_tag> &&
  std::is_lvalue_reference_v<std::iter_reference_t<I>> &&
  std::same_as<std::iter_value_t<I>, std::remove_cvref_t<std::iter_reference_t<I>>> &&
  requires(const I& i) {
  { std::to_address(i) } -> std::same_as<std::add_pointer_t<std::iter_reference_t<I>>>;
};

}  // namespace ranges

namespace ranges::detail {
template<class T>
concept class_or_enum = std::is_class_v<T> || std::is_enum_v<T> || std::is_union_v<T>;
}

namespace ranges::cust {
void
iter_move();

template<class T>
concept adl_imove = detail::class_or_enum<std::remove_reference_t<T>> && requires(T&& t) {
  iter_move(std::forward<T>(t));
};

struct IMove {
 private:
  template<class T>
  struct result {
    using type = std::iter_reference_t<T>;
  };

  template<class T>
    requires adl_imove<T>
  struct result<T> {
    using type = decltype(iter_move(std::declval<T>()));
  };

  template<class T>
    requires(!adl_imove<T>)
  &&std::is_lvalue_reference_v<std::iter_reference_t<T>> struct result<T> {
    using type = decltype(std::move(*std::declval<T>()));
  };

  template<class T>
  static constexpr bool
  _noexcept() {
    if constexpr (adl_imove<T>)
      return noexcept(iter_move(std::declval<T>()));
    else
      return noexcept(*std::declval<T>());
  }

 public:
  template<class T>
  using type = typename result<T>::type;

  template<detail::dereferenceable T>
  constexpr type<T>
  operator()(T&& e) const noexcept(_noexcept<T>()) {
    if constexpr (adl_imove<T>)
      return iter_move(std::forward<T>(e));
    else if constexpr (std::is_lvalue_reference_v<std::iter_reference_t<T>>)
      return std::move(*e);
    else
      return *e;
  }
};

template<class X, class Y>
constexpr std::iter_value_t<X>
iter_exchange_move(X&& x, Y&& y) noexcept(
  noexcept(std::iter_value_t<X>(IMove{}(x))) && noexcept(*x = IMove{}(y))) {
  std::iter_value_t<X> old_value(IMove{}(x));
  *x = IMove{}(y);
  return old_value;
}

void iter_swap(auto, auto) = delete;

template<class X, class Y>
concept adl_iswap = (detail::class_or_enum<std::remove_reference_t<X>> ||
                     detail::class_or_enum<std::remove_reference_t<Y>>)&&requires(X&& x, Y&& y) {
  iter_swap(std::forward<X>(x), std::forward<Y>(y));
};

template<class X, class Y>
concept stl_swap = indirectly_readable<X> && indirectly_readable<Y> &&
  std::swappable_with<std::iter_reference_t<X>, std::iter_reference_t<Y>>;

struct ISwap {
 private:
  template<class X, class Y>
  static constexpr bool
  _noexcept() {
    if constexpr (adl_iswap<X, Y>)
      return noexcept(iter_swap(std::declval<X>(), std::declval<Y>()));
    else if constexpr (stl_swap<X, Y>)
      return noexcept(std::ranges::swap(*std::declval<X>(), *std::declval<Y>()));
    else
      return noexcept(*std::declval<X>() =
                        iter_exchange_move(std::declval<Y>(), std::declval<X>()));
  }

 public:
  template<class X, class Y>
    requires adl_iswap<X, Y> || stl_swap<X, Y> ||
      (indirectly_movable_storable<X, Y>&& indirectly_movable_storable<Y, X>)constexpr void
      operator()(X&& x, Y&& y) const noexcept(_noexcept<X, Y>()) {
      if constexpr (adl_iswap<X, Y>)
        iter_swap(std::forward<X>(x), std::forward<Y>(y));
      else if constexpr (stl_swap<X, Y>)
        std::ranges::swap(*x, *y);
      else
        *x = iter_exchange_move(y, x);
    }
};

}  // namespace ranges::cust

namespace ranges {
inline constexpr cust::IMove iter_move;
inline constexpr cust::ISwap iter_swap;
}  // namespace ranges

namespace ranges {

template<class F, class I>
concept indirectly_unary_invocable =
  indirectly_readable<I> && std::copy_constructible<F> && std::invocable < F&,
        std::iter_value_t<I>
& > &&std::invocable<F&, std::iter_reference_t<I>>&&
      std::invocable<F&, std::iter_common_reference_t<I>>&&
        std::common_reference_with<std::invoke_result_t<F&, std::iter_value_t<I>&>,
                                   std::invoke_result_t<F&, std::iter_reference_t<I>>>;

template<class F, class I>
concept indirectly_regular_unary_invocable =
  indirectly_readable<I> && std::copy_constructible<F> && std::regular_invocable < F&,
        std::iter_value_t<I>
& > &&std::regular_invocable<F&, std::iter_reference_t<I>>&&
      std::regular_invocable<F&, std::iter_common_reference_t<I>>&&
        std::common_reference_with<std::invoke_result_t<F&, std::iter_value_t<I>&>,
                                   std::invoke_result_t<F&, std::iter_reference_t<I>>>;

template<class F, class I>
concept indirect_unary_predicate =
  indirectly_readable<I> && std::copy_constructible<F> && std::predicate < F&,
        std::iter_value_t<I>
& > &&std::predicate<F&, std::iter_reference_t<I>>&&
      std::predicate<F&, std::iter_common_reference_t<I>>;

template<class F, class I1, class I2>
concept indirect_binary_predicate = indirectly_readable<I1> && indirectly_readable<I2> &&
                                    std::copy_constructible<F> && std::predicate < F&,
        std::iter_value_t<I1>
&, std::iter_value_t<I2>& >
     &&std::predicate<F&, std::iter_value_t<I1>&, std::iter_reference_t<I2>>&&
       std::predicate<F&, std::iter_reference_t<I1>, std::iter_value_t<I2>&>&&
         std::predicate<F&, std::iter_reference_t<I1>, std::iter_reference_t<I2>>&&
           std::predicate<F&, std::iter_common_reference_t<I1>, std::iter_common_reference_t<I2>>;

template<class F, class I1, class I2 = I1>
concept indirect_equivalence_relation =
  indirectly_readable<I1> && indirectly_readable<I2> && std::copy_constructible<F> &&
  std::equivalence_relation < F&,
        std::iter_value_t<I1>
&, std::iter_value_t<I2>& >
     &&std::equivalence_relation<F&, std::iter_value_t<I1>&, std::iter_reference_t<I2>>&&
       std::equivalence_relation<F&, std::iter_reference_t<I1>, std::iter_value_t<I2>&>&&
         std::equivalence_relation<F&, std::iter_reference_t<I1>, std::iter_reference_t<I2>>&&
           std::equivalence_relation<F&, std::iter_common_reference_t<I1>,
                                     std::iter_common_reference_t<I2>>;

template<class F, class I1, class I2 = I1>
concept indirect_strict_weak_order = indirectly_readable<I1> && indirectly_readable<I2> &&
                                     std::copy_constructible<F> && std::strict_weak_order < F&,
        std::iter_value_t<I1>
&, std::iter_value_t<I2>& >
     &&std::strict_weak_order<F&, std::iter_value_t<I1>&, std::iter_reference_t<I2>>&&
       std::strict_weak_order<F&, std::iter_reference_t<I1>, std::iter_value_t<I2>&>&&
         std::strict_weak_order<F&, std::iter_reference_t<I1>, std::iter_reference_t<I2>>&&
           std::strict_weak_order<F&, std::iter_common_reference_t<I1>,
                                  std::iter_common_reference_t<I2>>;

template<class F, class... Is>
  requires(indirectly_readable<Is>&&...)
&&std::invocable<F, std::iter_reference_t<Is>...> using indirect_result_t =
  std::invoke_result_t<F, std::iter_reference_t<Is>...>;

template<indirectly_readable I, indirectly_regular_unary_invocable<I> Proj>
struct projected {
  using value_type = std::remove_cvref_t<indirect_result_t<Proj&, I>>;
  indirect_result_t<Proj&, I>
  operator*() const;
};

template<weakly_incrementable I, class Proj>
struct incrementable_traits<projected<I, Proj>> {
  using difference_type = std::iter_difference_t<I>;
};

template<class I1, class I2 = I1>
concept indirectly_swappable = indirectly_readable<I1> && indirectly_readable<I2> &&
  requires(const I1 i1, const I2 i2) {
  ranges::iter_swap(i1, i1);
  ranges::iter_swap(i2, i2);
  ranges::iter_swap(i1, i2);
  ranges::iter_swap(i2, i1);
};

template<class I1, class I2, class R, class P1 = std::identity, class P2 = std::identity>
concept indirectly_comparable = indirect_binary_predicate<R, projected<I1, P1>, projected<I2, P2>>;

template<class I>
concept permutable =
  forward_iterator<I> && indirectly_movable_storable<I, I> && indirectly_swappable<I, I>;

template<class I1, class I2, class Out, class R = std::ranges::less, class P1 = std::identity,
         class P2 = std::identity>
concept mergeable = input_iterator<I1> && input_iterator<I2> && weakly_incrementable<Out> &&
  indirectly_copyable<I1, Out> && indirectly_copyable<I2, Out> &&
  indirect_strict_weak_order<R, projected<I1, P1>, projected<I2, P2>>;

template<class I, class R = std::ranges::less, class P = std::identity>
concept sortable = permutable<I> && indirect_strict_weak_order<R, projected<I, P>>;

}  // namespace ranges
