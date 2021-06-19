#pragma once

#include <type_traits>

namespace ranges {

template<class T, class U>
using cond_type = decltype(false ? std::declval<T>() : std::declval<U>());

template<class T, class U>
concept can_cond_type = requires {
  typename cond_type<T, U>;
};

template<class T, class U>
struct common_type_impl {};

template<class T, class U>
  requires can_cond_type<T, U>
struct common_type_impl<T, U> {
  using type = std::decay_t<cond_type<T, U>>;
};

template<class T, class U>
  requires(!can_cond_type<T, U>)
&&can_cond_type<const T&, const U&> struct common_type_impl<T, U> {
  using type = std::remove_cvref_t<cond_type<const T&, const U&>>;
};

template<class...>
struct common_type;

template<class... Ts>
using common_type_t = common_type<Ts...>::type;

template<>
struct common_type<> {};

template<class T>
struct common_type<T> : common_type<T, T> {};

template<class T, class U, class D1 = std::decay_t<T>, class D2 = std::decay_t<U>>
struct common_type2 : common_type<D1, D2> {};

template<class T, class U>
struct common_type2<T, U, T, U> : common_type_impl<T, U> {};

template<class T, class U>
struct common_type<T, U> : common_type2<T, U> {};

template<class T, class U, class... Rest>
struct common_type3 {};

template<class T, class U, class... Rest>
  requires requires { typename common_type_t<T, U>; }
struct common_type3<T, U, Rest...> : common_type<common_type_t<T, U>, Rest...> {};

template<class T, class U, class... Rest>
struct common_type<T, U, Rest...> : common_type3<T, U, Rest...> {};

template<class From>
struct copy_cv_impl {
  template<class To>
  using apply = To;
};

template<class From>
struct copy_cv_impl<const From> {
  template<class To>
  using apply = const To;
};

template<class From>
struct copy_cv_impl<volatile From> {
  template<class To>
  using apply = volatile To;
};

template<class From>
struct copy_cv_impl<const volatile From> {
  template<class To>
  using apply = const volatile To;
};

template<class From, class To>
using copy_cv = copy_cv_impl<From>::template apply<To>;

template<class From>
struct add_qualifiers {
  template<class To>
  using apply = copy_cv<From, To>;
};

template<class From>
struct add_qualifiers<From&> {
  template<class To>
  using apply = std::add_lvalue_reference_t<copy_cv<From, To>>;
};

template<class From>
struct add_qualifiers<From&&> {
  template<class To>
  using apply = std::add_rvalue_reference_t<copy_cv<From, To>>;
};

template<class...>
struct common_reference;

template<class... Ts>
using common_reference_t = common_reference<Ts...>::type;

template<>
struct common_reference<> {};

template<class T>
struct common_reference<T> {
  using type = T;
};

template<class T, class U>
struct common_reference2C : common_type<T, U> {};

template<class T, class U>
using cond_res = decltype(false ? std::declval<T (&)()>()() : std::declval<U (&)()>()());

template<class T, class U>
  requires requires { typename cond_res<T, U>; }
struct common_reference2C<T, U> {
  using type = cond_res<T, U>;
};

template<class, class, template<class> class, template<class> class>
struct basic_common_reference {};

template<class T, class U>
using basic_specialization = basic_common_reference<std::remove_cvref_t<T>, std::remove_cvref_t<U>,
                                                    add_qualifiers<T>::template apply,
                                                    add_qualifiers<U>::template apply>::type;

template<class T, class U>
struct common_reference2B : common_reference2C<T, U> {};

template<class T, class U>
  requires requires { typename basic_specialization<T, U>; }
struct common_reference2B<T, U> {
  using type = basic_specialization<T, U>;
};

template<class T, class U>
struct common_reference2A : common_reference2B<T, U> {};

template<class T, class U, class Result = cond_res<copy_cv<T, U>&, copy_cv<U, T>&>>
  requires std::is_lvalue_reference_v<Result>
using ll_common_ref = Result;

template<class T, class U>
  requires requires { typename ll_common_ref<T, U>; }
struct common_reference2A<T&, U&> {
  using type = ll_common_ref<T, U>;
};

template<class T, class U>
  requires std::is_convertible_v<T&&, ll_common_ref<const T, U>>
struct common_reference2A<T&&, U&> {
  using type = ll_common_ref<const T, U>;
};

template<class T, class U>
  requires std::is_convertible_v<U&&, ll_common_ref<const U, T>>
struct common_reference2A<T&, U&&> {
  using type = ll_common_ref<const U, T>;
};

template<class T, class U>
using rr_common_ref = std::remove_reference_t<ll_common_ref<T, U>>&&;

template<class T, class U>
  requires std::is_convertible_v<T&&, rr_common_ref<T, U>> &&
    std::is_convertible_v<U&&, rr_common_ref<T, U>>
struct common_reference2A<T&&, U&&> {
  using type = rr_common_ref<T, U>;
};

template<class T, class U>
struct common_reference<T, U> : common_reference2A<T, U> {};

template<class T, class U, class... Rest>
struct common_reference3 {};

template<class T, class U, class... Rest>
  requires requires { typename common_reference_t<T, U>; }
struct common_reference3<T, U, Rest...> : common_reference<common_reference_t<T, U>, Rest...> {};

template<class T, class U, class... Rest>
struct common_reference<T, U, Rest...> : common_reference3<T, U, Rest...> {};

}  // namespace ranges