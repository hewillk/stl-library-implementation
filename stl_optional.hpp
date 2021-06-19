#pragma once

#include <cassert>
#include <exception>
#include <memory>
#include <utility>

namespace ranges {

struct nullopt_t {
  enum class Construct { Token };
  constexpr explicit nullopt_t(Construct) {}
};
inline constexpr nullopt_t nullopt{nullopt_t::Construct::Token};

class bad_optional_access : public std::exception {
 public:
  const char*
  what() const noexcept override {
    return "bad optional access";
  }
};

inline void
throw_bad_optional_access() {
  throw bad_optional_access{};
}

template<class T>
struct optional_payload_base {
  using stored_type = std::remove_const_t<T>;

  struct empty_base {};

  union storage {
    empty_base empty_;
    stored_type value_;

    constexpr storage() noexcept : empty_() {}

    storage(const storage&) = default;
    storage(storage&&)      = default;

    template<class... Args>
    constexpr storage(std::in_place_t, Args&&... args) : value_(std::forward<Args>(args)...) {}

    template<class V, class... Args>
    constexpr storage(std::initializer_list<V> il, Args&&... args)
    : value_(il, std::forward<Args>(args)...) {}

    ~storage() requires std::is_trivially_destructible_v<stored_type>
    = default;
    ~storage() {}
  };

  storage payload_;
  bool engaged_            = false;

  optional_payload_base()  = default;
  ~optional_payload_base() = default;

  template<class... Args>
  constexpr optional_payload_base(std::in_place_t, Args&&... args)
  : payload_(std::in_place, std::forward<Args>(args)...), engaged_(true) {}

  template<class U, class... Args>
  constexpr optional_payload_base(std::initializer_list<U> il, Args&&... args)
  : payload_(il, std::forward<Args>(args)...), engaged_(true) {}

  optional_payload_base(
    const optional_payload_base&) requires std::is_trivially_copy_constructible_v<T>
  = default;
  optional_payload_base(optional_payload_base&&) requires std::is_trivially_move_constructible_v<T>
  = default;

  constexpr optional_payload_base(const optional_payload_base& other) {
    if (other.engaged_) construct(other.get());
  }

  constexpr optional_payload_base(optional_payload_base&& other) {
    if (other.engaged_) construct(std::move(other.get()));
  }

  optional_payload_base&
  operator=(const optional_payload_base&) = default;
  optional_payload_base&
  operator=(optional_payload_base&&) = default;

  constexpr void
  copy_assign(const optional_payload_base& other) {
    if (engaged_ && other.engaged_)
      get() = other.get();
    else {
      if (other.engaged_)
        construct(other.get());
      else
        reset();
    }
  }

  constexpr void
  move_assign(optional_payload_base&& other) noexcept(
    std::is_nothrow_move_constructible_v<T>&& std::is_nothrow_move_assignable_v<T>) {
    if (engaged_ && other.engaged_)
      get() = std::move(other.get());
    else {
      if (other.engaged_)
        construct(std::move(other.get()));
      else
        reset();
    }
  }

  template<class... Args>
  void
  construct(Args&&... args) noexcept(std::is_nothrow_constructible_v<stored_type, Args...>) {
    ::new ((void*)std::addressof(payload_)) stored_type(std::forward<Args>(args)...);
    engaged_ = true;
  }

  constexpr T&
  get() noexcept {
    return payload_.value_;
  }

  constexpr const T&
  get() const noexcept {
    return payload_.value_;
  }

  constexpr void
  reset() noexcept {
    if (engaged_) {
      engaged_ = false;
      payload_.value_.~stored_type();
    }
  }
};

template<class T>
struct optional_payload : optional_payload_base<T> {
  using optional_payload_base<T>::optional_payload_base;

  optional_payload()                        = default;
  optional_payload(const optional_payload&) = default;
  optional_payload(optional_payload&&)      = default;

  optional_payload&
  operator=(const optional_payload&) requires std::is_trivially_copy_constructible_v<T> &&
    std::is_trivially_copy_assignable_v<T>
  = default;

  constexpr optional_payload&
  operator=(const optional_payload& other) {
    this->copy_assign(other);
    return *this;
  }

  optional_payload&
  operator=(optional_payload&&) requires std::is_trivially_move_constructible_v<T> &&
    std::is_trivially_move_assignable_v<T>
  = default;

  constexpr optional_payload&
  operator=(optional_payload&& other) noexcept(
    std::is_nothrow_move_constructible_v<T>&& std::is_nothrow_move_assignable_v<T>) {
    this->move_assign(std::move(other));
    return *this;
  }

  ~optional_payload() requires std::is_trivially_destructible_v<T>
  = default;
  ~optional_payload() { this->reset(); }
};

template<class T>
struct optional_base {
  using stored_type = std::remove_const_t<T>;

  template<class... Args>
  void
  construct(Args&&... args) noexcept(std::is_nothrow_constructible_v<stored_type, Args...>) {
    payload_.construct(std::forward<Args>(args)...);
  }

  constexpr void
  reset() noexcept {
    payload_.reset();
  }

  constexpr bool
  is_engaged() const noexcept {
    return payload_.engaged_;
  }

  constexpr T&
  get() noexcept {
    assert(is_engaged());
    return payload_.get();
  }

  constexpr const T&
  get() const noexcept {
    assert(is_engaged());
    return payload_.get();
  }

  optional_payload<T> payload_;

  constexpr optional_base() = default;

  template<class... Args>
    requires std::is_constructible_v<T, Args...>
  constexpr explicit optional_base(std::in_place_t, Args&&... args)
  : payload_(std::in_place, std::forward<Args>(args)...) {}

  template<class U, class... Args>
    requires std::is_constructible_v < T, std::initializer_list<U>
  &, Args... > constexpr explicit optional_base(std::in_place_t, std::initializer_list<U> il,
                                                Args&&... args)
  : payload_(std::in_place, il, std::forward<Args>(args)...) {}

  optional_base(const optional_base&) requires std::is_trivially_copy_constructible_v<T>
  = default;
  optional_base(optional_base&&) requires std::is_trivially_move_constructible_v<T>
  = default;

  constexpr optional_base(const optional_base& other) : payload_(other.payload_) {}

  constexpr optional_base(optional_base&& other) noexcept(std::is_nothrow_move_constructible_v<T>)
  : payload_(std::move(other.payload_)) {}

  constexpr optional_base&
  operator=(const optional_base&) = default;
  constexpr optional_base&
  operator=(optional_base&&) = default;
};

template<class>
class optional;

template<class T, class U>
concept converts_from_optional = std::is_constructible_v < T,
        const optional<U>
& > || std::is_constructible_v<T, optional<U>&> ||
  std::is_constructible_v<T, const optional<U>&&> || std::is_constructible_v<T, optional<U>&&> ||
  std::is_constructible_v<const optional<U>&, T> || std::is_constructible_v<optional<U>&, T> ||
  std::is_constructible_v<const optional<U>&&, T> || std::is_constructible_v<optional<U>&&, T>;

template<class T, class U>
concept assigns_from_optional = std::is_assignable_v < T&,
        const optional<U>
& > || std::is_assignable_v<T&, optional<U>&> || std::is_assignable_v<T&, const optional<U>&&> ||
  std::is_assignable_v<T&, optional<U>&&>;

template<class T, class U>
concept not_self = !std::same_as<optional<T>, std::remove_cvref_t<U>>;

template<class T, class U>
concept not_tag = !std::same_as<std::in_place_t, std::remove_cvref_t<U>>;

template<class T>
class optional : private optional_base<T> {
  static_assert(!std::same_as<std::remove_cv_t<T>, nullopt_t>);
  static_assert(!std::same_as<std::remove_cv_t<T>, std::in_place_t>);
  static_assert(!std::is_reference_v<T>);

  using Base = optional_base<T>;

 public:
  using value_type = T;

  constexpr optional() noexcept {}
  constexpr optional(nullopt_t) noexcept {}

  optional(const optional&) requires std::is_copy_constructible_v<T>
  = default;
  optional(optional&&) requires std::is_move_constructible_v<T>
  = default;

  optional(const optional&) = delete;
  optional(optional&&)      = delete;

  optional&
  operator=(
    const optional&) requires std::is_copy_constructible_v<T> && std::is_copy_assignable_v<T>
  = default;
  optional&
  operator=(optional&&) requires std::is_move_constructible_v<T> && std::is_move_assignable_v<T>
  = default;

  optional&
  operator=(const optional&) = delete;
  optional&
  operator=(optional&&) = delete;

  template<class U = T>
    requires(not_self<T, U>&& not_tag<T, U>&& std::is_constructible_v<T, U>)
  constexpr explicit(!std::is_convertible_v<U, T>)
    optional(U&& t) noexcept(std::is_nothrow_constructible_v<T, U>)
  : Base(std::in_place, std::forward<U>(t)) {}

  template<class U>
    requires(!std::same_as<T, U> && std::is_constructible_v<T, const U&> &&
             !converts_from_optional<T, U>)
  constexpr explicit(!std::is_convertible_v<const U&, T>)
    optional(const optional<U>& t) noexcept(std::is_nothrow_constructible_v<T, const U&>) {
    if (t) emplace(*t);
  }

  template<class U>
    requires(!std::same_as<T, U> && std::is_constructible_v<T, U> && !converts_from_optional<T, U>)
  constexpr explicit(!std::is_convertible_v<U, T>)
    optional(optional<U>&& t) noexcept(std::is_nothrow_constructible_v<T, U>) {
    if (t) emplace(std::move(*t));
  }

  template<class... Args>
    requires std::is_constructible_v<T, Args...>
  constexpr explicit optional(std::in_place_t,
                              Args&&... args) noexcept(std::is_nothrow_constructible_v<T, Args...>)
  : Base(std::in_place, std::forward<Args>(args)...) {}

  template<class U, class... Args>
    requires std::is_constructible_v < T, std::initializer_list<U>
  &, Args... >
       constexpr explicit optional(
         std::in_place_t, std::initializer_list<U> il,
         Args&&... args) noexcept(std::is_nothrow_constructible_v<T, std::initializer_list<U>&,
                                                                  Args...>)
  : Base(std::in_place, il, std::forward<Args>(args)...) {}

  optional&
  operator=(nullopt_t) noexcept {
    this->reset();
    return *this;
  }

  template<class U = T>
    requires(not_self<T, U> &&
             !(std::is_scalar_v<T> &&
               std::same_as<T, std::decay_t<U>>)&&std::is_constructible_v<T, U> &&
             std::is_assignable_v<T&, U>)
  optional&
  operator=(U&& u) noexcept(
    std::is_nothrow_constructible_v<T, U>&& std::is_nothrow_assignable_v<T&, U>) {
    if (this->is_engaged())
      this->get() = std::forward<U>(u);
    else
      this->construct(std::forward<U>(u));

    return *this;
  }

  template<class U>
    requires(!std::same_as<T, U> && std::is_constructible_v<T, const U&> &&
             std::is_assignable_v<T&, const U&> && !converts_from_optional<T, U> &&
             !assigns_from_optional<T, U>)
  optional&
  operator=(const optional<U>& u) noexcept(
    std::is_nothrow_constructible_v<T, const U&>&& std::is_nothrow_assignable_v<T&, const U&>) {
    if (u) {
      if (this->is_engaged())
        this->get() = *u;
      else
        this->construct(*u);
    } else
      this->reset();
    return *this;
  }

  template<class U>
    requires(!std::same_as<T, U> && std::is_constructible_v<T, U> && std::is_assignable_v<T&, U> &&
             !converts_from_optional<T, U> && !assigns_from_optional<T, U>)
  optional&
  operator=(optional<U>&& u) noexcept(
    std::is_nothrow_constructible_v<T, U>&& std::is_nothrow_assignable_v<T&, U>) {
    if (u) {
      if (this->is_engaged())
        this->get() = std::move(*u);
      else
        this->construct(std::move(*u));
    } else
      this->reset();
    return *this;
  }

  template<class... Args>
  requires std::is_constructible_v<T, Args...> T&
  emplace(Args&&... args) noexcept(std::is_nothrow_constructible_v<T, Args...>) {
    this->reset();
    this->construct(std::forward<Args>(args)...);
    return this->get();
  }

  template<class U, class... Args>
    requires std::is_constructible_v < T, std::initializer_list<U>
  &, Args... > T&
               emplace(std::initializer_list<U> il, Args&&... args) noexcept(
                 std::is_nothrow_constructible_v<T, std::initializer_list<U>&, Args...>) {
    this->reset();
    this->construct(il, std::forward<Args>(args)...);
    return this->get();
  }

  void
  swap(optional& other) noexcept(
    std::is_nothrow_move_constructible_v<T>&& std::is_nothrow_swappable_v<T>) {
    using std::swap;

    if (this->is_engaged() && other.is_engaged())
      swap(this->get(), other.get());
    else if (this->is_engaged()) {
      other.construct(std::move(this->get()));
      this->reset();
    } else if (other.is_engaged()) {
      this->construct(std::move(other.get()));
      other.reset();
    }
  }

  constexpr T*
  operator->() noexcept {
    return std::addressof(this->get());
  }

  constexpr const T*
  operator->() const noexcept {
    return std::addressof(this->get());
  }

  constexpr T&
  operator*() & noexcept {
    return this->get();
  }

  constexpr const T&
  operator*() const& noexcept {
    return this->get();
  }

  constexpr T&&
  operator*() && noexcept {
    return std::move(this->get());
  }

  constexpr const T&&
  operator*() const&& noexcept {
    std::move(this->get());
  }

  constexpr explicit operator bool() const noexcept { return this->is_engaged(); }

  constexpr bool
  has_value() const noexcept {
    return this->is_engaged();
  }

  constexpr T&
  value() & {
    return this->is_engaged() ? this->get() : throw_bad_optional_access();
  }

  constexpr const T&
  value() const& {
    return this->is_engaged() ? this->get() : throw_bad_optional_access();
  }

  constexpr T&&
  value() && {
    return this->is_engaged() ? std::move(this->get()) : throw_bad_optional_access();
  }

  constexpr const T&&
  value() const&& {
    return this->is_engaged() ? std::move(this->get()) : throw_bad_optional_access();
  }

  template<class U>
  constexpr T
  value_or(U&& u) const& {
    static_assert(std::is_copy_constructible_v<T>);
    static_assert(std::is_convertible_v<U, T>);

    return this->is_engaged() ? this->get() : static_cast<T>(std::forward<U>(u));
  }

  template<class U>
  constexpr T
  value_or(U&& u) && {
    static_assert(std::is_move_constructible_v<T>);
    static_assert(std::is_convertible_v<U, T>);

    return this->is_engaged() ? std::move(this->get()) : static_cast<T>(std::forward<U>(u));
  }

  void
  reset() noexcept {
    static_cast<Base*>(this)->reset();
  }
};

template<class T, class U>
  requires requires {
    { std::declval<const T&>() == std::declval<const U&>() } -> std::convertible_to<bool>;
  }
constexpr bool
operator==(const optional<T>& lhs, const optional<U>& rhs) {
  return static_cast<bool>(lhs) == static_cast<bool>(rhs) && (!lhs || *lhs == *rhs);
}

template<class T, class U>
  requires requires {
    { std::declval<const T&>() != std::declval<const U&>() } -> std::convertible_to<bool>;
  }
constexpr bool
operator!=(const optional<T>& lhs, const optional<U>& rhs) {
  return static_cast<bool>(lhs) != static_cast<bool>(rhs) ||
         (static_cast<bool>(lhs) && *lhs != *rhs);
}

template<class T, class U>
  requires requires {
    { std::declval<const T&>() < std::declval<const U&>() } -> std::convertible_to<bool>;
  }
constexpr bool
operator<(const optional<T>& lhs, const optional<U>& rhs) {
  return static_cast<bool>(rhs) && (!lhs || *lhs < *rhs);
}

template<class T, class U>
  requires requires {
    { std::declval<const T&>() > std::declval<const U&>() } -> std::convertible_to<bool>;
  }
constexpr bool
operator>(const optional<T>& lhs, const optional<U>& rhs) {
  return static_cast<bool>(lhs) && (!rhs || *lhs > *rhs);
}

template<class T, class U>
  requires requires {
    { std::declval<const T&>() <= std::declval<const U&>() } -> std::convertible_to<bool>;
  }
constexpr bool
operator<=(const optional<T>& lhs, const optional<U>& rhs) {
  return !lhs || (static_cast<bool>(rhs) && *lhs <= *rhs);
}

template<class T, class U>
  requires requires {
    { std::declval<const T&>() >= std::declval<const U&>() } -> std::convertible_to<bool>;
  }
constexpr bool
operator>=(const optional<T>& lhs, const optional<U>& rhs) {
  return !rhs || (static_cast<bool>(lhs) && *lhs >= *rhs);
}

template<class T, std::three_way_comparable_with<T> U>
constexpr std::compare_three_way_result_t<T, U>
operator<=>(const optional<T>& x, const optional<U>& y) {
  return x && y ? *x <=> *y : static_cast<bool>(x) <=> static_cast<bool>(y);
}

template<class T>
constexpr bool
operator==(const optional<T>& lhs, nullopt_t) noexcept {
  return !lhs;
}

template<class T>
constexpr std::strong_ordering
operator<=>(const optional<T>& x, nullopt_t) noexcept {
  return static_cast<bool>(x) <=> false;
}

template<class T, class U>
  requires requires {
    { std::declval<const T&>() == std::declval<const U&>() } -> std::convertible_to<bool>;
  }
constexpr bool
operator==(const optional<T>& lhs, const U& rhs) {
  return static_cast<bool>(lhs) && *lhs == rhs;
}

template<class T, class U>
  requires requires {
    { std::declval<const U&>() == std::declval<const T&>() } -> std::convertible_to<bool>;
  }
constexpr bool
operator==(const U& lhs, const optional<T>& rhs) {
  return static_cast<bool>(rhs) && lhs == *rhs;
}

template<class T, class U>
  requires requires {
    { std::declval<const T&>() != std::declval<const U&>() } -> std::convertible_to<bool>;
  }
constexpr bool
operator!=(const optional<T>& lhs, const U& rhs) {
  return !lhs || *lhs != rhs;
}

template<class T, class U>
  requires requires {
    { std::declval<const U&>() != std::declval<const T&>() } -> std::convertible_to<bool>;
  }
constexpr bool
operator!=(const U& lhs, const optional<T>& rhs) {
  return !rhs || lhs != *rhs;
}

template<class T, class U>
  requires requires {
    { std::declval<const T&>() < std::declval<const U&>() } -> std::convertible_to<bool>;
  }
constexpr bool
operator<(const optional<T>& lhs, const U& rhs) {
  return !lhs || *lhs < rhs;
}

template<class T, class U>
  requires requires {
    { std::declval<const U&>() < std::declval<const T&>() } -> std::convertible_to<bool>;
  }
constexpr bool
operator<(const U& lhs, const optional<T>& rhs) {
  return static_cast<bool>(rhs) || lhs < *rhs;
}

template<class T, class U>
  requires requires {
    { std::declval<const T&>() > std::declval<const U&>() } -> std::convertible_to<bool>;
  }
constexpr bool
operator>(const optional<T>& lhs, const U& rhs) {
  return static_cast<bool>(lhs) || *lhs > rhs;
}

template<class T, class U>
  requires requires {
    { std::declval<const U&>() > std::declval<const T&>() } -> std::convertible_to<bool>;
  }
constexpr bool
operator>(const U& lhs, const optional<T>& rhs) {
  return !rhs || lhs > *rhs;
}

template<class T, class U>
  requires requires {
    { std::declval<const T&>() <= std::declval<const U&>() } -> std::convertible_to<bool>;
  }
constexpr bool
operator<=(const optional<T>& lhs, const U& rhs) {
  return !lhs || *lhs <= rhs;
}

template<class T, class U>
  requires requires {
    { std::declval<const U&>() <= std::declval<const T&>() } -> std::convertible_to<bool>;
  }
constexpr bool
operator<=(const U& lhs, const optional<T>& rhs) {
  return static_cast<bool>(rhs) || *lhs <= rhs;
}

template<class T, class U>
  requires requires {
    { std::declval<const T&>() >= std::declval<const U&>() } -> std::convertible_to<bool>;
  }
constexpr bool
operator>=(const optional<T>& lhs, const U& rhs) {
  return static_cast<bool>(lhs) && *lhs >= rhs;
}

template<class T, class U>
  requires requires {
    { std::declval<const U&>() >= std::declval<const T&>() } -> std::convertible_to<bool>;
  }
constexpr bool
operator>=(const U& lhs, const optional<T>& rhs) {
  return !rhs || *lhs >= rhs;
}

template<class T, class U>
  requires(!std::same_as<U, optional<typename U::value_type>>)
&&std::three_way_comparable_with<T, U> constexpr std::compare_three_way_result_t<T, U>
operator<=>(const optional<T>& x, const U& v) {
  return static_cast<bool>(x) ? *x <=> v : std::strong_ordering::less;
}

template<class T>
  requires std::is_move_constructible_v<T> && std::is_swappable_v<T>
inline void
swap(optional<T>& lhs, optional<T>& rhs) noexcept(noexcept(lhs.swap(rhs))) {
  lhs.swap(rhs);
}

template<class T>
  requires std::is_constructible_v<std::decay_t<T>, T>
constexpr optional<std::decay_t<T>>
make_optional(T&& t) noexcept(std::is_nothrow_constructible_v<optional<std::decay_t<T>>, T>) {
  return optional<std::decay_t<T>>(std::forward<T>(t));
}

template<class T, class... Args>
  requires std::is_constructible_v<T, Args...>
constexpr optional<T>
make_optional(Args&&... args) noexcept(std::is_nothrow_constructible_v<T, Args...>) {
  return optional<T>(std::in_place, std::forward<Args>(args)...);
}

template<class T, class U, class... Args>
  requires std::is_constructible_v < T, std::initializer_list<U>
&, Args... > constexpr optional<T>
             make_optional(std::initializer_list<U> il, Args&&... args) noexcept(
               std::is_nothrow_constructible_v<T, std::initializer_list<U>&, Args...>) {
  return optional<T>(std::in_place, il, std::forward<Args>(args)...);
}

template<class T>
optional(T) -> optional<T>;

}  // namespace ranges