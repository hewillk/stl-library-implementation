#pragma once

#include "view_interface.hpp"

namespace ranges {

template <class W>
using iota_diff_t = std::conditional_t<
  std::integral<W>, std::conditional_t<sizeof(W) < sizeof(int), int, long long>,
  std::iter_difference_t<W>>;

template <class I>
concept decrementable = std::incrementable<I>&& requires(I i) {
  { --i }
  ->std::same_as<I&>;
  { i-- }
  ->std::same_as<I>;
};

template <class I>
concept advanceable =
  decrementable<I>&& std::totally_ordered<I>&& requires(I i, const I j, const iota_diff_t<I> n) {
  { i += n }
  ->std::same_as<I&>;
  { i -= n }
  ->std::same_as<I&>;
  I(j + n);
  I(n + j);
  I(j - n);
  { j - j }
  ->std::convertible_to<iota_diff_t<I>>;
};

template <std::weakly_incrementable W, std::semiregular Bound = std::unreachable_sentinel_t>
requires weakly_equality_comparable_with<W, Bound>&& std::semiregular<W> class iota_view
  : public view_interface<iota_view<W, Bound>> {
 private:
  struct sentinel;

  template <class T>
  struct ter_cat {};
  template <std::incrementable T>
  struct ter_cat<T> {
    using iterator_category = std::input_iterator_tag;
  };

  struct iterator : ter_cat<W> {
   private:
    W value_ = W();

    friend sentinel;

    static auto iter_concept() {
      if constexpr (advanceable<W>)
        return std::random_access_iterator_tag{};
      else if constexpr (decrementable<W>)
        return std::bidirectional_iterator_tag{};
      else if constexpr (std::incrementable<W>)
        return std::forward_iterator_tag{};
      else
        return std::input_iterator_tag{};
    }

   public:
    using iterator_concept = decltype(iter_concept());
    using value_type = W;
    using difference_type = iota_diff_t<W>;

    iterator() = default;

    constexpr explicit iterator(W value) : value_(value) {}

    constexpr W operator*() const noexcept(std::is_nothrow_copy_constructible_v<W>) {
      return value_;
    }

    constexpr iterator& operator++() {
      ++value_;
      return *this;
    }

    constexpr auto operator++(int) requires std::incrementable<W> {
      if constexpr (std::incrementable<W>) {
        auto tmp = *this;
        ++value_;
        return tmp;
      } else
        ++value_;
    }

    constexpr iterator& operator--() requires decrementable<W> {
      --value_;
      return *this;
    }

    constexpr iterator operator--(int) requires decrementable<W> {
      auto tmp = *this;
      --*this;
      return tmp;
    }

    constexpr iterator& operator+=(difference_type n) requires advanceable<W> {
      if constexpr (is_integer_like<W> && !is_signed_integer_like<W>) {
        if (n >= difference_type(0))
          value_ += static_cast<W>(n);
        else
          value_ -= static_cast<W>(-n);
      } else
        value_ += n;
      return *this;
    }

    constexpr iterator& operator-=(difference_type n) requires advanceable<W> {
      if constexpr (is_integer_like<W> && !is_signed_integer_like<W>) {
        if (n >= difference_type(0))
          value_ -= static_cast<W>(n);
        else
          value_ += static_cast<W>(-n);
      } else
        value_ -= n;
      return *this;
    }

    constexpr W operator[](difference_type n) const requires advanceable<W> {
      return W(value_ + n);
    }

    friend constexpr bool operator==(
      const iterator& x, const iterator& y) requires std::equality_comparable<W> {
      return x.value_ == y.value_;
    }

    friend constexpr bool operator<(
      const iterator& x, const iterator& y) requires std::totally_ordered<W> {
      return x.value_ < y.value_;
    }

    friend constexpr bool operator>(
      const iterator& x, const iterator& y) requires std::totally_ordered<W> {
      return y < x;
    }

    friend constexpr bool operator<=(
      const iterator& x, const iterator& y) requires std::totally_ordered<W> {
      return !(y < x);
    }

    friend constexpr bool operator>=(
      const iterator& x, const iterator& y) requires std::totally_ordered<W> {
      return !(x < y);
    }

    friend constexpr auto operator<=>(
      const iterator& x,
      const iterator& y) requires std::totally_ordered<W>&& std::three_way_comparable<W> {
      return x.value_ <=> y.value_;
    }

    friend constexpr iterator operator+(iterator i, difference_type n) requires advanceable<W> {
      return i += n;
    }

    friend constexpr iterator operator+(difference_type n, iterator i) requires advanceable<W> {
      return i + n;
    }

    friend constexpr iterator operator-(iterator i, difference_type n) requires advanceable<W> {
      return i -= n;
    }

    friend constexpr difference_type operator-(
      const iterator& x, const iterator& y) requires advanceable<W> {
      using D = difference_type;
      if constexpr (is_integer_like<W>) {
        if constexpr (is_signed_integer_like<W>)
          return D(D(x.value_) - D(y.value_));
        else
          return y.value_ > x.value_ ? D(-D(y.value_ - x.value_)) : D(x.value_ - y.value_);
      } else
        return x.value_ - y.value_;
    }
  };

  struct sentinel {
   private:
    Bound bound_ = Bound();

    constexpr bool equal(const iterator& x) const { return x.value_ == bound_; }

   public:
    sentinel() = default;
    constexpr explicit sentinel(Bound bound) : bound_(bound) {}

    friend constexpr bool operator==(const iterator& x, const sentinel& y) { return y.equal(x); }

    friend constexpr std::iter_difference_t<W> operator-(
      const iterator& x, const sentinel& y) requires std::sized_sentinel_for<Bound, W> {
      return x.value_ - y.bound_;
    }

    friend constexpr std::iter_difference_t<W> operator-(
      const sentinel& x, const iterator& y) requires std::sized_sentinel_for<Bound, W> {
      return -(y - x);
    }
  };

  W value_ = W();
  Bound bound_ = Bound();

 public:
  iota_view() = default;

  constexpr explicit iota_view(W value) : value_(value) {}

  constexpr iota_view(std::type_identity_t<W> value, std::type_identity_t<Bound> bound)
    : value_(value), bound_(bound) {
    if constexpr (std::totally_ordered_with<W, Bound>) assert(bool(value <= bound));
  }

  constexpr iota_view(iterator first, sentinel last) : iota_view(*first, last.bound_) {}

  constexpr iterator begin() const { return iterator{value_}; }

  constexpr auto end() const {
    if constexpr (std::same_as<Bound, std::unreachable_sentinel_t>)
      return std::unreachable_sentinel;
    else
      return sentinel{bound_};
  }

  constexpr iterator end() const requires std::same_as<W, Bound> { return iterator{bound_}; }

  constexpr auto size() const requires(
    (std::same_as<W, Bound> && advanceable<W>) || (std::integral<W> && std::integral<Bound>) ||
    std::sized_sentinel_for<Bound, W>) {
    if constexpr (is_integer_like<W> && is_integer_like<Bound>)
      return value_ < 0 ? (bound_ < 0 ? to_unsigned_like(-value_) - to_unsigned_like(-bound_)
                                      : to_unsigned_like(bound_) + to_unsigned_like(-value_))
                        : to_unsigned_like(bound_) - to_unsigned_like(value_);
    else
      return to_unsigned_like(bound_ - value_);
  }
};

template <class W, class Bound>
requires(
  !is_integer_like<W> || !is_integer_like<Bound> ||
  is_signed_integer_like<W> == is_signed_integer_like<Bound>) iota_view(W, Bound)
  ->iota_view<W, Bound>;

}  // namespace ranges

namespace views {

struct Iota {
  template <class T>
  constexpr auto operator()(T&& t) const {
    return ranges::iota_view{std::forward<T>(t)};
  }

  template <class T, class U>
  constexpr auto operator()(T&& t, U&& u) const {
    return ranges::iota_view{std::forward<T>(t), std::forward<U>(u)};
  }
};

inline constexpr Iota iota{};

}  // namespace views

template <class W, class Bound>
inline constexpr bool std::ranges::enable_borrowed_range<ranges::iota_view<W, Bound>> = true;