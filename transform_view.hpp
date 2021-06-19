#pragma once

#include <functional>

#include "ref_view.hpp"
#include "semiregular_box.hpp"

namespace ranges {

template <std::ranges::input_range V, std::copy_constructible F>
requires std::ranges::view<V>&& std::is_object_v<F>&&
  std::regular_invocable<F&, std::ranges::range_reference_t<V>>&&
    can_reference<std::invoke_result_t<F&, std::ranges::range_reference_t<V>>> class transform_view
  : public view_interface<transform_view<V, F>> {
 private:
  template <class Base>
  struct iter_cat {};

  template <std::ranges::forward_range Base>
  struct iter_cat<Base> {
   private:
    static auto iter_category() {
      if constexpr (std::is_lvalue_reference_v<
                      std::invoke_result_t<F&, std::ranges::range_reference_t<Base>>>) {
        using C = std::iterator_traits<std::ranges::iterator_t<Base>>::iterator_category;
        if constexpr (std::derived_from<C, std::contiguous_iterator_tag>)
          return std::random_access_iterator_tag{};
        else
          return C{};
      } else
        return std::input_iterator_tag{};
    }

   public:
    using iterator_category = decltype(iter_category());
  };

  template <bool Const>
  struct sentinel;

  template <bool Const>
  struct iterator : iter_cat<maybe_const<Const, V>> {
   private:
    using Parent = maybe_const<Const, transform_view>;
    using Base = maybe_const<Const, V>;
    std::ranges::iterator_t<Base> current_ = std::ranges::iterator_t<Base>();
    Parent* parent_ = nullptr;

    static auto iter_concept() {
      if constexpr (std::ranges::random_access_range<V>)
        return std::random_access_iterator_tag{};
      else if constexpr (std::ranges::bidirectional_range<V>)
        return std::bidirectional_iterator_tag{};
      else if constexpr (std::ranges::forward_range<V>)
        return std::forward_iterator_tag{};
      else
        return std::input_iterator_tag{};
    }

    friend iterator<!Const>;
    template <bool>
    friend struct sentinel;

   public:
    using iterator_concept = decltype(iter_concept());
    using value_type =
      std::remove_cvref_t<std::invoke_result_t<F&, std::ranges::range_reference_t<Base>>>;
    using difference_type = std::ranges::range_difference_t<Base>;

    iterator() = default;

    constexpr iterator(Parent& parent, std::ranges::iterator_t<Base> current)
      : current_(std::move(current)), parent_(std::addressof(parent)) {}

    constexpr iterator(iterator<!Const> i) requires Const&& std::convertible_to<
      std::ranges::iterator_t<V>, std::ranges::iterator_t<Base>> : current_(std::move(i.current_)),
                                                                   parent_(i.parent_) {}

    constexpr std::ranges::iterator_t<Base> base()
      const& requires std::copyable<std::ranges::iterator_t<Base>> {
      return current_;
    }
    constexpr std::ranges::iterator_t<Base> base() && { return std::move(current_); }
    constexpr decltype(auto) operator*() const { return std::invoke(*parent_->fun_, *current_); }

    constexpr iterator& operator++() {
      ++current_;
      return *this;
    }
    constexpr auto operator++(int) {
      if constexpr (std::ranges::forward_range<Base>) {
        auto tmp = *this;
        ++*this;
        return tmp;
      } else
        ++current_;
    }

    constexpr iterator& operator--() requires std::ranges::bidirectional_range<Base> {
      --current_;
      return *this;
    }
    constexpr iterator operator--(int) requires std::ranges::bidirectional_range<Base> {
      auto tmp = *this;
      --*this;
      return tmp;
    }

    constexpr iterator& operator+=(
      difference_type n) requires std::ranges::random_access_range<Base> {
      current_ += n;
      return *this;
    }
    constexpr iterator& operator-=(
      difference_type n) requires std::ranges::random_access_range<Base> {
      current_ -= n;
      return *this;
    }
    constexpr decltype(auto) operator[](difference_type n) const
      requires std::ranges::random_access_range<Base> {
      return std::invoke(*parent_->fun_, current_[n]);
    }

    friend constexpr bool operator==(
      const iterator& x,
      const iterator& y) requires std::equality_comparable<std::ranges::iterator_t<Base>> {
      return x.current_ == y.current_;
    }
    friend constexpr bool operator<(
      const iterator& x, const iterator& y) requires std::ranges::random_access_range<Base> {
      return x.current_ < y.current_;
    }
    friend constexpr bool operator>(
      const iterator& x, const iterator& y) requires std::ranges::random_access_range<Base> {
      return y < x;
    }
    friend constexpr bool operator<=(
      const iterator& x, const iterator& y) requires std::ranges::random_access_range<Base> {
      return !(y < x);
    }
    friend constexpr bool operator>=(
      const iterator& x, const iterator& y) requires std::ranges::random_access_range<Base> {
      return !(x < y);
    }
    friend constexpr auto operator<=>(const iterator& x, const iterator& y) requires std::ranges::
      random_access_range<Base>&& std::three_way_comparable<std::ranges::iterator_t<Base>> {
      return x.current_ <=> y.current_;
    }

    friend constexpr iterator operator+(
      iterator i, difference_type n) requires std::ranges::random_access_range<Base> {
      return iterator{*i.parent_, i.current_ + n};
    }
    friend constexpr iterator operator+(
      difference_type n, iterator i) requires std::ranges::random_access_range<Base> {
      return iterator{*i.parent_, i.current_ + n};
    }
    friend constexpr iterator operator-(
      iterator i, difference_type n) requires std::ranges::random_access_range<Base> {
      return iterator{*i.parent_, i.current_ - n};
    }
    friend constexpr difference_type operator-(const iterator& x, const iterator& y) requires std::
      sized_sentinel_for<std::ranges::iterator_t<Base>, std::ranges::iterator_t<Base>> {
      return x.current_ - y.current_;
    }

    friend constexpr decltype(auto) iter_move(const iterator& i) noexcept(noexcept(*i)) {
      if constexpr (std::is_lvalue_reference_v<decltype(*i)>)
        return std::move(*i);
      else
        return *i;
    }
  };

  template <bool Const>
  struct sentinel {
   private:
    using Parent = maybe_const<Const, transform_view>;
    using Base = maybe_const<Const, V>;
    std::ranges::sentinel_t<Base> end_ = std::ranges::sentinel_t<Base>();

    template <bool OtherConst>
    constexpr auto distance_from(const iterator<OtherConst>& i) const {
      return end_ - i.current_;
    }
    template <bool OtherConst>
    constexpr bool equal(const iterator<OtherConst>& i) const {
      return i.current_ == end_;
    }

   public:
    sentinel() = default;
    constexpr explicit sentinel(std::ranges::sentinel_t<Base> end) : end_(end) {}
    constexpr sentinel(sentinel<!Const> i) requires Const&& std::convertible_to<
      std::ranges::sentinel_t<V>, std::ranges::sentinel_t<Base>> : end_(std::move(i.end_)) {}

    constexpr std::ranges::sentinel_t<Base> base() const { return end_; }

    template <bool OtherConst>
    requires std::sentinel_for<
      std::ranges::sentinel_t<Base>,
      std::ranges::iterator_t<maybe_const<OtherConst, V>>> friend constexpr bool
    operator==(const iterator<OtherConst>& x, const sentinel& y) {
      return y.equal(x);
    }
    template <bool OtherConst>
    requires std::sized_sentinel_for<
      std::ranges::sentinel_t<Base>,
      std::ranges::iterator_t<maybe_const<OtherConst, V>>> friend constexpr std::ranges::
      range_difference_t<maybe_const<OtherConst, V>>
      operator-(const iterator<OtherConst>& x, const sentinel& y) {
      return -y.distance_from(x);
    }
    template <bool OtherConst>
    requires std::sized_sentinel_for<
      std::ranges::sentinel_t<Base>,
      std::ranges::iterator_t<maybe_const<OtherConst, V>>> friend constexpr std::ranges::
      range_difference_t<maybe_const<OtherConst, V>>
      operator-(const sentinel& y, const iterator<OtherConst>& x) {
      return y.distance_from(x);
    }
  };

  V base_ = V();
  semiregular_box<F> fun_;

 public:
  transform_view() = default;
  constexpr transform_view(V base, F fun) : base_(std::move(base)), fun_(std::move(fun)) {}

  constexpr V base() const& requires std::copy_constructible<V> { return base_; }
  constexpr V base() && { return std::move(base_); }

  constexpr iterator<false> begin() { return iterator<false>{*this, std::ranges::begin(base_)}; }
  constexpr iterator<true> begin() const requires std::ranges::range<
    const V>&& std::regular_invocable<const F&, std::ranges::range_reference_t<const V>> {
    return iterator<true>{*this, std::ranges::begin(base_)};
  }

  constexpr sentinel<false> end() { return sentinel<false>{std::ranges::end(base_)}; }
  constexpr iterator<false> end() requires std::ranges::common_range<V> {
    return iterator<false>{*this, std::ranges::end(base_)};
  }
  constexpr sentinel<true> end() const requires std::ranges::range<
    const V>&& std::regular_invocable<const F&, std::ranges::range_reference_t<const V>> {
    return sentinel<true>{std::ranges::end(base_)};
  }
  constexpr iterator<true> end() const requires std::ranges::common_range<
    const V>&& std::regular_invocable<const F&, std::ranges::range_reference_t<const V>> {
    return iterator<true>{*this, std::ranges::end(base_)};
  }

  constexpr auto size() requires std::ranges::sized_range<V> { return std::ranges::size(base_); }
  constexpr auto size() const requires std::ranges::sized_range<const V> {
    return std::ranges::size(base_);
  }
};

template <class R, class F>
transform_view(R&&, F) -> transform_view<views::all_t<R>, F>;

}  // namespace ranges

namespace views {

template <class R, class F>
concept can_transform_view = requires {
  ranges::transform_view(std::declval<R>(), std::declval<F>());
};

struct Transform : ranges::RangeAdaptor<Transform> {
  template <std::ranges::viewable_range R, class F>
  requires can_transform_view<R, F> constexpr auto operator()(R&& r, F&& f) const {
    return ranges::transform_view(std::forward<R>(r), std::forward<F>(f));
  }

  using RangeAdaptor<Transform>::operator();
  static constexpr int arity = 2;
};

inline constexpr Transform transform;

}  // namespace views