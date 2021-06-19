#include <algorithm>

#include "ref_view.hpp"
#include "semiregular_box.hpp"

namespace ranges {

template <
  std::ranges::input_range V, std::indirect_unary_predicate<std::ranges::iterator_t<V>> Pred>
requires std::ranges::view<V>&& std::is_object_v<Pred> class filter_view
  : public view_interface<filter_view<V, Pred>> {
 private:
  V base_ = V();
  semiregular_box<Pred> pred_;
  [[no_unique_address]] maybe_present_t<std::ranges::forward_range<V>, CachedIter<V>>
    cached_begin_{};

  class sentinel;

  template <class R>
  struct iter_cat {};

  template <std::ranges::forward_range R>
  struct iter_cat<R> {
   private:
    static auto iter_category() {
      using C = std::iterator_traits<std::ranges::iterator_t<R>>::iterator_category;
      if constexpr (std::derived_from<C, std::bidirectional_iterator_tag>)
        return std::bidirectional_iterator_tag{};
      else if constexpr (std::derived_from<C, std::forward_iterator_tag>)
        return std::forward_iterator_tag{};
      else
        return C{};
    }

   public:
    using iterator_category = decltype(iter_category());
  };

  class iterator : public iter_cat<V> {
   private:
    std::ranges::iterator_t<V> current_ = std::ranges::iterator_t<V>();
    filter_view* parent_ = nullptr;

    friend sentinel;

    static auto iter_concept() {
      if constexpr (std::ranges::bidirectional_range<V>)
        return std::bidirectional_iterator_tag{};
      else if constexpr (std::ranges::forward_range<V>)
        return std::forward_iterator_tag{};
      else
        return std::input_iterator_tag{};
    }

   public:
    using iterator_concept = decltype(iter_concept());
    using value_type = std::ranges::range_value_t<V>;
    using difference_type = std::ranges::range_difference_t<V>;

    iterator() = default;
    constexpr iterator(filter_view& parent, std::ranges::iterator_t<V> current)
      : parent_(std::addressof(parent)), current_(std::move(current)) {}

    constexpr std::ranges::iterator_t<V> base()
      const& requires std::copyable<std::ranges::iterator_t<V>> {
      return current_;
    }

    constexpr std::ranges::iterator_t<V> base() && { return std::move(current_); }

    constexpr std::ranges::range_reference_t<V> operator*() const { return *current_; }
    constexpr std::ranges::iterator_t<V> operator->() const
      requires has_arrow<std::ranges::iterator_t<V>>&& std::copyable<std::ranges::iterator_t<V>> {
      return current_;
    }

    constexpr iterator& operator++() {
      current_ = std::ranges::find_if(
        std::move(++current_), std::ranges::end(parent_->base_), std::ref(*parent_->pred_));
      return *this;
    }

    constexpr auto operator++(int) {
      if constexpr (std::ranges::forward_range<V>) {
        auto tmp = *this;
        ++*this;
        return tmp;
      } else
        ++*this;
    }

    constexpr iterator& operator--() requires std::ranges::bidirectional_range<V> {
      do --current_;
      while (!std::invoke(*parent_->pred_, *current_));
      return *this;
    }
    constexpr iterator operator--(int) requires std::ranges::bidirectional_range<V> {
      auto tmp = *this;
      --*this;
      return tmp;
    }

    friend constexpr bool operator==(
      const iterator& x,
      const iterator& y) requires std::equality_comparable<std::ranges::iterator_t<V>> {
      return x.current_ == y.current_;
    }

    friend constexpr std::ranges::range_rvalue_reference_t<V> iter_move(const iterator& i) noexcept(
      noexcept(std::ranges::iter_move(i.current_))) {
      return std::ranges::iter_move(i.current_);
    }
    friend constexpr void iter_swap(const iterator& x, const iterator& y) noexcept(
      noexcept(std::ranges::iter_swap(
        x.current_, y.current_))) requires std::indirectly_swappable<std::ranges::iterator_t<V>> {
      std::ranges::iter_swap(x.current_, y.current_);
    }
  };

  class sentinel {
   private:
    std::ranges::sentinel_t<V> end_ = std::ranges::sentinel_t<V>();

    constexpr bool equal(const iterator& x) const { return x.current_ == end_; }

   public:
    sentinel() = default;
    constexpr explicit sentinel(filter_view& parent) : end_(std::ranges::end(parent.base_)) {}
    constexpr std::ranges::sentinel_t<V> base() const { return end_; }

    friend constexpr bool operator==(const iterator& x, const sentinel& y) { return y.equal(x); }
  };

 public:
  filter_view() = default;
  constexpr filter_view(V base, Pred pred) : base_(std::move(base)), pred_(std::move(pred)) {}

  constexpr V base() const& requires std::copy_constructible<V> { return base_; }
  constexpr V base() && { return std::move(base_); }

  constexpr const Pred& pred() const { return *pred_; }

  constexpr iterator begin() {
    assert(pred_.has_value());
    if constexpr (std::ranges::forward_range<V>) {
      if (cached_begin_.has_value()) return {*this, cached_begin_.get(base_)};
    }

    auto it = std::ranges::find_if(base_, std::ref(*pred_));
    if constexpr (std::ranges::forward_range<V>) cached_begin_.set(base_, it);
    return {*this, std::move(it)};
  }

  constexpr auto end() {
    if constexpr (std::ranges::common_range<V>)
      return iterator{*this, std::ranges::end(base_)};
    else
      return sentinel{*this};
  }
};

template <class R, class Pred>
filter_view(R&&, Pred) -> filter_view<views::all_t<R>, Pred>;

}  // namespace ranges

namespace views {

template <class R, class Pred>
concept can_filter_view = requires {
  ranges::filter_view(std::declval<R>(), std::declval<Pred>());
};

struct Filter : ranges::RangeAdaptor<Filter> {
  template <std::ranges::viewable_range R, class Pred>
  requires can_filter_view<R, Pred> constexpr auto operator()(R&& r, Pred&& pred) const {
    return ranges::filter_view(std::forward<R>(r), std::forward<Pred>(pred));
  }

  using RangeAdaptor<Filter>::operator();
  static constexpr int arity = 2;
};

inline constexpr Filter filter;

}  // namespace views