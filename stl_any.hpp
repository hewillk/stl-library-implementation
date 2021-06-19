#pragma once

#include <iostream>
#include <typeinfo>
#include <utility>

namespace ranges {

template<class T>
constexpr static bool is_in_place_type_v = false;

template<class T>
constexpr static bool is_in_place_type_v<std::in_place_type_t<T>> = true;

class bad_any_cast : std::bad_cast {
 public:
  const char*
  what() const noexcept override {
    return "bad any_cast";
  }
};

inline void
throw_bad_any_cast() {
  throw bad_any_cast{};
}

class any {
  union Storage {
    constexpr Storage() : ptr_(nullptr) {}

    Storage(const Storage&) = delete;
    Storage(Storage&&)      = delete;

    void* ptr_;
    std::aligned_storage_t<sizeof(void*), alignof(void*)> buffer_;
  };

  enum class Op { access, get_type_info, clone, destory, xfer };

  union Arg {
    void* obj_;
    const std::type_info* typeinfo_;
    any* any_;
  };

  void (*manager_)(Op, const any*, Arg*);
  Storage storage_;

  template<class T>
  static constexpr bool use_internal = std::is_nothrow_move_constructible_v<T> &&
                                       (sizeof(T) <= sizeof(Storage) &&
                                        alignof(T) <= alignof(Storage));

  template<class T>
  struct manager_internal {
    static void
    manage(Op which, const any* anyp, Arg* arg) {
      auto ptr = reinterpret_cast<const T*>(&anyp->storage_.buffer_);
      switch (which) {
        case Op::access:
          arg->obj_ = const_cast<T*>(ptr);
          break;
        case Op::get_type_info:
          arg->typeinfo_ = &typeid(T);
          break;
        case Op::clone:
          ::new (&arg->any_->storage_.buffer_) T(*ptr);
          arg->any_->manager_ = anyp->manager_;
          break;
        case Op::destory:
          ptr->~T();
          break;
        case Op::xfer:
          ::new (&arg->any_->storage_.buffer_) T(std::move(*const_cast<T*>(ptr)));
          ptr->~T();
          arg->any_->manager_              = anyp->manager_;
          const_cast<any*>(anyp)->manager_ = nullptr;
          break;
      }
    }

    template<class... Args>
    static void
    create(Storage& storage, Args&&... args) {
      void* addr = &storage.buffer_;
      ::new (addr) T(std::forward<Args>(args)...);
    }

    static T*
    access(const Storage& storage) {
      const void* addr = &storage.buffer_;
      return static_cast<T*>(const_cast<void*>(addr));
    }
  };

  template<class T>
  struct manager_external {
    static void
    manage(Op which, const any* anyp, Arg* arg) {
      auto ptr = static_cast<const T*>(anyp->storage_.ptr_);
      switch (which) {
        case Op::access:
          arg->obj_ = const_cast<T*>(ptr);
          break;
        case Op::get_type_info:
          arg->typeinfo_ = &typeid(T);
          break;
        case Op::clone:
          arg->any_->storage_.ptr_ = new T(*ptr);
          arg->any_->manager_      = anyp->manager_;
          break;
        case Op::destory:
          delete ptr;
          break;
        case Op::xfer:
          arg->any_->storage_.ptr_         = anyp->storage_.ptr_;
          arg->any_->manager_              = anyp->manager_;
          const_cast<any*>(anyp)->manager_ = nullptr;
          break;
      }
    }

    template<class... Args>
    static void
    create(Storage& storage, Args&&... args) {
      storage.ptr_ = new T(std::forward<Args>(args)...);
    }

    static T*
    access(const Storage& storage) {
      return static_cast<T*>(storage.ptr_);
    }
  };

  template<class T>
  using manager = std::conditional_t<use_internal<T>, manager_internal<T>, manager_external<T>>;

  template<class T, class... Args, class Mgr = manager<T>>
  T&
  do_emplace(Args&&... args) {
    reset();
    Mgr::create(storage_, std::forward<Args>(args)...);
    manager_ = &Mgr::manage;
    return *Mgr::access(storage_);
  }

  template<class T, class U, class... Args, class Mgr = manager<T>>
  T&
  do_emplace(std::initializer_list<U> il, Args&&... args) {
    reset();
    Mgr::create(storage_, il, std::forward<Args>(args)...);
    manager_ = &Mgr::manage;
    return *Mgr::access(storage_);
  }

 public:
  constexpr any() noexcept : manager_(nullptr) {}

  any(const any& other) {
    if (!other.has_value())
      manager_ = nullptr;
    else {
      Arg arg;
      arg.any_ = this;
      other.manager_(Op::clone, &other, &arg);
    }
  }

  any(any&& other) {
    if (!other.has_value())
      manager_ = nullptr;
    else {
      Arg arg;
      arg.any_ = this;
      other.manager_(Op::xfer, &other, &arg);
    }
  }

  template<class T, class V = std::decay_t<T>, class Mgr = manager<V>>
    requires(!std::same_as<any, V> && !is_in_place_type_v<V> && std::is_copy_constructible_v<V>)
  any(T&& value) : manager_(&Mgr::manage) { Mgr::create(storage_, std::forward<T>(value)); }

  template<class T, class... Args, class V = std::decay_t<T>, class Mgr = manager<V>>
    requires std::is_copy_constructible_v<V> && std::is_constructible_v<V, Args...>
  explicit any(std::in_place_type_t<T>, Args&&... args) : manager_(&Mgr::manage) {
    Mgr::create(storage_, std::forward<Args>(args)...);
  }

  template<class T, class U, class... Args, class V = std::decay_t<T>, class Mgr = manager<V>>
    requires std::is_copy_constructible_v<V> && std::is_constructible_v < V,
      std::initializer_list<U>
  &, Args... > explicit any(std::in_place_type_t<T>, std::initializer_list<U> il, Args&&... args)
  : manager_(&Mgr::manage) {
    Mgr::create(storage_, il, std::forward<Args>(args)...);
  }

  ~any() { reset(); }

  any&
  operator=(const any& rhs) {
    *this = any(rhs);
    return *this;
  }

  any&
  operator=(any&& rhs) {
    if (!rhs.has_value())
      reset();
    else if (this != &rhs) {
      reset();
      Arg arg;
      arg.any_ = this;
      rhs.manager_(Op::xfer, &rhs, &arg);
    }
    return *this;
  }

  template<class T, class V = std::decay_t<T>>
    requires(!std::same_as<any, V> && std::is_copy_constructible_v<V>)
  any&
  operator=(T&& rhs) {
    *this = any(std::forward<T>(rhs));
    return *this;
  }

  template<class T, class... Args, class V = std::decay_t<T>>
  requires std::is_copy_constructible_v<V> && std::is_constructible_v<V, Args...> V&
  emplace(Args&&... args) {
    return do_emplace<V>(std::forward<Args>(args)...);
  }

  template<class T, class U, class... Args, class V = std::decay_t<T>>
    requires std::is_copy_constructible_v<V> && std::is_constructible_v < V,
      std::initializer_list<U>
  &, Args... > V&
               emplace(std::initializer_list<U> il, Args&&... args) {
    return do_emplace<V, U>(il, std::forward<Args>(args)...);
  }

  bool
  has_value() const noexcept {
    return manager_ != nullptr;
  }

  const std::type_info&
  type() const noexcept {
    if (!has_value()) return typeid(void);
    Arg arg;
    manager_(Op::get_type_info, this, &arg);
    return *arg.typeinfo_;
  }

  void
  reset() noexcept {
    if (has_value()) {
      manager_(Op::destory, this, nullptr);
      manager_ = nullptr;
    }
  }

  void
  swap(any& rhs) noexcept {
    if (!has_value() && !rhs.has_value()) return;

    if (has_value() && rhs.has_value()) {
      if (this == &rhs) return;

      any tmp;
      Arg arg;
      arg.any_ = &tmp;
      rhs.manager_(Op::xfer, &rhs, &arg);
      arg.any_ = &rhs;
      manager_(Op::xfer, this, &arg);
      arg.any_ = this;
      tmp.manager_(Op::xfer, &tmp, &arg);
    } else {
      any* empty = !has_value() ? this : &rhs;
      any* full  = !has_value() ? &rhs : this;
      Arg arg;
      arg.any_ = empty;
      full->manager_(Op::xfer, full, &arg);
    }
  }

  template<class T>
  friend void*
  any_caster(const any*);
};

inline void
swap(any& x, any& y) noexcept {
  x.swap(y);
}

template<class T, class... Args>
any
make_any(Args&&... args) {
  return any(std::in_place_type<T>, std::forward<Args>(args)...);
}

template<class T, class U, class... Args>
any
make_any(std::initializer_list<U> il, Args&&... args) {
  return any(std::in_place_type<T>, il, std::forward<Args>(args)...);
}

template<class T>
concept valid_any_cast = std::is_reference_v<T> || std::is_copy_constructible_v<T>;

template<class T, class U = std::remove_cvref_t<T>>
  requires valid_any_cast<T> && std::is_constructible_v<T, const U&>
inline T
any_cast(const any& a) {
  auto p = any_cast<U>(&a);
  if (!p) throw_bad_any_cast();
  return static_cast<T>(*p);
}

template<class T, class U = std::remove_cvref_t<T>>
  requires valid_any_cast<T> && std::is_constructible_v<T, U&>
inline T
any_cast(any& a) {
  auto p = any_cast<U>(&a);
  if (!p) throw_bad_any_cast();
  return static_cast<T>(*p);
}

template<class T, class U = std::remove_cvref_t<T>>
  requires valid_any_cast<T> && std::is_constructible_v<T, U>
inline T
any_cast(any&& a) {
  auto p = any_cast<U>(&a);
  if (!p) throw_bad_any_cast();
  return static_cast<T>(std::move(*p));
}

template<class T>
void*
any_caster(const any* a) {
  using U = std::remove_cv_t<T>;
  if constexpr (!std::same_as<std::decay_t<U>, U>)
    return nullptr;
  else if constexpr (!std::is_copy_constructible_v<U>)
    return nullptr;
  else if (a->manager_ == &any::manager<U>::manage || a->type() == typeid(T))
    return any::manager<U>::access(a->storage_);
  return nullptr;
}

template<class T>
inline const T*
any_cast(const any* a) noexcept {
  if constexpr (std::is_object_v<T>)
    if (a) return static_cast<T*>(any_caster<T>(a));
  return nullptr;
}

template<class T>
inline T*
any_cast(any* a) noexcept {
  if constexpr (std::is_object_v<T>)
    if (a) return static_cast<T*>(any_caster<T>(a));
  return nullptr;
}

}  // namespace ranges