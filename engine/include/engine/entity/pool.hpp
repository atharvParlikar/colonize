#pragma once

#include <cstdint>
#include <utility>
#include <vector>

namespace engine {

// A reference to one entity in a Pool<T>. Safe to keep around: once that
// entity is destroyed the handle goes stale and Pool::get returns null, even
// if the slot has since been reused by a new entity (the generation differs).
template <typename T>
struct Handle {
  std::uint32_t index = 0;
  std::uint32_t generation = 0; // 0 is never live: a default Handle is null

  explicit operator bool() const { return generation != 0; }
  bool operator==(const Handle &) const = default;
};

// Storage for one kind of entity: plain structs in a flat array, reused
// through a free list, so creating and destroying never moves the others.
//
//   Pool<Drone> drones;
//   Handle<Drone> h = drones.create(Drone{...});
//   if (Drone *d = drones.get(h)) ...           // null once destroyed
//   for (auto [handle, drone] : drones) ...      // every live entity
//   drones.destroy(h);
//
// Destroying during iteration is fine (also the current one; don't touch it
// afterwards). Creating during iteration is fine too, but it may resize the
// array: don't keep a T& across create(); go back through the handle. An
// entity created mid-loop may or may not be visited by that same loop.
template <typename T>
class Pool {
  struct Slot {
    T value{};
    std::uint32_t generation = 1; // odd: free, even: live
  };

public:
  template <typename... Args>
  Handle<T> create(Args &&...args) {
    std::uint32_t index;
    if (!free_.empty()) {
      index = free_.back();
      free_.pop_back();
    } else {
      index = static_cast<std::uint32_t>(slots_.size());
      slots_.emplace_back();
    }
    Slot &slot = slots_[index];
    slot.value = T{std::forward<Args>(args)...};
    ++slot.generation; // now even: live
    ++live_;
    return {index, slot.generation};
  }

  // Returns false if the handle was already stale.
  bool destroy(Handle<T> handle) {
    if (!get(handle)) return false;
    Slot &slot = slots_[handle.index];
    ++slot.generation; // odd again: free; every handle to it is now stale
    slot.value = T{};
    free_.push_back(handle.index);
    --live_;
    return true;
  }

  T *get(Handle<T> handle) {
    if (handle.index >= slots_.size()) return nullptr;
    Slot &slot = slots_[handle.index];
    return slot.generation == handle.generation && isLive(slot) ? &slot.value : nullptr;
  }
  const T *get(Handle<T> handle) const { return const_cast<Pool *>(this)->get(handle); }

  bool contains(Handle<T> handle) const { return get(handle) != nullptr; }
  std::size_t size() const { return live_; }
  bool empty() const { return live_ == 0; }

  void clear() {
    for (std::uint32_t i = 0; i < slots_.size(); ++i) {
      if (isLive(slots_[i])) destroy({i, slots_[i].generation});
    }
  }

  // ---- iteration over live entities: yields {handle, T&} -------------------
  template <typename PoolT, typename Ref>
  class Iterator {
  public:
    Iterator(PoolT *pool, std::uint32_t index) : pool_(pool), index_(index) { skipFree(); }
    std::pair<Handle<T>, Ref> operator*() const {
      auto &slot = pool_->slots_[index_];
      return {Handle<T>{index_, slot.generation}, slot.value};
    }
    Iterator &operator++() {
      ++index_;
      skipFree();
      return *this;
    }
    bool operator!=(const Iterator &other) const { return index_ != other.index_; }

  private:
    void skipFree() {
      while (index_ < pool_->slots_.size() && !isLive(pool_->slots_[index_])) ++index_;
    }
    PoolT *pool_;
    std::uint32_t index_;
  };

  auto begin() { return Iterator<Pool, T &>(this, 0); }
  auto end() { return Iterator<Pool, T &>(this, static_cast<std::uint32_t>(slots_.size())); }
  auto begin() const { return Iterator<const Pool, const T &>(this, 0); }
  auto end() const { return Iterator<const Pool, const T &>(this, static_cast<std::uint32_t>(slots_.size())); }

private:
  static bool isLive(const Slot &slot) { return slot.generation % 2 == 0; }

  std::vector<Slot> slots_;
  std::vector<std::uint32_t> free_;
  std::size_t live_ = 0;
};

} // namespace engine
