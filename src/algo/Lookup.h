// Copyright 2022 by Daniel Winkelman. All rights reserved.

#pragma once

#include <map>
#include <vector>

namespace algo {
template <typename T>
class Lookup {
 public:
  uint32_t getIndex(const T &object) {
    auto [it, success] = forward_.emplace(object, backward_.size());
    if (success) {
      backward_.push_back(object);
    }
    return it->second;
  }

  uint32_t operator()(const T &object) { return getIndex(object); }

  T &getValue(const uint32_t index) {
    if (index < backward_.size()) {
      return backward_[index];
    } else {
      throw std::out_of_range("Invalid reverse mapping");
    }
  }

  const T &getValue(const uint32_t index) const {
    if (index < backward_.size()) {
      return backward_[index];
    } else {
      throw std::out_of_range("Invalid reverse mapping");
    }
  }

  T &operator()(const uint32_t index) { return getValue(index); }
  const T &operator()(const uint32_t index) const { return getValue(index); }

 private:
  std::map<T, uint32_t> forward_;
  std::vector<T> backward_;
};

template <>
class Lookup<uint32_t> {
 public:
  uint32_t getIndex(const uint32_t &object) {
    auto [it, success] = forward_.emplace(object, backward_.size());
    if (success) {
      backward_.push_back(object);
    }
    return it->second;
  }

  uint32_t operator()(const uint32_t &object) { return getIndex(object); }

  uint32_t &getValue(const uint32_t index) {
    if (index < backward_.size()) {
      return backward_[index];
    } else {
      throw std::out_of_range("Invalid reverse mapping");
    }
  }

  const uint32_t &getValue(const uint32_t index) const {
    if (index < backward_.size()) {
      return backward_[index];
    } else {
      throw std::out_of_range("Invalid reverse mapping");
    }
  }

 private:
  std::map<uint32_t, uint32_t> forward_;
  std::vector<uint32_t> backward_;
};
}  // namespace algo
