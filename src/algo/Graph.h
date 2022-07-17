// Copyright 2022 by Daniel Winkelman. All rights reserved.

#pragma once

#include <iostream>
#include <map>
#include <memory>
#include <set>
#include <variant>

namespace algo {
class Unit {};

template <typename D, typename E, typename V>
class Graph {
 protected:
  template <typename I, typename R>
  class Iterator {
   public:
    Iterator(const I& it) : it_(it), ref_(it) {}
    const R& operator*() const { return ref_; }
    R& operator*() { return ref_; }
    const R* operator->() const { return &ref_; }
    R* operator->() { return &ref_; }
    Iterator& operator++() {
      ++it_;
      ref_ = R(it_);
      return *this;
    }
    bool operator==(const Iterator& other) const { return it_ == other.it_; }

   private:
    I it_;
    R ref_;
  };

  template <typename I>
  class Range {
   public:
    Range(const I& begin, const I& end) : begin_(begin), end_(end) {}
    I begin() const { return begin_; }
    I end() const { return end_; }

   private:
    I begin_, end_;
  };

 protected:
  using VertexMap = std::map<uint32_t, V>;
  using VertexMapIterator = typename VertexMap::iterator;
  using VertexMapConstIterator = typename VertexMap::const_iterator;

 public:
  class ConstVertex {
   public:
    ConstVertex(const VertexMapConstIterator& it) : it_(it) {}
    inline uint32_t getIndex() const { return it_->first; }
    inline const V& getValue() const { return it_->second; }

   private:
    VertexMapConstIterator it_;
  };

  class Vertex {
   public:
    Vertex(const VertexMapIterator& it) : it_(it) {}
    inline uint32_t getIndex() const { return it_->first; }
    inline const V& getValue() const { return it_->second; }
    inline V& getValue() { return it_->second; }
    operator ConstVertex() const { return ConstVertex(it_); }

   private:
    VertexMapIterator it_;
  };

  using VertexIterator = Iterator<VertexMapIterator, Vertex>;
  using VertexConstIterator = Iterator<VertexMapConstIterator, ConstVertex>;

 protected:
  class EdgeMapComparator;
  using EdgeMap = std::map<std::pair<uint32_t, uint32_t>,
                           std::shared_ptr<std::pair<D, E>>, EdgeMapComparator>;
  using EdgeMapIterator = typename EdgeMap::iterator;
  using EdgeMapConstIterator = typename EdgeMap::const_iterator;

 public:
  class ConstEdge {
   public:
    ConstEdge(const EdgeMapConstIterator& it) : it_(it) {}
    inline uint32_t getSource() const { return it_->first.first; }
    inline uint32_t getDest() const { return it_->first.second; }
    inline const D& getWeight() const { return it_->second->first; }
    inline const E& getValue() const { return it_->second->second; }
    inline const std::shared_ptr<std::pair<D, E>> getData() const {
      return it_->second;
    }

   private:
    EdgeMapConstIterator it_;
  };

  class Edge {
   public:
    Edge(const EdgeMapIterator& it) : it_(it) {}
    inline uint32_t getSource() const { return it_->first.first; }
    inline uint32_t getDest() const { return it_->first.second; }
    inline const D& getWeight() const { return it_->second->first; }
    inline D& getWeight() { return it_->second->first; }
    inline const E& getValue() const { return it_->second->second; }
    inline E& getValue() { return it_->second->second; }
    inline const std::shared_ptr<std::pair<D, E>> getData() const {
      return it_->second;
    }
    inline std::shared_ptr<std::pair<D, E>> getData() { return it_->second; }
    operator ConstEdge() const { return ConstEdge(it_); }

   private:
    EdgeMapIterator it_;
  };

 protected:
  class EdgeMapComparator {
   public:
    EdgeMapComparator(const bool isOutbound) : isOutbound_(isOutbound) {}

    inline bool operator()(const std::pair<uint32_t, uint32_t>& a,
                           const std::pair<uint32_t, uint32_t>& b) const {
      if (isOutbound_) {
        return a.first < b.first || (a.first == b.first && a.second < b.second);
      } else {
        return b.first < a.first || (b.first == a.first && b.second < a.second);
      }
    }

   private:
    bool isOutbound_;
  };

 public:
  using EdgeIterator = Iterator<EdgeMapIterator, Edge>;
  using EdgeConstIterator = Iterator<EdgeMapConstIterator, ConstEdge>;

 public:
  Graph() : outboundEdges_(EdgeMapComparator(true)) {}

  inline std::pair<VertexIterator, bool> emplaceVertex(const uint32_t a) {
    return vertices_.emplace(a, V());
  }
  virtual std::pair<EdgeIterator, bool> emplaceEdge(const uint32_t a,
                                                    const uint32_t b) {
    return outboundEdges_.emplace(std::pair<uint32_t, uint32_t>(a, b),
                                  std::make_shared<std::pair<D, E>>(D(), E()));
  }

  Range<VertexIterator> getVertices() {
    return Range<VertexIterator>(vertices_.begin(), vertices_.end());
  }
  Range<VertexConstIterator> getVertices() const {
    return Range<VertexConstIterator>(vertices_.begin(), vertices_.end());
  }
  Range<EdgeIterator> getEdges() {
    return Range<EdgeIterator>(outboundEdges_.begin(), outboundEdges_.end());
  }
  Range<EdgeConstIterator> getEdges() const {
    return Range<EdgeConstIterator>(outboundEdges_.begin(),
                                    outboundEdges_.end());
  }

 private:
  VertexMap vertices_;
  EdgeMap outboundEdges_;
};

template <typename D, typename E, typename V>
class DirectedGraph : public Graph<D, E, V> {
 public:
  using EdgeMapComparator = typename Graph<D, E, V>::EdgeMapComparator;
  using EdgeIterator = typename Graph<D, E, V>::EdgeIterator;

 public:
  DirectedGraph() : Graph<D, E, V>(), inboundEdges_(EdgeMapComparator(false)) {}

  std::pair<EdgeIterator, bool> emplaceEdge(const uint32_t a,
                                            const uint32_t b) override {
    auto [it, success] = Graph<D, E, V>::emplaceEdge(a, b);
    if (success) {
      inboundEdges_.emplace(std::pair<uint32_t, uint32_t>(a, b), it->getData());
    }
    return std::pair<EdgeIterator, bool>(it, success);
  }

 private:
  typename Graph<D, E, V>::EdgeMap inboundEdges_;
};
}  // namespace algo
