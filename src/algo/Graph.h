// Copyright 2022 by Daniel Winkelman. All rights reserved.

#pragma once

#include <functional>
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
   protected:
    using SkipFunction = std::function<bool(const I&)>;

   public:
    Iterator(const I& it) : it_(it), ref_(it) {}
    Iterator(const I& it, const SkipFunction& skipFunction)
        : it_(it), ref_(it), skipFunction_(skipFunction) {}
    const R& operator*() const { return ref_; }
    R& operator*() { return ref_; }
    const R* operator->() const { return &ref_; }
    R* operator->() { return &ref_; }
    Iterator& operator++() {
      do {
        ++it_;
        ref_ = R(it_);
      } while (skipFunction_ && (*skipFunction_)(it_));
      return *this;
    }
    bool operator==(const Iterator& other) const { return it_ == other.it_; }

   private:
    I it_;
    R ref_;
    std::optional<SkipFunction> skipFunction_;
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
  Graph()
      : outboundEdges_(typename Graph<D, E, V>::EdgeMapComparator(true)),
        inboundEdges_(typename Graph<D, E, V>::EdgeMapComparator(false)) {}

  inline std::pair<VertexIterator, bool> emplaceVertex(const uint32_t a) {
    return vertices_.emplace(a, V());
  }

  virtual std::pair<EdgeIterator, bool> emplaceEdge(const uint32_t a,
                                                    const uint32_t b) = 0;

  Range<VertexIterator> getVertices() {
    return Range<VertexIterator>(vertices_.begin(), vertices_.end());
  }

  Range<VertexConstIterator> getVertices() const {
    return Range<VertexConstIterator>(vertices_.begin(), vertices_.end());
  }

  virtual Range<EdgeIterator> getEdges() = 0;
  virtual Range<EdgeConstIterator> getEdges() const = 0;

  Range<EdgeIterator> getEdgesFromVertex(const uint32_t a) {
    return Range<EdgeIterator>(
        outboundEdges_.lower_bound(std::pair<uint32_t, uint32_t>(a, 0)),
        outboundEdges_.upper_bound(std::pair<uint32_t, uint32_t>(a + 1, 0)));
  }

  Range<EdgeConstIterator> getEdgesFromVertex(const uint32_t a) const;
  Range<EdgeIterator> getEdgesToVertex(const uint32_t a);
  Range<EdgeConstIterator> getEdgesToVertex(const uint32_t a) const;

 protected:
  VertexMap vertices_;
  EdgeMap outboundEdges_, inboundEdges_;
};

template <typename D, typename E, typename V>
class UndirectedGraph : public Graph<D, E, V> {
 public:
  template <typename I>
  using Range = typename Graph<D, E, V>::template Range<I>;
  using EdgeIterator = typename Graph<D, E, V>::EdgeIterator;
  using EdgeConstIterator = typename Graph<D, E, V>::EdgeConstIterator;

 public:
  UndirectedGraph() : Graph<D, E, V>() {}

  std::pair<EdgeIterator, bool> emplaceEdge(const uint32_t a,
                                            const uint32_t b) override {
    auto it = this->outboundEdges_.find(std::pair<uint32_t, uint32_t>(a, b));
    if (it != this->outboundEdges_.end()) {
      return std::pair<EdgeIterator, bool>(it, false);
    } else {
      std::pair<uint32_t, uint32_t> forward(a, b);
      std::pair<uint32_t, uint32_t> reverse(b, a);
      auto mem = std::make_shared<std::pair<D, E>>(D(), E());
      it = this->outboundEdges_.emplace(forward, mem).first;
      this->outboundEdges_.emplace(reverse, mem);
      this->inboundEdges_.emplace(forward, mem);
      this->inboundEdges_.emplace(reverse, mem);
      return std::pair<EdgeIterator, bool>(it, true);
    }
  }

  Range<EdgeIterator> getEdges() override {
    std::function skipper =
        [this](const typename Graph<D, E, V>::EdgeIterator& it) {
          return it != this->outboundEdges_.end() &&
                 it->getSource() > it->getDest();
        };
    return Range<EdgeIterator>(
        EdgeIterator(this->outboundEdges_.begin(), skipper),
        EdgeIterator(this->outboundEdges_.end()));
  }

  Range<EdgeConstIterator> getEdges() const override {
    std::function skipper =
        [this](const typename Graph<D, E, V>::EdgeConstIterator& it) {
          return it != this->outboundEdges_.end() &&
                 it->getSource() > it->getDest();
        };
    return Range<EdgeConstIterator>(
        EdgeConstIterator(this->outboundEdges_.begin(), skipper),
        EdgeConstIterator(this->outboundEdges_.end()));
  }
};

template <typename D, typename E, typename V>
class DirectedGraph : public Graph<D, E, V> {
 public:
  template <typename I>
  using Range = typename Graph<D, E, V>::template Range<I>;
  using EdgeIterator = typename Graph<D, E, V>::EdgeIterator;
  using EdgeConstIterator = typename Graph<D, E, V>::EdgeConstIterator;

 public:
  DirectedGraph() : Graph<D, E, V>() {}

  std::pair<EdgeIterator, bool> emplaceEdge(const uint32_t a,
                                            const uint32_t b) override {
    auto it = this->outboundEdges_.find(std::pair<uint32_t, uint32_t>(a, b));
    if (it != this->outboundEdges_.end()) {
      return std::pair<EdgeIterator, bool>(it, false);
    } else {
      std::pair<uint32_t, uint32_t> forward(a, b);
      auto mem = std::make_shared<std::pair<D, E>>(D(), E());
      it = this->outboundEdges_.emplace(forward, mem).first;
      this->inboundEdges_.emplace(forward, mem);
      return std::pair<EdgeIterator, bool>(it, true);
    }
  }

  Range<EdgeIterator> getEdges() override {
    return Range<EdgeIterator>(EdgeIterator(this->outboundEdges_.begin()),
                               EdgeIterator(this->outboundEdges_.end()));
  }

  Range<EdgeConstIterator> getEdges() const override {
    return Range<EdgeConstIterator>(
        EdgeConstIterator(this->outboundEdges_.begin()),
        EdgeConstIterator(this->outboundEdges_.end()));
  }
};
}  // namespace algo
