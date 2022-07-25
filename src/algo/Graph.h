// Copyright 2022 by Daniel Winkelman. All rights reserved.

#pragma once

#include <functional>
#include <iostream>
#include <map>
#include <memory>
#include <set>
#include <stack>
#include <variant>

namespace algo {
class Unit {};

template <typename D, typename E, typename V, bool IsDirected>
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
    uint32_t getCount() const {
      uint32_t count = 0;
      for (auto it = begin(); it != end(); ++it) {
        ++count;
      }
      return count;
    }

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

  using VertexVariant =
      std::variant<uint32_t, Vertex, VertexIterator, VertexConstIterator>;
  class VertexVariantVisitor {
   public:
    uint32_t operator()(const uint32_t index) { return index; }
    uint32_t operator()(const Vertex& vertex) { return vertex.getIndex(); }
    uint32_t operator()(const VertexIterator& vertexIt) {
      return vertexIt->getIndex();
    }
    uint32_t operator()(const VertexConstIterator& vertexIt) {
      return vertexIt->getIndex();
    }
  };

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
        return a.second < b.second ||
               (a.second == b.second && a.first < b.first);
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
      : outboundEdges_(EdgeMapComparator(true)),
        inboundEdges_(EdgeMapComparator(false)) {}

  inline std::pair<VertexIterator, bool> emplaceVertex(const uint32_t a) {
    return vertices_.emplace(a, V());
  }

  std::pair<EdgeIterator, bool> emplaceEdge(const VertexVariant& a,
                                            const VertexVariant& b) {
    uint32_t aIndex = std::visit(VertexVariantVisitor(), a);
    uint32_t bIndex = std::visit(VertexVariantVisitor(), b);
    auto it = this->outboundEdges_.find(
        std::pair<uint32_t, uint32_t>(aIndex, bIndex));
    if (it != this->outboundEdges_.end()) {
      return std::pair<EdgeIterator, bool>(it, false);
    } else {
      std::pair<uint32_t, uint32_t> forward(aIndex, bIndex);
      auto mem = std::make_shared<std::pair<D, E>>(D(), E());
      it = this->outboundEdges_.emplace(forward, mem).first;
      this->inboundEdges_.emplace(forward, mem);
      if (!IsDirected) {
        std::pair<uint32_t, uint32_t> reverse(bIndex, aIndex);
        this->inboundEdges_.emplace(reverse, mem);
        this->outboundEdges_.emplace(reverse, mem);
      }
      return std::pair<EdgeIterator, bool>(it, true);
    }
  }

  Range<VertexIterator> getVertices() {
    return Range<VertexIterator>(vertices_.begin(), vertices_.end());
  }

  Range<VertexConstIterator> getVertices() const {
    return Range<VertexConstIterator>(vertices_.begin(), vertices_.end());
  }

  std::optional<VertexIterator> getVertex(const VertexVariant& vertex) {
    auto it = vertices_.find(std::visit(VertexVariantVisitor(), vertex));
    return it == vertices_.end() ? std::nullopt
                                 : std::optional<VertexIterator>(it);
  }

  std::optional<VertexConstIterator> getVertex(
      const VertexVariant& vertex) const {
    auto it = vertices_.find(std::visit(VertexVariantVisitor(), vertex));
    return it == vertices_.end() ? std::nullopt
                                 : std::optional<VertexConstIterator>(it);
  }

  Range<EdgeIterator> getEdges() {
    std::function skipper = [this](const EdgeIterator& it) {
      return it != this->outboundEdges_.end() &&
             it->getSource() > it->getDest();
    };
    return Range<EdgeIterator>(
        IsDirected ? EdgeIterator(this->outboundEdges_.begin())
                   : EdgeIterator(this->outboundEdges_.begin(), skipper),
        EdgeIterator(this->outboundEdges_.end()));
  }

  Range<EdgeConstIterator> getEdges() const {
    std::function skipper = [this](const EdgeConstIterator& it) {
      return it != this->outboundEdges_.end() &&
             it->getSource() > it->getDest();
    };
    return Range<EdgeConstIterator>(
        IsDirected ? EdgeConstIterator(this->outboundEdges_.begin())
                   : EdgeConstIterator(this->outboundEdges_.begin(), skipper),
        EdgeConstIterator(this->outboundEdges_.end()));
  }

  std::optional<EdgeIterator> getEdge(const VertexVariant& a,
                                      const VertexVariant& b) {
    auto it = outboundEdges_.find(
        std::pair<uint32_t, uint32_t>(std::visit(VertexVariantVisitor(), a),
                                      std::visit(VertexVariantVisitor(), b)));
    return it == outboundEdges_.end() ? std::nullopt
                                      : std::optional<EdgeIterator>(it);
  }

  std::optional<EdgeConstIterator> getEdge(const VertexVariant& a,
                                           const VertexVariant& b) const {
    auto it = outboundEdges_.find(
        std::pair<uint32_t, uint32_t>(std::visit(VertexVariantVisitor(), a),
                                      std::visit(VertexVariantVisitor(), b)));
    return it == outboundEdges_.end() ? std::nullopt
                                      : std::optional<EdgeConstIterator>(it);
  }

  Range<EdgeIterator> getEdgesFromVertex(const VertexVariant& vertex) {
    uint32_t index = std::visit(VertexVariantVisitor(), vertex);
    return Range<EdgeIterator>(
        outboundEdges_.lower_bound(std::pair<uint32_t, uint32_t>(index, 0)),
        outboundEdges_.upper_bound(std::pair<uint32_t, uint32_t>(
            index, std::numeric_limits<uint32_t>().max())));
  }

  Range<EdgeConstIterator> getEdgesFromVertex(
      const VertexVariant& vertex) const {
    uint32_t index = std::visit(VertexVariantVisitor(), vertex);
    return Range<EdgeConstIterator>(
        outboundEdges_.lower_bound(std::pair<uint32_t, uint32_t>(index, 0)),
        outboundEdges_.upper_bound(std::pair<uint32_t, uint32_t>(
            index, std::numeric_limits<uint32_t>().max())));
  }

  Range<EdgeIterator> getEdgesToVertex(const VertexVariant& vertex) {
    uint32_t index = std::visit(VertexVariantVisitor(), vertex);
    return Range<EdgeIterator>(
        inboundEdges_.lower_bound(std::pair<uint32_t, uint32_t>(0, index)),
        inboundEdges_.upper_bound(std::pair<uint32_t, uint32_t>(
            std::numeric_limits<uint32_t>().max(), index)));
  }

  Range<EdgeConstIterator> getEdgesToVertex(const VertexVariant& vertex) const {
    uint32_t index = std::visit(VertexVariantVisitor(), vertex);
    return Range<EdgeConstIterator>(
        inboundEdges_.lower_bound(std::pair<uint32_t, uint32_t>(0, index)),
        inboundEdges_.upper_bound(std::pair<uint32_t, uint32_t>(
            std::numeric_limits<uint32_t>().max(), index)));
  }

 protected:
  using VertexFunction = std::function<void(const ConstVertex&)>;
  using EdgeFunction = std::function<void(const ConstEdge&)>;

  template <bool Reverse>
  void traverseDepthFirst(const VertexFunction& vertexFunc,
                          const EdgeFunction& edgeFunc, const uint32_t index,
                          std::set<uint32_t>& visited) const {
    if (visited.find(index) == visited.end()) {
      visited.insert(index);
      std::optional<VertexConstIterator> vertexIt = getVertex(index);
      if (!vertexIt) {
        throw std::exception();
      }
      vertexFunc(**vertexIt);
      for (const ConstEdge& edge : getEdgesFromVertex(index)) {
        uint32_t dest = edge.getDest();
        if (IsDirected || dest >= index) {
          edgeFunc(edge);
        }
        traverseDepthFirst<Reverse>(vertexFunc, edgeFunc, dest, visited);
      }
      if (Reverse) {
        for (const ConstEdge& edge : getEdgesToVertex(index)) {
          uint32_t source = edge.getSource();
          if (IsDirected || source <= index) {
            edgeFunc(edge);
          }
          traverseDepthFirst<Reverse>(vertexFunc, edgeFunc, source, visited);
        }
      }
    }
  }

 public:
  std::set<uint32_t> traverseDepthFirst(const VertexFunction& vertexFunc,
                                        const EdgeFunction& edgeFunc,
                                        const VertexVariant firstVertex) const {
    std::set<uint32_t> vertices;
    traverseDepthFirst<false>(vertexFunc, edgeFunc,
                              std::visit(VertexVariantVisitor(), firstVertex),
                              vertices);
    return vertices;
  }

  std::vector<Graph> getConnectedComponents() const {
    std::vector<Graph> output;
    std::set<uint32_t> vertices;
    while (vertices.size() < vertices_.size()) {
      for (const ConstVertex& vertex : getVertices()) {
        if (vertices.find(vertex.getIndex()) == vertices.end()) {
          Graph subgraph;
          traverseDepthFirst<true>(
              [&subgraph](const ConstVertex& vertex) {
                subgraph.emplaceVertex(vertex.getIndex());
              },
              [&subgraph](const ConstEdge& edge) {
                subgraph.emplaceVertex(edge.getDest());
                subgraph.emplaceEdge(edge.getSource(), edge.getDest());
              },
              vertex.getIndex(), vertices);
          output.push_back(subgraph);
        }
      }
    }
    return output;
  }

 protected:
  VertexMap vertices_;
  EdgeMap outboundEdges_, inboundEdges_;
};

template <typename D, typename E, typename V>
using UndirectedGraph = Graph<D, E, V, false>;

template <typename D, typename E, typename V>
using DirectedGraph = Graph<D, E, V, true>;
}  // namespace algo
