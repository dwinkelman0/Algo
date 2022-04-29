// Copyright 2022 by Daniel Winkelman. All rights reserved.

#pragma once

#include <map>
#include <set>

namespace algo {
class DirectedGraph {
 public:
  void connect(const uint32_t a, const uint32_t b);
  void unconnect(const uint32_t a, const uint32_t b);
  void eraseNode(const uint32_t a);

  std::set<uint32_t> getNodes() const;

  void breadthFirstTraverse(
      const std::function<void(const DirectedGraph &, const uint32_t)> &func,
      const std::vector<uint32_t> &origins) const;

  DirectedGraph pruneSourcesAndSinks() const;

 protected:
  bool isSource(const uint32_t a) const;
  bool isSink(const uint32_t a) const;

 private:
  std::map<uint32_t, std::set<uint32_t>> outgoingEdges_;
  std::map<uint32_t, std::set<uint32_t>> incomingEdges_;
};
}  // namespace algo
