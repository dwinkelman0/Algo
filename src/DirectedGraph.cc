// Copyright 2022 by Daniel Winkelman. All rights reserved.

#include "DirectedGraph.h"

#include <queue>

namespace algo {
void DirectedGraph::connect(const uint32_t a, const uint32_t b) {
  std::set<uint32_t> &outgoingNodes =
      outgoingEdges_.emplace(a, std::set<uint32_t>()).first->second;
  outgoingNodes.insert(b);
  std::set<uint32_t> &incomingNodes =
      incomingEdges_.emplace(b, std::set<uint32_t>()).first->second;
  incomingNodes.insert(a);
}

void DirectedGraph::unconnect(const uint32_t a, const uint32_t b) {
  auto outgoingNodesIt = outgoingEdges_.find(a);
  if (outgoingNodesIt != outgoingEdges_.end()) {
    outgoingNodesIt->second.erase(b);
    if (outgoingNodesIt->second.empty()) {
      outgoingEdges_.erase(outgoingNodesIt);
    }
  }
  auto incomingNodesIt = incomingEdges_.find(a);
  if (incomingNodesIt != incomingEdges_.end()) {
    incomingNodesIt->second.erase(b);
    if (incomingNodesIt->second.empty()) {
      incomingEdges_.erase(incomingNodesIt);
    }
  }
}

void DirectedGraph::eraseNode(const uint32_t a) {
  auto outgoingNodesIt = outgoingEdges_.find(a);
  if (outgoingNodesIt != outgoingEdges_.end()) {
    for (const uint32_t b : outgoingNodesIt->second) {
      unconnect(a, b);
    }
  }
  auto incomingNodesIt = incomingEdges_.find(a);
  if (incomingNodesIt != incomingEdges_.end()) {
    for (const uint32_t b : incomingNodesIt->second) {
      unconnect(b, a);
    }
  }
}

std::set<uint32_t> DirectedGraph::getNodes() const {
  std::set<uint32_t> indices;
  for (const auto &[index, _] : outgoingEdges_) {
    indices.insert(index);
  }
  for (const auto &[index, _] : incomingEdges_) {
    indices.insert(index);
  }
  return indices;
}

DirectedGraph DirectedGraph::pruneSourcesAndSinks() const {
  DirectedGraph output(*this);
  bool progress = true;
  while (progress) {
    progress = false;
    for (const auto &[index, nodes] : output.outgoingEdges_) {
      if (output.isSource(index)) {
        output.eraseNode(index);
        progress = true;
        break;
      }
    }
    for (const auto &[index, nodes] : output.incomingEdges_) {
      if (output.isSink(index)) {
        output.eraseNode(index);
        progress = true;
        break;
      }
    }
  }
  assert(output.outgoingEdges_.size() == output.incomingEdges_.size());
  return output;
}

bool DirectedGraph::isSource(const uint32_t a) const {
  auto outgoingNodesIt = outgoingEdges_.find(a);
  auto incomingNodesIt = incomingEdges_.find(a);
  return (incomingNodesIt == incomingEdges_.end() ||
          incomingNodesIt->second.empty()) &&
         (outgoingNodesIt != outgoingEdges_.end() &&
          !outgoingNodesIt->second.empty());
}

bool DirectedGraph::isSink(const uint32_t a) const {
  auto outgoingNodesIt = outgoingEdges_.find(a);
  auto incomingNodesIt = incomingEdges_.find(a);
  return (outgoingNodesIt == outgoingEdges_.end() ||
          outgoingNodesIt->second.empty()) &&
         (incomingNodesIt != incomingEdges_.end() &&
          !incomingNodesIt->second.empty());
}
}  // namespace algo