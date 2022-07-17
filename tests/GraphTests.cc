// Copyright 2022 by Daniel Winkelman. All rights reserved.

#include <algo/Graph.h>
#include <gtest/gtest.h>

#include <iostream>
#include <string>

namespace algo {
TEST(Graph, Basic) {
  using GraphType = DirectedGraph<float, std::string, std::vector<uint32_t>>;
  GraphType graph;
  GraphType::VertexIterator v0 = graph.emplaceVertex(0).first;
  GraphType::VertexIterator v1 = graph.emplaceVertex(1).first;
  GraphType::VertexIterator v2 = graph.emplaceVertex(2).first;
  GraphType::EdgeIterator e0 = graph.emplaceEdge(0, 1).first;
  GraphType::EdgeIterator e1 = graph.emplaceEdge(1, 2).first;
  GraphType::EdgeIterator e2 = graph.emplaceEdge(1, 0).first;
  v0->getValue() = {4, 5, 6};
  v1->getValue() = {1, 2, 3, 7};
  e0->getWeight() = 0.5;
  e0->getValue() = "hello";
  e1->getValue() = "world";
  for (GraphType::Vertex vertex : graph.getVertices()) {
    std::cout << vertex.getIndex() << ": " << vertex.getValue().size()
              << std::endl;
  }
  for (GraphType::Edge edge : graph.getEdges()) {
    std::cout << edge.getSource() << " -> " << edge.getDest() << ": "
              << edge.getValue() << std::endl;
  }
}
}  // namespace algo
