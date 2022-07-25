// Copyright 2022 by Daniel Winkelman. All rights reserved.

#include <algo/Graph.h>
#include <gtest/gtest.h>

#include <iostream>
#include <string>

namespace algo {
TEST(Graph, Basic) {
  using GraphType = UndirectedGraph<float, std::string, std::vector<uint32_t>>;
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
}

TEST(Graph, Vertices) {
  using GraphType = UndirectedGraph<Unit, Unit, uint32_t>;
  GraphType graph;
  const auto &[v0, s0] = graph.emplaceVertex(0);
  const auto &[v1, s1] = graph.emplaceVertex(1);
  const auto &[v2, s2] = graph.emplaceVertex(0);
  ASSERT_EQ(v0->getIndex(), 0);
  ASSERT_TRUE(s0);
  ASSERT_EQ(v1->getIndex(), 1);
  ASSERT_TRUE(s1);
  ASSERT_EQ(v2, v0);
  ASSERT_FALSE(s2);
  for (const GraphType::Vertex vertex : graph.getVertices()) {
    ASSERT_EQ(vertex.getValue(), 0);
  }
  std::optional<GraphType::VertexIterator> v3 = graph.getVertex(0u);
  (*v3)->getValue() = 5;
  ASSERT_EQ((*v3)->getValue(), 5);
}

TEST(Graph, DirectedDFS) {
  using GraphType = DirectedGraph<Unit, Unit, Unit>;
  GraphType graph;
  graph.emplaceVertex(0);
  graph.emplaceVertex(1);
  graph.emplaceVertex(2);
  graph.emplaceVertex(3);
  graph.emplaceVertex(4);
  graph.emplaceEdge(0, 1);
  graph.emplaceEdge(1, 2);
  graph.emplaceEdge(2, 0);
  graph.emplaceEdge(0, 2);
  graph.emplaceEdge(2, 2);
  graph.emplaceEdge(0, 3);
  graph.emplaceEdge(0, 4);
  graph.emplaceEdge(4, 0);
  graph.emplaceEdge(3, 2);
  graph.emplaceEdge(4, 4);
  uint32_t vertexCount = 0;
  uint32_t edgeCount = 0;
  graph.traverseDepthFirst(
      [&vertexCount](const GraphType::ConstVertex &vertex) { ++vertexCount; },
      [&edgeCount](const GraphType::ConstEdge &edge) { ++edgeCount; }, 2u);
  ASSERT_EQ(vertexCount, 5);
  ASSERT_EQ(edgeCount, 10);
}

TEST(Graph, UndirectedDFS) {
  using GraphType = UndirectedGraph<Unit, Unit, Unit>;
  GraphType graph;
  graph.emplaceVertex(0);
  graph.emplaceVertex(1);
  graph.emplaceVertex(2);
  graph.emplaceVertex(3);
  graph.emplaceVertex(4);
  graph.emplaceEdge(0, 1);
  graph.emplaceEdge(1, 2);
  graph.emplaceEdge(2, 0);
  graph.emplaceEdge(2, 2);
  graph.emplaceEdge(0, 3);
  graph.emplaceEdge(0, 4);
  graph.emplaceEdge(3, 2);
  graph.emplaceEdge(4, 4);
  uint32_t vertexCount = 0;
  uint32_t edgeCount = 0;
  graph.traverseDepthFirst(
      [&vertexCount](const GraphType::ConstVertex &vertex) { ++vertexCount; },
      [&edgeCount](const GraphType::ConstEdge &edge) { ++edgeCount; }, 2u);
  ASSERT_EQ(vertexCount, 5);
  ASSERT_EQ(edgeCount, 8);
}

TEST(Graph, DirectedConnectedComponents) {
  using GraphType = DirectedGraph<Unit, Unit, Unit>;
  GraphType graph;
  graph.emplaceVertex(0);
  graph.emplaceVertex(1);
  graph.emplaceVertex(2);
  graph.emplaceVertex(3);
  graph.emplaceVertex(4);
  graph.emplaceEdge(0, 0);
  graph.emplaceEdge(0, 1);
  graph.emplaceEdge(1, 2);
  graph.emplaceEdge(3, 4);
  graph.emplaceEdge(4, 3);
  graph.emplaceEdge(4, 4);
  std::vector<GraphType> components = graph.getConnectedComponents();
  ASSERT_EQ(components.size(), 2);
  ASSERT_EQ(components[0].getVertices().getCount(), 3);
  ASSERT_EQ(components[0].getEdges().getCount(), 3);
  ASSERT_EQ(components[1].getVertices().getCount(), 2);
  ASSERT_EQ(components[1].getEdges().getCount(), 3);
}
}  // namespace algo
