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
  GraphType::EdgeIterator e0 = graph.emplaceEdge(v0, v1).first;
  GraphType::EdgeIterator e1 = graph.emplaceEdge(v1, v2).first;
  GraphType::EdgeIterator e2 = graph.emplaceEdge(v1, v0).first;
  v0->getValue() = {4, 5, 6};
  v1->getValue() = {1, 2, 3, 7};
  e0->getWeight() = 0.5;
  e0->getValue() = "hello";
  e1->getValue() = "world";
  ASSERT_TRUE(graph.getEdge(0u, 1u));
  ASSERT_FALSE(graph.getEdge(2u, 2u));
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

TEST(Graph, DirectedErase) {
  using GraphType = DirectedGraph<Unit, Unit, Unit>;
  GraphType graph;
  graph.emplaceVertex(0);
  graph.emplaceVertex(1);
  graph.emplaceVertex(2);
  GraphType::EdgeIterator e0 = graph.emplaceEdge(0u, 0u).first;
  GraphType::EdgeIterator e1 = graph.emplaceEdge(0u, 1u).first;
  GraphType::EdgeIterator e2 = graph.emplaceEdge(0u, 2u).first;
  GraphType::EdgeIterator e3 = graph.emplaceEdge(1u, 1u).first;
  GraphType::EdgeIterator e4 = graph.emplaceEdge(1u, 2u).first;
  GraphType::EdgeIterator e5 = graph.emplaceEdge(2u, 0u).first;
  ASSERT_EQ(graph.getVertices().getCount(), 3);
  ASSERT_EQ(graph.getEdges().getCount(), 6);
  ASSERT_EQ(graph.getEdgesFromVertex(0u).getCount(), 3);
  ASSERT_EQ(graph.getEdgesToVertex(1u).getCount(), 2);
  graph.eraseEdge(e1);
  ASSERT_EQ(graph.getEdgesFromVertex(0u).getCount(), 2);
  ASSERT_EQ(graph.getEdgesToVertex(1u).getCount(), 1);
  graph.eraseVertex(0u);
  ASSERT_EQ(graph.getVertices().getCount(), 2);
  ASSERT_EQ(graph.getEdges().getCount(), 2);
}

TEST(Graph, UndirectedErase) {
  using GraphType = UndirectedGraph<Unit, Unit, Unit>;
  GraphType graph;
  graph.emplaceVertex(0);
  graph.emplaceVertex(1);
  graph.emplaceVertex(2);
  GraphType::EdgeIterator e0 = graph.emplaceEdge(0u, 0u).first;
  GraphType::EdgeIterator e1 = graph.emplaceEdge(0u, 1u).first;
  GraphType::EdgeIterator e2 = graph.emplaceEdge(0u, 2u).first;
  GraphType::EdgeIterator e3 = graph.emplaceEdge(1u, 1u).first;
  GraphType::EdgeIterator e4 = graph.emplaceEdge(1u, 2u).first;
  ASSERT_EQ(graph.getVertices().getCount(), 3);
  ASSERT_EQ(graph.getEdges().getCount(), 5);
  ASSERT_EQ(graph.getEdgesFromVertex(0u).getCount(), 3);
  ASSERT_EQ(graph.getEdgesToVertex(1u).getCount(), 3);
  graph.eraseEdge(e1);
  ASSERT_EQ(graph.getEdgesFromVertex(0u).getCount(), 2);
  ASSERT_EQ(graph.getEdgesToVertex(1u).getCount(), 2);
  graph.eraseVertex(0u);
  ASSERT_EQ(graph.getVertices().getCount(), 2);
  ASSERT_EQ(graph.getEdges().getCount(), 2);
}

TEST(Graph, DirectedDFS) {
  using GraphType = DirectedGraph<Unit, Unit, Unit>;
  GraphType graph;
  graph.emplaceVertex(0);
  graph.emplaceVertex(1);
  graph.emplaceVertex(2);
  graph.emplaceVertex(3);
  graph.emplaceVertex(4);
  graph.emplaceEdge(0u, 1u);
  graph.emplaceEdge(1u, 2u);
  graph.emplaceEdge(2u, 0u);
  graph.emplaceEdge(0u, 2u);
  graph.emplaceEdge(2u, 2u);
  graph.emplaceEdge(0u, 3u);
  graph.emplaceEdge(0u, 4u);
  graph.emplaceEdge(4u, 0u);
  graph.emplaceEdge(3u, 2u);
  graph.emplaceEdge(4u, 4u);
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
  graph.emplaceEdge(0u, 1u);
  graph.emplaceEdge(1u, 2u);
  graph.emplaceEdge(2u, 0u);
  graph.emplaceEdge(2u, 2u);
  graph.emplaceEdge(0u, 3u);
  graph.emplaceEdge(0u, 4u);
  graph.emplaceEdge(3u, 2u);
  graph.emplaceEdge(4u, 4u);
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
  graph.emplaceEdge(0u, 0u);
  graph.emplaceEdge(0u, 1u);
  graph.emplaceEdge(1u, 2u);
  graph.emplaceEdge(3u, 4u);
  graph.emplaceEdge(4u, 3u);
  graph.emplaceEdge(4u, 4u);
  std::vector<GraphType> components = graph.getConnectedComponents();
  ASSERT_EQ(components.size(), 2);
  ASSERT_EQ(components[0].getVertices().getCount(), 3);
  ASSERT_EQ(components[0].getEdges().getCount(), 3);
  ASSERT_EQ(components[1].getVertices().getCount(), 2);
  ASSERT_EQ(components[1].getEdges().getCount(), 3);
}

TEST(Graph, DirectedRenameVertex) {
  using GraphType = DirectedGraph<Unit, Unit, Unit>;
  GraphType graph;
  graph.emplaceVertex(0);
  graph.emplaceVertex(1);
  graph.emplaceVertex(2);
  graph.emplaceEdge(0u, 0u);
  graph.emplaceEdge(0u, 1u);
  graph.emplaceEdge(2u, 0u);
  graph.emplaceEdge(1u, 2u);
  graph.emplaceEdge(2u, 2u);
  ASSERT_FALSE(graph.renameVertex(0u, 2u));
  ASSERT_TRUE(graph.renameVertex(0u, 3u));
}
}  // namespace algo
