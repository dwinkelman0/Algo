// Copyright 2022 by Daniel Winkelman. All rights reserved.

#include <DirectedGraph.h>
#include <gtest/gtest.h>

namespace algo {
TEST(DirectedGraph, Connect) {
  DirectedGraph graph;
  graph.connect(0, 1);
  graph.connect(1, 2);
  graph.connect(2, 0);
  ASSERT_EQ(graph.getNodes().size(), 3);
  ASSERT_EQ(graph.getEdges().size(), 3);
  graph.unconnect(0, 1);
  ASSERT_EQ(graph.getEdges().size(), 2);
  graph.unconnect(1, 2);
  ASSERT_EQ(graph.getEdges().size(), 1);
  graph.unconnect(2, 0);
  ASSERT_EQ(graph.getEdges().size(), 0);
}

TEST(DirectedGraph, EraseNode) {
  DirectedGraph graph;
  graph.connect(0, 1);
  graph.connect(1, 2);
  graph.connect(2, 3);
  graph.connect(3, 0);
  graph.connect(1, 3);
  ASSERT_EQ(graph.getNodes().size(), 4);
  ASSERT_EQ(graph.getEdges().size(), 5);
  graph.eraseNode(3);
  ASSERT_EQ(graph.getNodes().size(), 3);
  ASSERT_EQ(graph.getEdges().size(), 2);
}
}  // namespace algo
