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

TEST(DirectedGraph, AddNode) {
  DirectedGraph graph;
  graph.addNode(0);
  ASSERT_EQ(graph.getNodes().size(), 1);
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

TEST(DirectedGraph, PruneSourcesAndSinks) {
  DirectedGraph graph;
  graph.connect(5, 6);
  graph.connect(6, 4);
  graph.connect(4, 5);
  graph.connect(6, 7);
  graph.connect(3, 6);
  graph.connect(3, 7);
  graph.connect(7, 1);
  graph.connect(2, 3);
  DirectedGraph pruned(graph.pruneSourcesAndSinks());
  ASSERT_EQ(pruned.getNodes().size(), 3);
  ASSERT_EQ(pruned.getEdges().size(), 3);
}
}  // namespace algo
