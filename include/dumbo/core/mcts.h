/*
 * Copyright (c) 2019. David Fridovich-Keil.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 *    1. Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *
 *    2. Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *
 *    3. Neither the name of the copyright holder nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS AS IS
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Please contact the author(s) of this library if you have any questions.
 * Author: David Fridovich-Keil   ( dfk@eecs.berkeley.edu )
 */

///////////////////////////////////////////////////////////////////////////////
//
// Generic implementation of Monte Carlo Tree Search.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef DUMBO_CORE_MCTS_H
#define DUMBO_CORE_MCTS_H

#include <dumbo/core/game_state.h>
#include <dumbo/core/move.h>
#include <dumbo/core/solver.h>

#include <glog/logging.h>
#include <algorithm>
#include <chrono>
#include <iterator>
#include <memory>
#include <unordered_map>
#include <vector>

namespace {

template <typename M, typename G>
struct Node {
  G state;
  std::unordered_map<M, std::shared_ptr<Node>, typename M::Hasher> children;
  std::shared_ptr<Node> parent = nullptr;
  double wins = 0.0;
  double total = 0.0;

  // Update this node and all of its parents with a win/loss/draw result.
  // Winning is encoded as 1.0, loss as 0.0, and draw as 0.5.
  // The input is whether the computer won the rollout; within each node, the
  // 'wins' variable stores the number of wins for the player who played the
  // move immediately leading to that node.
  void Update(double win) {
    total += 1.0;

    if (state.IsMyTurn())
      wins += 1.0 - win;
    else
      wins += win;

    if (parent) parent->Update(win);
  }
};  //\struct Node

}  // namespace

namespace dumbo {
namespace core {

template <typename M, typename G>
class MCTS : public Solver<M, G> {
 public:
  ~MCTS() {}
  MCTS(double max_time_per_move = 1.0) : Solver<M, G>(max_time_per_move) {}

  // Run the solver on the specified game state. Returns a move.
  M Run(const G& state);

private:
  typedef std::shared_ptr<Node<M, G>> NodePtr;
};  //\class MCTS

// ---------------------------- IMPLEMENTATION ----------------------------- //

template <typename M, typename G>
M MCTS<M, G>::Run(const G& state) {
  CHECK(state.IsMyTurn());

  // Start a clock.
  const auto start_time = std::chrono::high_resolution_clock::now();

  // Root an empty tree at the initial state.
  NodePtr root(new Node<M, G>);
  root->state = state;

  // Keep track of all nodes we've seen so far.
  std::vector<NodePtr> registry = {root};

  while (std::chrono::duration<double>(
             std::chrono::high_resolution_clock::now() - start_time)
             .count() < this->max_time_per_move_) {
    // (1) Pick a node to expand.
    auto compare_ucbs = [](const std::pair<M, NodePtr>& entry1,
                           const std::pair<M, NodePtr>& entry2) {
      const NodePtr& n1 = entry1.second;
      const NodePtr& n2 = entry2.second;
      constexpr double kNumStddevs = 1.414;  // std::sqrt(2.0);

      CHECK_GT(n1->total, 0.0);
      CHECK_GT(n2->total, 0.0);

      // Compute UCBs for both nodes.
      // NOTE: this is the UCT rule which may be found at
      // https://en.wikipedia.org/wiki/Monte_Carlo_tree_search.
      const double parent_total1 = (n1->parent) ? n1->parent->total : n1->total;
      const double ucb1 =
          (n1->wins / n1->total) +
          kNumStddevs * std::sqrt(std::log(parent_total1) / n1->total);

      const double parent_total2 = (n2->parent) ? n2->parent->total : n2->total;
      const double ucb2 =
          (n2->wins / n2->total) +
          kNumStddevs * std::sqrt(std::log(parent_total2) / n2->total);

      // Compare UCBs.
      return ucb1 < ucb2;
    };  // compare_ucbs

    // Start at the root and walk down to a leaf node, using the above
    // comparitor to choose moves for both players.
    // NOTE: always use max UCB since each node stores the wins for the
    // player whose move it is at that node.
    NodePtr node = root;
    while (!node->children.empty() &&
           node->children.size() == node->state.LegalMoves().size()) {
      auto iter = std::max_element(node->children.begin(), node->children.end(),
                                   compare_ucbs);
      node = iter->second;
    }

    // If this is a terminal node, then just try again.
    double win = 0.0;
    if (node->state.IsTerminal(&win)) {
      VLOG(1) << "Hit a terminal leaf node. Updating and continuing.";
      node->Update(win);
      continue;
    }

    // (2) Expand the node by choosing a random move not already tried yet.
    M move = node->state.RandomMove();
    while (node->children.count(move)) {
      move = node->state.RandomMove();
    }

    NodePtr expansion(new Node<M, G>);
    registry.emplace_back(expansion);

    CHECK(node->state.NextState(move, &expansion->state));
    expansion->parent = node;
    node->children.emplace(move, expansion);

    // (3) Sample random game trajectory from the expanded node.
    G current_state = expansion->state;
    while (!current_state.IsTerminal(&win)) {
      const M move = current_state.RandomMove();

      G next_state;
      CHECK(current_state.NextState(move, &next_state));

      current_state = next_state;
    }

    // (3) Update all ancestors of leaf node.
    expansion->Update(win);
  }

  // Ran out of time. Pick the best move from the root.
  const auto iter =
      std::max_element(root->children.begin(), root->children.end(),
                       [](const std::pair<M, NodePtr>& entry1,
                          const std::pair<M, NodePtr>& entry2) {
                         return entry1.second->wins / entry1.second->total <
                                entry2.second->wins / entry2.second->total;
                       });
  return iter->first;
}

}  // namespace core
}  // namespace dumbo

#endif
