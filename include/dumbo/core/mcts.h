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
#include <memory>
#include <unordered_map>
#include <vector>

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
  struct Node {
    G state;
    std::unordered_map<M, std::shared_ptr<Node>, typename M::Hasher> children;
    std::shared_ptr<Node> parent = nullptr;
    double wins = 0.0;
    double total = 0.0;

    // Update this node and all of its parents with a win/loss/draw result.
    // Winning is encoded as 1.0, loss as 0.0, and draw as 0.5.
    void Update(double win) {
      total += 1.0;

      if (state.IsMyTurn())
        wins += win;
      else
        wins += 1.0 - win;

      if (parent) parent->Update(win);
    }
  };  //\struct Node
};    //\class MCTS

// ---------------------------- IMPLEMENTATION ----------------------------- //

template <typename M, typename G>
M MCTS<M, G>::Run(const G& state) {
  // Start a clock.
  const auto start_time = std::chrono::high_resolution_clock::now();

  // Root an empty tree at the initial state.
  std::shared_ptr<Node> root(new Node);
  root->state = state;
  CHECK(state.IsMyTurn());

  // Keep track of all nodes we've seen so far.
  std::vector<std::shared_ptr<Node>> registry = {root};

  while (std::chrono::duration<double>(
             std::chrono::high_resolution_clock::now() - start_time)
             .count() < this->max_time_per_move_) {
    // (1) Pick a node to expand.
    auto compare_ucbs = [](const std::pair<M, std::shared_ptr<Node>>& entry1,
                           const std::pair<M, std::shared_ptr<Node>>& entry2) {
      const std::shared_ptr<Node>& n1 = entry1.second;
      const std::shared_ptr<Node>& n2 = entry2.second;
      constexpr double kNumStddevs = 1.414;  // std::sqrt(2.0);
      constexpr double kSmallNumber = 1e-8;

      // Compute UCBs for both nodes.
      // NOTE: this is the UCT rule which may be found at
      // https://en.wikipedia.org/wiki/Monte_Carlo_tree_search.
      const double parent_total1 = (n1->parent) ? n1->parent->total : n1->total;
      const double ucb1 =
          (n1->wins / (n1->total + kSmallNumber)) +
          kNumStddevs * std::sqrt(std::log(parent_total1) / n1->total);

      const double parent_total2 = (n2->parent) ? n2->parent->total : n2->total;
      const double ucb2 =
          (n2->wins / (n2->total + kSmallNumber)) +
          kNumStddevs * std::sqrt(std::log(parent_total2) / n2->total);

      // Compare UCBs.
      return ucb1 < ucb2;
    };  // compare_ucbs

    // Start at the root and walk down to a leaf node, using the above
    // comparitor to choose moves for both players.
    std::shared_ptr<Node> node = registry[0];

    while (!node->children.empty()) {
      const auto& iter =
          (node->state.IsMyTurn())
              ? std::max_element(node->children.begin(), node->children.end(),
                                 compare_ucbs)
              : std::min_element(node->children.begin(), node->children.end(),
                                 compare_ucbs);
      node = iter->second;
    }

    // HACK! If this is a terminal node, then just use its parent.
    double win = 0.0;
    if (node->state.IsTerminal(&win)) {
      CHECK(node->parent);
      node = node->parent;
    }

    // (2) Expand this node by adding all children.
    for (const auto& move : node->state.LegalMoves()) {
      std::shared_ptr<Node> expansion(new Node);
      registry.push_back(expansion);

      CHECK(node->state.NextState(move, &expansion->state));
      expansion->parent = node;

      // Add expansion as child of parent. By default this will not overwrite
      // an existing entry with the same key (which is desired behavior).
      node->children.emplace(move, expansion);
    }

    // (2) Expand the node by choosing a random move.
    const M move = node->state.RandomMove();
    std::shared_ptr<Node> expansion = node->children.at(move);

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

  // Ran out of time. Pick the best move in the tree.
  // Root is always first element of registry.
  // NOTE: this could change if registry container changes.
  double best_total_plays = 0.0;
  M best_move;
  for (const auto& entry : registry[0]->children) {
    const double total_plays = entry.second->total;
    std::cout << "move: " << entry.first << " / total plays: " << total_plays
              << " / total wins: " << entry.second->wins << std::endl;
    if (total_plays > best_total_plays) {
      best_total_plays = total_plays;
      best_move = entry.first;
    }
  }

  // Return just the chosen move.
  return best_move;
}

}  // namespace core
}  // namespace dumbo

#endif
