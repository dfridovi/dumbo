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
    auto compare_ucbs = [](const std::shared_ptr<Node>& n1,
                           const std::shared_ptr<Node>& n2) {
      constexpr double num_stddevs = 1.414;  // std::sqrt(2.0);

      // Compute UCBs for both nodes.
      // NOTE: this is the UCT rule which may be found at
      // https://en.wikipedia.org/wiki/Monte_Carlo_tree_search.
      const double parent_total1 = (n1->parent) ? n1->parent->total : n1->total;
      const double ucb1 =
          (n1->wins / n1->total) +
          num_stddevs * std::sqrt(std::log(parent_total1) / n1->total);

      const double parent_total2 = (n2->parent) ? n2->parent->total : n2->total;
      const double ucb2 =
          (n2->wins / n2->total) +
          num_stddevs * std::sqrt(std::log(parent_total2) / n2->total);

      // Compare UCBs.
      return ucb1 < ucb2;
    };

    // Find maximum UCB by linear search. NOTE: this could be accelerated by
    // storing in pre-sorted order.
    auto max_iter =
        std::max_element(registry.begin(), registry.end(), compare_ucbs);

    // (2) Expand the node by sampling a random game trajectory.
    std::shared_ptr<Node> node = *max_iter;
    double win = 0.0;
    while (!node->state.IsTerminal(&win)) {
      const M move = node->state.RandomMove();

      G next_state;
      CHECK(node->state.NextState(move, &next_state));

      // If we've already taken this move from this node then just restart from
      // the appropriate child node.
      if (node->children.count(move)) {
        node = node->children[move];
        continue;
      }

      // Create a new node in the registry for the next state.
      std::shared_ptr<Node> next_node(new Node);
      next_node->state = next_state;
      next_node->parent = node;
      registry.push_back(next_node);

      // Update current node's child list to include the next node.
      node->children.emplace(move, next_node);

      // Set node to next node.
      node = next_node;
    }

    // (3) Update all ancestors of terminal node.
    node->Update(win);

    std::cout << "Registry size: " << registry.size() << std::endl;
    std::cout << "Root children: " << registry[0]->children.size() << std::endl;
    std::cout << "Root empty squares: " << registry[0]->state.LegalMoves().size() << std::endl;
  }

  // Ran out of time. Pick the best move in the tree.
  // NOTE: as a heuristic, choosing the move with the best LCB.
  auto compare_lcbs = [](const std::pair<M, std::shared_ptr<Node>>& m1,
                         const std::pair<M, std::shared_ptr<Node>>& m2) {
    const std::shared_ptr<Node> n1 = m1.second;
    const std::shared_ptr<Node> n2 = m2.second;
    constexpr double num_stddevs = 1.414;  // std::sqrt(2.0);

    // Compute LCBs for both nodes.
    // NOTE: this is the UCT rule which may be found at
    // https://en.wikipedia.org/wiki/Monte_Carlo_tree_search.
    const double parent_total1 = (n1->parent) ? n1->parent->total : n1->total;
    const double lcb1 = (n1->wins / n1->total);  //-
    //        num_stddevs * std::sqrt(std::log(parent_total1) / n1->total);

    const double parent_total2 = (n2->parent) ? n2->parent->total : n2->total;
    const double lcb2 = (n2->wins / n2->total);  // -
    // num_stddevs * std::sqrt(std::log(parent_total2) / n2->total);

    // Compare LCBs.
    return lcb1 < lcb2;
  };

  // Root is always first element of registry.
  // NOTE: this could change if registry container changes.
  double best_total_plays = 0.0;
  M best_move;
  for (const auto& entry : registry[0]->children) {
    const double total_plays = entry.second->total;
    std::cout << "move: " << entry.first << " / total plays: " << total_plays
              << std::endl;
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
