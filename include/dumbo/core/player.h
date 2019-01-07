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
// Generic game playing client. Manages game from initial board configuration
// to final result. Templated on move type M, game state type G.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef DUMBO_CORE_PLAYER_H
#define DUMBO_CORE_PLAYER_H

#include <dumbo/core/game_state.h>
#include <dumbo/core/move.h>
#include <dumbo/core/solver.h>

#include <glog/logging.h>
#include <iostream>
#include <memory>

namespace dumbo {
namespace core {

template <typename M, typename G>
class Player {
 public:
  ~Player() {}
  Player(const G& initial_state, std::unique_ptr<Solver<M, G>> solver)
      : state_(initial_state), solver_(std::move(solver)) {
    CHECK_NOTNULL(solver_.get());
  }

  // Play out the rest of the game.
  void Play();

 private:
  // Current state of the game.
  G state_;

  // Solver.
  std::unique_ptr<Solver<M, G>> solver_;
};  //\class Player

// ----------------------------- IMPLEMENTATION ---------------------------- //

template <typename M, typename G>
void Player<M, G>::Play() {
  double win = 0.0;

  // Play until the game is over.
  while (!state_.IsTerminal(&win)) {
    state_.Render();
    G next_state;

    // If it's our move, then just play.
    if (state_.IsMyTurn()) {
      std::cout << "My turn." << std::endl;
      const M move = solver_->Run(state_);
      std::cout << move << std::endl;

      CHECK(state_.NextState(move, &next_state));
    } else {
      // Ask for a move from the human.
      std::cout << "Please input a 0-indexed row [ENTER] and column [ENTER]."
                << std::endl;

      M move;
      std::cin >> move;
      std::cout << move << std::endl;

      // Try to take this move. If not legal, then ask for another move.
      while (!state_.NextState(move, &next_state))
        std::cout << "Illegal move. Please try again." << std::endl;
    }

    // Update current board state.
    state_ = next_state;
  }

  // Print result.
  state_.Render();
  if (win < 0.5)
    std::cout << "Congratulations! You won the game." << std::endl;
  else if (win > 0.5)
    std::cout << "Sorry. You lost the game." << std::endl;
  else
    std::cout << "Tie game." << std::endl;
}

}  // namespace core
}  // namespace dumbo

#endif
