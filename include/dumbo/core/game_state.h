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
// Base class for all game states.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef DUMBO_CORE_GAME_STATE_H
#define DUMBO_CORE_GAME_STATE_H

#include <dumbo/core/move.h>

#include <memory>
#include <random>

namespace dumbo {
namespace core {

template <typename M>
class GameState {
 public:
  virtual ~GameState() {}

  // Enumerate all legal moves from this game state.
  virtual std::vector<M> LegalMoves() const = 0;

  // Choose a random move from this game state.
  virtual M RandomMove() const = 0;

  // Populate the GameState that occurs when the current player plays
  // the given move. Returns whether or not the move was legal.
  virtual bool NextState(const M& move, GameState* next_state) const = 0;

  // Is this a terminal state? If so, return whether it is a win/loss/draw
  // (encoded as 1.0/0.0/0.5).
  virtual bool IsTerminal(double* win) const = 0;

  // Check if it is the AI's turn or not.
  bool IsMyTurn() const { return my_turn_; }

  // Check if same as another game state.
  virtual bool operator==(const GameState<M>& rhs) const = 0;

  // Render.
  virtual void Render() const = 0;

 protected:
  GameState(bool my_turn = true) : my_turn_(my_turn) {}

  // Is it my turn?
  bool my_turn_;

  // Random number generation.
  static std::random_device rd_;
  static std::default_random_engine rng_;
};  //\class GameState

// Define static variables.
template <typename M>
std::random_device GameState<M>::rd_;

template <typename M>
std::default_random_engine GameState<M>::rng_(GameState<M>::rd_());

}  // namespace core
}  // namespace dumbo

#endif
