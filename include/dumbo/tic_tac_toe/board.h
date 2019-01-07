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
// Tic tac toe board.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef DUMBO_TIC_TAC_TOE_BOARD_H
#define DUMBO_TIC_TAC_TOE_BOARD_H

#include <dumbo/core/game_state.h>
#include <dumbo/tic_tac_toe/square.h>

#include <tuple>
#include <unordered_set>
#include <vector>

namespace dumbo {
namespace tic {

class Board : public core::GameState<Square> {
 public:
  ~Board() {}
  Board(bool my_turn = true) : Board({}, my_turn) {}
  Board(const std::unordered_set<Square, Square::Hasher>& occupied_squares,
        bool my_turn = true);

  // Enumerate all legal moves from this game state.
  std::vector<Square> LegalMoves() const { return empty_squares_; }

  // Choose a random move from this game state.
  Square RandomMove() const;

  // Populate the GameState that occurs when the current player plays
  // the given move. Returns whether or not the move was legal.
  bool NextState(const Square& move, GameState* next_state) const;

  // Is this a terminal state? If so, return whether it is a win/loss/draw
  // (encoded as 1.0/0.0/0.5).
  bool IsTerminal(double* win) const;

  // Check if it is the AI's turn or not.
  bool IsMyTurn() const { return my_turn_; }

  // Check if same as another game state.
  bool operator==(const GameState<Square>& rhs) const;
  // Render.
  void Render() const;

 private:
  // Set of occupied and empty squares on the board. Occupied squares stored as
  // unordered set but empty squares stored as vector for efficiency of
  // legality and random selection operations.
  std::unordered_set<Square, Square::Hasher> occupied_squares_;
  std::vector<Square> empty_squares_;

  // Set of possible winning and losing three-in-a-rows.
  typedef std::tuple<Square, Square, Square> SquareTuple;
  static const std::vector<SquareTuple> winning_sets_;
  static const std::vector<SquareTuple> losing_sets_;
};  //\class Board

}  // namespace tic
}  // namespace dumbo

#endif
