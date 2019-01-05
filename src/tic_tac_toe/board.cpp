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

#include <dumbo/core/game_state.h>
#include <dumbo/tic_tac_toe/board.h>
#include <dumbo/tic_tac_toe/square.h>

#include <glog/logging.h>
#include <iostream>
#include <random>
#include <tuple>
#include <unordered_set>
#include <vector>

namespace dumbo {
namespace tic {

const std::vector<Board::SquareTuple> Board::winning_sets_ = {
    // Rows.
    std::make_tuple(Square(0, 0, true), Square(0, 1, true), Square(0, 2, true)),
    std::make_tuple(Square(1, 0, true), Square(1, 1, true), Square(1, 2, true)),
    std::make_tuple(Square(2, 0, true), Square(2, 1, true), Square(2, 2, true)),

    // Columns.
    std::make_tuple(Square(0, 0, true), Square(1, 0, true), Square(2, 0, true)),
    std::make_tuple(Square(0, 1, true), Square(1, 1, true), Square(2, 1, true)),
    std::make_tuple(Square(0, 2, true), Square(1, 2, true), Square(2, 2, true)),

    // Diagonals.
    std::make_tuple(Square(0, 0, true), Square(1, 1, true), Square(2, 2, true)),
    std::make_tuple(Square(0, 2, true), Square(1, 1, true),
                    Square(2, 0, true))};

const std::vector<Board::SquareTuple> Board::losing_sets_ = {
    // Rows.
    std::make_tuple(Square(0, 0, false), Square(0, 1, false),
                    Square(0, 2, false)),
    std::make_tuple(Square(1, 0, false), Square(1, 1, false),
                    Square(1, 2, false)),
    std::make_tuple(Square(2, 0, false), Square(2, 1, false),
                    Square(2, 2, false)),

    // Columns.
    std::make_tuple(Square(0, 0, false), Square(1, 0, false),
                    Square(2, 0, false)),
    std::make_tuple(Square(0, 1, false), Square(1, 1, false),
                    Square(2, 1, false)),
    std::make_tuple(Square(0, 2, false), Square(1, 2, false),
                    Square(2, 2, false)),

    // Diagonals.
    std::make_tuple(Square(0, 0, false), Square(1, 1, false),
                    Square(2, 2, false)),
    std::make_tuple(Square(0, 2, false), Square(1, 1, false),
                    Square(2, 0, false))};

Board::Board(const std::unordered_set<Square, Square::Hasher>& occupied_squares,
             bool my_turn)
    : GameState<Square>(my_turn), occupied_squares_(occupied_squares) {
  // Check to make sure all occupied squares are legal.
  constexpr uint8_t kGridSize = 3;
  for (const auto& sq : occupied_squares_)
    CHECK(sq.row < kGridSize && sq.col < kGridSize);

  // Populate empty squares vector.
  for (uint8_t ii = 0; ii < kGridSize; ii++) {
    for (uint8_t jj = 0; jj < kGridSize; jj++) {
      const Square sq{ii, jj, my_turn_};

      if (!occupied_squares_.count(sq)) empty_squares_.emplace_back(sq);
    }
  }
}

// Choose a random move from this game state.
Square Board::RandomMove() const {
  CHECK(!empty_squares_.empty());
  std::uniform_int_distribution<size_t> unif(0, empty_squares_.size() - 1);
  return empty_squares_[unif(rng_)];
}

// Populate the GameState that occurs when the current player plays
// the given move. Returns whether or not the move was legal.
bool Board::NextState(const Square& move, GameState* next_state) const {
  CHECK_NOTNULL(next_state);

  // Check legality first.
  if (occupied_squares_.count(move)) return false;

  // Update next state.
  Board* next_board = static_cast<Board*>(next_state);
  CHECK_NOTNULL(next_board);

  next_board->occupied_squares_.clear();
  next_board->occupied_squares_.insert(occupied_squares_.begin(),
                                       occupied_squares_.end());
  next_board->occupied_squares_.emplace(move);

  next_board->empty_squares_.clear();
  for (const auto& empty : empty_squares_) {
    if (empty == move) continue;
    next_board->empty_squares_.emplace_back(empty);
  }

  return true;
}

// Is this a terminal state? If so, return whether it is a win/loss/draw
// (encoded as 1.0/0.0/0.5).
bool Board::IsTerminal(double* win) const {
  CHECK_NOTNULL(win);

  // Utility for checking if the board contains a winning/losing set.
  auto has_set = [this](const SquareTuple& s) {
    return this->occupied_squares_.count(std::get<0>(s)) &&
           this->occupied_squares_.count(std::get<1>(s)) &&
           this->occupied_squares_.count(std::get<2>(s));
  };

  // Check all winning sets.
  for (const auto& ws : winning_sets_) {
    if (has_set(ws)) {
      *win = 1.0;
      return true;
    }
  }

  // Check all losing sets.
  for (const auto& ls : losing_sets_) {
    if (has_set(ls)) {
      *win = 0.0;
      return true;
    }
  }

  // Draw if no remaining empty squares.
  if (empty_squares_.empty()) {
    *win = 0.5;
    return true;
  }

  return false;
}

// Check if same as another game state.
bool Board::operator==(const GameState<Square>& rhs) const {
  const Board& rhs_board = static_cast<const Board&>(rhs);

  // Make sure every element of occupied squares match. Empty squares will then
  // also have to match if they were constructed correctly.
  for (const auto& sq : rhs_board.occupied_squares_) {
    if (!occupied_squares_.count(sq)) return false;
  }

  return true;
}

// Render. By default, our moves are 'X's and humans are 'O's.
void Board::Render() const {
  constexpr uint8_t kNumRows = 3;

  Square sq{0, 0, true};
  for (uint8_t ii = 0; ii < kNumRows; ii++) {
    sq.row = ii;

    for (uint8_t jj = 0; jj < kNumRows; jj++) {
      sq.col = jj;
      sq.my_square = true;
      if (occupied_squares_.count(sq))
        std::cout << " X ";
      else {
        sq.my_square = false;
        if (occupied_squares_.count(sq))
          std::cout << " O ";
        else
          std::cout << "   ";
      }
    }

    std::cout << std::endl;
  }
}

}  // namespace tic
}  // namespace dumbo
