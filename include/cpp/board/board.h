/*
 * Copyright (c) 2018, David Fridovich-Keil.
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
 * Authors: David Fridovich-Keil   ( david.fridovichkeil@gmail.com )
 */
/////////////////////////////////////////////////////////////////////////////
//
// Defines a chess board, implemented as a collection of Pieces.
//
/////////////////////////////////////////////////////////////////////////////

#ifndef DUMBO_BOARD_BOARD_H
#define DUMBO_BOARD_BOARD_H

#include <board/square.h>
#include <utils/types.h>

#include <glog/logging.h>
#include <unordered_set>
#include <unordered_map>
#include <list>
#include <memory>

namespace dumbo {

class Board {
public:
  ~Board() {}
  explicit Board();

  // Return the set of possible Boards resulting from the next player's move.
  std::list<const Board> Options() const;

private:
  // Check if the given square is occupied.
  inline bool IsOccupied(const Square& square) const {
    if (white_pieces_.count(square) == 0 &&
        black_pieces_.count(square) == 0)
      return false;

    return true;
  }

  // Lists of possible moves for each type of piece.
  std::list<Square> KingMoves(const Square& square) const;
  std::list<Square> QueenMoves(const Square& square) const;
  std::list<Square> BishopMoves(const Square& square) const;
  std::list<Square> KnightMoves(const Square& square) const;
  std::list<Square> RookMoves(const Square& square) const;
  std::list<Square> PawnMoves(const Square& square) const;

  // Hash tables storing pieces and their locations.
  std::unordered_map<Square, Piece, Square::Hash> white_pieces_;
  std::unordered_map<Square, Piece, Square::Hash> black_pieces_;

  // Who's turn is it next?
  Color turn_;
}; // class Board

} // namespace dumbo

#endif
