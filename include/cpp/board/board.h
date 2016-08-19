/*
 * Copyright (c) 2016, The Regents of the University of California (Regents).
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
 * Authors:       David Fridovich-Keil   ( dfk@eecs.berkeley.edu )
 */
/////////////////////////////////////////////////////////////////////////////
//
// Defines a chess board, implemented as a collection of Pieces.
//
/////////////////////////////////////////////////////////////////////////////

#ifndef DUMBO_BOARD_BOARD_H
#define DUMBO_BOARD_BOARD_H

#include <board/pawn.h>
#include <board/square.h>
#include <board/piece.h>
#include <board/pawn.h>
#include <board/knight.h>
#include <board/bishop.h>
#include <board/rook.h>
#include <board/queen.h>
#include <board/king.h>

#include <glog/logging.h>
#include <unordered_set>
#include <memory>

namespace dumbo {
  class Board {
  public:
    typedef std::shared_ptr<Board> Ptr;
    typedef std::shared_ptr<const Board> ConstPtr;

    // Initialize a new board.
    Board();
    ~Board() {}

    // Move the given Piece to the specified square if legal. Perform capture if
    // appropriate.
    bool Move(const Piece& piece, const Square& square);

    // Check if there is a piece of the specified color at the given square.
    bool HasPiece(const Square& square, Color color);

  private:
    // Lists of all pieces.
    std::unordered_set<Piece> white_pieces_;
    std::unordered_set<Piece> black_pieces_;
  }; // class Piece
} // namespace dumbo

#endif
