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
// Defines a generic chess piece. A Piece knows it's position (Square) on the
// board, so that it can generate a list of valid Moves.
//
/////////////////////////////////////////////////////////////////////////////

#ifndef DUMBO_BOARD_PIECE_H
#define DUMBO_BOARD_PIECE_H

#include <board/move.h>

#include <glog/logging.h>
#include <vector>
#include <memory>

namespace dumbo {
  class Piece {
  public:
    typedef std::shared_ptr<Piece> Ptr;
    typedef std::shared_ptr<const Piece> ConstPtr;

    Piece(Square square) : square_(square) {}
    ~Piece() {}

    // Set and retrieve location on the board.
    void SetSquare(const Square& square) { square_ = square; }
    const Square& GetSquare() { return square_; }

    // Populates the given vector with all valid moves this Piece can make.
    virtual void GetMoves(std::vector<Move>& moves) const = 0;

  protected:
    // Current location of this Piece on the board.
    Square square_;

  }; // class Piece
} // namespace dumbo

#endif
