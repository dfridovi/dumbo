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
// Struct to represent an occupied square in tic tac toe. This will be used to
// encode moves.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef DUMBO_TIC_TAC_TOE_SQUARE_H
#define DUMBO_TIC_TAC_TOE_SQUARE_H

#include <dumbo/core/move.h>

#include <stdint.h>
#include <boost/functional/hash.hpp>
#include <iostream>

namespace dumbo {
namespace tic {

struct Square : public core::Move {
  uint8_t row = 0;
  uint8_t col = 0;
  bool my_square = true;

  // Constructor.
  Square() {}
  Square(uint8_t ii, uint8_t jj, bool mine)
      : row(ii), col(jj), my_square(mine) {}

  // Convert to/from text.
  void Print(std::ostream& output) const {
    if (my_square)
      output << "Computer: ";
    else
      output << "Human: ";

    output << "(" << row << ", " << col << ")";
  }

  void Load(std::istream& input) {
    my_square = false;
    input >> row >> col;
  }

  // Equality operator.
  bool operator==(const Square& rhs) const {
    return row == rhs.row && col == rhs.col && my_square == rhs.my_square;
  }

  // Hash functor.
  struct Hasher {
    size_t operator()(const Square& sq) const {
      size_t seed = 0;
      boost::hash_combine(seed, sq.row);
      boost::hash_combine(seed, sq.col);
      boost::hash_combine(seed, sq.my_square);
      return seed;
    }
  };  //\struct Hasher
};    //\struct Square

}  // namespace tic
}  // namespace dumbo

#endif
