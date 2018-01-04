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
// Defines a particular square on the chess board.
//
/////////////////////////////////////////////////////////////////////////////

#ifndef DUMBO_BOARD_SQUARE_H
#define DUMBO_BOARD_SQUARE_H

#include <glog/logging.h>
#include <boost/functional/hash.hpp>

namespace dumbo {

struct Square {
  // Rank and file (integers between 0 and 7).
  unsigned char rank_, file_;

  // Constructor/destructor.
  ~Square() {}
  explicit Square(unsigned char rank, unsigned char file)
    : rank_(rank), file_(file) {
#ifndef ENABLE_DEBUG_MESSAGES
    if (rank > 7) {
      VLOG(1) << "Rank was out of bounds. Set to 0.";
      rank_ = 0;
    }

    if (file > 7) {
      VLOG(1) << "File was out of bounds. Set to 0.";
      file_ = 0;
    }
#endif
  }

  // (In)equality operators.
  inline bool operator==(const Square& rhs) const {
    return rank_ == rhs.rank_ && file_ == rhs.file_;
  }

  inline bool operator!=(const Square& rhs) const {
    return rank_ != rhs.rank_ || file_ != rhs.file_;
  }

  // Custom hash functor.
  struct Hash {
    inline size_t operator()(const Square& square) const {
      size_t seed = 0;
      boost::hash_combine(seed, boost::hash_value(square.rank_));
      boost::hash_combine(seed, boost::hash_value(square.file_));

      return seed;
    }
  }; // struct Hash

}; // struct Square

} // namespace dumbo

#endif
