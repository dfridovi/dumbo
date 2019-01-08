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
// Tic tac toe player.
//
///////////////////////////////////////////////////////////////////////////////

#include <dumbo/core/game_state.h>
#include <dumbo/core/mcts.h>
#include <dumbo/core/player.h>
#include <dumbo/tic_tac_toe/board.h>
#include <dumbo/tic_tac_toe/square.h>

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <memory>

DEFINE_double(max_time_per_move, 0.05, "Maximum time (s) per move.");

using dumbo::tic::Board;
using dumbo::tic::Square;

int main(int argc, char** argv) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  // Set up logging.
  const std::string log_file =
      DUMBO_EXEC_DIR + std::string("/tic_tac_toe/out.log");
  google::SetLogDestination(0, log_file.c_str());
  FLAGS_logtostderr = true;
  FLAGS_minloglevel = 1;
  google::InitGoogleLogging(argv[0]);

  // Set up a clean board.
  const Board empty_board;

  // Set up a solver.
  std::unique_ptr<dumbo::core::MCTS<Square, Board>> solver(
      new dumbo::core::MCTS<Square, Board>(FLAGS_max_time_per_move));

  // Set up game player.
  dumbo::core::Player<Square, Board> player(empty_board, std::move(solver));

  // Play.
  player.Play();

  return EXIT_SUCCESS;
}
