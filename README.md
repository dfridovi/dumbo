# Dumbo -- Chess for Dummies
[![License](https://img.shields.io/badge/license-BSD-blue.svg)](LICENSE)

Why **Dumbo**? **Dumbo** is a new game engine meant for the [Dumbos](http://movies.disney.com/dumbo) among us. It is designed for first for simplicity, and second for performance. It should be easy to use and develop from, and also fun to play against.

## About the author
I've been playing chess since I was in kindergarden or so, and for the last few years I've been meaning to try and build my own chess engine. This is the first step -- a generic framework for building game engines and a toy implementation for tic tac toe.

## Overview
This repository is entirely written in C++, and is structured around the `cmake` cross-compilation paradigm.

## Dependencies
The main external dependencies are just for logging and command line flags. Installation should be available from your operating system's package manager.
* `glog` (Google's C++ logging tools)
* `gflags` (Google's C++ command line flags)
* `boost` (general C++ toolchain)

## Build instructions
As with any other `cmake` build, just download the repository and from the top directory type:

```bash
mkdir build && cd build && cmake .. && make
```

## Demo
To run the tic tac toe demo:

```bash
cd bin
./tic_tac_toe
```
