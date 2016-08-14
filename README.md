# Dumbo -- Chess for Dummies
[![License](https://img.shields.io/badge/license-BSD-blue.svg)](LICENSE)

Why **Dumbo**? **Dumbo** is a new chess engine meant for the [Dumbos](http://movies.disney.com/dumbo) among us. It is designed for first for beauty and simplicity, and second for performance. It should be easy to use and develop from, and also fun to play against.

## About the author
I've been playing chess since I was in kindergarden or so. I played tournaments all through elementary and middle school, but didn't play much through high school or college. I'm currently a second-year PhD student in EECS at UC Berkeley, and since starting the program I've been getting back into the game. I am rated ~1750 on [chess.com](https://www.chess.com).

## Overview
This repository is entirely written in C++, and is structured around the CMAKE cross-compilation paradigm. All source code is in the `src/cpp` directory.

## Status
Currently under active development. Not usable at the moment. More updates to follow.

## Build instructions
We follow the standard CMAKE build scheme. Just download the repository and from the top directory type:

```bash
mkdir build && cd build && cmake .. && make
```
