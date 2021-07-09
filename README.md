# icfpc2021
ICFPC2021 sanma team repository

## Prerequisites

* Linux (Ubuntu 20.04 LTS (not 18.04 LTS which lacks g++-10)) / WSL1 on Windows 10
  * `sudo apt install build-essential git-core cmake g++-10 libtool autoconf`
* Windows 10
  * Visual Studio 2019 (16.10.3)
  * CMake 3.17.3 (add to PATH)

## Build (Linux / WSL)

```
git clone --recursive https://github.com/nodchip/icfpc2021.git
# if you forgot to clone with --recursive, try: git submodule update --init
cd icfpc2021
bash libs/build.sh # to build external libraries
bash libs/build_local.sh # required only once
cd src
make -Bj # rebuild all in parallel.
make solver # or if you wish to build only the solver
make test # or if you wish to build only tests
```

## Build (Visual Studio)

```
git clone --recursive https://github.com/nodchip/icfpc2021.git
open Visual Studio 2019 Developer Console
cd icfpc2021
libs/build.bat # to build external libraries (Debug and Release)
start vs/ICFPC2021.sln
select Release;x64 or Debug;x64
Build Solution
```

## Run Tests

```
./test # run all tests
./test --gtest_filter=TestExample.* # run specific tests.
```

## Run Web REPL

```
./solver visualize_repl
start http://localhost:3333/
```

automatically loads `*.txt` from the data directory.

## Run Console REPL

```
./solver repl
```

automatically loads `*.txt` from the data directory.

## Add a Solver

1. duplicate existing solver (found in solvers/*.cpp)
2. register your solver with an unique name with `REGISTER_SOLVER("name", solver_func)`

## Notes

* all source codes are UTF-8.
* line endings are LF.
