# icfpc2021
ICFPC2021 sanma team repository

## Prerequisites

* Linux (Ubuntu 20.04 LTS (not 18.04 LTS which lacks g++-10)) / WSL1 on Windows 10
  * `sudo apt install build-essential git-core cmake g++-10 libtool autoconf texinfo libopencv-dev`
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

### GUI on WSL

1. start a X Server in Windows (VcXsrv https://sourceforge.net/projects/vcxsrv/)
2. ```
   export DISPLAY=0.0.0.0:0.0
   ./solver solve ManualSolver 2
   ````

## Build (Visual Studio)

```
git clone --recursive https://github.com/nodchip/icfpc2021.git
open Visual Studio 2019 Developer Console
cd icfpc2021
libs/build.bat # to build external libraries (Debug and Release)
libs/build_local.bat # required only once
start vs/ICFPC2021.sln
select Release;x64 or Debug;x64
Build Solution
```

## Run Tests

```
./test # run all tests
./test --gtest_filter=TestExample.* # run specific tests.
```

## Solve a problem

### Linux

```
cd src
./solve NaiveAnnealingSolver ../data/problems/1.problem.json out.json             # regular usage
./solve NaiveAnnealingSolver ../data/problems/1.problem.json out.json init.json   # .. or start with initial solution.
./solve NaiveAnnealingSolver 1 out.json                                           # shortcut
```

### Windows (console)

* current directory: vs\{solver,judge,test}
* exe path: vs\x64\{Release,Debug}

```
cd vs\solver
..\x64\Release\solve NaiveAnnealingSolver ../../data/problems/1.problem.json out.json             # regular usage
..\x64\Release\solve NaiveAnnealingSolver ../../data/problems/1.problem.json out.json init.json   # .. or start with initial solution.
..\x64\Release\solve NaiveAnnealingSolver 1 out.json                                           # shortcut
```

## Manual solver

```
./solver solve ManualSolver 52 output_53.pose.json [optional_input_pose_53.pose.json]
```

* d : toggle individual dislike score (large red circle means bad dislike)
* t : toggle "tolerated grid points" view (dark blue indicates some of the edges tolerate the point. light blue indicates all edges agree to the point.)
* e : toggle edge length annotation
* i : toggle node id annotation
* s : save current pose to intermediate.pose.json
* h : move left
* j : move down
* k : move up
* l : move right
* r : rotate clockwise
* ESC : save and exit

## Apply a solver to all problems

```
cd scripts
python solve_all.py SOLVER_NAME improvement   # overwrite existing solution file if the score improves
python solve_all.py SOLVER_NAME force         # always overwrite existing solution file
python solve_all.py SOLVER_NAME never         # do not overwrite existing solution file
```

* solves all problems in ./data/problems/
* results are stored in ./solutions/
* solve_all.csv (summary file) will be generated.

## Add a Solver

1. duplicate existing solver (found in solvers/*.cpp)
2. register your solver with an unique name with `REGISTER_SOLVER("name", solver_func)`

## Notes

* all source codes are UTF-8.
* line endings are LF.
