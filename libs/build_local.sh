#!/bin/bash
# This script needs to be run on bash.

echo "building gmp"
pushd $(dirname $0)/gmp-6.2.1
./configure --enable-cxx
make
popd
