#!/bin/bash
# This script needs to be run on bash.

#echo "building yasm"
#pushd $(dirname $0)/yasm
#./autogen.sh
#./configure
#make -j
#popd

echo "building mpir"
pushd $(dirname $0)/mpir
./autogen.sh
./configure --enable-cxx --enable-gmpcompat --with-yasm=$(cd ../yasm && pwd)/yasm
make -j
popd
