#!/bin/sh

cd build
cat ../data/*.txt | ./main "$@" || echo "run error code: $?"
