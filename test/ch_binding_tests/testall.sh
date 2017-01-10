#!/bin/bash

if [ $# -lt 1 ]; then
    echo "Usage: $1 <serial_id>"
    exit
fi

tests=`find -name "*.ch"`
for test in $tests; do
    echo "Press <enter> to run test $test:"
    read
    echo $1 | ch $test
    echo
    echo
done
