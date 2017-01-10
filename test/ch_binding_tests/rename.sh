#!/bin/bash

files=`ls *.ch`

for file in $files; do
    new_filename=`echo $file | sed -e "s/^test.._\(.*\)/\1/g"`
    echo $new_filename
    mv $file $new_filename
done
