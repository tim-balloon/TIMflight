#!/bin/sh
if [ $# -gt 0 ] ; then
  files="$*"
else
  files=$(find . include -name \*.c -o -name \*.h)
fi
# Run in parallel
echo $files | xargs -P 8 -n 8 python2 testing/cpplint.py \
  --root=include --verbose=2 

