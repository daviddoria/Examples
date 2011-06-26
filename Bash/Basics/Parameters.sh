#!/bin/bash

if [ $# -lt 1 ]; then #-lt is "less than"
  echo "You must enter an argument!";
  exit;
fi

testVariable=$1; #must NOT have spaces around '='
echo "You input $testVariable"
