#!/bin/bash
N=3
for i in `seq 1 $N`; do
  rosrun blackboard test_blackboard `expr $i - 1` $N &
done
wait
