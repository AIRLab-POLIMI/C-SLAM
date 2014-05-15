#!/bin/bash

for((i=1; i <= 20; i++))
do
	. testReasoner.bash &
	. testClassifier.bash &
	echo $i
done
