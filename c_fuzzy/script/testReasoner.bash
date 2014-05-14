#!/bin/bash

#Epic test script used to test the c_fuzzy_reasoner services
echo "Input=350 Prova=400"
rosservice call /reasoning "
inputs: 
-
  name: 'Input'
  value: 150"
echo "---------------------"
  
echo "Prova=150"
rosservice call /reasoning "
inputs: 
-
  name: 'Prova'
  value: 150"
echo "---------------------"
  
echo "Input=150 Prova=189"
rosservice call /reasoning "
inputs: 
-
  name: 'Input'
  value: 150
-
  name: 'Prova'
  value: 189"
echo "---------------------"
  
echo "Input=189 Prova=150"
rosservice call /reasoning "
inputs: 
-
  name: 'Input'
  value: 189
-
  name: 'Prova'
  value: 150"
echo "---------------------"

echo "Input=350 Prova=400"
rosservice call /reasoning "
inputs: 
-
  name: 'Input'
  value: 350
-
  name: 'Prova'
  value: 400"
echo "---------------------"