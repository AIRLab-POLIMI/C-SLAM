#!/bin/bash

#Epic test script used to test the c_fuzzy_reasoner classification service
echo "two concentric circles and a door (with handle)"
rosservice call /classification "
threshold: 0.0
objects: 
-
  id: 0
  variables:
  - 
    name: 'x'
    value: 100
  -
    name: 'y'
    value: 100
-
  id: 1
  variables:
  - 
    name: 'x'
    value: 105
  -
    name: 'y'
    value: 105
-
  id: 2
  variables:
  -
    name: 'Xmin'
    value: 0
  -
    name: 'Xmax'
    value: 20
  -
    name: 'Ymin'
    value: 0
  -
    name: 'Ymax'
    value: 40
  -
    name: 'FormFactor'
    value: 1
  -
    name: 'color'
    value: 12
-
  id: 3
  variables:
  - 
    name: 'x'
    value: 18
  -
    name: 'y'
    value: 20
"
echo "---------------------"
