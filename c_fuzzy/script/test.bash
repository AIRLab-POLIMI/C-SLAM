#!/bin/bash

#Epic test script used to test the c_fuzzy_reasoner services
rosservice call /reasoning "inputs: [{name: 'Input', value: 150}]"
rosservice call /reasoning "inputs: [{name: 'Prova', value: 150}]"
rosservice call /reasoning "inputs: [{name: 'Input', value: 150}, {name: 'Prova', value: 189}]"
rosservice call /reasoning "inputs: [{name: 'Input', value: 189}, {name: 'Prova', value: 150}]"
rosservice call /reasoning "inputs: [{name: 'Input', value: 350}, {name: 'Prova', value: 400}]"