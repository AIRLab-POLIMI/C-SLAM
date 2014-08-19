#!/bin/bash

rosservice call /getDependencyGraph | sed 's/graph: //' > ~/classifier.dot 
rosservice call /getReasoningGraph | sed 's/graph: //' > ~/reasoning.dot 

#wait

dot -Tpdf ~/classifier.dot -o ~/classifier.pdf 
dot -Tpdf ~/reasoning.dot -o ~/reasoning.pdf 

#wait

rm ~/classifier.dot
rm ~/reasoning.dot
