# robot_waiter

https://user-images.githubusercontent.com/85110793/208250465-d81af172-66f1-4d69-b29f-9b9be89327bc.mp4

## Overview
A path planning and control method is developed for the scenario of a non-holonomic robot serving food in a
restaurant. The path planning algorithm used is PRM*, where a semi-random point sampling algorithm is used. 
Graph search is performed by the Dijkstra algorithm, after which b-splines is used to smoothen the path. 
By using a pure pursuit model in combination with a PID-controller, the serving robot can follow the trajectory
within predefined limits. Finally, the results are discussed, commenting on the performance and limitations of
the system, followed by future recommendations. Take a look at [video](video.mp4) for a quick summary. 
Read the [report](report.pdf) provided for a detailed explanation of mathematics behind the code.
 
## Contributers
1. Chinmay Polyaramesh		5135125
2. Stan Zwinkels		4630726
3. Jelmer de Wolde		4705041

## Install packages
math \
numpy \
scipy \
PIL \
random \
os \
pygame

## Usage
Run the scipt `Main.py`. A PyGame windows will be opened. First run always takes a while, since borders 
are getting plotted and graph gets sampled. The location can be selected using mouse click. Map can be 
changed on line 32. Three maps are supplied (A, B and C).

## Explanation
![page1](docs/0001.jpg)
![page1](docs/0002.jpg)
![page1](docs/0003.jpg)
![page1](docs/0004.jpg)
