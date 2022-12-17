# robot_waiter


A path planning and control method is developed for the scenario of a non-holonomic robot serving 
food in a restaurant. Take a look at [video](video.mp4) for a quick summary. Read the [report](report.pdf) 
provided for a detailed explanation of mathematics behind the code.
 
## Contributers
----------------------------------------------------------------
1. Chinmay Polyaramesh		5135125
2. Stan Zwinkels		4630726
3. Jelmer de Wolde		4705041

## Requirements
----------------------------------------------------------------
* math
* numpy
* scipy
* PIL

* random

* os
* pygame

## Contents

-----------------------------------------------------------------
1. `Main.py
`
2. `create_object_borders.py
`
3. `sample_points.py`

4. `nearest_neighb.py`
5. `edge_check.py`

6. `Dijkstra.py`
7. `splines.py`

8. `PurePursuit_PID.py`

9. `simulation_plot_functions.py`

10. `colour_picking_check.py`


------------------------------------------------------------------



## Usage

Run the scipt `Main.py`. A PyGame windows will be opened. 
First run always takes a while, since borders 
are
 getting plotted and graph gets sampled.

 The location can be selected using mouse click. Map can be 
changed on line 32. Three maps are supplied (A, B and C).


