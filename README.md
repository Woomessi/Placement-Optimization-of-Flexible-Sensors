# Placement Optimization of Flexible Sensors for Human-Robot Collaboration
## Abstract
Flexible proximity sensors mounted on robot arms boost obstacle detection in human-robot collaboration (HRC). However, most of the flexible sensor placements lack further analysis to exploit the flexibility, leading to an inefficient and overpriced sensing system. 

In this work, we propose a systematic method to optimize the placement of the flexible proximity sensor for HRC. To prepare for the optimization, the geometric model of a flexible sensor is built.

<img src="img/fig_photo.png" title="Geodesic model of a flexible sensor's mounting location.">


An evaluation metric for the detection ability is established. 

<img src="img/fig_mc.png" title="Monte Carlo simulation case for the successful detection ratio.">


Based on a global search algorithm, we obtain the optimized sensor placement with a sufficient detection ability and a minimum number of sensors. 

<img src="img/framework3.png" title="Outline of the proposed placement optimization method.">


An experiment was conducted to verify the reliability of the method. 

<img src="img/fig_exp.png">


The comparison between the optimized placement results and the conventional ones indicates that the proposed method could achieve better detection performance with much fewer sensors. This method also takes the flexibility into account by customizing the placement for different tasks. 

<img src="img/box_1-4.png" title="Box plots of the results.">


Emphasizing the merits of flexibility, this paper provides new insights into the application and the design of the flexible sensor for HRC. 

## Details of codes
Branch `release` contains the programs of the proposed method. It runs on at least MATLAB R2016b. Please modify the file path in the `addpath( )` and `stlread( )` functions according to your download path. `geodesic_generation.m` is the straightest geodesic generation algorithm, `detection_simulation.m` is the simulation for the detection process, and `bayesian_optimization.m` is the specific optimization algorithm.  

Branch `experiment` contains the ROS workspace of the detection experiment implementation. Sensors transmit the data via serial ports and a ROS subscriber receives these data and processes them.
