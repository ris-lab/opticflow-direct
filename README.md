# opticflow-direct

Example code accompanying the following paper:

#############################################

A direct optic flow-based strategy for inverse flight altitude estimation with monocular vision and IMU measurements
-------
http://iopscience.iop.org/article/10.1088/1748-3190/aaa2be

Pakpong Chirarattananon<br />
pakpong.c@cityu.edu.hk<br />
http://ris.mbe.cityu.edu.hk/<br />
City University of Hong Kong

Published 20 March 2018 • © 2018 IOP Publishing Ltd
Bioinspiration & Biomimetics, Volume 13, Number 3

#############################################

The main file is "flow_comparison.m" (Matlab). This file calls the python script "compute_optic_flow.py" to calculate the optic flow using the proposed direct method and the traditional LK method. The example image dataset is located in the "images60fps" folder. The measurements from the motion capture system (mocap_data.mat) are used as the ground truth for comparison.

