# Hybrid Localization for UAV-based Charging of Wireless Sensor Networks


## Matlab directory

All source code © 2019 Paul Durham, School of Computer Science, Carleton University.

ESDPrun.m
Main program for cooperative localization simulations.

pitile.m
Main program for original PI simulations.

pong.m
Main program for augmented PI simulations.

Utilities:

circlereduce.m
GetIntersect.m
Intersect.m
IntersectP.m
LCscan.m
pchg.m
PIscan.m
PIscan1.m
PIscanP.m
TOClocate3.m
TOCscan.m
UAVmove.m


## matlabJava directory

All source code © 2019 Paul Durham, School of Computer Science, Carleton University

WSNode.java
WSNode.class


## COL - Matlab Modules from Computational Optimization Laboratory

All files available from:

https://web.stanford.edu/~yyye/Col.html

Included here:

test5-100.mat
This data file defines the node coordinates of the WSN for all simulations.

Not included here:

ESDP.p
This program file contains code for performing the cooperative localization simulations only.

generateD.p

This program file generates the connectivity graph for the WSN with log-normal fading.
Call is commented out in the PI localization simulations, as the connectivity graph is not currently
used.

SeDuMi 1.3

This directory contains program files for performing semidefinite optimization required by ESDP.p
