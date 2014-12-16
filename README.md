GPS Signal Following - Mobile Robots miniproject
================================================


Goal
----
The quadrotor has to follow another quadrotor (refered to as target).
The target sends its GPS data each 4 seconds.


Approach
--------
The idea is to use a Kalman filter as predictor to predict the target's
trajectory. The prediction is fed into a PID position controller to improve the
quadrotor's responsiveness.


### Kalman predictor
The Kalman implementation for our case is documented
[here](Documents/Notes/SalahNotes.pdf).

It runs the following steps:

1. Kalman prediction steps: the trajectory is predicted using a constant
acceleration motion model

2. If there is a new measurement
  * Estimate acceleration with Bézier interpolation
  * Run the Kalman correction steps: this accounts for the new measurement
  received to correct the prediction

3. Return the state estimate as waypoint prediction


### PID position controller
This takes the predictor's output (waypoint prediction) and apply a PID using
the already implemented PID controller.

The role of this PID is to project the prediction to a new waypoint that is
further so the quadrotor moves faster.


Repository structure
--------------------
* [Code](Code) contains all the source files. We only worked on
  * [kalman_predictor.c](Code/Library/control/kalman_predictor.c)
  * [kalman_predictor.h](Code/Library/control/kalman_predictor.h)
  * [track_following.c](Code/Library/control/track_following.c)
  * [track_following.h](Code/Library/control/track_following.h)
* [Documents](Documents) contains some papers that inspired our work and also
some notes we wrote in treating the problem (see directory
[Notes](Documents/Notes))
* [Presentation](Presentation) contains the presentation in PowerPoint format
