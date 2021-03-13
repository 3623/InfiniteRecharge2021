Project: Differential Drive Simulation

The project is a simulation of a differential drive robot. It is a 2D, top-down
simulation using models of motor voltage-speed-torque, inertia, friction, and
collisions with fixed objects. The goal is to produce a simulation which is easily
adaptable to test different control methods and algorithms, by including adaptable
graphing, path visualization, and user friendly waypoints.

To run: call main.py

Libraries: Needs no external libraries except for tkinter (included by default)
and cmu_112_graphics.

Shortcuts:
    Arrow keys to move the robot when not in auto driving mode.
        Click on the screen to create a waypoint, and click
          on any already created waypoint to select it.
          Double clicking on the waypoint to make it critical
          (the robot will slow down and stop there, else
          it will just drive through it). The yellow tick
          indicates the direction of the waypoint, and can be
          dragged to change the direction. The speed is
          indicated by the color and controlled by 'w' and 's'.
          The waypoint can be deleted by 'del'.
        To start the robot autodriving, there must be 2 waypoints
          and a path should be shown in white. Then 'a' will start
          and stop autodriving.
        Once the robot has been autdriven, the values for specific
          points on the graph can be found be hovering over it.