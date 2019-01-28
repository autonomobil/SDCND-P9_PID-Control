[//]: # (Image References)

[img1]: ./images/PID_comparison.png "PID_comparison.png"
[img2]: ./images/PID_tuning.png "PID_tuning.png"
[img3]: ./images/Working-of-PID-controller.jpg "Working.png"
___
# SDCND Term 2 Project 9: PID Controller
## PID Controller Project for Udacity's Self-Driving Car Engineer Nanodegree Program
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

This project involves the implementation of a Proportional-Integral-Derivative (PID) controller in C++ to control a vehicle in the [simulator of Udacity](https://github.com/udacity/self-driving-car-sim/releases). The simulator sends values of the Cross-track-error (CTE) to the PID controller (PID) via WebSocket and receives the steering signal ([-1, 1] normalized). [Here](https://github.com/udacity/CarND-PID-Control-Project) you can find the seed project of Udacity.

#### The results can be viewed here(Youtube):
[![result1](https://img.youtube.com/vi/GynEJXByuOo/0.jpg)](https://www.youtube.com/watch?v=GynEJXByuOo)

## Reflection
PID controllers are found in a wide range of applications for industrial process control. They are used in most automatic process control applications in industry. PID controllers can be used to regulate flow, temperature, pressure, level, and many other industrial process variables. PID stands for Proportional-Integral-Derivative. These three controllers are combined in such a way that it produces a control signal. As a feedback controller, it delivers the control output at desired levels. [source](https://www.elprocus.com/the-working-of-a-pid-controller/)

![img3]


The implementation of the controller was rather easy, but tuning of the parameters Kp (Proportional), Ki (Integral) and Kd (Derivative) took some time. At first I started with values which seemed realistic compared to the lessons. I tested each controller on their own.

##### P
* The proportional part of the controller tries to steer the vehicle to the centre line proportional to the negative CTE. Due to inertia and other physical properties, a P-controller is not sufficient, as it starts to oscillate and overload the vehicle from left to right.

##### I
* The integral part of the controller tries to mitigate the effect of a bias towards one direction. This part is the least effective of all and fails almost instantly.


##### D
* The differential part of the controller looks at the change of cte and counteracts accordingly.


I then combined P and D controller, which (with the right parameters) get good results:
* Completing the track without mistakes @max 40mph

I wanted more speed so after that I combined PD-controller with the I-controller with a small Ki parameter just to take out some mistakes. The error term for the I-controller is reset when CTE is close to zero (see ``PID.cpp`` function ``UpdateError``) As you can see in the graphic below, this produces smaller error spikes than no I-controller:
![img1]

The magenta line indicates when the vehicle has left the track and the simulator is reset.

As final parameter ``Kp = 0.15``, ``Ki = 0.003``  and ``Kd = 5.0`` were chosen. The maximal throttle value was set to ``max_throttle = 0.6``. The actual throttle value is calculated with this formula: ``throttle = max_throttle - fabs(steer_value);``. These parameters allowed the controller to finish the track with max 60mph.

These formulas calculate the actual steering value(from -1 to 1) depended on the car speed, which is sent to the simulator:
``double p_val = -(Kp_ - speed/500) * p_error;``

``double i_val = -Ki_ * i_error;``

``double d_val = -(Kd_ + speed/50) * d_error;``

``steering_val = p_val + i_val + d_val;``

More detail can be found in ``PID.cpp`` lines 56-81.

## Comments
* Output data will be generated to the folder ``build``
* If you have Matlab you can visualize the data with the script  ``PID_data_visualizer.m``
* In ***line 140*** in ``main.cpp`` you can change the throttle and thus the maximum speed of the vehicle, now it is set to ``msgJson["throttle"] = 0.8;``, so around 80 mph Maximum.
* In ***line 75*** in ``main.cpp`` you can activate/uncomment the function ``read_parameter`` for reading in the parameters (Kp, Ki, Kd) from the file  **parameter.dat**, so you don't have to recompile the controller every time you change these parameters. Just change parameter.dat, save it and run ``./pid`` in Ubuntu command again.
* In ***lines 112-119*** in ``main.cpp`` a termination of the PID controller and a reset of the simulator are implemented if ``cte > 7``
* I tried to use Matlab's PID tuner, but wasn't really successful. Nevertheless a great experience. Part of the the process of Plant Identification and PID tuning is shown here:
![img2]



## Dependencies

* cmake >= 3.5
* make >= 4.1
* gcc/g++ >= 5.4
* Udacity's simulator

## Setup and Running
These are the suggested steps for Windows setup:

* Follow these [instructions](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) for setting up Ubuntu BASH.
* Download Windows simulator [here](https://github.com/udacity/self-driving-car-sim/releases).
* Open Ubuntu Bash (write following commands to Ubuntu Bash command window)
* ``sudo apt-get update``
* ``sudo apt-get install git``
* ``sudo apt-get install cmake``
* ``sudo apt-get install gcc``
* ``sudo apt-get install g++``
* ``sudo apt-get install openssl``
* ``sudo apt-get install libssl-dev``
* navigate to where you want to clone this repository to, for example:
 ``cd /mnt/c/Users/Bob``
* ``git clone https://github.com/autonomobil/SDCND-P9_PID-Control``
* ``sudo rm /usr/lib/libuWS.so``
* navigate to project folder: ``cd SDCND-P9_PID-Control``
* ``./install-ubuntu.sh``
* navigate to the build folder: ``cd build``
* Execute ``cmake .. && make``
* Launch the **term2_sim.exe** from Windows simulator folder
* Execute ``./pid``
* If you see ``Listening to port 4567 Connected!!!``, it is working
* Press **Start**


All C++ files were modified compared to the [original repository](https://github.com/udacity/CarND-PID-Control-Project):  
