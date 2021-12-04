### ME 133A Final Project ###

This is the code base of the ME 133A Final Project. In this project, we make use of a Stewart Platforms to prevent balls from entering a sphere.

We would like to mention that this would not have been possible without the SDF file from https://github.com/daniel-s-ingram/stewart.

## How to Run ##

Clone this repo to your catkin workspace `src/` directory and build it using `catkin`. Run the following commands

```
catkin build stewart
source ~/your_catkin_ws/devel/setup.bash
```

We now need to build the plugin for the ball and the stewart platform. To do that,

```
cd plugin
mkdir build
cd build
cmake ../
make
```

If there are no errors until this point, we will now launch the sphere protecting Stewart Platform.

```
roslaunch stewart stewart.launch
```
