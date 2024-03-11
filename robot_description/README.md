# Robot description for Solo.

## Launching the simulation

Clone the package into the ```src``` folder of a workspace.

And then launch the robot by using command:

```
roslaunch robot_description robot_sim.launch
```

## Launching with arguments

Use the syntax:

```
roslaunch robot_description robot_sim.launch {arg_name_1}:={arg_value_1} {arg_name_2}:={arg_value_2}
```
*Replace {arg_name_x} by argument name and {arg_value_x} by argument value*

Example:

```
roslaunch robot_description robot_sim.launch gui:="false"
```


### Arguments available

- **gui** *(default:"true")* : Toggles the Gazebo GUI on (*true*) or off (*false*)
- **paused** *(default:"false")* : Starts the simulation paused if set to true
