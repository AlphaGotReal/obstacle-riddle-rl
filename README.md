## Reinforcement-Learning on an obstacle ridden course

This contains the ros packages to setup the simulation (enviroment) and the agent that runs on the enviroment.
It uses the policy-gradient algorithm which returns a continous velocity (linear and angular action space) by stochastically choosing the action from a predicted mean and standard diveation of a gaussian distrubition.
The reward is divided into three parts with weights which have to be tuned accodringly
- orientation reward:
		Reward based on the orientation of the bot with respect to the goal.
- distance reward:
		Euclidian distance from the goal.
- obstacle reward:
		A reward for maintaining a distance from the obstacles around.

## Setup

```zsh
mkdir -p ~/ws/src
cd ~/ws/src
git clone -b devel 
cd ../
catkin b # or catkin_make
source devel/setup.zsh # or devel/setup.bash
```

## Structure

In the lamp/src there are two files
- skeleton.py contains the pytorch model used for the policy network and the agent class.
- trainer is the python ros node that gets input from simulation, trains the model and saves the weights.

## How to use

To launch the simulation

```zsh
roslaunch simulation sim.launch gui:=false # avoid having gazebo open while the training process
```

To start training
```zsh
rosrun lamp trainer <from-weights> <to-weights>
```

- from-weights : this is the path of pretrained weights you want to use, leave it as "none" if training from scratch
- to-weight : this is the path to where you want to store your trained weights

## Here is the image of the agent 
-- The yellow marker represents the confidence of the action it chose and the blue marker is the goal it has to reach with some tolerance
![alt text](https://github.com/AlphaGotReal/obstacle-riddle-rl/blob/devel/imgs/ss.png?raw=true)
