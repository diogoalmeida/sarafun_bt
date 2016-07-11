Behavior Trees for the SARAFun project
=====

This repository contains code that showcases a potential application of behavior trees.
It provides:
*  A ```sarafun_tree``` ROS package. It shows a possible implementation of the ```ROS-Behavior-Tree``` package in the context of SARAFun.
*  Dummy packages ```sarafun_manipulation```, ```sarafun_generic_al_server``` and ```sarafun_assembly```, that implement [actionlib](http://wiki.ros.org/actionlib) servers for interacting with the behavior tree.

It depends on a modified version of the [ROS-Behavior-Tree](https://github.com/miccol/ROS-Behavior-Tree) package by Michele Colledanchise. The main changes are
* Support for the creation of a Behavior Tree from an input file
* Inclusion of the classes ```ActionTemplate``` and ```ConditionTemplate```, and their exposure as libraries.

The modified package is found [here](https://github.com/diogoalmeida/ROS-Behavior-Tree).

Instalation
-----

### Compiler
There is an [issue](https://github.com/nlohmann/json/pull/212) with gcc 4.8 that prevents compiling the json parser used in this project. So first, make sure you have a more recent version of gcc. In ubuntu this can be achieved by doing
```
$ sudo add-apt-repository ppa:ubuntu-toolchain-r/test
$ sudo apt-get update
$ sudo apt-get install gcc-4.9 g++-4.9
```
and your system should now be correctly configured.

### ROS packages
If you do not have a created catkin workspace, create one by doing
```
$ mkdir ~/catkin_ws
$ mkdir ~/catkin_ws/src
$ cd ~/catkin_ws
$ catkin_make
```
Otherwise skip this step. Go to the src folder of your catkin workspace and introduce the following commands
```
$ wstool init
$ wstool merge https://raw.githubusercontent.com/diogoalmeida/sarafun_bt/master/.rosinstall
$ wstool update
```
This will download the ```ROS-Behavior-Tree``` package, as well as the SARAFun packages required for running the demo. Compile the packages by doing ```$ catkin_make -DCMAKE_CXX_COMPILER=/usr/bin/g++-4.9 install``` at the root of your workspace. After compiling, do not forget to source the workspace:
```
$ source devel/setup.bash
```

Running the demo
----
This package provides two demos:
*  The first demo is used to illustrate the BT functionality. It runs the tree defined in ```sarafun_tree/data/example_demo1.json```, and each action will prompt the user to enter the key "p" for preempting the action, "a" to abort it and anything else to succeed. Preempting or aborting an action results in a ```FAILURE``` state in the behavior tree. Edit the ```sarafun_tree/data/example_demo1.json``` file in order to test different tree configurations!
To run this demo, type
```
$ roslaunch sarafun_tree run_sarafun_bt_demo.launch demo1:=true
```
*  The second demo showcases preemption and conditions. We modified the [online motion generator provided CERTH](https://github.com/auth-arl/sarafun_online_motion_generation) so that it implements an actionlib server. The modified code can be found [here](https://github.com/diogoalmeida/sarafun_online_motion_generation) and illustrates how an action should take into account that it can be preempted.
To run this demo, type
```
$ roslaunch sarafun_tree run_sarafun_bt_demo.launch demo2:=true
```

This will run a simple tree with 5 actions. Each action will command the robot to move to a different position. The two main subtrees execution depends on the output of the simple condition in the first one. This will be true for the first x ticks on the tree (defined in the launch file under ```/IsSimple/count_limit```. When the condition becomes false, the running action will be preempted, and the second subtree is executed, where the first action under the selector asks the robot to achieve an unreachable goal, and is thus preempted after the timeout period has expired. 

Creating a different tree
----
The behavior tree package takes a json file as input in order to generate the tree. The file is located in ```sarafun_tree/data``` and its name is given by the ROS parameter ```/sarafun/bt/file```. If the parameter is not set, it will use the default name ```example.json```.

The file defines the tree by specifying its root and a list of nodes. A node can be a flow control node or a leaf node. It is defined by its ```id```, ```type``` and ```name``` tags and, for the flow control nodes only, by a list of children (```children```) which contains the ```id```s of the children nodes.
The ```id``` tag should be unique, and it is used to refer to a particular instance of the behavior tree node. ```type``` determines the what is the node functionality (see [below](#Currently-supported-node-types)). Finally, the ```name``` tag is currently only used by the leaf nodes. It specifies the actionlib server name that the concrete action implementation uses and it is used by the behavior tree in order to call the server.

A simple example is as follows
```json
{
  "root": "sequence1",
  "nodes": {
    "sequence1": {
      "id": "sequence1",
      "type": "SequenceStar",
      "name": "SequenceStar",
      "children": [
        "action1",
        "selector1",
      ]
    },
    "selector1": {
      "id": "selector1",
      "type": "Selector",
      "name": "Selector",
      "children": [
        "action2",
        "action3"
      ]
    },
    "action1": {
      "id": "action1",
      "type": "Action",
      "name": "action_1",
    },
    "action2": {
      "id": "action2",
      "type": "Action",
      "name": "action_2",
    },
    "action3": {
      "id": "action3",
      "type": "Action",
      "name": "action_3",
    }
    }
  }
}
```

### Currently supported node types
There are four currently supported flow control nodes and two possible leaf node types, which are defined in the json file by the ```type``` property:
*  **Sequence**: The sequence node ticks its children sequentially every time it receives a tick from its parent node. Returns ```FAILURE``` as soon as one of its children returns ```FAILURE```, or ```SUCCESS``` in case all of its children are successful
*  **Selector**: The selector node ticks its children sequentially every time it receives a tick from its parent node. Returns ```SUCCESS``` as soon as one of its children returns ```SUCCESS```, or ```FAILURE``` in case all of its children are successful
* **SequenceStar**: The same as Sequence, but with memory, that is, every time it receives a tick from its parent, it will skip tick'ing all the children that previously returned ```SUCCESS```
*  **SelectorStar**: The same as Selector, but with memory, that is, every time it receives a tick from its parent, it will skip tick'ing all the children that previously returned ```FAILURE```
*  **Action**: An action will execute some well defined program when it is tick'ed, and will return ```SUCCESS``` or ```FAILURE``` at the end of its execution, according to the implementation. While executing, it will return ```RUNNING```
*  **Condition**: A condition is similar to an action, but it is meant to return ```SUCCESS``` or ```FAILURE``` as soon as it tick'ed, and should be used to help the behavior tree logic by verifying some condition.
