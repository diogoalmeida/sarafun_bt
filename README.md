Behavior Trees for the SARAFun project [![Build Status](https://travis-ci.org/diogoalmeida/sarafun_bt.svg?branch=master)](https://travis-ci.org/diogoalmeida/sarafun_bt)
=====

This repository contains the software the implements the Behavior Trees framework used in the [SARAFun EU project](http://h2020sarafun.eu/).
It relies on a modified version of the [ROS-Behavior-Tree](https://github.com/miccol/ROS-Behavior-Tree) package by Michele Colledanchise, which adds support for using the action and condition templates as libraties (found [here](https://github.com/diogoalmeida/ROS-Behavior-Tree)).

The framework wraps the ROS-Behavior-Tree package (dubbed 'the engine') in order to allow generating behavior trees from input files.
It also decouples the action implementations from the tree structure, by assuming that all the system actions are implemented as [actionlib](http://wiki.ros.org/actionlib) servers.
From the action definitions, the framework initialized BT actions as actionlib clients which, once tick'ed, will send a proper actionlib goal to the server implementing the action.

Additionally, a generator package allows for trees to be generated through the aggregation of pre-defined subtrees, representing a high level task.
Given a list of tasks to be executed, the package generates a tree description connecting the actions that implement each task.

![framework](https://raw.githubusercontent.com/diogoalmeida/sarafun_bt/master/docs/images/sarafun_bt_framework_generic.png)

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

### Dependencies
The provided packages depend on [yaml-cpp](https://github.com/oftc/yaml-cpp.git).
The framework currently depends on the `sarafun_msgs` package in order to use its tree generator. You can get a reduced version of the package [here](https://github.com/diogoalmeida/sarafun_msgs.git).

### ROS packages
If you do not have a catkin workspace, create one by doing
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
This will download the required packages for running the framework. Compile the packages by doing ```$ catkin_make -DCMAKE_CXX_COMPILER=/usr/bin/g++-4.9 install``` at the root of your workspace. After compiling, do not forget to source the workspace:
```
$ source devel/setup.bash
```

### YAML parser
These packages depend on yaml-cpp, version 0.5.3, which can be obtained [here](https://github.com/jbeder/yaml-cpp/tree/yaml-cpp-0.5.3)

Services
----
The tree client provides services to start, stop and restart a tree. When starting, the tree client will parse the input file and generate a tree based on it.
Restarting a tree stops the currently running tree, parses the input file and generates a new one based on it. The provided services are:
* ```/sarafun/start_tree```
* ```/sarafun/stop_tree```
* ```/sarafun/restart_tree```

The start service requires a file path to be given. This will be used by the json parser to load the appropriate tree.

Running the demo
----
This package provides two demos:
*  The first demo is used to illustrate the BT functionality. It runs the tree defined in ```sarafun_tree/data/example_demo1.json```, and each action will prompt the user to enter the key "p" for preempting the action, "a" to abort it and anything else to succeed. Preempting or aborting an action results in a ```FAILURE``` state in the behavior tree. Edit the ```sarafun_tree/data/example_demo1.json``` file in order to test different tree configurations!
To run this demo, type
```
$ roslaunch sarafun_tree run_sarafun_bt_demo.launch demo1:=true
```

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
				"selector1"
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
			"name": "action_1"
		},
		"action2": {
			"id": "action2",
			"type": "Action",
			"name": "action_2"
		},
		"action3": {
			"id": "action3",
			"type": "Action",
			"name": "action_3"
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

Tree visualization
----
The BT engine provides a basic openGL tree visualization. An arbitrary tree may not be rendered adequately, in which case the user can use the keyboard to ajust the tree:
*  The directional arrows on the keyboard will translate the tree on the visualization window;
*  Page up will zoom the tree in, page down will zoom out.
