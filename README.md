Behavior Trees for the SARAFun project
=====

This repository contains code that showcases a potential application of behavior trees.
It provides:
*  A modified version of the [ROS-Behavior-Tree](https://github.com/miccol/ROS-Behavior-Tree) package by Michele Colledanchise. The main changes are
    * Support for the creation of a Behavior Tree from an input file
    * Inclusion of the classes ```ActionTemplate``` and ```ConditionTemplate```, and their exposure as libraries.
*  A ```sarafun_tree``` ROS package. It shows a possible implementation of the ```ROS-Behavior-Tree``` package in the context of SARAFun.
*  Dummy packages ```sarafun_manipulation```, ```sarafun_generic_al_server``` and ```sarafun_assembly```, that implement [actionlib](http://wiki.ros.org/actionlib) servers for interacting with the behavior tree.

Instalation
-----
