# CS184/284 Final Project Idea Proposal: Drone Sim

## Title: Realistic 3D drone path simulator in an urban environment

## Summary

A 3D drone flight simulator incorporating open-source data (Google Earth) to path plan in realistic world scenarios. The simulation demo will be built in a free game engine Unreal Engine. For the purposes of this project, we will focus on 1 scenario and spend time comparing different path-planning algorithms to select the best path and add realism to the simulation such as constraints on where the drone can fly (for example, stay within the drone air space and don’t fly too close to buildings). The output would be a video of the virtual camera either following the drone path or on the drone itself that shows a realistic simulated 3D world and path planned around obstacles through certain points in space.

Team Memebers:
Irina Hallinan irina_hallinan@berkeley.edu
Buyi  Geng uyikatarina@berkeley.edu
Tianyun Yuan(184) 7ianyun@berkeley.edu
Xinyu Deng(184) xinyudeng@berkeley.edu

## Problem Description

The idea is to make a 3D animated simulation of a drone flying in a realistic urban setting, such as the city of San Francisco. The simulation loads a realistic 3D scene of an existing place available on Google Earth with obstacles such as buildings and trees (and possibly cars or pedestrians). The challenge is to make the drone path realistic: i.e. follow certain rules and constraints in calculating the course while being fast (as it flies). The project will explore different algorithms for path planning and compare the speed vs accuracy tradeoffs. The game engine will automatically compute the flight path and will display it on the screen as the drone flies. The program outputs a video taken from the point of view of the drone to show a realistically rendered scene. We can tell the drone where to fly (which points of interest it should fly by) and take a video from a specific point of view.

Additionally, we will implement animation such as the drone flying, turning, etc. A virtual camera will follow the path of the drone as it flies or will be mounted on the drone itself.

We will implement a GUI for the player to start, pause, take photos, as well as for placing targets in the scene the drone will fly through.

#### Relevant Topics from Class

- Animation: we will animate the drone flying through the scene on the path we compute.
- Kinematics: we will calculate the path the drone will fly through in 3D such that the targets lie on the path.
- Collision Detection: we will calculate a path that avoids collisions with obstacles and the environment such as the drone doesn’t crash.
- Meshes: we will create 3D world assets like the drone and world scene in Unreal Engine and represent all objects as 3D meshes.

#### Technical Concepts

- Collision-detection can be implemented in different ways. In class, we used Bounding Volume Hierarchy to efficiently calculate if there’s an intersection between a ray and an object. We could use a different, newer collision-detection algorithm. The algorithm will need to be fast enough to run in real-time.
- Path tracing can be implemented in different ways. In class, we used Bezier curves and surfaces to find a smooth path through the start and end points by using control points. We can use a more sophisticated technique to find a smooth path through predefined targets in 3D such that the given constraints are met.
- We will learn how to integrate available Google Earth data into a Unreal Engine simulation.

#### Challenges

The most challenging part of this project would be to achieve path planning and animation that is realistic, fast, and accurate. All team members have little experience with animation, so learning the basics of Unreal Engine and animation in the allotted time will be a challenge.

## Goals and Deliverables

We will create a 3D simulation of a drone flying through a realistic environment. We will be able to set and change parameters of the simulations, such as obstacles to avoid, the speed of the drone, and where the drone should fly by. As a final result, we will provide a video in which we will plot a path the drone will take and then the drone flying on that path. We will set limits on where the targets can be placed. The deliverable will be an interactive program built in Unreal Engine.

We think it is realistic to accomplish building a small simulation like this because it synthesizes many concepts developed in class and class projects. We will not be building the simulation from scratch but rather, we would be using resources available to us in the course as well as publicly available libraries and the game engine.

The final result would be a recorded video of different scenarios with the idea that the simulation can easily be extended to other Google Earth locations.

We would measure success if we are able to create interactive components of the simulation, create a realistic path plan, and animate the drone flying through an interesting scene such as a busy San Francisco block.

#### Questions Planned to Answer

1. How to use Unreal Engine to make a realistic drone flight simulation (how to integrate Google Earth data into Unreal Engine).
2. How to animate the drone and its motion in real-time?
3. How to plan a path the drone should take to avoid collisions and stay within the given constraints?

We hope this simulation helps people flying real drones plan a path. Ideally, we would like to compare our simulated fly path to a real drone (if we can get one).

#### Primary Goals

What we plan to accomplish: a realistic simulation that can inform drone operators on where to fly in a complex environment.

1. Create an interactive 3D scene from data in Google Earth in Unreal Engine.
2. Create the drone that automatically flies through the targets the user sets and stays within the constraints of the simulation (e.g.  drone doesn’t crash).
3. Create a virtual camera that follows the drone as it flies to produce a video footage of the 3D scene.

#### Additional Goal

If we have time, the simulation can be extended to more complex environments, and by adding other realistic simulation effects such as wind, moving cars, drone battery life, etc.

## Schedule

#### 4/6 - 4/12 Week 1:

Setup Unreal Engine, gather Google Earth data, choose a complex scene, and integrate it into the engine as a 3D model.

#### 4/13 - 4/19 Week 2:

Develop basic path planning algorithm that avoids obstacles & passes through certain points given the 3D model of the scene. Compare/combine different algorithms based on speed and accuracy.

#### 4/20 - 4/26 Week 3:

Add more complex constraints such as stay away from buildings, re-calculate the path if a new obstacle appears, account for battery life to return to start without crashing, etc.

#### 4/27 - 5/3 Week 4:

Evaluate how well our simulation does compared to real drones (gather data or find available data/footage to compare with); prepare final deliverables (write-up, video, presentation).

## Resources

#### Resources we would need

- Google Earth data for a specific location (such as San Francisco).
- Unreal Engine to create the simulation.

#### Relevant Research:

[1] Microsoft AirSim built on Unreal Engine simulating drone flight: https://microsoft.github.io/AirSim/
https://github.com/microsoft/airsim 

[2] Collision-avoidance and optimal flight path planning: improved Rapid-exploration Random Tree algorithm: https://www.hindawi.com/journals/ddns/2022/4544499/

[3] Survey of collision-detection algorithms: http://graphics.stanford.edu/courses/cs164-09-spring/Handouts/paper_colldect_survey2.pdf
