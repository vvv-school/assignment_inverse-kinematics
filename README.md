Assignment on Inverse Kinematics
================================

# Assignment

### Introduction
You are provided with the **3-links planar manipulator** shown in the sketch below,
where the lengths **ℓ<sub>1</sub>**, **ℓ<sub>2</sub>**, **ℓ<sub>3</sub>** are all
equal whereas the tip 𝐸 is identified by the triplet **[x<sub>e</sub>, y<sub>e</sub>, φ<sub>e</sub>]**,
being (x<sub>e</sub>, y<sub>e</sub>) the Cartesian coordinates of the tip and φ<sub>e</sub>
the angle between the link 3 and the x-axis of the root frame.

![kinematics](/misc/kinematics.png)

The manipulator is actuated by means of 3 motors controlled in velocity mode that move
the joints angles represented by **[θ<sub>1</sub>, θ<sub>2</sub>, θ<sub>3</sub>]**.

### Goal
Design a controller by filling in the missing code in the [**src/main.cpp**](src/main.cpp)
module, which implements a suitable Inverse Kinematics algorithm to meet
the following requirements given in descending order of importance:
- **Primary task**: reach the target position with the tip of the manipulator.
- **Secondary task**: align the tip frame along with the target frame.

An example of a successful controller is visible in the animation below:

![robot](/misc/robot.gif)

# [How to complete the assignment](https://github.com/vvv-school/vvv-school.github.io/blob/master/instructions/how-to-complete-assignments.md)
