Assignment on Inverse Kinematics
================================

# Assignment

### Introduction
You are provided with the **3-links planar manipulator** shown in the sketch below,
whose link lengths **ℓ<sub>1</sub>**, **ℓ<sub>2</sub>**, **ℓ<sub>3</sub>** are all
equal and whose tip 𝐸 is identified by the triplet **[x<sub>e</sub>, y<sub>e</sub>, φ<sub>e</sub>]**,
being (x<sub>e</sub>, y<sub>e</sub>) the Cartesian coordinates of the tip, whereas φ<sub>e</sub>
is the angle between the link 3 and the x-axis of the root frame.

![kinematics](/misc/kinematics.png)

Moreover, the manipulator is actuated by means of 3 motors controlled in velocity that move
the joints angles **[θ<sub>1</sub>, θ<sub>2</sub>, θ<sub>3</sub>]**.

### Goal
Design a controller by filling in the missing code in the [**src/main.cpp**](src/main.cpp)
module, which implements a suitable Inverse Kinematics algorithm to meet
the following requirements given in descending order of importance:
- **Primary task**: reach the target position with the tip of the manipulator.
- **Secondary task**: align the tip frame along with the target frame.

:point_right: The test will verify the capabilities of your controller in different
conditions, assigning **scores** according to the recorded performance, considering
in particular cases where the primary and secondary tasks cannot be achieved
at the same time or even cannot be fully completed.

:warning: In the latter setting, the test will evaluate the **best effort** made to
attain the primary task solely.

An example of a successful control is visible in the animation below:

![robot](/misc/robot.gif)

# [How to complete the assignment](https://github.com/vvv-school/vvv-school.github.io/blob/master/instructions/how-to-complete-assignments.md)
