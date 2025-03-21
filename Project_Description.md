# Project Description
Author: Tony Shilati

## Project Motivation
This past summer, Northwestern was named the lead institution of an NSF Engineering Research Center. This center is called the Human AugmentatioN via Dexterity (HAND) Center and is directed by my PhD Faculty Advisor, Professor Ed Colgate. The center aims to revolutionize the ability of robots to assist with human labor by developing robotic hands capable of dexterous manipulation tasks. Recently, I began a project related to the HAND Center. The goal of my project is to create a test platform for robot fingers so people within the center can rapidly test and characterize prototypes of fingers they have built. 

## Project Implementation
There are many aspects to this finger testbed project: mechanical systems, electrical systems, and software systems. Much of the software for controlling the brushless DC motors I use for my project has already been developed. Still, this test platform will also require software that can convert physical commands, such as a specific finger configuration, into commands for the motors. This test platform will also require that the finger can execute sequences of commands as specified by the user. Through the class project for ME 495: Advanced Programming Concepts, I will createa C++ library that allows users to easily specify a sequence of actions for the robot finger to execute. Since robot fingers are actuated by pulling on strings connected to the finger links, I will call this library Geppetto in reference to the puppet master in Pinocchio. This project will require that I use existing libraries, such as robotics kinematics solver libraries and linear algebra libraries, to determine how much the motors should move based on the commanded finger configuration. I will also have to create an interpreter that reads the commands line by line and executes the commands. If time permits, I will also create a GUI that allows users to program sequences of actions using block programming.

The creation of library is partially dependent on the associated hardware being built, which may not be possible in the available time. Since the problem of driving brushless DC motors has been solved, I will first render a robot finger using a gui. Then I will create the Geppetto library and interface it with the gui to control the motion of the finger. 

My first step in this project will be identifying existing libraries that my library will depend on and adding them to my project git repository. 

## Dependencies

- SFML: C++ cross platform graphics library
- Eigen: C++ linear algebra library

### Finger Parameters
The parameters of the finger, as used in the Dynamics, are shown in the figure below.
![Finger Parameters](media/Finger%20Parameters.png)
