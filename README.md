# Recycable Trash Sorter
## Team Members:
LAWS: Ana Rath, Suha Chang, William Zeng, Liuhao Wu

# Project Writeup
Like in the previous projects, your Github README will serve as your project writeup. Your writeup should include the following:

Please note: Since we have 2 graduating seniors in our group (Suha and Ana), our write up is for the MVP as of now, by the graduating senior deadline. 

## Project Description: 
Describe the goal of your project, why it's interesting, what you were able to make your robot do, and what the main components of your project are and how they fit together - please include diagrams and gifs when appropriate.

## Goal: 
The goal of the project is to have the robot act as a trash sorter. The robot should be able to pick up an object, identify if it is metal or not depending on the clicker sensor, and accordingly place it in the correct “pile of trash” at the corresponding AR tags. 

## Motivation: 
This topic is interesting to us because our project is inspired by the amount of unsorted trash and wasted plastics in the world. Sorting trash is also not the most appealing job for humans, so having a robot do this task would be helpful. The amount of pollution caused by plastic is very detrimental to the future generations and so our robot aims to be able to sort recyclable materials.

## Main components and how they fit together: 
1. Object recognition: TODO
2. Picking up the object using inverse kinematics: We used inverse kinematics for the robot arm to learn to adjust itself depending on the distance away from the object. 
   - We used gradient descent to minimize the distance from the object 
3. Classifying the object as metal or non-metal depending on clicker sensor readings: 
  - We used a clicker sensor
  - This sensor returns 2 if the it is unclicked (representing paper/soft/ non metal objects)
  - The sensor returns 1 if it is clicked (for hard/ metal objects) 
  - The return value from the sensor will help us classify the object as metal (1) or non-metal (1). 
4. AR Tag recognition: 
Now that the object is picked up, and is classified as metal or non-metal, the next task is to find the right AR tag to drop the object in front of. 
  - AR tag 1 represents the “recycling bin”, or where all non-metal objects should be placed. 
  - AR tag 2 represents where all metal objects should be placed. 
5. Dropping the object:
Once the robot has identified which tag it should drop the object at, for the MVP we implemented the robot to drop the object a fixed distance away from the correct AR tag. 
  - For the stretch goal, we will implement inverse kinematics for this step as well. 

## System Architecture: 
Describe in detail the robotics algorithm you implemented and each major component of your project, highlight what pieces of code contribute to these main components
1. Camera and LiDAR detection: 
2. Location: in object recognition
3. Arm movement: Inverse kinematics: 
4. Clicker sensor and sensor integration: 

## Execution: 
Describe how to run your code, e.g., step-by-step instructions on what commands to run in each terminal window to execute your project code.
Roscore 
Bring up 
Run inverse_kinematics.py 

## Challenges, Future Work, and Takeaways:
These should take a similar form and structure to how you approached these in the previous projects (1 paragraph each for the challenges and future work and a few bullet points for takeaways)

1. Challenges:
One of the big initial challenges we had was setting up the sensor and integrating it with the system. We ended up settling on using the clicker sensor because it outputted a binary value that made it easier to work with. **Liuhao can add some more challenges that he faced in this step** 
Another challenge we had was implementing the forward kinematics step which was necessary for running the gradient descent for inverse kinematics. We had to work out the trigonometry involved and the unique distances and angles that were present in how our turtlebot was set up. 

2. Future work:
  - For future work, we could explore converting the camera coordinates when the camera sees an object to real life coordinates so the robot can approach the object by interpreting this. 
  - We could also explore using inverse kinematics for dropping the object, because in this iteration of our project, we only implemented inverse kinematics for picking up the object. 
3. Takeaways:
  - Connecting a sensor to the turtlebot involved a lot of manipulation on the hardware and software side. 
  - Although the logic for gradient descent itself is understandable, working out the trigonometry equations for the forward kinematics aspect was harder than expected and included quirks specific to our turtlebot that we did not expect and had to work through. 


## Proposal 5/11
[Google Doc](https://docs.google.com/document/d/1U5GDX519xxsTQEZ-CdnZdTD-etKLyl8Cw83zIqQJ3U4/edit?usp=sharing) 


### [Installing Arduino](https://emanual.robotis.com/docs/en/parts/controller/opencr10/#install-on-linux)
