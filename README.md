# Recyclable Trash Sorter - 6/3
## Team Members:
LAWS: Liuhao Wu, Ana Rath, William Zen, Suha Chang

# Project Writeup

## Project Description: 
Describe the goal of your project, why it's interesting, what you were able to make your robot do, and what the main components of your project are and how they fit together - please include diagrams and gifs when appropriate

- The goal of the project is to have the robot act as a trash sorter. The robot should be able to pick up an object, identify if it is metal or not depending on the clicker sensor, and accordingly place it in the correct “pile of trash” at the corresponding AR tags. 

- Here is a diagram of what we are trying to accomplish.

<img width="544" alt="Screen Shot 2022-05-27 at 2 03 23 AM" src="https://user-images.githubusercontent.com/65791750/170648626-fc0493ca-0857-49ee-b86a-28f2c2c7aea9.png">

- This topic is interesting to us because our project is inspired by the amount of unsorted trash and wasted plastics in the world. Sorting trash is also not the most appealing job for humans, so having a robot do this task would be helpful. The amount of pollution caused by plastic is very detrimental to the future generations and so our robot aims to be able to sort recyclable materials.
- We were able to make our robot recognize and pick up colored batons that either were just made of paper (representing soft, non-metal objects) or paper with cardboard behind them (representing hard, metal objects) by having it go close to the object with proportional control, stop at a fixed distance, and specify a constant xyz location for the arm to move to in our inverse kinematics algorithm to solve for the joint angles that would allow it to pick up the object. Then a clicker sensor classified if it was metal or nonmetal and allowed us to place the object in front of the corresponding AR tag. 

### Our main components and how they fit together:
- **Robot perception - object recognition:** We used color detection with camera images to first have the robot recognize our set of colored “trash” objects and then proportional control to go towards the object. 
- **Picking up the object using inverse kinematics:** Then, we used inverse kinematics (using the gradient descent optimization algorithm) to solve for joint angles that will make the robot arm move towards a specified goal position in real XYZ space that is where the object is located so that it could pick it up.
- **Classifying the object as metal or non-metal depending on clicker sensor readings:** We integrated a clicker sensor and attached it to the gripper of the robot arm. It sends the message that it is clicked when holding a hard (metal) object and unclicked when holding a soft (non-metal) object. 
- **Robot perception - AR Tag recognition:** Now that the object is picked up, and is classified as metal or non-metal, it uses camera images to find the corresponding AR tag to drop the object in front of. 
  - AR tag 1 represents the “recycling bin”, or where all non-metal objects should be placed. 
  - AR tag 3 represents where all metal objects should be placed. 
 
## System Architecture: 
Describe in detail the robotics algorithm you implemented and each major component of your project, highlight what pieces of code contribute to these main components

- **Robot perception**: This section primarily made use of camera and scan information. We had the robot rotate and recognize different colored batons. As described earlier, the batons act as placeholders for our metal/non metal objects. The colors of the batons were not relevant, we just used different colored batons so our robot could recognize the baton. We then averaged the color pixels to find the center of the object, and used proportional control to make the robot move towards the object. We then used the LiDAR data to determine an accurate stopping distance for the robot, away from the object, depending on a certain tolerance level found via testing. 
  - Location: function find_color, scan_callback, image_callback
- **Clicker sensor and sensor integration**: #TODO (NOTE: Liuhao will fill in more details of actually implementing and integrating the sensor, but seniors are just writing the basics of this component with regards to how it fits together with other robot perception). 
  - We subscribe to the ‘/sensor_state’ robot topic that holds the state of the button click in the illumination attribute of this topic’s message. The value is 2 if the sensor is unclicked (which occurs when the gripper holds paper/soft/non-metal objects) The sensor returns 1 if it is clicked (for hard/metal objects). The return value from the sensor allowed us to classify the object as metal or non-metal and we changed a self.is_metal attribute in the sensor state callback function depending on the result. The AR tag recognition code sees the value of this attribute to know what corresponding tag to look for. 
  - Code location: find_tag, button_callback
- **Inverse kinematics**: Instead of solving for a closed-form set of equations that yield the correct joint angles to make an end effector reach a desired xyz position in space, we used the gradient descent optimization algorithm to solve for the joint angles in a numerical, iterative manner. We used this resource to help write the algorithm. 
  - https://www.alanzucconi.com/2017/04/10/gradient-descent/
  - https://www.alanzucconi.com/2017/04/10/robotic-arms/

- Below is an explanation of the algorithm/code. 
1. We obtained the current end effector (the gripper) xyz location from 4-DOF forward kinematics equations. The 4-DOF FK equations were obtained from the paper linked below, and we used the robot arm specifications from the diagram below to get the lengths of the arm components between the four joints. 
https://www.researchgate.net/publication/279201859_Kinematics_Modeling_of_a_4-DOF_Robotic_Arm
- Here is a diagram showing the four joint angles and the lengths used in calculating the FK equations. 

<img width="609" alt="Screen Shot 2022-05-27 at 2 11 26 AM" src="https://user-images.githubusercontent.com/65791750/170650230-c141e19e-c613-4bd8-bd40-c0632eb6d9b5.png">

- However, some transformations had to be performed to match the FK calculations from theoretical equations to the setup of the robot. 
  - In the robot arm, the joint angles are all equal to 0 radians when it is in this upside-down L-shaped configuration as shown above in the diagram. However, in theoretical FK equations we usually assume that when the joint angles are all 0, the arm is completely straight (not in this bent configuration). Therefore, we had to account for this addition of 90 degrees when considering the joint angle values attained by the robot before plugging them into the FK equations, as well as other transformations between the arm angle specifications (e.g. what direction is considered to be a positive vs. negative angle for the turtlebot arm system?) when inputted into the FK equations. 
  - Also, we assumed that the connection between joint 2 and joint 3 is direct, even though there is an arm segment and perpendicular block (as indicated by the blue asterisk) between those two joints. Therefore, the robot arm in its initial state (all joint angles = 0) is assumed to actually be in the shape drawn by the pink lines (which represent l1, l2, l3, l4) instead of the perfectly bent-L shape. 
  - We also had to figure out offsets to x, y, and z coordinates that needed to be accounted for to calculate the correct position of the end effector in space. For instance, through experimentation with the ROBOTIS GUI we found that z = 0 m on the ground of the robot, so we had to measure the height of the turtlebot up to the first joint and fine tune our parameters.
  - Code location: get_current_location
2. Then we performed gradient descent optimization to update the joint angles to ones that minimize the distance between current xyz and goal xyz (the error). We then move the arm once the algorithm has converged and we have found the right set of joint angles that minimize this error within a set threshold. 
  - The main updates in the gradient descent algorithm are performed in either the function do_gradient_descent in old_task.py or the run function in ik_demo.py. In these main updates, first the Euclidean distance between the current and goal position was found with the function get_joint_dist. If it was above a threshold distance value, we performed a gradient descent update on each of the joint angles by finding the partial gradient (slope of the error) when changing a given angle by a small amount specified by the variable delta (located in the partial_gradient function). We multiplied that partial gradient by a variable tau (the learning rate, controls how much we should modify the angles) and subtracted the current joint angle by that product to make the joint angle value change in a direction such that the error will travel down the gradient towards a minimum value. We also make sure that a given angle does not exceed its physical limits on the robot and thus give an invalid command for the arm by clamping them within their range (specified by attributes self.angle_min or self.angle_max) in the clamp_angles function. After this update is performed for each angle, we check again if the distance between the current xyz and goal xyz is less than threshold. If it is, then we move the arm according to those joints. If not, we repeat this whole gradient descent update. 
   - Code locations: specified in the description above
 
## Execution: Describe how to run your code, e.g., step-by-step instructions on what commands to run in each terminal window to execute your project code.

- To run the code for ik_demo.py (just showing inverse kinematics functionality)
  - 1st terminal: roscore 
  - 2nd terminal: SSH into robot, set_ip, bringup 
  - 3rd terminal: roslaunch turtlebot3_manipulation_bringup turtlebot3_manipulation_bringup.launch
  - 4th terminal: roslaunch turtlebot3_manipulation_moveit_config move_group.launch
  - 5th terminal: rosrun recyclable_trash_sorter ik_demo.py

- To run the code for our full robot perception demo:
  - 1st terminal: roscore
  - 2nd terminal: SSH into robot, set_ip, bringup
  - 3rd terminal: SSH into robot, set_ip, bringup_cam
  - 4th terminal: roslaunch turtlebot3_manipulation_bringup turtlebot3_manipulation_bringup.launch
  - 5th terminal: roslaunch turtlebot3_manipulation_moveit_config move_group.launch
  - 6th terminal: rosrun image_transport republish compressed in:=raspicam_node/image raw out:=camera/rgb/image_raw
  - 7th terminal: rosrun recyclable_trash_sorter recyclable_trash_sorter.py

### Demo:
- **ik_demo.py**: This demo shows the functionality of the inverse kinematics algorithm. We specify a list of seven goal positions in xyz space that the end effector should move to. It first moves outwards, and then draw a star shape in the next 6 locations (starts from lower left corner, then apex corner of star, lower right corner, middle left corner, middle right corner, then lower left corner again).

https://user-images.githubusercontent.com/65791750/170651352-a79440ff-4127-45ca-a5ea-d81d75b8196a.mp4

- **recyclable_trash_sorter.py**: This demo shows how all the components come together to have the robot properly recognize, pick up, classify objects, and put it in front of the right AR tag. 


https://user-images.githubusercontent.com/59663733/171243806-2e74bf01-49c0-4cc1-9ad8-e95c34526cae.mp4


## Challenges, Future Work, and Takeaways: 
These should take a similar form and structure to how you approached these in the previous projects (1 paragraph each for the challenges and future work and a few bullet points for takeaways)

### Challenges:
- One of the big initial challenges we had was setting up the sensor and integrating it with the system. We ended up settling on using the clicker sensor because it outputted a binary value that made it easier to work with. 
- Another challenge we had was implementing the forward kinematics step which was necessary for running the gradient descent for inverse kinematics. It took quite a bit to understand how the theoretical kinematics equations did not match up to the robot setup, and use trigonometry to think about how to adjust the angles of the turtlebot setup. It also took a while to figure out through testing all of the xyz planes/positions relative to the robot because the GUI was pretty unreliable, but it was ultimately really important for visualizing the axes and how changing each joint angle changes the xyz coordinates in concerted ways.

### Future work:
- For future work, we could explore converting the camera coordinates when the camera sees an object to real life coordinates so that we could have our IK algorithm solve for joint angles to reach different goal_locations depending on where the object xyz position is (instead of having the robot move to the same place relative to the object every time and feed in the same goal_location for the IK solver). This could be done with using a checkerboard pattern and OpenCV libraries that compute the transformation matrices between real world coordinates of checkerboard corners and the image coordinates of those detected corners on the screen (similar to how the cyborg robot dog group performed those transformations I believe). 
- For another future work, we could also explore more complicated optimization algorithms or versions of gradient descent and compare how long they take to converge or whether they are better at getting closer to the goal position (distance thresholds between goal and current xyz position can be set much lower without too large of an increase in convergence time). 
- Sensor: since we did not have access to percise metal sensor, the robot is currently using a digital button to differentiate cardboard as metal (button clicked) and soft paper as other stuff (i.e., button unclicked). In the future, it would be more intersting if we can have better and more precise sensor that can differentiate different kinds of materials so we can actually make this preliminary idea into a real trash sorter.

### Takeaways
- Connecting a sensor to the turtlebot involved a lot of manipulation on the hardware and software side. Though Liuhao will likely expand on sensor-related points, as a team figuring out how to start research from knowing nothing at the beginning about sensor implementation or IK-solver things was a valuable experience that taught us to be resourceful in exploring different references and interpreting diagrams to understand trigonometry or other concepts. It taught us to be more independent in the process of thinking about robotics projects, which was pretty cool in giving us the confidence to explore different ideas on our own even after the end of this course.
- Using optimization methods for figuring out inverse kinematics equations is a good method to use when the number of degrees of freedom becomes too large and finding closed-form solutions is not possible. Learning about how to implement methods like gradient descent for a robotics context and optimizing parameters to find the right conditions for convergence might be helpful in the type of later research I (Suha) might want to do in the future with robotic prosthetic limb/hand manipulation that requires many DOFs to consider. Working out the trigonometry equations for the forward kinematics aspect was harder than expected and gave a lot of practical experience with matching any theoretical equations to practical robotic arms setup in different configurations. 


## Proposal 5/11
[Google Doc](https://docs.google.com/document/d/1U5GDX519xxsTQEZ-CdnZdTD-etKLyl8Cw83zIqQJ3U4/edit?usp=sharing) 


### [Installing Arduino](https://emanual.robotis.com/docs/en/parts/controller/opencr10/#install-on-linux)
