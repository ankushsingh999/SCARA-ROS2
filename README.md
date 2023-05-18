# Control of SCARA using PD controller on ROS2

**Created** and developed a Model of SCARA (URDF) on ROS2 to be simulated on GAZEBO.

![image](https://github.com/ankushsingh999/SCARA-ROS2/assets/64325043/a1cdc903-2750-4766-bdbc-49f7d155983c)


**Forward Kinematics:** Implemented a forward kinematics node that:
1) Subscribes to the joint values topic and reads them to gazebo simulator
2) Calculates the end effector pose
3) Publishes the pose as a ROS topic

Test Cases:
Case 1 - [30 deg, 30 deg, -0.3] == [ 0.52, 0.52, -0.3] in radians 

![image](https://github.com/ankushsingh999/SCARA-ROS2/assets/64325043/ccde1ccb-f5c5-4d42-811d-d8aee5dd65f2)

Case2: [60 deg, 60 deg, -0.7] == [ 1.042, 1.042, -0.7] in radians

![image](https://github.com/ankushsingh999/SCARA-ROS2/assets/64325043/b96124cb-d5a9-44ff-92df-add61c526206)

Case 3 [90 deg, 90 deg, -1] == [ 1.57, 1.57, -1] in radians

![image](https://github.com/ankushsingh999/SCARA-ROS2/assets/64325043/720f7331-cb83-42c0-a6d3-48eb5a9e2236)


**Inverse Kinmeatics:** Implemented an inverse kinematics node that has a service client that takes a desired pose of the end effector from th euser and returns joint positions as aresponse.

![image](https://github.com/ankushsingh999/SCARA-ROS2/assets/64325043/bf96bd2f-b9b8-4f2a-a0ca-f792879b3dbd)

# Controlling the joints

Implement a position controller for your robot joints. The package will read the joint values from the Gazebo simulator, receive a reference value for the joints through a service, and publish joint efforts (continuously with high sampling rates) to make the joints move to these locations.
Tuned the PD gains. Do my best to have fast convergence with minimal overshoot. 

For three different sets of joint position references, recorded the reference positions and current positions of the joints in a text file for a period of time and visualized them.
 
 Test case: 
 
![image](https://github.com/ankushsingh999/SCARA-ROS2/assets/64325043/f9c72a7e-093c-4a61-9197-6a39d857764d)
![image](https://github.com/ankushsingh999/SCARA-ROS2/assets/64325043/92f25f9f-8432-49ae-9ee0-f28a668b21bb)

# Moving the robot in a linear path 

Implemented a node with two services. One takes joint velocities and converts them to end effector velocities, and the second one takes end effector velocities and converts them to joint velocities.Designed velocity controllers for all the joints.

Gave a constant velocity reference in the positive ‘y’ direction of the Cartesian space. Converted this velocity to the joint space velocities using my Jacobian and fed it as a reference to your velocity controllers.


![ezgif com-video-to-gif (5)](https://github.com/ankushsingh999/SCARA-ROS2/assets/64325043/6881ebd1-1c24-4a89-8bcc-5284daa61f13)

![image](https://github.com/ankushsingh999/SCARA-ROS2/assets/64325043/06233573-4573-4aa5-94fd-0d7a4fa5e450)
