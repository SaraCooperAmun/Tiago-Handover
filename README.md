# Tiago-Handover

This is the MSc project of Sara Cooper, for the MSc in Robotics, Autonomous and Interactive Systems at Heriot-Watt University, Edinburgh (UK), in year 2018-2019, under supervision of Mauro Dragone. It consists of a joint human-robot interaction and psychology study with University of Stirling, alongside Stuart Gow and Dimitrious Kourtis. The main goal was to research how human action planning differed when engaging in a joint-action - such as an object exchange or handover- with a robot from when doing an individual action. For that it uses PAL Robotics Tiago robot and EEG (Electroencephalogram) to detect brain activity. Experiments were conducted with 16 participants.
Sample demo of user study: 
https://youtu.be/rbWynDuErvs
https://youtu.be/w_uDgpDpouc


This repository includes the code used for the development of the project and the user-studies. 

## **Components needed**

### Hardware


-	Amplifier
-	EEG caps: small, medium, large (sizes) depending on participant’s head size
-	Projector, to project cues onto the table
-	Keypad, to detect action onnset
-	Object + aruco marker
-	Ethernet cable
-	3 laptops:
  -	Windows 7 for EEG acquisition
  -	Windows 7 for behavioral analysis
  -	Ubuntu ROS laptop to control Tiago’s actions
-	Tiago robot (PAL Robotics)
-	2 cameras, positioned perpendicularly to the handover and another from the ceiling, facing the participant, for later video annotation
-	Astra Orbbec camera for hand tracking

### Software

-	Matlab for cue presentation and interconnection of EEG and robot (Behavioral analysis laptop). Required toolboxes:

        -	Robotics system toolbox: https://www.mathworks.com/products/robotics.html
        -	Psychtoolbox (http://psychtoolbox.org/)
-	Curry 7  (EEG acquisition laptop)
-	Nuitrack for skeleton tracking (http://download.3divi.com/Nuitrack/doc/Installation_page.html)  (ROS laptop) and nuitrack-ros connection node (https://github.com/shinselrobots/nuitrack_body_tracker)
-	Python + ROS (ROS laptop). Additional packages to install,

        - pal_face_detector_opencv (https://github.com/pal-robotics/pal_face_detector_opencv)

### Other components

-	Syringes
-	GEL
-	Face cream
-	Object (used object is of 25 x 5 x 5.5 pringles box)
-	Aruco marker (tested of ID 582, size 60)


The behavioural analysis laptop must be connected to the LCD projector as well as the EEG acquisition PC through parallel port. Keypad attached for recording action onset as well.  The EEG acquisition laptop is connected to the behavioural analysis laptop and the EEG amplifier. Lastly the Ubuntu ROS laptop is connected to Tiago by ethernet cable, and will access the inner contents of Tiago through ssh (ssh pal@10.68.0.1).

Firstly, input the tiago_handover package into the catkin workspace inside Tiago, and tiago_pc package into the catkin workspace of the Ubuntu ROS laptop.


## **Set-up**


Firstly, input the tiago_handover package into the catkin workspace inside Tiago, and tiago_pc package into the catkin workspace of the Ubuntu ROS laptop. The EEG_Tiago_study.m maltab file should be added to the working directory of the behavioural analysis laptop. The object should be placed hafway between the participant with the aruco marker facing Tiago. Switch on the projector. 


1.	Change in the Matlab script  (EEG_Tiago_study.m) the participant ID, block number (initially 0), shape_id (corresponding to the cue types used). Ensure the ROS connection parameters are adequate (e.j Tiago’s IP address)
2.	Ensure marker size and ID are adequate in the tiago_handover/launch/single.launch file

3.	In the tiago_handover/src/main.py, adjust the participantID, rosbag file path in order to save all the ROS logs. 

## **Main code**

In order to run the complete code:


1.	On the ROS laptop,

    - On PC: roslaunch my_tiago tiago_pc.launch. Runs the following the nodes
    
     - aruco_tranform.py (node /xtion_aruco). Publishes the  transform between /xtion_rgb_optical_frame and the /aruco_marker_frame through topic /xtion_aruco (PoseStamped() message) to enable object tracking, due to not being to perform the look up directly from within Tiago

     - pal_face_detector_opencv/launch/detector.launch file, for face tracking

     - image_view.py node to visualise the tracked person on topic /pal_face/debug



     - On Tiago (ssh pal@10.68.0.1):
      - roslaunch tiago_handover tiago_handovers.launch. Runs the following nodes

      - static_transform_publisher.py (tf package): publishes transform between /aruco_marker_frame and /arm_goal (-0.09 $(arg distance) 0 0.707 -0.707 0 0), indicating the pregrasp position in robot-to-human handover, where distance = 0.3 (default distance from object to pregrasp). Grasp point -0.09 is adjustable

      - static_transform_publisher.py (tf package): publishes transform between /aruco_marker_frame and /place_goal (-0.16 $(arg distance) 0 0.707 -0.707 0 0), indicating the place goal position in human-to-robot handover. Grasp point -0.16 is adjustable.

      - static_transform_publisher.py (tf package): publishes transform between /aruco_marker_frame and /human_goal (-0.16 $(arg distance) 0 0.707 -0.707 0 0), indicating the pregrasp goal position in human-to-robot handover. Grasp point -0.16 is adjustable.
      - single.launch. Runs the aruco marker detector for marker ID 582, size 70. If a different marker is used change in


       - plan_smooth.cpp. Contains the code that applies time parameterization to the waypoints sent by main.py script and executes the cartesian path trajectory with MoveIt. If desired, possible to adjust the eef_step to speed up motion duration (higher step, quicker motion, but more jerky).

       - main.py. Contains the main script to carry out the experiment and robot-to-human as well as human-to-robot handovers

     -On Tiago (ssh): roslaunch tiago_handover backup.launch, includes necessary codes to react if Tiago is not responding correctly.

Input the following in order to return Tiago to resting position after pausing Matlab script (cue presentation),
#2) Stop Matlab script
#1) Return Tiago's gripper to initial position
#3) Restart Matlab script

#Other commands include
#4) Open gripper. Used for instance when the person starts to pull too soon
#5) Open gripper and retreat
#7) Indicate to Matlab action was unsuccesfull

If desired it is possible to include new commands, merely add a new number or expected string input.

2. Run Matlab script on behavioural laptop
3. Switch on the LCD projector, if not done already, and when the initialization screen shows up, press ENTER on keypad to start the experiment

If hand tracking with Astra/Nuitrack is enabled, the only difference is step 1)
Roslaunch my_tiago tiago_pc_camera (running main_astra.py). Aparts from the nodes mentioned, it launches

- nuitrack_body_tracker.launch file, to run hand tracking with Nuitrack
- hand_speed.py to send hand coordinate data across to Tiago

- static_transform_publisher.py (tf package): publishes transform between /base_link and /nuitrack_camera_link  (1.0 -0.5 0.82 1.0 0 0) for a rough estimation on camera location with respect to Tiago (not accurate as camera is not added to TF tree of Tiago)

Sample output in Rviz for hand tracking: https://youtu.be/8yc4a9Qg9nI



## **ADDITIONAL CODES**

Other scripts included in the repository which can be used, but are optional, include.

-	My_tiago/launch/pick_demo.launch (moveit_simple_grasps_test.py). Runs the moveit_simple_grasps sample to determine all possible grasp positions for the selected object, taking as example http://wiki.ros.org/Robots/TIAGo/Tutorials/Pick_and_place/Pick%20%26%20place%20demo
-	My_tiago/launch/pick_demo_real.launch. Runs the modified movit_simple_grasps sample to pick the object from the table and offer it to human (pick_client.py, spherical_grasps_server.py, pick_and_place_server.py)
-	My_tiago/launch/rosbag_launch.launch. Records data of the following topics to a rosbag
  -	/arm_controller/state, used for analysisng Tiago’s arm trajectory
  -	/wrist_ft, to evaluate gripper behavior
-	Camera_human.launch. Same as tiago_pc.launch but runs main_astra.py instead. It shows human-to-robot handover where Tiago takes into account human hand movement
-	Tiago_handover/launch/pick_human.launch (human_grasp_play_motion.py) runs human-to-robot handover using learning by demonstration movements

Other nodes, which can be run through their respective launch files, of tiago_handover package include

-	VelocityProfiles.py, used to test different ways to perform grasping (cartesian path planning, smoothing, end-effector goal specficiation)
-	Record_camera.py, to record hand coordinates and velocity and output it to a rosbag for further analysis. Must be run alongside tiago_pc_camera.launch file in order to record


