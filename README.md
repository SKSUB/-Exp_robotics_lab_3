# Exp_robotics_lab Assignment3
## INTRODUCTION
Purpose of this repository is to develop the assignment 3 of Experimental Robotics Laboratory. In this assigment the robot moves between different rooms in the Gazebo simulation. The rooms has aruco markers, robot reads the marker and collects the hint. The robot navigate in the simulation with the help of slam gmapping and move base navigation with the help of laser and fixed goals. And checks for consistency and complete solution. And return to oracle room to check for solution. For the correct solution the simulation ends or the robot navigates again to other rooms from the first.

## SOFTWARE ARCHITECTURE
In order to better understand the software architecture we can utilise the UML component diagram

![cluedo_mapping](https://user-images.githubusercontent.com/82164428/219997455-ae5f1b28-13f5-4b94-b0a2-33d4bc6ebafd.jpg)

The state machine is the main node of the architecture effectively communicates with all other nodes to run the simulation. It request the Oracle node for hint and asks the solution. It sends the move_base action goal to move the robot in the move state. It receives all the IDs from the aruco ros via /marker id. 

And it defines the place to initiate the scene. And it communicate with the ontology interface for adding, updating and consistency check via add_hint service, update_request service and check_consistency service.  

The ontology interface node performs all the querying operation with the armor service with the help of myArmor class.

Movearm node effectively moves the arm no matter the state of the simulation to read the marker ID. Aruco_ros publish the ID to when the aruco marker detected on the camera. 

Oracle node generates the hint and solution of the simulation. The gmapping and Move base navigation to localise the robot and move the robot in the simulation with the help of laser and move base actio goal. 
