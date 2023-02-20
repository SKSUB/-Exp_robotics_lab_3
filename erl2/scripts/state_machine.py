#! /usr/bin/env python3

'''
.. module:: place
   :platform: Unix
   :synopsis: Node for simulating the cludo game steps. 
.. moduleauthor:: SUBRAMANI SATHISH KUMAR

This node for simulating the cludo game steps. 


This is the main node of the architecture, it implements the finite state machine. The robot will be in the one of these three states, (move, check consistency and solution).

MOVE state: It is moving state class, in which the robot is move from one room to another in the continuous to collect all the hints. It checks whether the robot should move in a oracle room or other room to check the soution or to collect the hints. 

CHECK CONSISTENCY state: In this state all the aruco IDs are received and the corresponding hints are collected from the /oracle_hint. Then it stores the hints and check for consistency and if they are consistent it ask the robot to move to the oracle ro0m to check the solution or it ask to move to other room otherwise and also remove the hint from the ontology if they are inconsistent.

SOLUTION state: This is the state where the robot checks the solution received via /oracle_solution service, if the solution is correct the game ends or the game is continues i.e to the move state.


Subscriber:
    /marker_id: To get the ID of a marker when it's detected by the camera

Client: 
    /oracle_hint: ask the hint

    /ontology_interface/add_hint: to send the new hint to the ontology interface. 

    /ontology_interface/update_request: for adding the hints and perform reason on ontology interface.
    
    /ontology_interface/check_consistency: to check the consistency
    
    /oracle_solution: solution ID of the cluedo game 
    
    /ontology_interface/ask_solution: to query

ActionClient: movebase to move the robot. 

'''

from time import sleep
import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import smach
import smach_ros
from classes.place import Place
import actionlib
from std_msgs.msg import Int32
from erl2.msg import ErlOracle
from erl2.srv import MarkerRequest, Marker, Hint, HintRequest, Consistent, ConsistentRequest, Update, Oracle, OracleRequest, Solution, SolutionRequest


pub_move_base=None
''' 
Initialize the action client for performing the motion of the robot via move_base action 

'''
sub_ID=None
''' 
Subscriber to /marker_id topic

'''
client_ID_msg=None
''' 
Initialize the /oracle_hint service client

'''
client_add_hint=None
''' 
Initialize the /ontology_interface/add_hint service client

'''
client_update_ontology=None
''' Initialize the /ontology_interface/update_request service client

'''
client_try_solution=None
''' 
Initialize the /ontology_interface/check_consistency service client

'''
ask_solution=None
''' 
Initialize the /oracle_solution service client

'''
armor_solution=None
''' 
Initialize the /ontology_interface/ask_solution service client

'''

IDs=[]
''' 
list: List with the new IDs that are received

'''
tried_IDs=[]
''' list: List with the IDs that have already been included in the ontology

'''
consistent_ids=[]
''' 
list: List with the consistent_ids that have been found

'''

places=[]
''' list: List with the places of the scene

'''
actual_pose=None
''' 
Actual position of the robot in the environment

'''
oracle=False
''' bool: Boolean to know if the robot must go to the Oracle_Room

'''
oracle_room=None
''' 
Initialize the Oracle_Room

'''
stop=False
''' 
bool: Flag to stop storing new IDs in the IDs list

'''

def IDs_callback(id):
    '''
    This is the callback function to get the ID from the marker with the aruco_ros node.
    '''
    global IDs
    if id.data not in IDs and stop==False and id.data<=40:
        IDs.append(id.data)


def init_scene():
    '''
    This is the funcition to initialise the rooms in the simulation.
    '''
    global places, actual_pose, oracle_room
    places.append(Place('ROOM1', -4, -3))
    places.append(Place('ROOM2', -4, 2))
    places.append(Place('ROOM3', -4, 7))
    places.append(Place('ROOM4', 5, -7))
    places.append(Place('ROOM5', 5, -3))
    places.append(Place('ROOM6', 5, 1))

    oracle_room=Place('Oracle_Room', 0, -1)
    actual_pose=Place('Oracle_Room', 0, -1) #Starts in the oracle room



class Explore(smach.State):
    '''
    This is the node replicate the robot movement.

    '''
    
    def __init__(self):

        smach.State.__init__(self, 
                             outcomes=['check_consistency', 'solution'])
        
    def execute(self, userdata):
        '''
        Thia ia the method to pass the coordinates of the place to move_base action goal to move the robot and as well as oracle to check the solution to reach the oracle room.
        '''

        global pub_move_base, actual_pose, places, oracle

        if oracle==False:
            destination=places.pop()
            
        else:
            destination=oracle_room

        ## Initialize a MoveBaseActionGoal target to move my robot
        move_goal = MoveBaseGoal()
        move_goal.target_pose.header.frame_id="map"
        move_goal.target_pose.pose.orientation.w=1
        move_goal.target_pose.pose.orientation.z=0
        move_goal.target_pose.pose.orientation.x=0
        move_goal.target_pose.pose.orientation.y=0

        move_goal.target_pose.pose.position.x = destination.x
        move_goal.target_pose.pose.position.y = destination.y
        move_goal.target_pose.pose.position.z = 0

        pub_move_base.wait_for_server()
        print("\nMoving from " + actual_pose.name + " to " + destination.name +"\n")
        pub_move_base.send_goal(move_goal)
        pub_move_base.wait_for_result()
        
        actual_pose.x=destination.x
        actual_pose.y=destination.y
        actual_pose.name=destination.name


        # If it is going to the oracle room it sets oracle to False for the future iterations of the state_machine, then returns 'solution'
        if oracle == True:
            oracle=False
            return 'solution'
        
        else:
            sleep(5)

            return 'check_consistency'

class Check_Consistency(smach.State):
    '''
    This is the class which ask the ARMOR onotoogy for consistency checking.
    '''

    def __init__(self):

        smach.State.__init__(self, 
                             outcomes=['explore'])

    def execute(self, userdata):
        '''
        This is the method check the for new IDs and sends them to the ontology_interface. It updates the ontolody and check for consistency and remove the inconsistent ones.
        '''

        global IDs, tried_IDs, client_ID_msg, oracle, client_update_ontology, stop, consistent_ids

        #Stop adding elements in the list
        stop=True

        for id in IDs:
            if id not in tried_IDs:
                req=MarkerRequest()
                req.markerId=id
                rospy.wait_for_service('/oracle_hint')
                res=client_ID_msg(req)
                print("Reply: "+str(res))
                oracle_hint=ErlOracle()
                oracle_hint.ID=res.oracle_hint.ID
                oracle_hint.key=res.oracle_hint.key
                oracle_hint.value=res.oracle_hint.value
                req=HintRequest()
                req.oracle_hint=oracle_hint
                client_add_hint(req)
            
                tried_IDs.append(id)

        #After having added all the hints to the ontology_interface node, update the ontology performing "reason" operation
        rospy.wait_for_service('/ontology_interface/update_request')
        client_update_ontology()

        #Clear ID list and update oracle flag to send robot in the oracle room to try a solution
        IDs.clear()

        #Ask for complete and consistent hypothesis as potential solutions
        res=client_try_solution(ConsistentRequest())

        if len(res.consistent)>0:
            consistent_ids=res.consistent
            oracle=True
        
        stop=False
        return 'explore'


class Try_Solution(smach.State):
    '''
     This is the class to represent the behavior of the robot when it tries to generate a solution

    '''

    def __init__(self):

        smach.State.__init__(self, 
                             outcomes=['explore', 'correct'])

    def execute(self, userdata):
        '''
        This is the method which checks the complete and consistent one for the solution. And the games ends there or switch to the move state.
        '''

        global consistent_ids

        solution=ask_solution(OracleRequest())
        for id in consistent_ids:
            if str(solution.ID) == id:
                print("\nSolution found!: "+ str(solution.ID))

                req=SolutionRequest()
                req.ID=id
                res=armor_solution(req)
                print("Person: "+res.person+" Weapon: "+res.weapon+" Place: "+res.place)

                return 'correct'
            else:
                print("Solution ID "+id +" is not correct")

        consistent_ids.clear()
        return 'explore'

        


def main():
    '''
    This is the main function of the state_machine node which initialise the node and defines the SMACH state.
    '''

    global pub_move_base, sub_ID, client_ID_msg, client_add_hint, client_update_ontology, client_try_solution, ask_solution
    global armor_solution

    # Initialize the node
    rospy.init_node('state_machine')
 
    pub_move_base=actionlib.SimpleActionClient('move_base', MoveBaseAction)
    sub_ID=rospy.Subscriber('/marker_id', Int32, IDs_callback)
    client_ID_msg=rospy.ServiceProxy('/oracle_hint', Marker)
    client_add_hint=rospy.ServiceProxy('/ontology_interface/add_hint', Hint)
    client_update_ontology=rospy.ServiceProxy('/ontology_interface/update_request', Update)
    client_try_solution=rospy.ServiceProxy('/ontology_interface/check_consistency', Consistent)
    ask_solution=rospy.ServiceProxy('/oracle_solution', Oracle)
    armor_solution=rospy.ServiceProxy('/ontology_interface/ask_solution', Solution)

    init_scene()

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['state_machine'])
    sm.userdata.sm_counter = 0


     # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('MOVE', Explore(), 
                               transitions={'check_consistency':'CHECK_CONSISTENCY',
                                            'solution':'SOLUTION' })

        smach.StateMachine.add('CHECK_CONSISTENCY', Check_Consistency(), 
                               transitions={'explore':'MOVE'})
        
        smach.StateMachine.add('SOLUTION', Try_Solution(),
                                transitions={'explore':'MOVE',
                                             'correct':'state_machine'})


    # Create and start the introspection server for visualization
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    # Execute the state machine
    sm.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()




if __name__ == '__main__':
    main()
