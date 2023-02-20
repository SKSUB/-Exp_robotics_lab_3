#! /usr/bin/env python3

'''
.. module:: ontology:interface
   :platform: Unix
   :synopsis: Node for implementing the ontology.
.. moduleauthor:: SUBRAMANI SATHISH KUMAR

Node implements the onotology, perform queries and other ontology functions with the Armor service.
   
   This node recieves the hint from the state_machine node node and add them. And it checks the received hint is whether consistent and complete. When the recieved solutio is consistent and complete it gives the solution ID to the /ask_solution, otherwise removes them from the onotology. 
     
	                                                                                                                            
Services:
    /check_consistency, for complete and consistent hypothesis check.
    /update_request, for adding and reasoning the hints 
    /add_hint receives the hint form state machine
    /ask_solution receive solution from the final oracle node 
'''

import rospy
from classes.myArmor import MyArmor
from erl2.srv import Update, UpdateResponse, Consistent, ConsistentResponse, Hint, HintResponse, Solution, SolutionResponse
from armor_msgs.srv import *
from armor_msgs.msg import *

people_ontology=["missScarlett", "colonelMustard", "mrsWhite", "mrGreen", "mrsPeacock", "profPlum"]
''' 
list[str]: Define all the people of the scene
'''
places_ontology=["conservatory", "lounge", "kitchen", "library", "hall", "study", "bathroom", "diningRoom", "billiardRoom"]
''' 
list[str]: Define all the places of the scene
'''
weapons_ontology=["candlestick", "dagger", "leadPipe", "revolver", "rope", "spanner"]
''' 
list[str]: Define all the weapons of the scene
'''

ID=[]
''' 
list: Initialize ID list 
'''
key=[]
''' 
list: Initialize key list (each element is referred to the corresponding element in the ID list)
'''
value=[]
''' 
list: Initialize value list (each element is referred to the corresponding element in the ID list)
'''
update_service=None
''' 
Initialize /update_request service server
'''
consistency_service=None
''' 
Initialize /consistency_request service server
'''


def init_scene():
    '''
    This is the function to initialize the scene and add all the information to the armor ontology. It pefroms disjoint to seperate them respect to same class element. And at last it performs reason to update the ontology.      
    '''

    ## Add people, weapons and places o the ontology

    j=0
    while j!=len(people_ontology):
        res=MyArmor.add_item(people_ontology[j], 'PERSON')

        if res.armor_response.success==False:
            print("\nError in loading PERSON in the ontology")
        ## Disjoint all the elements
        if j!=0:
            count=j
            while count!=0:
                MyArmor.disjoint(people_ontology[j], people_ontology[count-1])
                count = count -1
        j=j+1

    j=0
    while j!=len(places_ontology):
        res=MyArmor.add_item(places_ontology[j], 'PLACE')

        if res.armor_response.success==False:
            print("\nError in loading PERSON in the ontology")
        ## Disjoint all the elements
        if j!=0:
            count=j
            while count!=0:
                MyArmor.disjoint(places_ontology[j], places_ontology[count-1])
                count = count -1
        j=j+1

    j=0
    while j!=len(weapons_ontology):
        res=MyArmor.add_item(weapons_ontology[j], 'WEAPON')

        if res.armor_response.success==False:
            print("\nError in loading PERSON in the ontology")
        ## Disjoint all the elements
        if j!=0:
            count=j
            while count!=0:
                MyArmor.disjoint(weapons_ontology[j], weapons_ontology[count-1])
                count = count -1
        j=j+1

    res=MyArmor.reason()


def receive_hint(hint):
    '''
    This is the callback function to get the hints from the state machine and append them.
    '''
    global ID, key, value
    
    print("Received hint: "+str(hint))
    ID.append(str(hint.oracle_hint.ID))
    key.append(hint.oracle_hint.key)
    value.append(hint.oracle_hint.value)
    return HintResponse()


def update_ontology(msg):
    '''
    This is the callback function to add the hypothesis to the ontology when the /update_request is called.
    '''

    global ID, key, value
    i=0
    while i<len(ID):
        MyArmor.add_hypothesis(key[i], ID[i], value[i])
        i+=1
    #Clear the lists after having sent the hypothesis
    ID.clear()
    key.clear()
    value.clear()
    
    MyArmor.reason()
    res=UpdateResponse()
    res.updated=True
    return res

        
def find_consistent(msg):
    '''
    This is the callback function to check the complete and conistent hypothesis when /check_consistency is called and removes the inconsistent ones.
    '''

    response_complete=MyArmor.ask_complete()
    list_complete=[]
    print(str(response_complete))
    if response_complete.armor_response.success==False:
        print("\nError in asking query")
    else:
        if len(response_complete.armor_response.queried_objects)!=0:

            for complete in response_complete.armor_response.queried_objects:
                complete=complete[40:]
                id_complete=complete[:-1]
                print("ID_inconsistent "+str(id_complete) +"\n")
                list_complete.append(id_complete)

            response_inconsistent=MyArmor.ask_inconsistent()
            print(str(response_inconsistent))
            print("Complete responses: "+str(list_complete))
        
            # Look for possible inconsistent hypothesis and in case remove them
            if len(response_inconsistent.armor_response.queried_objects)!=0:

                for str_inconsistent in response_inconsistent.armor_response.queried_objects:
                    str_inconsistent=str_inconsistent[40:]
                    id_inconsistent=str_inconsistent[:-1]
                    print("ID_inconsistent "+str(id_inconsistent) +"\n")
                    res=MyArmor.remove(id_inconsistent)
                    list_complete.remove(id_inconsistent)

                    if res.armor_response.success==False:
                        print("Error in removing\n")

        print("Consistent hypotheses: "+str(list_complete))
        res=ConsistentResponse()
        if len(list_complete)>0:
            res.consistent=list_complete
        return res

def ask_solution(msg):
    '''
    This is the callback function to find the solution when the /ask_solution is called.
    '''

    res=SolutionResponse()

    #Ask weapon
    armor_res=MyArmor.ask_item(ID=msg.ID, type="what")
    print(str(armor_res))
    if len(armor_res.armor_response.queried_objects)==0:
        print("Error in asking query")
    else:
        for str_what in armor_res.armor_response.queried_objects:
                    str_what=str_what[40:]
                    res.weapon=str_what[:-1]
     #Ask person
    armor_res=MyArmor.ask_item(ID=msg.ID, type="who")
    print(str(armor_res))
    if len(armor_res.armor_response.queried_objects)==0:
        print("Error in asking query")
    else:
        for str_who in armor_res.armor_response.queried_objects:
                    str_who=str_who[40:]
                    res.person=str_who[:-1]
     #Ask place
    armor_res=MyArmor.ask_item(ID=msg.ID, type="where")
    print(str(armor_res))
    if len(armor_res.armor_response.queried_objects)==0:
        print("Error in asking query")
    else:
        for str_where in armor_res.armor_response.queried_objects:
                    str_where=str_where[40:]
                    res.place=str_where[:-1]
    return res


def main():
    '''
    This is the main fucniton of the ontology_interface node intializes the module.

    '''

    global update_service, consistency_service
    rospy.init_node('ontology_interface')
   
    consistency_service=rospy.Service('/ontology_interface/check_consistency', Consistent, find_consistent)
    update_service= rospy.Service('/ontology_interface/update_request', Update, update_ontology)
    rospy.Service('/ontology_interface/add_hint', Hint, receive_hint)
    rospy.Service('/ontology_interface/ask_solution', Solution, ask_solution)

    rospy.wait_for_service("/armor_interface_srv")
    rospy.wait_for_service("/armor_interface_serialized_srv")

    path=rospy.get_param("~ontology")
    print("PATH: "+str(path))
    
    # Loads the ontology
    response=MyArmor.load(path)
    if response.armor_response.success==True:
        print("\nOntology loaded successfully")
    else:
        print("\nERROR: Ontology not loaded correctly")
    
    init_scene()
    rospy.spin()



if __name__ == '__main__':
    main()
