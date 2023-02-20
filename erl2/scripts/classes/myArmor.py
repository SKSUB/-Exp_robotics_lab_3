#! /usr/bin/env python

"""
.. module:: myArmor.py
   :platform: Unix
   :synopsis: CLASS to use the ARMOR
.. moduleauthor:: SUBRAMANI SATHISH KUMAR

This is the class to implement the ARMOR operations.

This is created to use the ARMOR commands and sends the armor command to armor_interface_srv service. 
"""


from posixpath import dirname, realpath
from armor_msgs.srv import *
from armor_msgs.msg import *
from rospy.impl.tcpros_service import ServiceProxy


armor_interface=ServiceProxy('/armor_interface_srv', ArmorDirective)
'''
Defines the armor_interface_srv service to send the messages to ARMOR
'''


class MyArmor(object):
    '''
    This is the call sends the ARMOR commands prpèerly.
    '''


    def __init__(self):
        '''
        Initialize the class
        '''
        print("\nClass initialized")

    def load(path):
        '''
        This load method loads the ontology.
        '''

        request=ArmorDirectiveReq()
        request.client_name='tutorial'
        request.reference_name='ontoTest'
        request.command='LOAD'
        request.primary_command_spec='FILE'
        request.secondary_command_spec=''
        request.args=[path, 'http://www.emarolab.it/cluedo-ontology', 'true', 'PELLET', 'true']
        response=armor_interface(request)
        return response
                
#Below are the methods to request appropriate command to the ARMOR         
    def add_hypothesis(type, ID, arg):
        '''
        This is the method to add the hupothesis to the ontology.
        '''

        req=ArmorDirectiveReq()
        req.client_name='tutorial'
        req.reference_name='ontoTest'
        req.command='ADD'
        req.primary_command_spec='OBJECTPROP'
        req.secondary_command_spec='IND'
        req.args=[type, ID, arg]
        res=armor_interface(req)
        return res

    def reason():
        '''
        Reason performer to update the onotology

        '''

        req=ArmorDirectiveReq()
        req.client_name= 'tutorial'
        req.reference_name= 'ontoTest'
        req.command= 'REASON'
        req.primary_command_spec= ''
        req.secondary_command_spec= ''
        req.args= []
        res=armor_interface(req)
        return res

    def ask_complete():
        '''
        Request to complete the hypothesis of the ontology.
        '''

        req=ArmorDirectiveReq()
        req.client_name= 'tutorial'
        req.reference_name= 'ontoTest'
        req.command= 'QUERY'
        req.primary_command_spec= 'IND'
        req.secondary_command_spec= 'CLASS'
        req.args= ['COMPLETED']
        res=armor_interface(req)
        return res


    def ask_inconsistent():
        '''
        Request to incnsistent with the hypothesis of the ontology.
        '''

        req=ArmorDirectiveReq()
        req.client_name= 'tutorial'
        req.reference_name= 'ontoTest'
        req.command= 'QUERY'
        req.primary_command_spec= 'IND'
        req.secondary_command_spec= 'CLASS'
        req.args= ['INCONSISTENT']
        res=armor_interface(req)
        return res

    def remove(name):
        '''
        This is the method to remove the ontology
        '''

        req=ArmorDirectiveReq()
        req.client_name= 'tutorial'
        req.reference_name= 'ontoTest'
        req.command= 'REMOVE'
        req.primary_command_spec= 'IND'
        req.secondary_command_spec= ''
        req.args= [name]
        res=armor_interface(req)
        return res

    def ask_item(type, ID):
        '''
        Request the specific element in the hypthesis
        '''

        req=ArmorDirectiveReq()
        req.client_name= 'tutorial'
        req.reference_name= 'ontoTest'
        req.command= 'QUERY'
        req.primary_command_spec= 'OBJECTPROP'
        req.secondary_command_spec= 'IND'
        req.args= [type,ID]
        res=armor_interface(req)
        return res

    def add_item(name, type):
        '''
        This is the method to add element in the ontology
        '''

        req=ArmorDirectiveReq()
        req.client_name= 'tutorial'
        req.reference_name= 'ontoTest'
        req.command= 'ADD'
        req.primary_command_spec= 'IND'
        req.secondary_command_spec= 'CLASS'
        req.args= [name, type]
        res=armor_interface(req)
        return res

    def disjoint(ind1, ind2):
        '''
        This is the disjoint method to notify the ARMOR they are different elements from one another 
        '''

        req=ArmorDirectiveReq()
        req.client_name= 'tutorial'
        req.reference_name= 'ontoTest'
        req.command= 'DISJOINT'
        req.primary_command_spec= 'IND'
        req.secondary_command_spec= ''
        req.args= [ind1, ind2]
        res=armor_interface(req)
        return res


    def save():
        '''
        This save method saves the current ontology (soltuion ontology). 
        '''

        req=ArmorDirectiveReq()
        path = dirname(realpath(__file__))
        path = path[:-15] + "solution_cluedo_ontology.owl"
        req.client_name= 'tutorial'
        req.reference_name= 'ontoTest'
        req.command= 'SAVE'
        req.primary_command_spec= ''
        req.secondary_command_spec= ''
        req.args= [path]
        res=armor_interface(req)
        return res

