#! /usr/bin/env python

"""
.. module:: place
   :platform: Unix
   :synopsis: CLASS to define the places in the scene 
.. moduleauthor:: SUBRAMANI SATHISH KUMAR

This is the class for defining the places of the scene. It defines the postion of the place in the scene.
"""


class Place(object):

    name=None
    '''
    Name of the place
    '''
    x=0
    '''
    x coordinate of the place
    '''
    y=0
    '''
    y coordinate of the place
    '''

    def __init__(self, name, x_coord, y_coord):
        self.name=name
        self.x=x_coord
        self.y=y_coord
