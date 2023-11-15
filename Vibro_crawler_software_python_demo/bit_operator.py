# -*- coding: utf-8 -*-
"""
Created on Wed Oct 18 08:43:38 2023

@author: zchen
"""

## function to set or clear system status bit


def setBit(variable, bit):
    
    SetBIT = variable | bit
    return SetBIT


def ClearBit(variable, bit):
    
    SetBIT = variable &  ( ~ bit)
    return SetBIT    
    