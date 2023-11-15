# -*- coding: utf-8 -*-
"""
Created on Wed Oct 18 09:20:53 2023

@author: zchen
"""

def Seat_valvecontrol(model_control_switch,seat_Valve_button, autoSitzventilMaxTime,autoEnableSitzventilTimer,autodisableSitzventilTimer, autoEnableSitzventile,autodisableSitzventile , sitzventil_High):
    
    # enable seat valve before experiment
    if (seat_Valve_button == 1):
      if (autoEnableSitzventilTimer < autoSitzventilMaxTime):
        autoEnableSitzventilTimer += 1  
        autoEnableSitzventile = 1 
        # '          autodisableSitzventile = 1 
        sitzventil_High = 0
      else:
        autoEnableSitzventile = 0
        model_control_switch = 1
        sitzventil_High = 1
        
      seat_valve_channel_0_output =  autoEnableSitzventile
      seat_valve_channel_1_output =  0
      # P2_digout(TRA16_01_module, TRA16_Out_enable_Sitzventil, autoEnableSitzventile) 
      # P2_digout(TRA16_01_module, TRA16_Out_disable_Sitzventil, 0) 
      
    else:
      model_control_switch = 0
      if (autodisableSitzventilTimer < autoSitzventilMaxTime):
        autodisableSitzventilTimer += 1    
        # '        autoEnableSitzventile = 1 
        autodisableSitzventile = 1 
        sitzventil_High = 0
      else:
        autodisableSitzventile = 0
      
      seat_valve_channel_0_output =  0
      seat_valve_channel_1_output =  autodisableSitzventile
      # P2_digout(TRA16_01_module, TRA16_Out_enable_Sitzventil, 0) 
      # P2_digout(TRA16_01_module, TRA16_Out_disable_Sitzventil, autodisableSitzventile)
      
    return autoEnableSitzventilTimer,autodisableSitzventilTimer, model_control_switch, seat_valve_channel_0_output, seat_valve_channel_1_output, sitzventil_High 
     