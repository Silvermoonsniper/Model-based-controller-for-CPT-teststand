# -*- coding: utf-8 -*-
"""
Created on Wed Oct 18 08:10:59 2023

@author: zchen
"""

######################################

# function to propaget operating state of vibro controller

import bit_operator
def vibro_crawler_state_machine(BitStatus_01,vibroOpCmd_model_control,autodisableSitzventilTimer, autoSitzventilMaxTime,_startOperation , _stopOperation, _enableHandventil, _disableHandventil, _Taster_start, _Taster_stop, _Taster_frei1, _Taster_frei2, sysState_bit,IstWegMTS,v_fullFLAG, switch_flag, local_index,SollWeg,sitzventil_High):
    
    
    
    
    # disable the seat valve before the start button is pressed
    if (vibroOpCmd_model_control == 1):
      if (autodisableSitzventilTimer < autoSitzventilMaxTime):
        autodisableSitzventilTimer += 1  
        # '        autoEnableSitzventile = 1 
        autodisableSitzventile = 1 
        sitzventil_High = 0
      else:
        autodisableSitzventile = 0
      
      
      
      
      
      
      
      
      
      # P2_digout(TRA16_01_module, TRA16_Out_enable_Sitzventil, 0) 
      # P2_digout(TRA16_01_module, TRA16_Out_disable_Sitzventil, autodisableSitzventile)
      # rem if start button is pressed, goto state 2
    if (((_startOperation == 1) or (_Taster_start == 1)) and (vibroOpCmd_model_control == 1)):
              
        vibroOpCmd_model_control = 2
     
        _stopOperation = 0
        _startOperation = 0
        _enableHandventil = 0
        _disableHandventil = 0
        
        sitzventil_High = 1
      
      
      
       # when stop button is pressed
      
    if ((_stopOperation == 1) or (_Taster_stop == 1)):
        # rem update status of state machine
        vibroOpCmd_model_control = 3
        model_control_cycle_num = 0
        autodisableSitzventilTimer = 0
        # set desired distance
        SollWeg = IstWegMTS
        _stopOperation = 0
        _startOperation = 0
        _enableHandventil = 0
        _disableHandventil = 0
        
    
    
      # rem if frei button 1 is pressed, enable handvalve
    if ((_enableHandventil == 1) or (_Taster_frei1 == 1)):
        # rem reset all buttons
        _stopOperation = 0
        _startOperation = 0
        _enableHandventil = 0
        _disableHandventil = 0
        # '        autodisableSitzventilTimer = 0 
        # rem update status for state machine
        vibroOpCmd_model_control = 4
        BitStatus_01 = bit_operator.ClearBit(BitStatus_01, sysState_bit['sysState_vibroPushingEnd'])
        BitStatus_01 = bit_operator.setBit(BitStatus_01, sysState_bit['SysState_vibro_Hand'])
      
        
      # rem if frei button 2 is pressed, restart model control
      
    if ((_disableHandventil == 1) or (_Taster_frei2 == 1)):
         
        # rem update status for state machine
        vibroOpCmd_model_control = 5
       
        # rem reset all buttons
        _stopOperation = 0
        _startOperation = 0
        _enableHandventil = 0
        _disableHandventil = 0
      
     # when stop button is pressed
    if (vibroOpCmd_model_control == 3):
       
       
       # ' disable seat valves, when stop button is pressed  
       if (autodisableSitzventilTimer < autoSitzventilMaxTime):
         autodisableSitzventilTimer += 1 
         autodisableSitzventile = 1 
         sitzventil_High = 0
       else:
         # '          autoEnableSitzventile = 0
         autodisableSitzventile = 0 
       
       BitStatus_01 = bit_operator.ClearBit(BitStatus_01, sysState_bit['sysState_vibroPushing'])
       BitStatus_01 = bit_operator.setBit(BitStatus_01, sysState_bit['sysState_vibroPushingEnd'])
      
     
       
       # rem reinitialize linear regression module, when stop button is pressed
       v_fullFLAG = 0
       switch_flag = 0
       local_index = 1
     
    
     # rem LED control
    if (vibroOpCmd_model_control == 4):
     
     
       # ' disable seat valves, when we use hand valve     
       if (autodisableSitzventilTimer < autoSitzventilMaxTime):
         autodisableSitzventilTimer += 1  
         autodisableSitzventile = 1 
         sitzventil_High = 0
       else:
         autodisableSitzventile = 0  
      
     
    if (vibroOpCmd_model_control == 5):
        
        SysModus = 12
        # rem disable seat valve
        sitzventil_High = 0
        
     # enable control valve and disable seat valve, when we use handvalve
     
    if (vibroOpCmd_model_control == 4):
        
     
       
       # rem disable seat valve
       sitzventil_High = 0
      
      
       
    return  v_fullFLAG, switch_flag, local_index, vibroOpCmd_model_control, _stopOperation, _startOperation, _enableHandventil,_disableHandventil,  BitStatus_01, sitzventil_High, SollWeg
  
    
  
        
          