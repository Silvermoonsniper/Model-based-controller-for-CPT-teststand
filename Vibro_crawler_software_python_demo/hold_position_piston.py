# -*- coding: utf-8 -*-
"""
Created on Wed Oct 18 10:44:52 2023

@author: zchen
"""

def hold_position_PI_control(Stellwert,static_penetration_activate, vibroOpCmd_model_control,IstWegMTS , SollWeg,Kp_wegHalten, Soll_Position_Regelungswert):
    
    # rem move piston a little bit down when we start penetration, 
    # rem when static penetration mode is activated, don#t use open loop control
    # rem else, we use open loop control 
    if (static_penetration_activate == 0):
     
      # rem hold the piston position when piston moves down sufficiently 
           
      Regelabweichung = IstWegMTS - SollWeg
      Stellwert = Regelabweichung * Kp_wegHalten          # fester Kp bei  weggesteuert Positioin halten, muss nicht weiter parametriert werden, funktioniert so!
      Stellwert = Stellwert_LimitAndScale(Stellwert)
      #        stellwert = amplitude_train * sin(2 * 3.1415926 * (vibration_counter / point_num_singlecycle)) + 32768
        
      #        stellwert = amplitude_train * sin(2 * 3.1415926 * (vibration_counter / point_num_singlecycle)) + 32000

      #      endif
      Soll_Position_Regelungswert = IstWegMTS
      
    else:
    
      # rem firstly hold the position of piston, when start button to start static penetration is not pressed
      if ((vibroOpCmd_model_control > 2) or (vibroOpCmd_model_control < 2)):
        Regelabweichung = IstWegMTS - SollWeg
        Stellwert = Regelabweichung * Kp_wegHalten        # fester Kp bei  weggesteuert Positioin halten, muss nicht weiter parametriert werden, funktioniert so!
        Stellwert = Stellwert_LimitAndScale(Stellwert)
        # rem initialize 
        Soll_Position_Regelungswert = IstWegMTS
         
    
    return Stellwert,Soll_Position_Regelungswert
    
def Stellwert_LimitAndScale(Stellwert):
    
     if (Stellwert < -32768):
         Stellwert = -32768
  
     if (Stellwert > 32768):
         Stellwert = 32768
  
     Stellwert_LimitAndScale = Stellwert + 32768 
     
     return Stellwert_LimitAndScale
 
    
 