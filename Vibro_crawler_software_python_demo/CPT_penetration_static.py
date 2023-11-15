# -*- coding: utf-8 -*-
"""
Created on Wed Oct 18 14:59:13 2023

@author: zchen
"""

def static_CPT(IstWegMTS,SummeRegelabweichung, Samplingrate_ ,_Kp,_Ki,model_control_cycle_num,Stellwert, _endeMessbereich,Soll_Position_Regelungswert):
    
    
    Vorschub = (20) / 5000

      
    model_control_cycle_num += 1
  
  
                                                              
    # rem calculate desired control signal
     
    Soll_Position_Regelungswert = Soll_Position_Regelungswert + Vorschub  #'activate vorschub  
        
    Regelabweichung = IstWegMTS - Soll_Position_Regelungswert  #'Model activated  
        
                   
    if (IstWegMTS > _endeMessbereich):            #EndE des Reglers nahe dem Ende  
      _Kp = 1  
      _Ki = 1  
    
          
    SummeRegelabweichung = Regelabweichung + SummeRegelabweichung 

    # '        If (SummeRegelabweichung < -65536) Then SummeRegelabweichung = -65536 
    # '        If (SummeRegelabweichung > 65536) Then SummeRegelabweichung = 65536  

       
    Stellwert = (_Kp * Regelabweichung) + (_Ki * Samplingrate_ * SummeRegelabweichung)  
       
    # rem controller output for distance sensor data  
    Stellwert = Stellwert + 32678
  
    return Stellwert, model_control_cycle_num