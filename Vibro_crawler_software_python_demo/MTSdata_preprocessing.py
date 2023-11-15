# -*- coding: utf-8 -*-
"""
Created on Wed Oct 18 12:45:02 2023

@author: zchen
"""

def distance_data_regression(WegInd_Digits,  realdistance_data,vel_full_flag, intervalp , Programm_counter):
    
    
    if (Programm_counter < intervalp):
        
        Programm_counter += 1 
        realdistance_data[Programm_counter] = WegInd_Digits
   
   
    # rem when number of velocity data exceeds size of velocity data array, we use ring buffer to insert new velocity data and remove old one
    else:
        vel_full_flag = 1
    # '    ringbuffer(reg_para[2],realdistance_data,intervalp)
        Programm_counter = -1
        Programm_counter += 1 
        realdistance_data[Programm_counter] = WegInd_Digits
     
  
  # '  if (Programm_counter > -1) then
    if (vel_full_flag == 1):
        vel_local = velocity_dataval(realdistance_data, Programm_counter, intervalp)
    else:
        vel_local = 0
 
    smoothed_dist = WegInd_Digits
    
    return smoothed_dist,vel_full_flag, Programm_counter,vel_local


# function to calculate velocity
def velocity_dataval(distance_data, Programm_counter, intervalp):
  
  if (Programm_counter < intervalp-1):
    velocity_dataval = (distance_data[Programm_counter] - distance_data[Programm_counter + 1]) / intervalp
  else:
    velocity_dataval = (distance_data[Programm_counter] - distance_data[0]) / intervalp
  
  return velocity_dataval


def Distance_data_smooth(distance_data, pointer_average, windowsize, WegInd_Digits):
  
  # rem modified by zhengyu 2022-06-08########
      
 
  # rem record distance data
  # rem for the first 'data_sizetrain' data
  if (pointer_average < windowsize - 1):
    # rem transform digital position reading into mm
    pointer_average += 1 
    distance_data[pointer_average] = WegInd_Digits 
    
       
  
    # rem when number of velocity data exceeds size of velocity data array, we use ring buffer to insert new velocity data and remove old one
  else:
    # rem spilt distance data buffer into small piece, for each piece calculat its average portion
       
    pointer_average = -1
  
  return distance_data, pointer_average
     
        
  
  
   # record distance data for velocity calculation
      
def vel_controloutput_array(WegInd_RegOut, local_c, controlleroutput, vel_local, velocity_data, local_vel_pointer, data_sizetrain):
  
 
                     
                              
  # rem record velocity data into varible for fifo transmission
                           
  if (local_vel_pointer < data_sizetrain):
   
    velocity_data[local_vel_pointer] = vel_local
    local_vel_pointer = local_vel_pointer + 1                         
    # # rem check if velocity data has spike
    if (abs(vel_local) > 10):
        
      if (local_vel_pointer > 1): 
        vel_local = velocity_data[local_vel_pointer - 2] 
        velocity_data[local_vel_pointer-1] = velocity_data[local_vel_pointer - 2] 
      else:
        vel_local = velocity_data[data_sizetrain] 
        velocity_data[local_vel_pointer-1] = velocity_data[local_vel_pointer - 2] 
      
    
                            
  else:
    # rem reinitialize index
    local_vel_pointer = 0

    velocity_data[local_vel_pointer] = vel_local
    local_vel_pointer = local_vel_pointer + 1 

  
                            
                      
                      
  ########################################
                          
  # rem store controller output data 
                         
                                        
                                
  if (local_c < data_sizetrain):
   
    controlleroutput[local_c] = WegInd_RegOut
    local_c = local_c + 1
    
  else:
    # rem reinitialie index, when data array is full
                            
    local_c = 0
    controlleroutput[local_c] = WegInd_RegOut
    local_c = local_c + 1                        
  
    
  return velocity_data, controlleroutput, local_vel_pointer,local_c
  

  
