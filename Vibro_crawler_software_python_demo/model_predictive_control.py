# -*- coding: utf-8 -*-
"""
Created on Wed Oct 18 16:56:38 2023

@author: zchen
"""
import numpy as np
# rem function to calculate predicted distance based on local linear regression parameters
# 'predicted distance at current time step is done as follows, when we get one new distance sensor measurement, we trace back
# '#to past sensor measurement by L steps, L is number of datapoints which is equal to maximum correlation, then we calculate 
# '# predict distance at current time step as distance data before L points plus the summation of estimated distance difference
# '# for past L points using linear regression model

# rem input args:
# rem         distance_data[]: distance data array
# rem         controlleroutput[]: controller output array
# rem         linear_coff: linea regression parameters
# rem         datasize_train: number of datapoints in the buffer
# rem         MAX_correlationstep: time step for maximum correlation
# rem         predict_distance_pointer:pointer for incremental calculation
# rem         predict_full: flag to indicate array is full
# rem         estima_vel_sub[]: array to store local velocity calculated from model
# rem         windowsize. size of distance data array
# rem         pointer_average: pointer for distance data array
# rem         local_c: pointer for controller output array
def predict_distance(predict_distance_pointer, predict_full, estima_vel_sub, WegInd_Digits, windowsize, pointer_average, local_c, distance_data, controlleroutput, linear_coff, datasize_train, MAX_correlationstep):
  # rem define summation of estimated distance difference for past 'max_correlationstep' datapoints
  # DIM delta_s_local as float
  

  
  # rem distance data before 'max_correlationstep' datapoints
  
  if (pointer_average >= MAX_correlationstep):
    
      
      delta_s_local = distance_data[pointer_average - MAX_correlationstep]
       
     
       
  else:
      
      delta_s_local = distance_data[windowsize + (pointer_average - MAX_correlationstep)]
      
     
        
  
  # rem predict distance in the future steps with current smoothed distance
  # '  delta_s_local = distance_data[pointer_average]
  # rem store estimated velocity into array
  if (predict_full == 0):
    if (predict_distance_pointer <= MAX_correlationstep - 1):
      if (local_c >= predict_distance_pointer):
        estima_vel_sub[predict_distance_pointer] = ((linear_coff[0] * controlleroutput[local_c - predict_distance_pointer] + linear_coff[1]))
      else:
        estima_vel_sub[predict_distance_pointer] = ((linear_coff[0] * controlleroutput[datasize_train + local_c - predict_distance_pointer] + linear_coff[1]))
      
      predict_distance_pointer += 1
    else:
      predict_distance_pointer = 1
      # rem array is full
      predict_full = 1
    
  
  if (predict_full == 0):
    # '    if (local_c > predict_distance_pointer) then
    # '      delta_s_local = delta_s_local + estima_vel_sub[predict_distance_pointer]
    # '        
    # '    else
    delta_s_local = delta_s_local + estima_vel_sub[predict_distance_pointer]
          
    # '  endif
  
        
  
      
  if (predict_full == 1):
    if (predict_distance_pointer < MAX_correlationstep - 1): 
      if (local_c >= predict_distance_pointer):
        delta_s_local = delta_s_local - estima_vel_sub[predict_distance_pointer + 1] + ((linear_coff[0] * controlleroutput[local_c - predict_distance_pointer] + linear_coff[1]))
      else:
        delta_s_local = delta_s_local - estima_vel_sub[predict_distance_pointer + 1] + ((linear_coff[0] * controlleroutput[datasize_train + local_c - predict_distance_pointer] + linear_coff[1]))
        
      
    else:
      if (local_c >= predict_distance_pointer):
        delta_s_local = delta_s_local - estima_vel_sub[1] + ((linear_coff[0] * controlleroutput[local_c - predict_distance_pointer] + linear_coff[1]))
    
      else:
        delta_s_local = delta_s_local - estima_vel_sub[1] + ((linear_coff[0] * controlleroutput[datasize_train + local_c - predict_distance_pointer] + linear_coff[1]))
    
     
 
  # '  rem obtain predict distance
  predict_distance = delta_s_local
  
  return predict_distance, predict_distance_pointer, predict_full
  
   ####################################################################################################################################
  
  
  
  
  # '  delta_s_local = distance_data[pointer_average]
  # '    
  # '    
  # '  rem predict distance in the future steps with current smoothed distance
  # '    
  # '  rem store estimated velocity into array
  # '  for j = 1 to MAX_correlationstep
  # '    if (predict_full = 0) then
  # '      if (predict_distance_pointer <= MAX_correlationstep - 1) then
  # '        if (local_c > predict_distance_pointer) then
  # '          estima_vel_sub[predict_distance_pointer] = ((linear_coff[1] * controlleroutput[local_c - predict_distance_pointer] + linear_coff[2]))
  # '        else
  # '          estima_vel_sub[predict_distance_pointer] = ((linear_coff[1] * controlleroutput[datasize_train + local_c - predict_distance_pointer] + linear_coff[2]))
  # '        endif
  # '        inc(predict_distance_pointer)
  # '      else
  # '        predict_distance_pointer = 1
  # '        rem array is full
  # '        predict_full = 1
  # '      endif
  # '    endif
  # '  
  # '    delta_s_local = delta_s_local + estima_vel_sub[predict_distance_pointer - 1]
  # '          
  # '    '  endif
  # '    'endif 
  # '          
  # '  next j
  # '   
  # '  rem obtain predict distance
  # '  predict_distance = delta_s_local
  
  # ' #################################################################################################################################### 
  


# rem function to calculate desired distance data at current step
# rem      initialWegInd_Digits:distance sensor measurement at controller starting point
# rem      Amplitude_Vibro: vibration amplitude for desired sine+linear cone penetration movement
# rem      Sinusfrequenz_Vibro: vibration frequency
# rem      Vibro_internal_counter: number for calculate sine movement value, cyclically repeated by sine period
# rem      Programm_counter: measurement cycles from programm  start
# rem      penetration_vel:linear penetration velocity in physical unit (mm)
# rem      controlstart_step: controller starting step
def desired_distance(desired_dist, IstWegMTS_data, switch_flag, total_shiftdist,  local_index, penetration_vel, global_pointer,  total_shift, initialWegInd_Digits, samplingfrequency_, _OpAmplitude, _OpFrequency, vibration_counter):
  
  
  # '  total_shiftdist = 110      
  if (switch_flag == 1):
   
    if (local_index == 1):
      # '      total_shiftdist_1 = vibration_counter 
       # record original vibration frequency
      original_vib_freq = _OpFrequency
      # rem record initial piston position  
                                                                                       
      initialWegInd_Digits = IstWegMTS_data
      
      global_pointer = 0
            
      local_index += 1
    
    else:
    
      local_index += 1
    
  
  
  
    # rem desired distance in physical unit (mm) 
    # rem desired trajectory is defined with combination of linear penetration movment of cone and sinsuidal damping
    
  
   
   
    # rem calculate initial distance shifted back by total_shift steps in mm rem
    # '    desired_dist = initialWegInd_Digits + _OpAmplitude * cos(2 * 3.1415926 * (_OpFrequency / samplingfrequency_) * (vibration_counter - total_shiftdist + 1) + 0.5 * 3.1415926) + (penetration_vel / samplingfrequency_) * global_pointer
    # '    new_deisred_dist = initialWegInd_Digits + (_OpAmplitude) * sin(2 * 3.1415926 * (_OpFrequency / samplingfrequency_) * (vibration_counter - total_shiftdist + 1)) + (penetration_vel / samplingfrequency_) * global_pointer 
    
    # rem modified desired distance profile, 2023-05-12
    desired_dist = initialWegInd_Digits + _OpAmplitude * np.cos(2 * np.pi * (_OpFrequency / samplingfrequency_) * (vibration_counter - total_shiftdist + 1) + 1 * np.pi) - _OpAmplitude * np.cos(1 * np.pi) + (penetration_vel / samplingfrequency_) * global_pointer
   
    
    global_pointer += 1
    
  else:
        
    desired_dist = IstWegMTS_data

  return desired_dist ,global_pointer ,local_index ,initialWegInd_Digits  



def estimated_parameters_justification(safe_m, safe_b,fulldata_number, full_flag, estimated_parameter_pointer, linear_coff, estimated_slope_array, estimated_intercept_array):



  
  
  if (estimated_parameter_pointer <= 10):
       
    estimated_slope_array[estimated_parameter_pointer] = linear_coff[0]
    estimated_intercept_array[estimated_parameter_pointer] = linear_coff[1]                        
    estimated_parameter_pointer = estimated_parameter_pointer + 1
    
      
    safe_m = -5.59393E-6
            
   
    safe_b = 0.185342
           
     
  else:
    # rem reinitialize index
    estimated_parameter_pointer = 1
    full_flag = 1
   
  
     
  
  
  #  if (full_flag = 1) then
  # rem for slope
  #  dec(estimated_parameter_pointer)
  if (estimated_parameter_pointer >= 1):
    if (abs((estimated_slope_array[estimated_parameter_pointer] - safe_m) / safe_m) > 0.01):
      # rem justify estimated parameters
      linear_coff[0] = safe_m
    
  else:
    if (abs((linear_coff[0] - safe_m) / safe_m) > 0.01):
        
      linear_coff[0] = safe_m
   
    
  # rem for intercept
  if (estimated_parameter_pointer > 1):
    if (abs((estimated_intercept_array[estimated_parameter_pointer] - safe_b) / safe_b) > 0.01):
      # rem justify estimated parameters
      linear_coff[1] = safe_b  
    
  else:
    if (abs((estimated_intercept_array[1] - safe_b) / safe_b) > 0.01):
      linear_coff[1] = safe_b
  
    
  return safe_m, safe_b, linear_coff, estimated_parameter_pointer




# rem subroutie for model controller
# rem input args:
# rem     desired_dist: desired distance
# rem     intervalp: number of points for velocity calculation
# rem     linear_coff[]: estimated parameters:
# rem     Abtastfrequenz: sampling frequency
# rem     Sinusfrequenz_Vibro: current set vibration frequency
# rem     Vibro_internal_counter: counter for vibration motion
# rem     total_shiftdist: initial phase of desired distance profile
# rem     WegInd_RegOut: controller output
# rem     sollvel: desired velocity
# rem     initialWegInd_Digits: distance data when we switch on model controller
# rem     predict_dist: predicted distance
# rem     estimatedWegInd_RegOut: estimated controller output
# rem     total_shift: number of points for time delay
# rem     switch_flag: flag to switch
# rem     WegInd_Digits: position of piston in digital value
# rem     vel_deviation: velocity deviation
# rem     dist_err: distance error
# rem     proportion_str: constant for proportional control
  
def model_control( desired_velraw, desired_P,  pro_const, vel_local, predict_distance_array, datasize_train, pre_dist_p,  local_index, penetration_vel, IstWegMTS, samplingfrequency_, _OpAmplitude, _OpFrequency, vibration_counter, desired_dist, intervalp, linear_coff,  Vibro_internal_counter, total_shiftdist, WegInd_RegOut, sollvel, initialWegInd_Digits, predict_dist, estimatedWegInd_RegOut, total_shift, switch_flag, global_pointer, WegInd_Digits, vel_deviation, dist_err):
  
  

  ##############################################################################
  ##############################################################################
  # rem switch when distance cross zero
  # rem shift points for get same phase between desired and measured distance
  
  
  
  # # rem  testing code 2 ########################
  # '  total_shiftdist = vibration_counter
  # rem  testing code 2 ########################
  # rem switch to model controller when velocity is 0, always starting at smallest change point
  # rem  testing code 1 ########################
  if ((abs(vel_local) < 0.005) and ((switch_flag == 0))):
    # rem  testing code 1 ########################
    # rem  testing code 2 ########################
    # '    if ((vibration_counter = total_shiftdist) and ((switch_flag = 0))) then 
    
    # rem  testing code 2 ########################
    switch_flag = 1
    
    # rem  testing code 1 ########################
    
    total_shiftdist = vibration_counter 
    # rem  testing code 1 ########################
    # rem record velocity at switch point
  
    initial_desired_velocity = vel_local
   
    # rem calculate initial desired velocity
    
    desired_dist ,global_pointer ,local_index , initialWegInd_Digits = desired_distance(desired_dist, IstWegMTS, switch_flag, total_shiftdist,  local_index, penetration_vel, global_pointer,  total_shift, initialWegInd_Digits, samplingfrequency_, _OpAmplitude, _OpFrequency, vibration_counter)
  
   

  
  

       
  if (switch_flag == 1):
    
   
    # rem append desired velocity array
    
   
    # rem calculate difference between desired and predict distance
     
       
    # rem calculate difference between desired and predicted distance
    # '    dist_err = desired_dist - predict_dist
    # '    dec(pre_dist_p)
    # '           
    # '    if (pre_dist_p > total_shift) then
    # '      dist_err = desired_dist - predict_distance_array[pre_dist_p - total_shift]
    # '    else
    # '      dist_err = desired_dist - predict_distance_array[datasize_train + pre_dist_p - total_shift]
    # '    endif 
    
    # '  ###########################################################################################
    # rem testing code
     desired_P -= 1
     if (desired_P >= total_shift):
         dist_err = desired_velraw[desired_P - total_shift] - predict_dist
     else:
         dist_err = desired_velraw[datasize_train + desired_P - total_shift] - predict_dist
    
    
    # rem desired velocity for current step (anayltical solution)
     if (desired_P >= total_shift):
     
         anayltical_vel = (desired_dist - desired_velraw[desired_P - total_shift]) / intervalp 
     
     else:
     
         anayltical_vel = (desired_dist - desired_velraw[datasize_train + desired_P - total_shift]) / intervalp 
    
    
    # rem calculate velocity deviation at current step
    
     vel_deviation = dist_err / pro_const
   
    
    # rem desired velocity 
     sollvel = (desired_dist - WegInd_Digits) / intervalp
    
    # rem calculate difference between anayltical desired velocity at current step and actual velocity
    # rem this term is served as a precontrol term 
    # e_k = sollvel - anayltical_vel
    # if (e_k_pointer < 3):
    #   ek_array[e_k_pointer] = e_k
    #   inc(e_k_pointer)
    # else
      
    #   ek_array[1] = ek_array[2]
    #   ek_array[2] = e_k
    # endif
    # rem calculate anayltical control output
     analytical_Regout = ((vel_deviation + anayltical_vel) - linear_coff[1]) / (linear_coff[0])
    
    # rem calculate adaptive parameter w_k based on LEAST square solution
    # w_k = (1 / e_k) * (analytical_Regout * linear_coff[1] + linear_coff[2] - (sollvel + vel_deviation))
    
    
    # rem desired velocity is calculated as difference between desired distance and measured distance 'intervalp-1' points, and then divide by 'interval_p'
    # rem  this is same as how we calculate veloicty data in modelling process.  
    # '    if (pointer_average > (intervalp - 1)) then
    # '      sollvel = (desired_dist - distance_data[pointer_average - (intervalp - 1)]) / intervalp
    # '    else
    # '      sollvel = (desired_dist - distance_data[windowsize + pointer_average - (intervalp - 1)]) / intervalp
    # '       
    # '    endif
    
    # rem controller output from model 
   
     estimatedWegInd_RegOut = ((vel_deviation + sollvel ) -  linear_coff[1]) / ( linear_coff[0])
         
    
                         
 
   
    # rem assign real controller output
    # rem ensure control output no sudden peak
    # '    if (abs(controlleroutput[local_c - 1] - estimatedWegInd_RegOut) < 200) then 
     WegInd_RegOut = estimatedWegInd_RegOut
    # '  endif
    
    
    # '    #######################################################################################################
    # '    testing code halle testing, vibro model control 2023.04.26
    
    # rem  switch to final closed loop control when estimated control output is very close to real control
    # '    if (abs(estimatedWegInd_RegOut - WegInd_RegOut) < 20) then
    # '      final_switch = 1
    # '        
    # '    endif
    # '        
    # '    if (final_switch = 1) then
    # '          
    # '      WegInd_RegOut = estimatedWegInd_RegOut
    # '        
    # '    endif 

         
  return  WegInd_RegOut,switch_flag, total_shiftdist  


    
