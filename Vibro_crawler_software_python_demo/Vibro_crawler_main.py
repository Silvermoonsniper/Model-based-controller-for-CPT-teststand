# -*- coding: utf-8 -*-
"""
Created on Tue Oct 17 14:32:48 2023

@author: zchen
"""

 #-----------------------------------------------------------------------------------------------------------------------------      
     
 




# rem varibales declaration






import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
#from Definition import *
import matplotlib.gridspec as gridspec
#from BC5NewApplicationAnalysis import *
from cycler import cycler
import vibro_state_machine
import bit_operator  
import seat_valve_control
import CPT_depth_calc
import hold_position_piston
import MTSdata_preprocessing
import Linear_regression_incremental
import model_predictive_control
import CPT_penetration_static
import plot_functions

if   __name__ == "__main__":

      # read testing data
      
      datapath =  r'field_test_2023_07_25\1000_p_const_20hz_6mm_control.dat.fast'  
      
      simulation_step = 3000
     
      raw_measurementdata = pd.read_table(datapath,skiprows = 5 , nrows = simulation_step, delimiter='\t',encoding='latin-1',engine='python')
      
      # remove spikes in the data
      raw_measurementdata = raw_measurementdata[abs(raw_measurementdata.Stellwert) < 65536]

      # first switch to model control mode, initial paras
      vibroOpCmd_model_control = 1
      
      autodisableSitzventilTimer = 0 
      
      sitzventil_High = 0
      
      autodisableSitzventile = 0
      
      autoEnableSitzventile = 0

      autoSitzventilMaxTime  = 2500
      
      autodisableSitzventilTimer = 0
      
      autoEnableSitzventilTimer = 0
      
      _startOperation = 0
      
      _stopOperation = 0
      
      _enableHandventil = 0
      
      _disableHandventil = 0
      
      _Taster_start = 0
      
      _Taster_stop  = 0 
      
      _Taster_frei1 = 0
      
      _Taster_frei2 = 0
      
      sysState_bit = {
          'sysState_vibroPushingEnd': 0x10  , 
          'SysState_vibro_Hand': 0x200  ,
          'sysState_vibroPushing':  0x08,
          'sysState_vibroActivated': 0x04
          }
      
      BitStatus_01 = sysState_bit['sysState_vibroActivated']
      
      offset_index = 1
      
      CPT_depth_offset  = 0
      
      intervalp = 60
      
      model_true_time_step = 13
      
      total_shift = 81
      
      vibration_counter = 1
      
      samplingfrequency_  = 5000
      
      _OpAmplitude = 10
      
      _OpFrequency = 20
      
      v_fullFLAG = 0
      
      switch_flag = 0
      
      local_index = 1
      
      SollWeg = raw_measurementdata.IstWegMTS[0]
      
      model_control_switch = 0
      
      seat_Valve_button = 0
      
      static_penetration_activate = 0
      
      Kp_wegHalten = 3000
      
      realdistance_data = np.zeros(100)
      
      windowsize = 1320
      
      distance_data = np.zeros(windowsize)
      
      data_sizetrain = 10000
      
      velocity_data = np.zeros(data_sizetrain)
      
      controlleroutput = np.zeros(data_sizetrain)#
      
      newvelocity_data = np.zeros(10500)
      
      newcontrolleroutput = np.zeros(10500)
      
      Programm_counter = -1
      
      pointer_average = -1
      
      local_vel_pointer = 0
      
      local_c = 0
      
      vel_local = 0
      
      sitzventil_High = 0
      
      Stellwert = 0
      
      SummeRegelabweichung = 0
      
      _Kp = 1500
      
      _Ki = 2000
      
      _endeMessbereich = 670 
      
      fulldata_number = 1 * samplingfrequency_ / _OpFrequency
      
      subxmean = 0
      
      subymean = 0  
      
      subxydiffpsum = 0
      
      subxsquaresum = 0
      
      linear_coff = np.zeros(2)
      
      linear_coff[0] = 3.379455683109773e-6
  
      linear_coff[1] = -0.10541358321865224
      
      pointer_vel_regression = 0
      
      start_control_time = 0
      
      end_control_time = 15 
      
      model_control_cycle_num = 0
      
      Soll_Position_Regelungswert = 0
      
      linear_model = 1
      
      measure_dist_array = []
      
      predict_dist_array = []
      
      desired_dist_array = []
      
      fitted_velocity_array = []
      
      controloutput_model_array = []
      
      actual_vel_array = []
      
      m_array = []
      
      b_array = []
      
      predict_distance_pointer = 1
      
      predict_full = 0
      
      estima_vel_sub = np.zeros(20000)
      
      estimated_slope_array = np.zeros(30)
      
      estimated_intercept_array = np.zeros(30)
      
      pre_dist_p = 0
      
      predict_distance_array = np.zeros(data_sizetrain+1)
      
      desired_P = 0
      
      desired_velraw = np.zeros(data_sizetrain)
      
      pro_const = 5000
      
      total_shiftdist = 0
      
      penetration_vel = 20
      
      global_pointer = 0
      
      initialWegInd_Digits = 0
      
      safe_m = 0
      
      safe_b = 0
      
      full_flag = 0
      
      estimated_parameter_pointer = 0
      
      initial_desired_velocity = 0
      
      sollvel = 0
      
      switch_flag = 1
      
      model_control_switch = 1
      
      estimatedWegInd_RegOut = 0
      
      vel_deviation = 0
      
      dist_err = 0
      
      start,end = 230, 2200
      
      # rem if time delay obtained by correlation appears wrong value, correct with confident value 
      # rem check if cone is connected, if so, step is double. 
      # if (synced_ = 0):
      #   if (abs(par_31 - 81) < 5) then
      #     total_shift = par_31
      #   else:
      #     total_shift = 81
        
      # else:
      #   if (abs(par_31 - 41) < 5) then
      #     total_shift = par_31
      #   else:
      #     total_shift = 41
          
          
      
      
      j = 0
      for j in range(len(raw_measurementdata)):
                                
      ###############################################
          ############################################
           if ( j<= (end_control_time/(1/samplingfrequency_)) and (j>= (start_control_time/(1/samplingfrequency_)))):
              
              _startOperation = 1
              
              seat_Valve_button = 1
           else:
              
              _stopOperation = 0
              
              seat_Valve_button = 0
      
          # obtain data from distance sensor
      
           IstWegMTS = np.array(raw_measurementdata.IstWegMTS)[j]
      
           WegInd_RegOut = np.array(raw_measurementdata.Stellwert)[j]
      
        
      
           # vibro crawler operating state
          

           v_fullFLAG, switch_flag, local_index, vibroOpCmd_model_control, _stopOperation, _startOperation, _enableHandventil,_disableHandventil,  BitStatus_01, sitzventil_High, SollWeg = vibro_state_machine. vibro_crawler_state_machine(BitStatus_01,vibroOpCmd_model_control,autodisableSitzventilTimer, autoSitzventilMaxTime,_startOperation , _stopOperation, _enableHandventil, _disableHandventil, _Taster_start, _Taster_stop, _Taster_frei1, _Taster_frei2, sysState_bit,IstWegMTS,v_fullFLAG, switch_flag, local_index, SollWeg, sitzventil_High)            

            # control seat valve
                 
           autoEnableSitzventilTimer,autodisableSitzventilTimer, model_control_switch, seat_valve_channel_0_output, seat_valve_channel_1_output, sitzventil_High  = seat_valve_control.Seat_valvecontrol(model_control_switch,seat_Valve_button, autoSitzventilMaxTime,autoEnableSitzventilTimer,autodisableSitzventilTimer, autoEnableSitzventile,autodisableSitzventile , sitzventil_High)
            # calculate CPT depth
           
           if (IstWegMTS > 670): 
               _stopOperation = 1
         
           CPT_depth, offset_index,CPT_depth_offset = CPT_depth_calc.CPT_depth_calulation(_stopOperation, _Taster_stop, offset_index, IstWegMTS, CPT_depth_offset)
           
           
           _stopOperation = 0
           offset_index = 1
      
           
          # '-----------------------------------------------------------------------------------------------------------------------------      
          ###  control simulation loop
      
          # update pointer for sinus displacement generation
      
           point_num_singlecycle = round(samplingfrequency_ / _OpFrequency)

           if (vibration_counter < point_num_singlecycle):
             vibration_counter += 1
           else:
             vibration_counter = 1
      
           Stellwert, Soll_Position_Regelungswert = hold_position_piston.hold_position_PI_control(Stellwert,static_penetration_activate, vibroOpCmd_model_control,IstWegMTS , SollWeg,Kp_wegHalten, Soll_Position_Regelungswert)
       
           # rem smoothing processing for noisy distance data measurement and      
           # rem calculate velocity based on smoothed distance data
           # rem smooth method; perform linear regression and take intercept 
           
   
         
         
         
           smoothed_dist,vel_full_flag, Programm_counter,vel_local = MTSdata_preprocessing.distance_data_regression(IstWegMTS,  realdistance_data,v_fullFLAG, intervalp , Programm_counter)  
           
           # rem record smoothed distance
           
           distance_data, pointer_average = MTSdata_preprocessing.Distance_data_smooth(distance_data, pointer_average, windowsize, IstWegMTS)         
           # rem record control output and velocity with array
        
           velocity_data, controlleroutput, local_vel_pointer,local_c = MTSdata_preprocessing.vel_controloutput_array(WegInd_RegOut, local_c, controlleroutput, vel_local, velocity_data, local_vel_pointer, data_sizetrain)  
          
           
    
    
    
    
    
          
    ################################################################################################################
           
           if ((seat_Valve_button == 1)):
               
    ################################################################################################################
              
             if ((static_penetration_activate == 1) and (vibroOpCmd_model_control == 2)): 
             
            
               # rem change systemstatusbit
               
               BitStatus_01 = bit_operator.ClearBit(BitStatus_01, sysState_bit['sysState_vibroActivated'])
              
               # rem set systemstatusbit
               BitStatus_01 = bit_operator.setBit(BitStatus_01, sysState_bit['sysState_vibroPushing'])
               
               # perform Static CPT
               CPT_penetration_static.static_CPT(IstWegMTS,SummeRegelabweichung, (1/samplingfrequency_) ,_Kp,_Ki,model_control_cycle_num,Stellwert, _endeMessbereich,Soll_Position_Regelungswert)
    
      
    ################################################################################################################
     # linear regression with incremental approach
      
           linear_coff,pointer_vel_regression,v_fullFLAG, subxmean, subymean, subxydiffpsum, subxsquaresum = Linear_regression_incremental.sub_blockcalculation(samplingfrequency_, _OpFrequency, v_fullFLAG, local_vel_pointer-1, data_sizetrain, velocity_data, linear_coff, newvelocity_data, newcontrolleroutput, subxmean, subymean, subxydiffpsum, subxsquaresum, pointer_vel_regression, WegInd_RegOut, vel_local, round(82 * 2.5), fulldata_number)
      
           
      # rem assign estimated paras
           estimated_slope = linear_coff[0]  
           estimated_intercept = linear_coff[1]
           
           m_array.append(linear_coff[0]  )
           b_array.append(linear_coff[1]  )
           # rem calculate fitted velocity
       
           fitted_vel = linear_coff[0] * WegInd_RegOut + linear_coff[1]
           
           
 
      # rem predicted distance calculation
             
           MAX_correlationstep = round(total_shift) 
           
      # rem the time delay used to calculate predicted distance should be subtracted by additional time delay added during velocity calculation stage
           predict_dist, predict_distance_pointer, predict_full = model_predictive_control.predict_distance(predict_distance_pointer, predict_full, estima_vel_sub, WegInd_RegOut, windowsize, pointer_average, local_c, distance_data, controlleroutput, linear_coff, data_sizetrain, MAX_correlationstep) 
             
      # rem store predicted distance into an array
           if (pre_dist_p <= data_sizetrain):
                predict_distance_array[pre_dist_p] = predict_dist
                pre_dist_p += 1  
           else:
                pre_dist_p = 1
                predict_distance_array[pre_dist_p] = predict_dist
                pre_dist_p += 1  
      
      # rem assign desired distance tobe measured distance when we don't switch to model control
           desired_dist = IstWegMTS
           
      #' ------------------------------------------------------------------------------------------------------------------------------------     
     
      #' ------------------------------------------------------------------------------------------------------------------------------------     
           if (seat_Valve_button == 1):
        # rem if we use linear model and also the start button is pressed
                    if ((linear_model == 1) and (vibroOpCmd_model_control == 2)):
        
                             
                             BitStatus_01 = bit_operator.ClearBit(BitStatus_01, sysState_bit['sysState_vibroActivated'])
        
          # rem set systemstatusbit
                             BitStatus_01 = bit_operator.setBit(BitStatus_01, sysState_bit['sysState_vibroPushing'])
        
                             model_control_cycle_num += 1
        
       
        
        
        
          # rem wait for some time to open seat valve, then automatically switch to model control
          
          # '--------------------------------------------------------------------------------------------------------------------------------------
          # '      closed loop control with Model Predictive controller
      
                             
         
                    
        
          
          ##################################################################################################################
        
        
          # rem when we press the start model control button 
                             
                             if (model_control_switch == 1):
            # rem desired distance calculation
                                    
                                     desired_dist ,global_pointer ,local_index , initialWegInd_Digits =   model_predictive_control.desired_distance(desired_dist, smoothed_dist, switch_flag, total_shiftdist,  local_index, penetration_vel, global_pointer,  total_shift, initialWegInd_Digits, samplingfrequency_, _OpAmplitude, _OpFrequency, vibration_counter)
                                     desired_dist = np.array(raw_measurementdata.SollWeg)[j]
          
        
            # rem justify paramaters when it deviates too much
          
                                  
                                     safe_m, safe_b, linear_coff, estimated_parameter_pointer = model_predictive_control.estimated_parameters_justification(safe_m, safe_b,fulldata_number, full_flag, estimated_parameter_pointer, linear_coff, estimated_slope_array, estimated_intercept_array)         
            
           
         
            # rem closed loop control with controller output calculated from model 
            
                                     WegInd_RegOut, switch_flag, total_shiftdist = model_predictive_control.model_control(desired_velraw, desired_P,  pro_const, vel_local, predict_distance_array, data_sizetrain, pre_dist_p, local_index, penetration_vel, IstWegMTS, samplingfrequency_, _OpAmplitude, _OpFrequency, vibration_counter, desired_dist, intervalp, linear_coff, vibration_counter, total_shiftdist, WegInd_RegOut, sollvel, initialWegInd_Digits, predict_dist, estimatedWegInd_RegOut, 0, switch_flag, global_pointer, smoothed_dist, vel_deviation, dist_err)
                                     
                             else:
          
            # rem calculate difference between smoothed and predicted distance
          
        
                                     desired_P = desired_P -1
                                     
                                     if (desired_P > MAX_correlationstep):
                                            
                                            dist_err = desired_velraw[desired_P - MAX_correlationstep] - predict_dist
                                     else:
                                            
                                            dist_err = desired_velraw[data_sizetrain + desired_P - MAX_correlationstep] - predict_dist
            
          
            # rem calculate velocity deviation at current step
    
                                     vel_deviation = dist_err / pro_const
       
           
            # rem controller output from model 
                                     estimatedWegInd_RegOut = (vel_deviation + 1 * (vel_local) -  linear_coff[1]) / ( linear_coff[0])     
      
      
          
        
      # rem append desired distance into one array
           if (desired_P < data_sizetrain):
                  
                desired_velraw[desired_P] = desired_dist
        
                desired_P += 1
      
           else:
        
                desired_P = 0
        
                desired_velraw[desired_P] = desired_dist
        
                desired_P += 1
      
      
      # rem assign desired distance into fast dataset
           sollweg = desired_dist
            
            #append for distance data
            
           measure_dist_array.append(IstWegMTS)
            
           predict_dist_array.append(predict_dist)
            
           desired_dist_array.append(desired_dist)
           
           controloutput_model_array.append(WegInd_RegOut)
      
           fitted_velocity_array.append(fitted_vel)
           
           actual_vel_array.append(vel_local)
      
      ##################################
      # plot function for control performance
      plot_functions.control_performance_Plot(measure_dist_array,desired_dist_array,predict_dist_array,controloutput_model_array)
     
      # plot function for model accuracy
      plot_functions.fitted_velocity_plot(measure_dist_array,start,end,actual_vel_array, fitted_velocity_array)
      
      # ################################
      # start,end = 230, 1200
      # time_delay = 205
      # plt.plot(controloutput_model_array[start+ time_delay:end +time_delay],actual_vel_array[start:end])
      # start,end = 1230, 2200
      # plt.plot(controloutput_model_array[start:end],actual_vel_array[start:end])
      # start,end = 2230, 7200
      # plt.plot(controloutput_model_array[start:end],actual_vel_array[start:end])
      
 
      
      
      
      
      # '       closed loop control with Model Predictive controller
      
      # '--------------------------------------------------------------------------------------------------------------------------------------
    
      
      
      
     
   
