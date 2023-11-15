# -*- coding: utf-8 -*-
"""
Created on Wed Oct 18 15:19:57 2023

@author: zchen
"""



def sub_blockcalculation(Abtastfrequenz, Sinusfrequenz_Vibro, v_fullFLAG, local_vel_pointer, data_sizetrain, velocity_data, linear_coff, newvelocity_data, newcontrolleroutput, subxmean, subymean, subxydiffpsum, subxsquaresum, pointer_vel_regression, WegInd_RegOut, vel_local, total_shift, fulldata_number):

 
  
 
  # rem calculate sub mean, at each cycle we update mean and sum calculations until buffer is full
 
   
  # rem record velocity and controller output in the array 
  if ((pointer_vel_regression < fulldata_number)):
      
    # rem when data buffer are not full
    # rem velocity data some steps before, as velocity data is delayed by some time steps
    if ((v_fullFLAG == 0)):
      if ((local_vel_pointer >= total_shift)):
        newvelocity_data[pointer_vel_regression] = velocity_data[local_vel_pointer - total_shift]
    
      else:
        newvelocity_data[pointer_vel_regression] = velocity_data[data_sizetrain + (local_vel_pointer - total_shift)]
      
   
      # rem current controller output
      newcontrolleroutput[pointer_vel_regression] = WegInd_RegOut
    
    # rem update x and y means
    if (v_fullFLAG == 0):
      subxmean = subxmean + (newcontrolleroutput[pointer_vel_regression] / fulldata_number)
     
      subymean = subymean + (newvelocity_data[pointer_vel_regression] / fulldata_number)
      # rem calculate sub xy sum and xx sum for linear regression 
      subxydiffpsum = subxydiffpsum + newcontrolleroutput[pointer_vel_regression] * newvelocity_data[pointer_vel_regression]
      subxsquaresum = subxsquaresum + newcontrolleroutput[pointer_vel_regression] * newcontrolleroutput[pointer_vel_regression]
    
    
    if ((pointer_vel_regression == fulldata_number-1) and (v_fullFLAG == 0)):
      # rem set data buffer full flag
      v_fullFLAG = 1
      # rem calculate estimated parameters
      linear_coff[0] = (subxydiffpsum - subymean * fulldata_number * subxmean) / (subxsquaresum - fulldata_number * subxmean * subxmean)  #'m Steigung  
  
      linear_coff[1] = subymean - linear_coff[0] * subxmean  #'b yAchsenabsch
    
    # rem when data buffer are full, subtract old term values and add new term
    if (v_fullFLAG == 1):
     
      # rem subtract old value
      subxmean = subxmean - (newcontrolleroutput[pointer_vel_regression] / fulldata_number)
     
      subymean = subymean - (newvelocity_data[pointer_vel_regression] / fulldata_number)
      subxydiffpsum = subxydiffpsum - newcontrolleroutput[pointer_vel_regression] * newvelocity_data[pointer_vel_regression]
      subxsquaresum = subxsquaresum - newcontrolleroutput[pointer_vel_regression] * newcontrolleroutput[pointer_vel_regression]
      # rem assign current values
      if ((local_vel_pointer >= total_shift)):
        newvelocity_data[pointer_vel_regression] = velocity_data[local_vel_pointer - total_shift]
    
      else:
        newvelocity_data[pointer_vel_regression] = velocity_data[data_sizetrain  + (local_vel_pointer - total_shift)]
      
      newcontrolleroutput[pointer_vel_regression] = WegInd_RegOut
      
        
      # rem add new value
      subxmean = subxmean + (newcontrolleroutput[pointer_vel_regression] / fulldata_number)
      subymean = subymean + (newvelocity_data[pointer_vel_regression] / fulldata_number)
      subxydiffpsum = subxydiffpsum + newcontrolleroutput[pointer_vel_regression] * newvelocity_data[pointer_vel_regression]
      subxsquaresum = subxsquaresum + newcontrolleroutput[pointer_vel_regression] * newcontrolleroutput[pointer_vel_regression]
      
      
      # rem calculate estimated parameters
      linear_coff[0] = (subxydiffpsum - subymean * fulldata_number * subxmean) / (subxsquaresum - fulldata_number * subxmean * subxmean)  #'m Steigung  
  
      linear_coff[1] = subymean - linear_coff[0] * subxmean  #'b yAchsenabsch
     
    
    
   
    
    # rem update index
    pointer_vel_regression = pointer_vel_regression + 1
      
    
  else:
    # rem if we reach the end of array, reinitialize index 
    pointer_vel_regression = 0
    # rem full flag 
    v_fullFLAG = 1
    
    
    # rem when data buffer are not full
    # rem velocity data some steps before, as velocity data is delayed by some time steps
    if ((v_fullFLAG == 0)):
      if ((local_vel_pointer >= total_shift)):
        newvelocity_data[pointer_vel_regression] = velocity_data[local_vel_pointer - total_shift]
    
      else:
        newvelocity_data[pointer_vel_regression] = velocity_data[data_sizetrain + (local_vel_pointer - total_shift)]
      
   
      # rem current controller output
      newcontrolleroutput[pointer_vel_regression] = WegInd_RegOut
    
    # rem update x and y means
    if (v_fullFLAG == 0):
      subxmean = subxmean + (newcontrolleroutput[pointer_vel_regression] / fulldata_number)
     
      subymean = subymean + (newvelocity_data[pointer_vel_regression] / fulldata_number)
      # rem calculate sub xy sum and xx sum for linear regression 
      subxydiffpsum = subxydiffpsum + newcontrolleroutput[pointer_vel_regression] * newvelocity_data[pointer_vel_regression]
      subxsquaresum = subxsquaresum + newcontrolleroutput[pointer_vel_regression] * newcontrolleroutput[pointer_vel_regression]
    
    
    if ((pointer_vel_regression == fulldata_number-1) and (v_fullFLAG == 0)):
      # rem set data buffer full flag
      v_fullFLAG = 1
      # rem calculate estimated parameters
      linear_coff[0] = (subxydiffpsum - subymean * fulldata_number * subxmean) / (subxsquaresum - fulldata_number * subxmean * subxmean)  #'m Steigung  
  
      linear_coff[1] = subymean - linear_coff[0] * subxmean  #'b yAchsenabsch
    
    # rem when data buffer are full, subtract old term values and add new term
    if (v_fullFLAG == 1):
     
      # rem subtract old value
      subxmean = subxmean - (newcontrolleroutput[pointer_vel_regression] / fulldata_number)
     
      subymean = subymean - (newvelocity_data[pointer_vel_regression] / fulldata_number)
      subxydiffpsum = subxydiffpsum - newcontrolleroutput[pointer_vel_regression] * newvelocity_data[pointer_vel_regression]
      subxsquaresum = subxsquaresum - newcontrolleroutput[pointer_vel_regression] * newcontrolleroutput[pointer_vel_regression]
      # rem assign current values
      if ((local_vel_pointer >= total_shift)):
        newvelocity_data[pointer_vel_regression] = velocity_data[local_vel_pointer - total_shift]
    
      else:
        newvelocity_data[pointer_vel_regression] = velocity_data[data_sizetrain  + (local_vel_pointer - total_shift)]
      
      newcontrolleroutput[pointer_vel_regression] = WegInd_RegOut
      
        
      # rem add new value
      subxmean = subxmean + (newcontrolleroutput[pointer_vel_regression] / fulldata_number)
      subymean = subymean + (newvelocity_data[pointer_vel_regression] / fulldata_number)
      subxydiffpsum = subxydiffpsum + newcontrolleroutput[pointer_vel_regression] * newvelocity_data[pointer_vel_regression]
      subxsquaresum = subxsquaresum + newcontrolleroutput[pointer_vel_regression] * newcontrolleroutput[pointer_vel_regression]
      
      
      # rem calculate estimated parameters
      linear_coff[0] = (subxydiffpsum - subymean * fulldata_number * subxmean) / (subxsquaresum - fulldata_number * subxmean * subxmean)  #'m Steigung  
  
      linear_coff[1] = subymean - linear_coff[0] * subxmean  #'b yAchsenabsch
     
    
    
   
    
    # rem update index
    pointer_vel_regression = pointer_vel_regression + 1
  
  return linear_coff,pointer_vel_regression,v_fullFLAG, subxmean, subymean, subxydiffpsum, subxsquaresum
   
 