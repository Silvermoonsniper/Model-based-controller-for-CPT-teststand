# -*- coding: utf-8 -*-
"""
Created on Mon Oct 23 15:06:02 2023

@author: zchen
"""



import numpy as np
import matplotlib.pyplot as plt
def  control_performance_Plot(measure_dist_array,desired_dist_array,predict_dist_array,controloutput_model_array):
    
    
    
    time = (1/5000)*np.arange(len(measure_dist_array))
    
    start,end = 270, 2200
    
    plt.figure(figsize = [10,8])
    
    plt.subplot(311)
    
    timedelay = 106
    
    timedelay_1 = 83
          
    plt.plot(time[start:end], measure_dist_array[start:end])#
    
    plt.plot(time[start:end], desired_dist_array[start-timedelay:end- timedelay])
    
    plt.plot(time[start:end], predict_dist_array[start:end])
    
    plt.plot(time[start:end], desired_dist_array[start-24:end-24]) 
    
    plt.plot(time[start:end], predict_dist_array[start+timedelay_1:end+timedelay_1])
    
    plt.xlabel('time (s)')
    plt.ylabel('penetration depth (mm)')
    plt.legend(['measure', 'desire', 'predicted' ,'shifted desire', 'shifted predicted'])
    plt.title('Model control simulation and analyse for CPT Teststand')
    
    plt.subplot(312)
    
    plt.plot(time[start:end], controloutput_model_array[start:end])
    plt.xlabel('time (s)')
    plt.ylabel('digital control output')
    
   # plt.plot(time[start:end], raw_measurementdata.Stellwert[start:end])
    
    plt.subplot(313)
    
    plt.plot(time[start:end], np.array(predict_dist_array[start:end]) - np.array(desired_dist_array[start-timedelay:end- timedelay]))
    
    plt.plot(time[start:end], np.array(measure_dist_array[start:end]) - np.array(desired_dist_array[start-24:end-24]))
   
    plt.plot(time[start:end], np.array(predict_dist_array[start+timedelay_1:end+timedelay_1]) - np.array(measure_dist_array[start:end]))
      
    plt.xlabel('time (s)')
    plt.ylabel('error (mm)')
    plt.legend(['predicted - desired ', 'shifted desire - measured', 'shifted measured  -  desire'])
    
    plt.tight_layout()