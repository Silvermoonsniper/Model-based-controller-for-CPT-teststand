# -*- coding: utf-8 -*-
"""
Created on Wed Oct 18 10:32:11 2023

@author: zchen
"""

def CPT_depth_calulation(_stopOperation, _Taster_stop, offset_index, IstWegMTS, CPT_depth_offset):
    
    # rem record depth during CPT test
    # rem if vibro pushing is not finished, record depth as distance data measurement
    # '      if ((bitStatus_01 = sysState_vibroPushingEnd) or (bitStatus_01 = SysState_vibro_Hand)) then
    # rem depth is final position of piston, when one push is over
    if ((_stopOperation == 1) or (_Taster_stop == 1)):
      if (offset_index == 1):
        CPT_depth_offset = CPT_depth_offset + IstWegMTS
        CPT_depth = CPT_depth_offset
        offset_index += 1
      else:
        CPT_depth = CPT_depth_offset
      
    else:
      if (offset_index == 1):
        CPT_depth = CPT_depth_offset + IstWegMTS
      else:
        CPT_depth = CPT_depth_offset
    return CPT_depth, offset_index,CPT_depth_offset