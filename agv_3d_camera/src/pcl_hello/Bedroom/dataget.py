# -*- coding:utf-8 -*-


import os




f_in = open('floorplan-v2-2-_0_bedroom-2_r0.asc', 'r')
f_out= open('floorplan-v2-2-_0_bedroom-2_r0.pcd', 'w')
for line in f_in:
    items = line.split(' ')
    f_out.write(items[0]+" "+items[1]+" "+items[2]+'\n')

