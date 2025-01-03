# -*- coding: utf-8 -*-
"""
Created on Fri May 26 19:13:04 2023

@author: dal
"""

import numpy as np
import matplotlib.pyplot as plt
import math as m
import random
import time
from numpy import linalg as LA

ind = 50
dim = 9
gamma = 1000
z = 0
# F = random.uniform(0,1)
F = 0.6
te1 = []
te2 = []
terr = []
qi = []
part = []
muta = []
errr = []
ua = []
newo = []
x = []
y = []
y1= []
y2= []
y3= []
def t(dx,dy,t0,t1,t2,t3,t4,t5,t6):
    
    
    s0 = m.sin(t0)
    s1 = m.sin(t1)
    s2 = m.sin(t2)
    s3 = m.sin(t3)
    s4 = m.sin(t4)
    s5 = m.sin(t5)
    s6 = m.sin(t6)
    
    c0 = m.cos(t0)
    c1 = m.cos(t1)
    c2 = m.cos(t2)
    c3 = m.cos(t3)
    c4 = m.cos(t4)
    c5 = m.cos(t5)
    c6 = m.cos(t6)
    
    a1 = 175
    a2 = 890
    a3 = 50
    x = dx
    y = dy
    d2 = 575
    d5 = 1035
    z = 100
    
    r11 = s5*(s4*(s2*s3*(c0*c1 - s0*s1) - c2*c3*(c0*c1 - s0*s1)) + c4*(c0*s1 + c1*s0)) - c6*(s5*(c2*s3*(c0*c1 - s0*s1) + c3*s2*(c0*c1 - s0*s1)) + c5*(c4*(s2*s3*(c0*c1 - s0*s1) - c2*c3*(c0*c1 - s0*s1)) - s4*(c0*s1 + c1*s0)))
    r12 = s6*(s5*(c2*s3*(c0*c1 - s0*s1) + c3*s2*(c0*c1 - s0*s1)) + c5*(c4*(s2*s3*(c0*c1 - s0*s1) - c2*c3*(c0*c1 - s0*s1)) - s4*(c0*s1 + c1*s0))) + c6*(s4*(s2*s3*(c0*c1 - s0*s1) - c2*c3*(c0*c1 - s0*s1)) + c4*(c0*s1 + c1*s0))
    r13 = c5*(c2*s3*(c0*c1 - s0*s1) + c3*s2*(c0*c1 - s0*s1)) - s5*(c4*(s2*s3*(c0*c1 - s0*s1) - c2*c3*(c0*c1 - s0*s1)) - s4*(c0*s1 + c1*s0))

    
    r21 = c6*(s5*(c2*c3 - s2*s3) + c4*c5*(c2*s3 + c3*s2)) - s4*s5*(c2*s3 + c3*s2)
    r22 = -s6*(s5*(c2*c3 - s2*s3) + c4*c5*(c2*s3 + c3*s2)) - c6*s4*(c2*s3 + c3*s2)
    r23 = c4*s5*(c2*s3 + c3*s2) - c5*(c2*c3 - s2*s3)
    
    r31 = c6*(s5*(c2*s3*(c0*s1 + c1*s0) + c3*s2*(c0*s1 + c1*s0)) + c5*(c4*(s2*s3*(c0*s1 + c1*s0) - c2*c3*(c0*s1 + c1*s0)) + s4*(c0*c1 - s0*s1))) - s5*(s4*(s2*s3*(c0*s1 + c1*s0) - c2*c3*(c0*s1 + c1*s0)) - c4*(c0*c1 - s0*s1))
    r32 = -s6*(s5*(c2*s3*(c0*s1 + c1*s0) + c3*s2*(c0*s1 + c1*s0)) + c5*(c4*(s2*s3*(c0*s1 + c1*s0) - c2*c3*(c0*s1 + c1*s0)) + s4*(c0*c1 - s0*s1))) - c6*(s4*(s2*s3*(c0*s1 + c1*s0) - c2*c3*(c0*s1 + c1*s0)) - c4*(c0*c1 - s0*s1))
    r33 = s5*(c4*(s2*s3*(c0*s1 + c1*s0) - c2*c3*(c0*s1 + c1*s0)) + s4*(c0*c1 - s0*s1)) - c5*(c2*s3*(c0*s1 + c1*s0) + c3*s2*(c0*s1 + c1*s0))
    
    ex = x - a3*(s2*s3*(c0*c1 - s0*s1) - c2*c3*(c0*c1 - s0*s1)) + a1*(c0*c1 - s0*s1) + d2*(c0*s1 + c1*s0) + d5*(s4*(s2*s3*(c0*c1 - s0*s1) - c2*c3*(c0*c1 - s0*s1)) + c4*(c0*s1 + c1*s0)) + a2*c2*(c0*c1 - s0*s1)
    ey = y + a2*s2 + a3*(c2*s3 + c3*s2) - d5*s4*(c2*s3 + c3*s2) + 100
    ez = a3*(s2*s3*(c0*s1 + c1*s0) - c2*c3*(c0*s1 + c1*s0)) - a1*(c0*s1 + c1*s0) + d2*(c0*c1 - s0*s1) - d5*(s4*(s2*s3*(c0*s1 + c1*s0) - c2*c3*(c0*s1 + c1*s0)) - c4*(c0*c1 - s0*s1)) - a2*c2*(c0*s1 + c1*s0)


    
    T = [[r11,r12,r13,ex],
         [r21,r22,r23,ey],
         [r31,r32,r33,ez],
         [0,0,0,1]]
    return T

init = [-2300, -570, 0, 0.02, -112.56, 112.6, 0, 90, 0]        

#目標座標
T0 =np.array([[0, -0.961, 0.276, -1500], #目標座標
              [-0.005, -0.961, 0.276,-900],
              [1, 0 , 0.0017, 800],
              [0, 0, 0, 1]])

bounds=[(-3750, -900), (-1030, -500),  (m.radians(0), m.radians(90)), (m.radians(-2.86), m.radians(90)),\
(m.radians(-116.75), m.radians(-70)), (m.radians(90), m.radians(130)),\
(m.radians(-1), m.radians(1)), (m.radians(40),m.radians(125)), (m.radians(-1), m.radians(1))]

muta1=[]
q = []
q1 = []

for i in range(ind):#50個體xri,G
    for j in range(len(bounds)):
        r = random.random()
        new = bounds[j][0]+(bounds[j][1]-bounds[j][0])*r
        q1.append(new)

q = np.array(q1)
q = np.reshape(q,(ind,9))

for i in range(ind): #初代50個算出來的座標直
    qi = t(q[i][0],q[i][1],q[i][2],q[i][3],q[i][4],q[i][5],q[i][6],q[i][7],q[i][8])
    # qi = np.array(qi)
    part.append(qi)  
    
for k in range(ind):#初代50個體的目標數函數

    te1 = T0 -part[k]
    te2.append(te1) 
    err = LA.norm(te1, ord ='fro')
    err = err.tolist()
    terr.append(err)

g0 = sorted(terr)
t1 = time.time()
while True:
    print(z) 
    
    for i in range(ind):  #rand/1/bin
        pen = 0
        u = [0]*9
        ran = random.sample(range(ind),3)         
        muta = q[ran[0]]+ F*(q[ran[1]]-q[ran[2]]) #變異函數
        muta1.append(muta)

        for n in range(len(bounds)): 

            p1 = random.uniform(0,1)
            # cr = random.uniform(0,1)
            cr = 0.9
            p2 = random.randint(1,9)
            
            if p1 <= cr or n == p2: #交配
                u[n] = muta[n]
            elif p1 > cr and n != p2:#交配
                u[n] = q[i][n]
            if u[n] < bounds[n][0] or u[n] > bounds[n][1]:   #懲罰函數
                pen = pen + 1
            
            # if u[7]>-0.001745 and u[7]<0.001745:
            #     pen = pen + 1
            #     print(u[7])
            #     print('第五軸',u[7])
        # print(pen)
        ua.append(u)
    #u是第一代後代   
        newo = t(u[0],u[1],u[2],u[3],u[4],u[5],u[6],u[7],u[8])
        qq = LA.norm((q[i] - u))
        newe = 0.9*LA.norm((T0 - newo ), ord ='fro') + 0.1*qq + pen*gamma
        errr.append(newe)
        if newe < terr[i]:
            q[i] = u
            terr[i] = newe
    g = sorted(terr)
    
    x.append(z)
    z = z+1
    y.append(g[0])
    r = terr.index(min(terr))
    result = t(q[r][0],q[r][1],q[r][2],q[r][3],q[r][4],q[r][5],q[r][6],q[r][7],q[r][8])
    xx = abs(T0[0][3])-abs(result[0][3])
    yy = abs(T0[1][3])-abs(result[1][3])
    zz = abs(T0[2][3])-abs(result[2][3])
    y1.append(abs(xx))
    y2.append(abs(yy))
    y3.append(abs(zz))
    # ite = [abs(xx),abs(yy),abs(zz)]
    # y.append(ite)
    if z == 1000:
    # if abs(xx) < 1 and abs(yy) < 1 and abs(zz) < 1:
        r = terr.index(min(terr))
        print(terr.index(min(terr)))
        result = t(q[r][0],q[r][1],q[r][2],q[r][3],q[r][4],q[r][5],q[r][6],q[r][7],q[r][8])
        print('result:', result)
        q[r] = q[r][0],q[r][1],m.degrees(q[r][2]),m.degrees(q[r][3]),m.degrees(q[r][4]),m.degrees(q[r][5]),m.degrees(q[r][6]),m.degrees(q[r][7]),m.degrees(q[r][8])
        print('9dof:', q[r])
        t2 = time.time()
        print(t2-t1)
        break
# # x = np.array(x)
# # y = np.array(y)
# # print(x.shape, y.shape)

# plt.xlabel('iter')
plt.plot(x, y)
# plt.subplot(1,1,1)
# plt.title('x_error')
# plt.xlabel('iter')
# plt.plot(x, y1)
# plt.subplot(1,1,2)
# plt.title('y_error')
# plt.xlabel('iter')
# plt.plot(x, y2)
# plt.subplot(1,1,3)
# plt.title('z_error')
# plt.xlabel('iter')
# plt.plot(x, y3)
