#!/usr/bin/python2
# -*- coding: utf-8 -*
mass = input("mass value = ")
print(mass)
x = input("x = ")
print(x)
y = input("y = ")
print(y)
z = input("z = ")
print(z)
#转动惯量Ixx=∫(y*y+z*z)dm=∫∫(y*y+z*z)dy*dz*m/(ly*lz)（恒正）
ixx = mass*(y*y + z*z)*(0.333)
iyy = mass*(x*x + z*z)*(0.333)
izz = mass*(x*x + y*y)*(0.333)
#惯性积Ixy=∫(x*y)dm=∫∫(x*y)dx*dy*m/(lx*ly)（可正可负）
ixy = mass*(x*y)*(0.25)
ixz = mass*(x*z)*(0.25)
iyz = mass*(y*z)*(0.25)

print("      <inertia")
temp = '        ixx="' + str(ixx) + '"'
print(temp)
temp = '        ixy="' + str(ixy) + '"'
print(temp)
temp = '        ixz="' + str(ixz) + '"'
print(temp)
temp = '        iyy="' + str(iyy) + '"'
print(temp)
temp = '        iyz="' + str(iyz) + '"'
print(temp)
temp = '        izz="' + str(izz) + '"'
print(temp)
