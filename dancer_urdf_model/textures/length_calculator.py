#!/usr/bin/python2
# -*- coding: utf-8 -*
mass = input("mass value = ")
print(mass)
ixx = input("ixx = ")
print(ixx)
iyy = input("iyy = ")
print(iyy)
izz = input("izz = ")
print(izz)

whole = (ixx + iyy + izz)*0.5
x2 = (whole - ixx)*3/mass
y2 = (whole - iyy)*3/mass
z2 = (whole - izz)*3/mass

print("x2 = ", x2)
print("y2 = ", y2)
print("z2 = ", z2)

if x2 < 0:
  x2 = -1*x2
if y2 < 0:
  y2 = -1*y2
if z2 < 0:
  z2 = -1*z2

x = x2 ** 0.5
y = y2 ** 0.5
z = z2 ** 0.5

print("x = ", x)
print("y = ", y)
print("z = ", z)
