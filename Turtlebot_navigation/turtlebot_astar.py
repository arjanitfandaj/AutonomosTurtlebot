#!/usr/bin/python

'''

Author: Arjanit Fandaj(@arjanitfandaj)

All right reserved.

Contact: Arjanitfandaj@gmail.com





'''






from os import read


c = []

filename = 'text.txt'

def readfile(filename):
    with open(filename) as f:
        c =  [[int(num) for num in line.split(",")] for line in f]
        return c


b = readfile('x_y.txt')

print(b[0][::-1])