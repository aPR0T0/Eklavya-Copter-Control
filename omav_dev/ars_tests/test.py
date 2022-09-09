import numpy as np
import math

def control(a1):
    print(5)
    return (5)

def control1(a1):
    print(10)
    return(10)

def med(a1):
    b1 = control(a1)
    b2 = control1(a1)
    print(b1*b2)

if __name__=='__main__':
    med(5)