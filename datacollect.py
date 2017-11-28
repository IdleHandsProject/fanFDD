import time
import serial
import numpy as np
#import matplotlib.pyplot as plt
import sys

numsamples = 5
numcond = 3

datasetinput = np.zeros(shape=(numsamples*numcond,65))

datasetoutput = np.zeros(shape=(numsamples*numcond,2))

filename = sys.argv[1]



ser = serial.Serial('/dev/ttyACM0', 115200)
time.sleep(1)

def collectdata():
    ser.flush()
    ser.write('t')
    time.sleep(5)
    line = ser.readline()
    #print line
    data = map(float, line.split(','))
    #print data
    return data

print ("This program will collect the data to create a training set")
print ("Follow the on screen inscructions")
print ("Press enter to continue")
text = raw_input()

print ("The system will now collect HEALTHY data")
print ("Turn on HEALTHY fan with no faults")
print ("Wait for fan to reach full speed and Press Enter")
text = raw_input()

for x in range(0,(numsamples)):
    #collectdata()
    print x
    datasetinput[x] = collectdata()
    print datasetinput[x]
    datasetoutput[x] = [0,0]



print ("The system will now collect BLOCKED data")
print ("Add cover to inlet of fan and Turn on")
print ("Wait for fan to reach full speed and Press Enter")
text = raw_input()

for x in range(numsamples,(numsamples*2)):
    #collectdata()
    print x
    datasetinput[x] = collectdata()
    print datasetinput[x]
    datasetoutput[x] = [1,0]


print ("The system will now collect BALANCE data")
print ("Add WEIGHTS to fan and turn on")
print ("Wait for fan to reach full speed and Press Enter")
text = raw_input()

for x in range(numsamples*2,(numsamples*3)):
    #collectdata()
    print x
    datasetinput[x] = collectdata()
    print datasetinput[x]
    datasetoutput[x] = [0,1]

inputname = filename + '_input.npy'
outputname = filename + '_output.npy'

np.save(inputname, datasetinput)
np.save(outputname, datasetoutput)

print ("The Training Data has now been collected")
print ("Press Enter to Continue")
text = raw_input()
time.sleep(2)
