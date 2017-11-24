import time
import serial
import numpy as np
import matplotlib.pyplot as plt
import sys
import pylab as plt

from sklearn.neural_network import MLPClassifier 
from sklearn.externals import joblib

filename = sys.argv[1]

def collectdata():
    ser.flush()
    ser.write('t')
    time.sleep(5)
    line = ser.readline()
    #print line
    data = map(float, line.split(','))
    #print data
    return data

ser = serial.Serial('/dev/ttyACM0', 115200)
time.sleep(1)

# mlp = MLPClassifier(hidden_layer_sizes=(100, 100), max_iter=400, alpha=1e-4,
#                     solver='sgd', verbose=10, tol=1e-4, random_state=1)
mlp = MLPClassifier(hidden_layer_sizes=(66,), max_iter=1000, alpha=1e-4, solver='sgd', verbose=10, tol=1e-4, random_state=1, learning_rate_init=.1)

inputfile = filename + '_input.npy'
outputfile = filename + '_output.npy'

X = np.load(inputfile)
y = np.load(outputfile)

print ("The System will now detect the Fault")
print ("Press Enter to Continue, CTRL+C to Quit at Any Time")
text = raw_input()

#X_train, X_test = X[:60000], X[60000:]
#y_train, y_test = y[:60000], y[60000:]


mlp = joblib.load(filename + '_network.nn')

G = np.linspace(0,65,65)
livesample = collectdata()
drawfreq = livesample

plt.ion()
graph = plt.plot(G,drawfreq)[0]
while True:
    livesample = collectdata()
    drawfreq = livesample
    
    graph.set_ydata(drawfreq)
    plt.draw()
    prediction =  mlp.predict([livesample])
    

    if (prediction[0,0] == 0 and prediction[0,1] == 0):
        print "HEALTHY"
    if (prediction[0,0] == 1 and prediction[0,1] == 0):
        print "AIR BLOCKAGE"
    if (prediction[0,0] == 0 and prediction[0,1] == 1):
        print "BALANCE ISSUE"
    if (prediction[0,0] == 1 and prediction[0,1] == 1):
        print "AIR BLOCKAGE + BALANCE ISSUE"
