import time
import serial
import numpy as np
import matplotlib.pyplot as plt
import sys


from sklearn.neural_network import MLPClassifier 
from sklearn.externals import joblib

filename = sys.argv[1]




# mlp = MLPClassifier(hidden_layer_sizes=(100, 100), max_iter=400, alpha=1e-4,
#                     solver='sgd', verbose=10, tol=1e-4, random_state=1)
mlp = MLPClassifier(hidden_layer_sizes=(66,), max_iter=1000, alpha=1e-4, solver='sgd', verbose=10, tol=1e-4, random_state=1, learning_rate_init=.1)

inputfile = filename + '_input.npy'
outputfile = filename + '_output.npy'

X = np.load(inputfile)
y = np.load(outputfile)


print ("The Neural Network will now be trained with the Collected Data")
print ("Press Enter to begin Training, CTRL+C to exit")
text = raw_input()

#X_train, X_test = X[:60000], X[60000:]
#y_train, y_test = y[:60000], y[60000:]
mlps = []
mlp.fit(X, y)
mlps.append(mlp)

print mlps

print("Training set score: %f" % mlp.score(X, y))
#print("Test set score: %f" % mlp.score(X_test, y_test))
joblib.dump(mlp, (filename + '_network.nn'))



#fig, axes = plt.subplots(4, 4)
# use global min / max to ensure all weights are shown on the same scale
#vmin, vmax = mlp.coefs_[0].min(), mlp.coefs_[0].max()
#for coef, ax in zip(mlp.coefs_[0].T, axes.ravel()):
#    ax.matshow(coef.reshape(65, 65), cmap=plt.cm.gray, vmin=.5 * vmin,
#               vmax=.5 * vmax)
#    ax.set_xticks(())
#    ax.set_yticks(())

#plt.show()

print mlp.predict(X[1:])
