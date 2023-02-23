from Solver import solver 
import numpy as np

def writeToFile(data):
    for i in range (0,np.size(data)): 
        solver.textFile.write(str(data[0,i]))
        solver.textFile.write('\t')
    return
