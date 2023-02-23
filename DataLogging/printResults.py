from Solver import solver 
import numpy as np

def writeToFile(data):

    # solver.textFile.write('test')
    # solver.textFile.write('\n')

    for i in range (0,np.size(data)): 
        solver.textFile.write(str(data[0,i]))
        solver.textFile.write('\t')
    return
