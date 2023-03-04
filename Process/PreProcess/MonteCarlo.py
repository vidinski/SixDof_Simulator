import numpy as np
import matplotlib.pyplot as plt

textFile = open('MonteCarlos.sh','w')

for i in range(1,5): 
    text = 'python Main.py --finalTime 1.0 --run ' + str(i)
    textFile.write(text + '\n')

textFile.close()