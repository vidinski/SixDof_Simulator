import numpy as np
import matplotlib.pyplot as plt

textFile = open('sweep.sh','w')

for i in range(1,5): 
    text = 'python Main.py --finalTime 1.0 --run ' + str(i)
    textFile.write(text + '\n')

textFile.close()