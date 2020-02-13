import pylab as pl
import os
from math import *

resistor = 4700
farad = 1 * 10**-9

frequency = [ i**3 for i in range(1, 100)]
pl.grid()
capacitor_impedance = [1/(2 * pi * i * farad) for i in frequency]
passthrough = [x / (resistor + x) for x in capacitor_impedance]
pl.plot(frequency, passthrough)
pl.xscale("log")
pl.waitforbuttonpress()
pl.savefig(os.path.abspath(".") + os.path.sep + "lowpass_impedance.png")
