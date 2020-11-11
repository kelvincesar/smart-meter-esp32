import math

samples = 2048

hann_k = 2 * math.pi / (samples-1)

values = []


for n in range(0, samples):
    values.append((1 - math.cos(hann_k * n)) / 2)

import json
with open('hanning-window.txt', 'w') as outfile:
    json.dump(values, outfile)
