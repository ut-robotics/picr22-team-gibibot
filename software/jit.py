from numba import jit

@jit(nopython=True)
def color_sequence(column):
    count_thresh = 10
    counter = 0
    last_colour = -1
    sequence = []

    if len(column) == 0:
        return sequence

    for i in range(len(column)):
        element = column[i] 
        if element == 0:
            continue
        if element == last_colour:
            counter += 1
        else:
            last_colour = element
            counter = 0
        if counter > count_thresh:
            sequence.append(element)
    return sequence

@jit(nopython=True)
def is_inside(sequence):
    last_colour = -1
    value = False
    
    if 6 not in sequence:
        value = True
        return value
    for i in sequence:
        if i == 6:
            if last_colour == 5:
                value = True
            elif last_colour == 4:
                value = False
        last_colour = i
    return value    

# Real Data:  0 0 0 43690
# FILTERED LINE:  [4, 4, 4, 4, 6, 6, 6, 6, 5, 6, 5, 5, 5, 5, 4, 4, 4, 4]
# True
# FILTERED LINE:  [4, 4, 4, 4, 6, 6, 6, 6, 5, 5, 5, 5, 4, 4, 4, 4]
# False
