from numba import jit

@jit(nopython=True)
def color_sequence(column):
    count_thresh = 2
    counter = 0
    last_colour = -1
    sequence = []
    for i in range(len(column)):
        element = column[i] 
        if element == 0:
            continue
        if element == last_colour:
            counter += 1
        else:
            last_colour = element
            counter = 0
        if counter < count_thresh:
            sequence.append(element)
    return sequence

@jit(nopython=True)
def is_inside(sequence):
    last_colour = ""
    value = True

    for i in sequence:
        if i == 6:
            if last_colour == 5:
                value = False
            elif last_colour == 4:
                value = True
    return value


