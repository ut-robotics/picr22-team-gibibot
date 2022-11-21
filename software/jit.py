from numba import jit

@jit(nopython=True)
def color_sequence(column):
    sequence = []
    for i in len(column):
        element = column[i] 
        if element == column[i+1] and element == column[i-1]:
            sequence.append(element)
    return sequence