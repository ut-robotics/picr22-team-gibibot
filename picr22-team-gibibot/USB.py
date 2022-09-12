import serial as ser
import serial.tools.list_ports as prtlst

global COMs
COMs=[]
def getCOMs():
    global COMs
    pts= prtlst.comports()
    print(pts)
    for pt in pts:
        if 'USB' in pt[1]: #check 'USB' string in device description
            COMs.append(pt[0])
    