import math
import numpy as np
import owl
import time
import matplotlib.pyplot as plt
import matplotlib.animation as anim
from mpl_toolkits.mplot3d import Axes3D
import serial

def normalize(v):
    norm = np.sqrt(np.sum(v**2))
    if norm == 0: 
       return v
    return v / norm

#markers = 2d list with double xyz positions of each marker
def calculateangles(markers):
    m0 = np.array(markers[0], dtype=float)
    m1 = np.array(markers[1], dtype=float)
    m2 = np.array(markers[2], dtype=float)
    m3 = np.array(markers[3], dtype=float)
    m4 = np.array(markers[4], dtype=float)
    m5 = np.array(markers[5], dtype=float)
    m6 = np.array(markers[6], dtype=float)
    m7 = np.array(markers[7], dtype=float)
    m8 = np.array(markers[8], dtype=float)
    m9 = np.array(markers[9], dtype=float)
    m10 = np.array(markers[10], dtype=float)
    m11 = np.array(markers[11], dtype=float)
    
    #generate rotation matrices with respect to global coordinate system
    y1 = m1 - m0
    v1 = m1 - m2
    z1 = np.cross(y1, v1)
    x1 = np.cross(z1, y1)
    y2 = m5 - m4
    v2 = m4 - m3
    z2 = np.cross(v2, y2)
    x2 = np.cross(z2, y2)
    y3 = m8 - m7
    v3 = m7 - m6
    z3 = np.cross(v3, y3)
    x3 = np.cross(z3, y3)
    y4 = m11 - m10
    v4 = m10 - m9
    z4 = np.cross(v4, y4)
    x4 = np.cross(z4, y4)
    x1 = normalize(x1)
    y1 = normalize(y1)
    z1 = normalize(z1)
    x2 = normalize(x2)
    y2 = normalize(y2)
    z2 = normalize(z2)
    x3 = normalize(x3)
    y3 = normalize(y3)
    z3 = normalize(z3)
    x4 = normalize(x4)
    y4 = normalize(y4)
    z4 = normalize(z4)
    #notation: Rab = rotation matrix of b from a; g = global
    #print(x1)
    Rg1 = np.transpose(np.array([x1, y1, z1]))
    Rg2 = np.transpose(np.array([x2, y2, z2]))
    Rg3 = np.transpose(np.array([x3, y3, z3]))
    Rg4 = np.transpose(np.array([x4, y4, z4]))
    #print(Rg1)
    #print(Rg2)
    #print(Rg3)
    #print(Rg4)
    #new rot matrices
    
    R12 = np.matmul(np.linalg.inv(Rg1), Rg2)
    R23 = np.matmul(np.linalg.inv(Rg2), Rg3)
    R34 = np.matmul(np.linalg.inv(Rg3), Rg4)
    #print(R12)
    #print(R23)
    #print(R34)

    #calculate euler angles
    '''
    alphag1 = math.degrees(np.arctan(Rg1[1, 0]/Rg1[0, 0]))
    betag1 = math.degrees(np.arcsin(-1 * Rg1[2, 0]))
    gammag1 = math.degrees(np.arctan(Rg1[2, 1]/Rg1[2, 2]))
    
    alpha12 = math.degrees(np.arctan(R12[1, 0]/R12[0, 0]))
    beta12 = math.degrees(np.arcsin(-1 * R12[2, 0]))
    gamma12 = math.degrees(np.arctan(R12[2, 1]/R12[2, 2]))

    

    alpha23 = math.degrees(np.arctan(R23[1, 0]/R23[0, 0]))
    beta23 = math.degrees(np.arcsin(-1 * R23[2, 0]))
    gamma23 = math.degrees(np.arctan(R23[2, 1]/R23[2, 2]))
    
    alpha34 = math.degrees(np.arctan(R34[1, 0]/R34[0, 0]))
    beta34 = math.degrees(np.arcsin(-1 * R34[2, 0]))
    gamma34 = math.degrees(np.arctan(R34[2, 1]/R34[2, 2]))
    '''

    betag1 = np.arctan2(-1*Rg1[2,0], np.sqrt(Rg1[0,0]**2 + Rg1[1,0]**2))
    alphag1 = math.degrees(np.arctan2(Rg1[1,0]/np.cos(betag1), Rg1[0,0]/np.cos(betag1)))
    gammag1 = math.degrees(np.arctan2(Rg1[2,1]/np.cos(betag1), Rg1[2,2]/np.cos(betag1)))
    betag1 = math.degrees(betag1)

    beta12 = np.arctan2(-1*R12[2,0], np.sqrt(R12[0,0]**2 + R12[1,0]**2))
    alpha12 = math.degrees(np.arctan2(R12[1,0]/np.cos(beta12), R12[0,0]/np.cos(beta12)))
    gamma12 = math.degrees(np.arctan2(R12[2,1]/np.cos(beta12), R12[2,2]/np.cos(beta12)))
    beta12 = math.degrees(beta12)

    beta23 = np.arctan2(-1*R23[2,0], np.sqrt(R23[0,0]**2 + R23[1,0]**2))
    alpha23 = math.degrees(np.arctan2(R23[1,0]/np.cos(beta23), R23[0,0]/np.cos(beta23)))
    gamma23 = math.degrees(np.arctan2(R23[2,1]/np.cos(beta23), R23[2,2]/np.cos(beta23)))
    beta23 = math.degrees(beta23)

    beta34 = np.arctan2(-1*R34[2,0], np.sqrt(R34[0,0]**2 + R34[1,0]**2))
    alpha34 = math.degrees(np.arctan2(R34[1,0]/np.cos(beta34), R34[0,0]/np.cos(beta34)))
    gamma34 = math.degrees(np.arctan2(R34[2,1]/np.cos(beta34), R34[2,2]/np.cos(beta34)))
    beta34 = math.degrees(beta34)
    
    #print(f"1 to 2 angles - alpha: {alpha12}° beta: {beta12}° gamma: {gamma12}°")
    #print(f"2 to 3 angles - alpha: {alpha23}° beta: {beta23}° gamma: {gamma23}°")
    #print(f"3 to 4 angles - alpha: {alpha34}° beta: {beta34}° gamma: {gamma34}°")

    #print(f"Approximate angles: Shoulder Rotated ≈ {int(gamma12)}° - Shoulder Tilted ≈ {int(alpha12)}° - Elbow Tilted ≈ {int(alpha23)}° - Wrist Tilted ≈ {int(alpha34 if alpha34 > 0 else (alpha34 + 360))}° - Wrist Rotated ≈ {-1*int(gamma34)}°")
    return (int(alpha12), int(beta12), int(alpha23), int(alpha34), int(gamma34))

#import numpy as np
#import pandas as pd



#SERVER = sys.argv[1]
SERVER = "192.168.1.230"
# instantiate context
o = owl.Context()
# connect to server with timeout of 10000000 microseconds 

o.open(SERVER, "timeout=10000000") # initialize session
o.initialize("streaming=1")
 


# main loop
evt = None
def getLatestEvent():
    global evt
    global o
    #print("getting latest event...")
    temp = None
    while o.isOpen() and o.property("initialized") and evt:
        temp = evt
        evt = o.nextEvent()
    evt = temp

while not evt:
    evt = o.nextEvent(1000000)
    print("searching for event")

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.axes.set_xlim3d(left=-2000, right=2000)
ax.axes.set_ylim3d(bottom=-2000, top=2000)
ax.axes.set_zlim3d(bottom=-2000, top=2000)
ax.set_autoscale_on(False)

lastmarkers = [[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0]]


#serial port init

ser = serial.Serial('COM17', baudrate=19200)
#hello world ping + arm setup
ser.write(bytearray([255, 13, 1, 0, 64, 64, 64, 69, 69, 69, 69, 69, 39]))
print(list(ser.read(size=13)))
ser.write(bytearray([255, 13, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 15]))
print(list(ser.read(size=13)))
ser.write(bytearray([255, 13, 1, 2, 0, 3, 0, 0, 1, 0, 0, 0, 20]))
print(list(ser.read(size=13)))
ser.write(bytearray([255, 13, 41, 1, 1, 0, 0, 0, 182, 78, 0, 0, 60]))
ser.write(bytearray([255, 13, 42, 10, 0, 1, 0, 0, 0, 0, 0, 0, 66]))
ser.write(bytearray([255, 13, 42, 10, 0, 2, 0, 0, 0, 0, 0, 0, 67]))
ser.write(bytearray([255, 13, 42, 10, 0, 3, 0, 0, 0, 0, 0, 0, 68]))
ser.write(bytearray([255, 13, 2, 2, 0, 0, 0, 0, 0, 0, 0, 0, 17]))
ser.write(bytearray([255, 13, 42, 10, 0, 20, 0, 0, 0, 0, 0, 0, 85]))
ser.write(bytearray([255, 13, 42, 10, 0, 21, 0, 0, 0, 0, 0, 0, 86]))
ser.write(bytearray([255, 13, 42, 10, 0, 22, 0, 0, 0, 0, 0, 0, 87]))
ser.write(bytearray([255, 13, 42, 10, 0, 23, 0, 0, 0, 0, 0, 0, 88]))
ser.write(bytearray([255, 13, 42, 10, 0, 24, 0, 0, 0, 0, 0, 0, 89]))
ser.write(bytearray([255, 13, 82, 10, 0, 1, 0, 0, 0, 0, 0, 0, 106]))
ser.write(bytearray([255, 13, 82, 10, 0, 2, 0, 0, 0, 0, 0, 0, 107]))
ser.write(bytearray([255, 13, 82, 10, 0, 3, 0, 0, 0, 0, 0, 0, 108]))
ser.write(bytearray([255, 13, 42, 20, 0, 0, 0, 0, 0, 0, 0, 0, 75]))
ser.write(bytearray([255, 13, 42, 20, 1, 0, 0, 0, 0, 0, 0, 0, 76]))
ser.write(bytearray([255, 13, 42, 20, 2, 0, 0, 0, 0, 0, 0, 0, 77]))
ser.write(bytearray([255, 13, 42, 20, 3, 0, 0, 0, 0, 0, 0, 0, 78]))
ser.write(bytearray([255, 13, 42, 20, 4, 0, 0, 0, 0, 0, 0, 0, 79]))
ser.write(bytearray([255, 13, 42, 20, 5, 0, 0, 0, 0, 0, 0, 0, 80]))
ser.write(bytearray([255, 13, 42, 21, 0, 1, 0, 0, 0, 0, 0, 0, 77]))
ser.write(bytearray([255, 13, 42, 21, 0, 2, 0, 0, 0, 0, 0, 0, 78]))
ser.write(bytearray([255, 13, 42, 21, 1, 1, 0, 0, 0, 0, 0, 0, 78]))
ser.write(bytearray([255, 13, 42, 21, 1, 2, 0, 0, 0, 0, 0, 0, 79]))
ser.write(bytearray([255, 13, 42, 21, 2, 1, 0, 0, 0, 0, 0, 0, 79]))
ser.write(bytearray([255, 13, 42, 21, 2, 2, 0, 0, 0, 0, 0, 0, 80]))
ser.write(bytearray([255, 13, 42, 21, 3, 1, 0, 0, 0, 0, 0, 0, 80]))
ser.write(bytearray([255, 13, 42, 21, 3, 2, 0, 0, 0, 0, 0, 0, 81]))
ser.write(bytearray([255, 13, 42, 21, 4, 1, 0, 0, 0, 0, 0, 0, 81]))
ser.write(bytearray([255, 13, 42, 21, 4, 2, 0, 0, 0, 0, 0, 0, 82]))
ser.write(bytearray([255, 13, 42, 21, 5, 1, 0, 0, 0, 0, 0, 0, 82]))
ser.write(bytearray([255, 13, 42, 21, 5, 2, 0, 0, 0, 0, 0, 0, 83]))
ser.write(bytearray([255, 13, 42, 22, 0, 1, 0, 0, 0, 0, 0, 0, 78]))
ser.write(bytearray([255, 13, 42, 22, 0, 2, 0, 0, 0, 0, 0, 0, 79]))
ser.write(bytearray([255, 13, 32, 1, 0, 64, 0, 0, 0, 0, 0, 0, 110]))
ser.write(bytearray([255, 13, 11, 0, 1, 10, 0, 0, 4, 0, 0, 0, 39]))
ser.write(bytearray([255, 13, 11, 0, 2, 10, 0, 0, 4, 0, 0, 0, 40]))
ser.write(bytearray([255, 13, 11, 0, 3, 10, 0, 0, 4, 0, 0, 0, 41]))
ser.write(bytearray([255, 13, 11, 0, 4, 10, 0, 0, 4, 0, 0, 0, 42]))
ser.write(bytearray([255, 13, 11, 0, 5, 10, 0, 0, 4, 0, 0, 0, 43]))
ser.write(bytearray([255, 13, 11, 0, 6, 10, 0, 0, 0, 0, 0, 0, 40]))
ser.write(bytearray([255, 13, 11, 0, 6, 11, 0, 0, 16, 0, 0, 0, 57]))
ser.write(bytearray([255, 13, 31, 0, 0, 9, 0, 0, 0, 0, 0, 0, 53]))
ser.write(bytearray([255, 13, 31, 0, 0, 11, 0, 0, 3, 0, 0, 0, 58]))
ser.write(bytearray([255, 13, 31, 0, 0, 11, 0, 0, 3, 0, 0, 0, 58]))
ser.write(bytearray([255, 13, 31, 0, 0, 44, 0, 0, 200, 0, 0, 0, 32]))
ser.write(bytearray([255, 13, 31, 0, 0, 52, 0, 0, 0, 0, 0, 0, 96]))
ser.write(bytearray([255, 13, 31, 0, 0, 48, 0, 0, 255, 15, 0, 0, 106]))
ser.write(bytearray([255, 13, 31, 0, 0, 12, 0, 0, 255, 0, 0, 0, 55]))
#turn on torque for all joints after 5 secs:
print("turning torque on for all joints in 5 seconds...")
time.sleep(5)
ser.write(bytearray([255, 13, 21, 1, 0, 64, 0, 0, 1, 0, 0, 0, 100]))
ser.write(bytearray([255, 13, 21, 1, 1, 64, 0, 0, 1, 0, 0, 0, 101]))
ser.write(bytearray([255, 13, 21, 1, 2, 64, 0, 0, 1, 0, 0, 0, 102]))
ser.write(bytearray([255, 13, 21, 1, 3, 64, 0, 0, 1, 0, 0, 0, 103]))
ser.write(bytearray([255, 13, 21, 1, 4, 64, 0, 0, 1, 0, 0, 0, 104]))
ser.write(bytearray([255, 13, 21, 1, 5, 64, 0, 0, 1, 0, 0, 0, 105]))
print("torque is on.")
for joint in range(0,6):
    ser.write(bytearray([255, 13, 21, 1, joint, 112, 0, 0, 208, 7, 0, 0, 106+joint]))
    ser.write(bytearray([255, 13, 21, 1, joint, 108, 0, 0, 232, 3, 0, 0, 122+joint]))
    ser.write(bytearray([255, 13, 51, 20, joint, 0, 0, 0, 0, 0, 0, 0, (84+joint)%256]))
time.sleep(3)


def send(shoulder, elbow, forearm, wrist, hand):
    global ser
    speedbytes = (0).to_bytes(4, byteorder='little', signed=True)
    accelbytes = (0).to_bytes(4, byteorder='little', signed=True)
    shoulder = max(-180, min(180, shoulder))
    elbow = max(0, min(120, elbow))
    forearm = max(0, min(180, forearm))
    wrist = max(70, min(270, wrist))
    hand = max(-180, min(180, hand))
    #accelbytes = (0).to_bytes(4, byteorder='little', signed=True)
    for joint in range(2, 3):
        ser.write(bytearray([255, 13, 21, 1, joint, 112, 0, 0, speedbytes[0], speedbytes[1], speedbytes[2], speedbytes[3], (147+joint+speedbytes[0]+speedbytes[1]+speedbytes[2]+speedbytes[3])%256]))
        ser.write(bytearray([255, 13, 21, 1, joint, 108, 0, 0, accelbytes[0], accelbytes[1], accelbytes[2], accelbytes[3], (143+joint+accelbytes[0]+accelbytes[1]+accelbytes[2]+accelbytes[3])%256]))
        if joint == 0:
            angle = int(shoulder*10)
        elif joint == 1:
            angle = int((90-elbow)*10)
        elif joint == 2:
            angle = int((90-forearm)*10)
        elif joint == 3:
            angle = int(wrist*10)
        elif joint == 4:
            angle = int(hand*10)
        anglebytes = (angle).to_bytes(4, byteorder='little', signed=True)
        ser.write(bytearray([255, 13, 51, 20, joint, 0, 0, 0, anglebytes[0], anglebytes[1], anglebytes[2], anglebytes[3], (84+joint+anglebytes[0]+anglebytes[1]+anglebytes[2]+anglebytes[3])%256]))
    #print("commands sent.")


#while evt or (o.isOpen() and o.property("initialized")):
def animate(i):   
    global evt
    global lastmarkers
    #print(lastmarkers)
    t1 = time.time()
    ax.cla()
    ax.axes.set_xlim3d(left=-2000, right=2000)
    ax.axes.set_ylim3d(bottom=-2000, top=2000)
    ax.axes.set_zlim3d(bottom=-2000, top=2000)
    ax.set_autoscale_on(False)
    ax.scatter(0,0,0)
    ax.text(0,0,0,"(0, 0, 0)")
    
    # poll for events with a timeout (microseconds)
    getLatestEvent()
    # nothing received, keep waiting
   
    if not evt:
        return
   
    # process event
    if evt.type_id == owl.Type.FRAME:
        # print markers
        if "markers" in evt:
            count = 0
            for m in evt.markers:
                ax.scatter(m.x, m.y, m.z)
                if not (m.x == 0 or m.y == 0 or m.z == 0):
                    lastmarkers[count][0] = m.x
                    lastmarkers[count][1] = m.y
                    lastmarkers[count][2] = m.z
                #print(str(m.x) + ", " + str(m.y) + ", " + str(m.z))
                count += 1
            if [0,0,0] not in lastmarkers:
                angles = calculateangles(lastmarkers)
                ax.text(-1500,0,0,f"Shoulder Rotated ≈ {angles[0]}°\nShoulder Tilted ≈ {angles[1]}°\nElbow Tilted ≈ {angles[2]}°\nWrist Tilted ≈ {angles[3]}°\nWrist Rotated ≈ {angles[4]}°")
                #if i % 2 == 0:
                send(angles[1], angles[0], angles[2], angles[3], angles[4])
         
    elif evt.type_id == owl.Type.ERROR:
        # handle errors
        print(evt.name, evt.data)
        if evt.name == "fatal":
            return
    elif evt.name == "done":
        # done event is sent when master connection stops session
        print("done")
        return
    t2 = time.time()
    #print(f"Completed 1 iteration in {t2 - t1} seconds.")


ani = anim.FuncAnimation(fig, animate, interval=0)
'''
cont = input("Enter any text to close server & untorque servoes.")
if cont:
    o.done()
    o.close()
    print("torque off in 3 seconds...")
    time.sleep(3)
    ser.write(bytearray([255, 13, 21, 1, 0, 64, 0, 0, 0, 0, 0, 0, 99]))
    ser.write(bytearray([255, 13, 21, 1, 1, 64, 0, 0, 0, 0, 0, 0, 100]))
    ser.write(bytearray([255, 13, 21, 1, 2, 64, 0, 0, 0, 0, 0, 0, 101]))
    ser.write(bytearray([255, 13, 21, 1, 3, 64, 0, 0, 0, 0, 0, 0, 102]))
    ser.write(bytearray([255, 13, 21, 1, 4, 64, 0, 0, 0, 0, 0, 0, 103]))
    ser.write(bytearray([255, 13, 21, 1, 5, 64, 0, 0, 0, 0, 0, 0, 104]))
    ser.close()
'''
    #result_available.wait(0.020)
# end main loop
 
# end session
#o.done()
# close socket
#o.close()
 
