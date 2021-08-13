import serial
import time
from tkinter import *

print("hello")
#gui setup
root = Tk()
root.title("Arm Instruction Sender")
root.geometry("800x1000")
stop = False
shoulder = DoubleVar(value=0.0)
elbow = DoubleVar(value=90.0)
forearm = DoubleVar(value=90.0)
wrist = DoubleVar(value=180.0)
hand = DoubleVar(value=0.0)
fingers = DoubleVar(value=22.0)
speed = IntVar(value=2000)

def stope():
    global stop 
    stop = True

def send():
    global ser
    speedbytes = (speed.get()).to_bytes(4, byteorder='little', signed=True)
    accelbytes = (int(speed.get()/2)).to_bytes(4, byteorder='little', signed=True)
    #accelbytes = (0).to_bytes(4, byteorder='little', signed=True)
    for joint in range(0, 6):
        ser.write(bytearray([255, 13, 21, 1, joint, 112, 0, 0, speedbytes[0], speedbytes[1], speedbytes[2], speedbytes[3], (147+joint+speedbytes[0]+speedbytes[1]+speedbytes[2]+speedbytes[3])%256]))
        ser.write(bytearray([255, 13, 21, 1, joint, 108, 0, 0, accelbytes[0], accelbytes[1], accelbytes[2], accelbytes[3], (143+joint+accelbytes[0]+accelbytes[1]+accelbytes[2]+accelbytes[3])%256]))
        if joint == 0:
            angle = int(shoulder.get()*10)
        elif joint == 1:
            angle = int((90-elbow.get())*10)
        elif joint == 2:
            angle = int((90-forearm.get())*10)
        elif joint == 3:
            angle = int((180-wrist.get())*10)
        elif joint == 4:
            angle = int(hand.get()*10)
        else:
            angle = int(fingers.get()*10)
        anglebytes = (angle).to_bytes(4, byteorder='little', signed=True)
        ser.write(bytearray([255, 13, 51, 20, joint, 0, 0, 0, anglebytes[0], anglebytes[1], anglebytes[2], anglebytes[3], (84+joint+anglebytes[0]+anglebytes[1]+anglebytes[2]+anglebytes[3])%256]))
    print("commands sent.")

def updatevalues(event=None):
    #try:
        speed.set(int(float(speedentry.get())))
        speedscale.set(speed.get())
        shoulder.set(int(float(shoulderentry.get())))
        shoulderscale.set(shoulder.get())
        elbow.set(int(float(elbowentry.get())))
        elbowscale.set(elbow.get())
        forearm.set(int(float(forearmentry.get())))
        forearmscale.set(forearm.get())
        wrist.set(int(float(wristentry.get())))
        wristscale.set(wrist.get())
        hand.set(int(float(handentry.get())))
        handscale.set(hand.get())
        fingers.set(int(float(fingerentry.get())))
        fingerscale.set(fingers.get())
    #except:
        #print("invalid text in box.")


speedlabel = Label(root, text = "Speed of action in ms")
speedscale = Scale(root, variable=speed, from_=500, to=8000, orient=HORIZONTAL, length=600, width=30, resolution=10)
speedentry = Entry(root, textvariable=speed)
#speedentry.bind('<Return>', updatevalues)
shoulderlabel = Label(root, text = "Shoulder Rotation Joint (degrees)")
shoulderscale = Scale(root, variable=shoulder, from_=-180, to=180, orient=HORIZONTAL, length=600, width=30, resolution=1)
shoulderentry = Entry(root, textvariable=shoulder)
#shoulderentry.bind('<Return>', updatevalues)
elbowlabel = Label(root, text = "Shoulder Joint (degrees)")
elbowscale = Scale(root, variable=elbow, from_=-10, to=196, orient=HORIZONTAL, length=600, width=30, resolution=1)
elbowentry = Entry(root, textvariable=elbow)
#elbowentry.bind('<Return>', updatevalues)
forearmlabel = Label(root, text = "Elbow Joint (degrees)")
forearmscale = Scale(root, variable=forearm, from_=-5, to=192, orient=HORIZONTAL, length=600, width=30, resolution=1)
forearmentry = Entry(root, textvariable=forearm)
#forearmentry.bind('<Return>', updatevalues)
wristlabel = Label(root, text = "Wrist Joint (degrees)")
wristscale = Scale(root, variable=wrist, from_=57, to=280, orient=HORIZONTAL, length=600, width=30, resolution=1)
wristentry = Entry(root, textvariable=wrist)
#wristentry.bind('<Return>', updatevalues)
handlabel = Label(root, text = "Wrist Rotate Joint (degrees)")
handscale = Scale(root, variable=hand, from_=-180, to=180, orient=HORIZONTAL, length=600, width=30, resolution=1)
handentry = Entry(root, textvariable=hand)
#handentry.bind('<Return>', updatevalues)
fingerlabel = Label(root, text = "Gripper Joint")
fingerscale = Scale(root, variable=fingers, from_=-90, to=22, orient=HORIZONTAL, length=600, width=30, resolution=1)
fingerentry = Entry(root, textvariable=fingers)
#fingerentry.bind('<Return>', updatevalues)

sendb = Button(root, text="Send Commands", command=send, bg="green", height=8, width=100)
stopb = Button(root, text="Stop", command=root.destroy, bg="red", height=5, width=100)
l1 = Label(root)

speedscale.pack(anchor=CENTER)
speedentry.pack()
speedlabel.pack()
shoulderscale.pack(anchor=CENTER)
shoulderentry.pack()
shoulderlabel.pack()
elbowscale.pack(anchor=CENTER)
elbowentry.pack()
elbowlabel.pack()
forearmscale.pack(anchor=CENTER)
forearmentry.pack()
forearmlabel.pack()
wristscale.pack(anchor=CENTER)
wristentry.pack()
wristlabel.pack()
handscale.pack(anchor=CENTER)
handentry.pack()
handlabel.pack()
fingerscale.pack(anchor=CENTER)
fingerentry.pack()
fingerlabel.pack()
sendb.pack(anchor=CENTER)
stopb.pack(anchor=CENTER)
l1.pack()


#serial port init

ser = serial.Serial('COM16', baudrate=19200)
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
root.mainloop()
'''
keepgoing = input("Type \"quit\" to quit, any other to keep going:")
while keepgoing != "quit":
    joint = int(input("Which joint? [0] Waist (-180~180), [1] Shoulder (-106~100), [2] Elbow (-102~95), [3] Wrist Tilt (-100~123), [4] Wrist Rotate (-180~180), [5] Gripper (-90~22):"))
    angle = 10*int(input("Angle:"))
    #registering joint:
    ser.write(bytearray([255, 13, 21, 1, joint, 112, 0, 0, 208, 7, 0, 0, 106+joint]))
    ser.write(bytearray([255, 13, 21, 1, joint, 108, 0, 0, 232, 3, 0, 0, 122+joint]))
    anglebytes = (angle).to_bytes(4, byteorder='little', signed=True)
    ser.write(bytearray([255, 13, 51, 20, joint, 0, 0, 0, anglebytes[0], anglebytes[1], anglebytes[2], anglebytes[3], (84+joint+anglebytes[0]+anglebytes[1]+anglebytes[2]+anglebytes[3])%256]))
    keepgoing = input("Type \"quit\" to quit, any other to keep going:")
while not stop:
    ser.write(bytearray([255, 13, 21, 1, 2, 112, 0, 0, 0, 0, 0, 0, 149]))
    ser.write(bytearray([255, 13, 21, 1, 2, 108, 0, 0, 0, 0, 0, 0, 145]))
    anglebytes = (int((90-forearm.get())*10)).to_bytes(4, byteorder='little', signed=True)
    ser.write(bytearray([255, 13, 51, 20, 2, 0, 0, 0, anglebytes[0], anglebytes[1], anglebytes[2], anglebytes[3], (84+2+anglebytes[0]+anglebytes[1]+anglebytes[2]+anglebytes[3])%256]))
    root.update()
    time.sleep(0.02)
'''
#returning to base position:
'''
for joint in range(0,6):
    ser.write(bytearray([255, 13, 21, 1, joint, 112, 0, 0, 208, 7, 0, 0, 106+joint]))
    ser.write(bytearray([255, 13, 21, 1, joint, 108, 0, 0, 232, 3, 0, 0, 122+joint]))
    ser.write(bytearray([255, 13, 51, 20, joint, 0, 0, 0, 0, 0, 0, 0, (84+joint)%256]))
time.sleep(2)
ser.write(bytearray([255, 13, 21, 1, 1, 112, 0, 0, 208, 7, 0, 0, 106+1]))
ser.write(bytearray([255, 13, 21, 1, 1, 108, 0, 0, 232, 3, 0, 0, 122+1]))
ser.write(bytearray([255, 13, 51, 20, 1, 0, 0, 0, 255, 154, 0, 0, (84+1+255+154)%256]))
ser.write(bytearray([255, 13, 21, 1, 2, 112, 0, 0, 208, 7, 0, 0, 106+2]))
ser.write(bytearray([255, 13, 21, 1, 2, 108, 0, 0, 232, 3, 0, 0, 122+2]))
ser.write(bytearray([255, 13, 51, 20, 2, 0, 0, 0, 95, 0, 0, 0, (84+2+95)%256]))
ser.write(bytearray([255, 13, 21, 1, 3, 112, 0, 0, 208, 7, 0, 0, 106+3]))
ser.write(bytearray([255, 13, 21, 1, 3, 108, 0, 0, 232, 3, 0, 0, 122+3]))
ser.write(bytearray([255, 13, 51, 20, 3, 0, 0, 0, 255, 156, 0, 0, (84+3+255+156)%256]))
'''
#torquing all joints off:
print("torque off in 3 seconds...")
time.sleep(3)
ser.write(bytearray([255, 13, 21, 1, 0, 64, 0, 0, 0, 0, 0, 0, 99]))
ser.write(bytearray([255, 13, 21, 1, 1, 64, 0, 0, 0, 0, 0, 0, 100]))
ser.write(bytearray([255, 13, 21, 1, 2, 64, 0, 0, 0, 0, 0, 0, 101]))
ser.write(bytearray([255, 13, 21, 1, 3, 64, 0, 0, 0, 0, 0, 0, 102]))
ser.write(bytearray([255, 13, 21, 1, 4, 64, 0, 0, 0, 0, 0, 0, 103]))
ser.write(bytearray([255, 13, 21, 1, 5, 64, 0, 0, 0, 0, 0, 0, 104]))
ser.close()

print(ser.name)