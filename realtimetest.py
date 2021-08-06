#!/usr/bin/python
 
#
# Copyright (c) PhaseSpace, Inc 2019
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR # IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, # FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL # PHASESPACE, INC BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER # IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN # CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
#
 
import owl
#import sys
import time
import matplotlib.pyplot as plt
import matplotlib.animation as anim
from mpl_toolkits.mplot3d import Axes3D
#import numpy as np
#import pandas as pd



#SERVER = sys.argv[1]
SERVER = "192.168.1.230"
'''
print("Enter the duration of recording (seconds) ")
r_duration = float(input())
 
print("Enter the recorded file name (ex. output01.txt) ")
fn = input()
'''
# instantiate context
o = owl.Context()
# connect to server with timeout of 10000000 microseconds 

o.open(SERVER, "timeout=10000000") # initialize session
o.initialize("streaming=1")
 


# main loop
t1 = time.time()
count = 0
evt = None
def getLatestEvent():
    global evt
    global o
    while evt:
        temp = evt
        evt = o.nextEvent()
    evt = temp

while not evt:
    evt = o.nextEvent(1000000)
    print("searching for event")

#fig = plt.figure()
#ax = fig.add_subplot(111, projection='3d')

while evt or (o.isOpen() and o.property("initialized")):
#def animate(i):   
    #ax.cla()
    # poll for events with a timeout (microseconds)
    getLatestEvent()
    # nothing received, keep waiting
   
    if not evt:
        continue
   
    # process event
    if evt.type_id == owl.Type.FRAME:
        # print markers
        if "markers" in evt:
            for m in evt.markers:
                #ax.scatter(m.x, m.y, m.z)
                print(str(m.x) + ", " + str(m.y) + ", " + str(m.z))
            
        # print rigids
        '''
        if "rigids" in evt:
            for r in evt.rigids: print(r)
            
        if count == 1:
            b = a
        else:             
            b = np.vstack((b,a)) 
        '''
        
    elif evt.type_id == owl.Type.ERROR:
        # handle errors
        print(evt.name, evt.data)
        if evt.name == "fatal":
            break
    elif evt.name == "done":
        # done event is sent when master connection stops session
        print("done")
        break

#ani = anim.FuncAnimation(fig, animate, interval=200)
    #result_available.wait(0.020)
# end main loop
 
# end session
o.done()
# close socket
o.close()
 
