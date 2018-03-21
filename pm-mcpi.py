from pymetawear.client import MetaWearClient
from pymetawear import libmetawear
import mcpi.minecraft as minecraft
import mcpi.block as block
import time
import math
import pyautogui as pag
import datetime as dt


#Places stone 5 m in front of player
def placeBlocks(blocktype):
  pos=mc.player.getPos()
  mc.setBlock(pos.x+5,pos.y+1,pos.z,blocktype)

def changeBlock(blocktype):
  blocktype+=1


# Create a buffer of values, with last average and last timestamp
class RingBuffer:
  def __init__(self, size):
    self.data = [ 0 for i in xrange(size)]
    self.size = size
    self.last_avg = 0
    self.last_time = 0
  
  def append(self, x):
    self.data.pop(0)
    self.data.append(x)
  
  def get(self):
    return self.data 
  
  def get_last_avg(self):
    return self.last_avg

  def set_last_avg(self, avg):
    self.last_avg = avg
  
  def get_last_time(self):
    return self.last_time
  
  def set_last_time(self,time):
    self.last_time = time

  def average(self):
    sm = 0
    for i in xrange(self.size):
      sm = sm + self.data[i]
    avg = sm / self.size
    return avg 
"""
  def derivative(self, stride):
    if stride > self.size-2: 
      return [0]
    der = [ 0 for i in xrange(self.size-stride)]
    for i in xrange(len(der)):
      der[i] = (self.data[i+stride] - self.data[i])/stride
    return der
"""

def magnitude(a_tup):
  (xa, ya, za ) =  a_tup
  sumsq =  xa * xa + ya * ya + za * za
  # calculate acceleration by removing gravity
  acc = math.sqrt(sumsq) - 1.0
  return acc

def spin_around(a_tup):
  (xg, yg, zg) = a_tup
  if zg > 100 :
    print("Angle = {0}".format(zg))
    xmouse=1   #  100*zg/1000.0  # move 100 pixels if xrot = 1000 deg/sec
    pag.moveRel(0,2)
  if zg < -100:
    print("Angle = {0}".format(zg))
    xmouse=-1 # 10*yg/1000.0  # move 100 pixels if xrot = 1000 deg/sec
    pag.moveRel(0, -2)

def move_around(a_tup):
  (xa, ya, za) = a_tup
  pag.keyUp('w')
  pag.keyUp('s')
  if xa > 0.4:
   print("Xaccel= {0}".format(xa))
   pag.keyDown('w')
  if xa < -0.4:
   print("Xaccel= {0}".format(xa))
   pag.keyDown('s')

def gyr_callback(data):
  
  global last_spin_time
  """Handle a (epoch, (x,y,z)) accelerometer tuple."""
  # print("{0}, {1}, {2}, {3}".format(data[0], *data[1]))
  if data[0] - last_spin_time > spin_time_thresh:
    last_spin_time = data[0] 
    spin_around(data[1])

def acc_callback(data):
  global last_move_time
  """Handle a (epoch, (x,y,z)) accelerometer tuple."""
  # print("Epoch time: [{0}]".format(data[0]))
  # Check for move
  if data[0] - last_move_time > move_time_thresh:
   last_move_time = data[0]
   start_time = dt.datetime.now()
   move_around(data[1])
   end_time = dt.datetime.now()
   #print(end_time-start_time)
  acc = magnitude(data[1])
  acc_buf.append(acc)
  current_avg = acc_buf.average()
  last_avg_acc = acc_buf.get_last_avg()
  delta_avg =  abs(current_avg - last_avg_acc) 
  delta_time = data[0] - acc_buf.get_last_time()
  if delta_avg > 0.2 and delta_time > 160 :
    print("Changed [ {0}]".format(data[0]))
    #x, y, z = mc.player.getPos()
    #mc.setBlock(x+5, y+2, z, 1)
    #mc.player.setPos(0,0,z)
    blocktype=1
    #placeBlocks(blocktype)
    #changeBlock(blocktype)
    pag.click(button="right")
  acc_buf.set_last_avg(current_avg)
  acc_buf.set_last_time(data[0])
  

# start main code

# Initialize minecraft
mc=minecraft.Minecraft.create()

#pyautogui.AUSE

# Initialize metawear
backend = 'pygatt'  # Or 'pybluez'
c = MetaWearClient('D5:05:98:AF:47:1D', backend)
time.sleep(1.0)

acc_buf = RingBuffer(4)
gyr_buf = RingBuffer(4)
last_move_time = 0
last_spin_time = 0
# constant threshold time between move/spin checks in ms
move_time_thresh = 300
spin_time_thresh = 300
sample_rate = 12.5 

# Set accelerometer settings 
c.accelerometer.set_settings(data_rate=sample_rate, data_range=4.0)
time.sleep(1.0)

c.gyroscope.set_settings(data_rate=25.0, data_range=1000.0)
time.sleep(1.0)

# Get current settings
#settings=c.accelerometer.get_current_settings()
#print(settings)

#print("connected")

# Enable high frequency stream
c.accelerometer.high_frequency_stream = True
# Enable acc notifications and register a callback for them.
c.accelerometer.notifications(acc_callback)

# Enable gyro notifications and register a callback for them.
c.gyroscope.notifications(gyr_callback)

mc.postToChat("YO DUDEE WHAT IS UP ")


time.sleep(1)
mc.postToChat("Another message")
time.sleep(100)

print("Unsubscribe notifications")
c.accelerometer.notifications(None)

time.sleep(1.0)

c.disconnect()
