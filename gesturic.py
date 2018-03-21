# This program is used to capture data for gesture modeling.

from pymetawear.client import MetaWearClient
from pymetawear import libmetawear
import sys
import time
import math, numpy
import datetime as dt




# Look for lift-down-twist-right gesture
def check_ldtr_pattern():
  global acc_xbuf, acc_ybuf, acc_zbuf
  
  #define the activation pattern arrays
  xpat = [0,0,0,0,1,1,-1,-1,-1,0,1,1,0,0,0,0,0,0,0,0]
  ypat = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,-1,-1,-1,0]
  zpat = [0,0,0,0,1,-1,0,0,0,0,-1,1,0,0,0,0,-1,0,0,0] 

  xacc = numpy.zeros(20)
  yacc = numpy.zeros(20)
  zacc = numpy.zeros(20)
  
  for i in xrange(20):
    xacc[i] = acc_xbuf.get()[i]
    yacc[i] = acc_ybuf.get()[i]
    zacc[i] = acc_zbuf.get()[i]
  
  activation_val = numpy.dot(xpat,xacc) + numpy.dot(ypat, yacc) + numpy.dot(zpat,zacc)
  if activation_val > 12.0:
    print("ldtr: {:05.2f}".format(activation_val))
    return True
  else:
    return False
 
# Look for put_block gesture
def check_ud_pattern():
  global acc_xbuf, acc_ybuf, acc_zbuf
  
  #define the activation pattern arrays
  xpat = [0,0,0,0,0,0,0,-1,-1,1,-1,-1,0,0,0,0,0,0,0,0]
  ypat = [0,0,0,0,0,0,0,0,0,-1,1,0,0,0,0,0,0,0,0,0]
  zpat = [0,0,0,0,0,0,0,0,-1,-1,1,1,0,0,0,0,0,0,0,0] 

  xacc = numpy.zeros(20)
  yacc = numpy.zeros(20)
  zacc = numpy.zeros(20)
  
  for i in xrange(20):
    xacc[i] = acc_xbuf.get()[i]
    yacc[i] = acc_ybuf.get()[i]
    zacc[i] = acc_zbuf.get()[i] 
  
  activation_val = numpy.dot(xpat,xacc) + numpy.dot(ypat, yacc) + numpy.dot(zpat,zacc)
  if activation_val > 8.0:
    print("ud  {:05.2f}".format(activation_val))
    return True
  else: 
    return False
 
  
# Look for wall gesture
def check_wall_pattern():
  global acc_xbuf, acc_ybuf, acc_zbuf
  
  #define the activation pattern arrays
  xpat = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
  ypat = [0,0,0,0,0,0,0,0,0,-1,-1,0,1,1,-1,0,1,1,1,1]
  zpat = [0,0,0,0,1,2,0,-1,-1,-1,1,1,2,0,-1,-1,0,2,1,0] 

  xacc = numpy.zeros(20)
  yacc = numpy.zeros(20)
  zacc = numpy.zeros(20)
  
  for i in xrange(20):
    xacc[i] = acc_xbuf.get()[i]
    yacc[i] = acc_ybuf.get()[i]
    zacc[i] = acc_zbuf.get()[i] - 1.0
  
  activation_val = numpy.dot(xpat,xacc) + numpy.dot(ypat, yacc) + numpy.dot(zpat,zacc)
  if activation_val > 10.0:
    print("wall: {:05.2f}".format(activation_val))
    return True
  else:
    return False

  

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
   
  def show(self):
    print("["),
    for i in xrange(self.size):
      print('{:03.2f}'.format(self.data[i])),
      print(","),
    print("]")
    
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


def gyr_callback(data):
  
  global last_spin_time, spin_time_thresh
  """Handle a (epoch, (x,y,z)) accelerometer tuple."""
  #print("Gyr {0}, {1}, {2}, {3}".format(data[0], *data[1]))

  gyr = magnitude(data[1])
  gyr_buf.append(gyr)
  gyr_xbuf.append(data[1][0])
  gyr_ybuf.append(data[1][1])
  gyr_zbuf.append(data[1][2])

  delta_time = data[0] - gyr_buf.get_last_time()
  if delta_time > 1600:
    gyr_buf.set_last_time(data[0])

    gyr_mag_data = gyr_buf.get()
    gyr_x_data = gyr_xbuf.get()
    gyr_y_data = gyr_ybuf.get()
    gyr_z_data = gyr_zbuf.get()
    print("Gyr\tX\tY\tZ")
    
    for i in xrange(len(gyr_mag_data)):
      print('{:03.2f} \t {:03.2f} \t {:03.2f} \t{:03.2f}'.format(gyr_mag_data[i]/250.0,gyr_x_data[i]/250.0,gyr_y_data[i]/250.0,gyr_z_data[i]/250.0))
    print(" ")
       

def acc_callback(data):
  global last_move_time, move_time_thresh, current_block_id
  # print("Epoch time: [{0}]".format(data[0]))
  acc = magnitude(data[1])
  acc_buf.append(acc)
  acc_xbuf.append(data[1][0])
  acc_ybuf.append(data[1][1])
  acc_zbuf.append(data[1][2])

  delta_time = data[0] - acc_buf.get_last_time()
  # Print acceleration stats for gesture recognition
  if delta_time > 1600:
    acc_buf.set_last_time(data[0])

    acc_mag_data = acc_buf.get() 
    acc_x_data = acc_xbuf.get() 
    acc_y_data = acc_ybuf.get() 
    acc_z_data = acc_zbuf.get() 
    print("Acc\tX\tY\tZ")
     
    for i in xrange(len(acc_mag_data)):
      print('{:03.2f} \t {:03.2f} \t {:03.2f} \t{:03.2f}'.format(acc_mag_data[i],acc_x_data[i],acc_y_data[i],acc_z_data[i]))
    print(" ")


# start main code


# Initialize metawear
backend =  'pygatt'  # Or 'pybluez'
while True:
  try:
    c = MetaWearClient('D5:05:98:AF:47:1D', backend)
    time.sleep(1.0)
    break
  except:
    print("Connecting to Wand...")

acc_buf = RingBuffer(20)   # magnitude
acc_xbuf = RingBuffer(20)
acc_ybuf = RingBuffer(20)
acc_zbuf = RingBuffer(20)
gyr_buf = RingBuffer(20)   # magnitude
gyr_xbuf = RingBuffer(20)
gyr_ybuf = RingBuffer(20)
gyr_zbuf = RingBuffer(20)

sample_rate = 12.5 

# Set accelerometer settings 
c.accelerometer.set_settings(data_rate=sample_rate, data_range=4.0)
time.sleep(0.2)

c.gyroscope.set_settings(data_rate=25.0, data_range=1000.0)
time.sleep(0.2)

# Get current settings
#settings=c.accelerometer.get_current_settings()
#print(settings)

print("Connected")

# Enable high frequency stream
#c.accelerometer.high_frequency_stream = True
# Enable acc notifications and register a callback for them.
c.accelerometer.notifications(acc_callback)

# Enable gyro notifications and register a callback for them.
c.gyroscope.notifications(gyr_callback)


while True:
  time.sleep(20)

print("Unsubscribe notifications")
c.accelerometer.notifications(None)
c.gyroscope.notifications(None)

time.sleep(1.0)

c.disconnect()
