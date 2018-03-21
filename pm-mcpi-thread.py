from pymetawear.client import MetaWearClient
from pymetawear import libmetawear
import sys
import mcpi.minecraft as minecraft
import mcpi.block as block
import time
import math, numpy
import pyautogui as pag
import datetime as dt
import threading
import RPi.GPIO as GPIO



#Places stone 5 m in front of player
def placeBlock():
  global current_block_id
  pos=mc.player.getPos()
  mc.setBlock(pos.x,pos.y+1,pos.z+1,current_block_id)

def changeBlock(pin_pressed):
  global current_block_id, block_pin
  current_block_id = block_pin[pin_pressed][0]
  msg=" You selected " + block_pin[pin_pressed][1]
  print(msg)
  mc.postToChat(msg)
  
def build_wall():
  pos=mc.player.getPos()
  mc.setBlocks(pos.x-50,pos.y-1,pos.z+4,pos.x+50,pos.y+49,pos.z+5,block.GLOWING_OBSIDIAN)
  mc.postToChat("We have finally built Donald Trump's wall! You have discovered the TRUMP'S WALL spell!")

def build_staircase():
  global current_block_id
  pos=mc.player.getPos()
  for i in range(0,20):
    mc.setBlock(pos.x,pos.y+i-1,pos.z+i,current_block_id)
    mc.setBlock(pos.x+1,pos.y+i-1,pos.z+i,current_block_id)
  mc.postToChat("You discovered the STAIRCASE spell! Climb up and up and up...")

def dancefloor():
  global current_block_id
  pos=mc.player.getPos()
  mc.postToChat("Let's PAARTY!You discovered the DANCEFLOOR spell!")
  while True:
    mc.setBlocks(pos.x-10,pos.y-1,pos.z-10,pos.z+10,pos.y-1,pos.z+10,89)


  
def frozen():
  mc.postToChat("You have discovered the FROZEN spell and acquired Elsa's magical power! You will leave a trail of ice wherever you go.")
  while mc.getBlock(pos.x,pos.y-1,pos.z)!=41:
    pos = mc.player.getTilePos()
    #mc.setBlock(pos.x,pos.y-1,pos.z,79)
    if mc.getBlock(pos.x,pos.y-1,pos.z)!=41:
        mc.setBlocks(pos.x-1,pos.y-1,pos.z-1,pos.x+1,pos.y-1,pos.z+1,79)

def build_tunnel():
  global current_block_id
  pos=mc.player.getPos()
  mc.postToChat("You have discovered the TUNNEL spell!")
  mc.setBlocks(pos.x-2,pos.y-1,pos.z-2,pos.x+2,pos.y+3,pos.z+30,current_block_id)
  mc.setBlocks(pos.x-1,pos.y,pos.z-2,pos.x+1,pos.y+2,pos.z+30,0)

# Initialize GPIO pins
def init_gpio():
  GPIO.setmode(GPIO.BCM)
  GPIO.setup(2, GPIO.IN, pull_up_down = GPIO.PUD_UP)
  GPIO.add_event_detect(2, GPIO.RISING, callback=changeBlock, bouncetime=10)
  GPIO.setup(3, GPIO.IN, pull_up_down = GPIO.PUD_UP)
  GPIO.add_event_detect(3, GPIO.RISING, callback=changeBlock, bouncetime=10)
  GPIO.setup(4, GPIO.IN, pull_up_down = GPIO.PUD_UP)
  GPIO.add_event_detect(4, GPIO.RISING, callback=changeBlock, bouncetime=10)
  GPIO.setup(5, GPIO.IN, pull_up_down = GPIO.PUD_UP)
  GPIO.add_event_detect(5, GPIO.RISING, callback=changeBlock, bouncetime=10)
  GPIO.setup(6, GPIO.IN, pull_up_down = GPIO.PUD_UP)
  GPIO.add_event_detect(6, GPIO.RISING, callback=changeBlock, bouncetime=10)
  GPIO.setup(13, GPIO.IN, pull_up_down = GPIO.PUD_UP)
  GPIO.add_event_detect(13, GPIO.RISING, callback=changeBlock, bouncetime=10)
  GPIO.setup(19, GPIO.IN, pull_up_down = GPIO.PUD_UP)
  GPIO.add_event_detect(19, GPIO.RISING, callback=changeBlock, bouncetime=10)
  

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
  if activation_val > 8.0:
    print("ldtr: {:05.2f}".format(activation_val))
    return True
  else:
    return False
 
# Look for put_block gesture
def check_ud_pattern():
  # only acc_xbuf, acc_zbuf and gyr_ybuf are discriminative
  global acc_xbuf, acc_zbuf, gyr_ybuf
  
  #define the activation pattern arrays
  xacof = [0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,0,0,0,0,0]
  zacof = [0,0,0,0,0,0,0,0,1,1,-1,-1,-1,-1,1,1,0,0,0,0] 
  ygcof = [1,1,1,0,-1,-1,-1,-1,-1,-1,0,0,0,0,0,0,0,0,0,0]

  xacc = numpy.zeros(20)
  zacc = numpy.zeros(20)
  ygyr = numpy.zeros(20)
  
  for i in xrange(20):
    xacc[i] = acc_xbuf.get()[i]
    zacc[i] = acc_zbuf.get()[i] 
    ygyr[i] = gyr_ybuf.get()[i]/250.0
  
  activation_val = numpy.dot(xacof,xacc) + numpy.dot(ygcof, ygyr) + numpy.dot(zacof,zacc)
  if activation_val > 24.0:
    print("ud  {:05.2f}".format(activation_val))
    return True
  else: 
    return False

  # Look for put_block gesture
def check_tunnel_pattern():
  # only gyr_xbuf, gyr_zbuf and gyr_ybuf are discriminative
  global gyr_xbuf, gyr_zbuf, gyr_ybuf
  
  #define the activation pattern arrays
  gxcof = [1,1,1,1,-1,-1,-1,-1,1,1,1,1,0,-1,-1,-1,-1,1,1,1]
  gycof = [1,1,1,0,0,0,-1,-1,1,1,1,0,0,0,0,-1,-1,0,1,1] 
  gzcof = [0,0,0,0,0,0,1,1,0,0,1,1,-1,0,0,1,1,0,1,0]

  xgyr = numpy.zeros(20)
  ygyr = numpy.zeros(20)
  zgyr = numpy.zeros(20)
  
  for i in xrange(20):
    xgyr[i] = gyr_xbuf.get()[i]/250.0
    ygyr[i] = gyr_ybuf.get()[i]/250.0
    zgyr[i] = gyr_zbuf.get()[i]/250.0
  
  activation_val = numpy.dot(gxcof,xgyr) + numpy.dot(gycof, ygyr) + numpy.dot(gzcof,zgyr)
  if activation_val > 40.0:
    print("tunnel {:05.2f}".format(activation_val))
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

def spin_around(a_tup):
  (xg, yg, zg) = a_tup
  # print("Position is X={0}, Y={1}".format(x,y))
  if zg > 300 :
    #print("Angle = {0}".format(zg))
    xmouse=1   #  100*zg/1000.0  # move 100 pixels if xrot = 1000 deg/sec
    pag.moveRel(2,0)
  if zg < -300:
    #print("Angle = {0}".format(zg))
    xmouse=-1 # 10*yg/1000.0  # move 100 pixels if xrot = 1000 deg/sec
    pag.moveRel(2, 0)
    #pag.moveTo(x-2, y)

def move_around():
  global gyr_buf, gyr_xbuf, gyr_ybuf, gyr_zbuf
  global acc_buf, acc_xbuf, acc_ybuf, acc_zbuf
  global sw_pressed
 
  if sw_pressed == False :
    pag.keyUp('w')
    pag.keyUp('a')
    pag.keyUp('s')
    pag.keyUp('d')
    return

  gx_avg = gyr_xbuf.average()
  gy_avg = gyr_ybuf.average()
  gz_avg = gyr_zbuf.average()
  ax_avg = acc_xbuf.average()
  ay_avg = acc_ybuf.average()
  az_avg = acc_zbuf.average()

#  print("ax {0}, ay {1}, az {2}, gx {3}, gy {4}, gz {5}".format(ax_avg,ay_avg,az_avg,gx_avg,gy_avg,gz_avg))

  # Check for left/right motion
  if ay_avg > 0.5 and az_avg > 0.4:
    print("Left")
    pag.keyDown('a')
  elif ay_avg < -0.5 and az_avg > 0.4:
    print("Right")
    pag.keyDown('d')

  # Check for forward/back motion
  if ax_avg > 0.5 and az_avg > 0.4 :
    print("Forward")
    pag.keyDown('w')
  # Check for fly motion
  elif ax_avg < -0.8 and az_avg < 0.1 :
    print("Fly")
    pag.press(['space', 'space'])
  elif ax_avg < -0.5 and az_avg < 0.4:
    print("Back")
    pag.keyDown('s')

  
  time.sleep(0.05)
  pag.keyUp('w')
  pag.keyUp('s')
  pag.keyUp('a')
  pag.keyUp('d')
  pag.keyUp('space')
  

def stop_move():
  return True
  pag.keyUp('w')
  pag.keyUp('a')
  pag.keyUp('s')
  pag.keyUp('d')
  pag.keyUp('space')

def gyr_callback(data):
  
  global last_spin_time, spin_time_thresh
  #global gyr_buf, gyr_xbuf, gyr_ybuf, gyr_zbuf
  """Handle a (epoch, (x,y,z)) accelerometer tuple."""
  #print("Gyr {0}, {1}, {2}, {3}".format(data[0], *data[1]))
  if data[0] - last_spin_time > spin_time_thresh:
    last_spin_time = data[0] 
    # thread.start_new_thread( spin_around, (data[1],))
    #spin_thread = threading.Thread(target=spin_around, args=(data[1],))
    #spin_thread.start()
  gyr = magnitude(data[1])
  gyr_buf.append(gyr)
  gyr_xbuf.append(data[1][0])
  gyr_ybuf.append(data[1][1])
  gyr_zbuf.append(data[1][2])
  current_avg = gyr_buf.average()
  last_avg_gyr = gyr_buf.get_last_avg()
  delta_avg =  abs(current_avg - last_avg_gyr) 
  delta_time = data[0] - gyr_buf.get_last_time()
  if delta_time > 1600:
    gyr_buf.set_last_time(data[0])

    gyr_mag_data = gyr_buf.get() 
    gyr_x_data = gyr_xbuf.get() 
    gyr_y_data = gyr_ybuf.get() 
    gyr_z_data = gyr_zbuf.get() 
    #print("Gyr\t\tX\t\tY\t\tZ")
    
    for i in xrange(len(gyr_mag_data)):
      d=1
     #  print('{:03.2f} \t\t {:03.2f} \t\t {:03.2f} \t\t{:03.2f}'.format(gyr_mag_data[i],gyr_x_data[i],gyr_y_data[i],gyr_z_data[i]))
    #print(" ")
       
def acc_callback(data):
  global last_move_time, move_time_thresh, current_block_id, sw_pressed
  #global acc_buf, acc_xbuf, acc_ybuf, acc_zbuf
  """Handle a (epoch, (x,y,z)) accelerometer tuple."""
  # print("Epoch time: [{0}]".format(data[0]))
  # Check for move
  if data[0] - last_move_time > move_time_thresh and sw_pressed:
    last_move_time = data[0]
    #thread.start_new_thread( move_around, (data[1],))
    move_thread = threading.Thread(target=move_around, args=())
    move_thread.start()
    #move_around()
  acc = magnitude(data[1])
  acc_buf.append(acc)
  acc_xbuf.append(data[1][0])
  acc_ybuf.append(data[1][1])
  acc_zbuf.append(data[1][2])
  current_avg = acc_buf.average()
  last_avg_acc = acc_buf.get_last_avg()
  delta_avg =  abs(current_avg - last_avg_acc) 
  delta_time = data[0] - acc_buf.get_last_time()
  if check_ud_pattern():
    placeBlock()
  elif check_ldtr_pattern():
    build_staircase()
  elif check_tunnel_pattern():
    build_tunnel()

  """ 
  # Print acceleration stats for gesture recognition
  if delta_time > 1600:
    acc_buf.set_last_time(data[0])

    
    acc_mag_data = acc_buf.get() 
    acc_x_data = acc_xbuf.get() 
    acc_y_data = acc_ybuf.get() 
    acc_z_data = acc_zbuf.get() 
    print("Acc\t\tX\t\tY\t\tZ")
    
    for i in xrange(len(acc_mag_data)):
      print('{:03.2f} \t\t {:03.2f} \t\t {:03.2f} \t\t{:03.2f}'.format(acc_mag_data[i],acc_x_data[i],acc_y_data[i],acc_z_data[i]))
    print(" ")
    
  """       


  """
  if delta_avg > 0.3 and delta_time > 100 :
    print("Building. Block id is {0}".format(current_block_id))
    x, y, z = mc.player.getPos()
    #mc.setBlock(x+5, y+2, z, current_block_id)
    #mc.player.setPos(0,0,z)
    placeBlocks(current_block_id)
  acc_buf.set_last_avg(current_avg)
  acc_buf.set_last_time(data[0])
  """

def sw_callback(data):
  global last_move_time, move_time_thresh, sw_pressed
  if data[1]:
    print("Switch pressed")
    sw_pressed=True
  else: 
    print("Switch released")
    sw_pressed=False
    stop_move()

def battery_callback(data):
  print("Voltage: {0}, Charge: {1}%".format(data[1][0]/1000.0,data[1][1]))

# start main code

# Initialize minecraft
mc=minecraft.Minecraft.create()

#Block - pin mappings
block_pin = {2: [block.DIRT.id, "DIRT"],
             3: [block.WOOD.id, "WOOD"],
             4: [block.COBBLESTONE.id, "COBBLESTONE"],
             5: [block.GLASS.id, "GLASS"],
             6: [block.DIAMOND_BLOCK.id, "DIAMOND"],
             13:[block.STONE.id, "STONE"],
             19:[block.TNT.id, "TNT"] }


# Initialize metawear
backend =  'pygatt'  # Or 'pybluez'
while True:
  try:
    c = MetaWearClient('D5:05:98:AF:47:1D', backend)
    time.sleep(1.0)
    break
  except:
    mc.postToChat(" Connecting to Wand...")

acc_buf = RingBuffer(20)   # magnitude
acc_xbuf = RingBuffer(20)
acc_ybuf = RingBuffer(20)
acc_zbuf = RingBuffer(20)
gyr_buf = RingBuffer(20)   # magnitude
gyr_xbuf = RingBuffer(20)
gyr_ybuf = RingBuffer(20)
gyr_zbuf = RingBuffer(20)
last_move_time = 0
last_spin_time = 0
# constant threshold time between move/spin checks in ms
move_time_thresh = 200 
spin_time_thresh = 400 
sample_rate = 12.5 
current_block_id=block_pin[2][0]
sw_pressed = False

# Set accelerometer settings 
c.accelerometer.set_settings(data_rate=sample_rate, data_range=4.0)
time.sleep(0.2)

c.gyroscope.set_settings(data_rate=25.0, data_range=1000.0)
time.sleep(0.2)

# Get current settings
#settings=c.accelerometer.get_current_settings()
#print(settings)


print("connected")

# Enable high frequency stream
#c.accelerometer.high_frequency_stream = True
# Enable acc notifications and register a callback for them.
c.accelerometer.notifications(acc_callback)

# Enable gyro notifications and register a callback for them.
c.gyroscope.notifications(gyr_callback)

# Enable switch notifications and register a callback for them.
c.switch.notifications(sw_callback)

# Enable battery notifications
c.battery.notifications(battery_callback)

# Trigger battery callback
c.battery.read_battery_state()

# Enable GPIO
init_gpio()

mc.postToChat("Start !")
while True:
  time.sleep(20)

print("Unsubscribe notifications")
c.accelerometer.notifications(None)
c.gyroscope.notifications(None)

time.sleep(1.0)

c.disconnect()
