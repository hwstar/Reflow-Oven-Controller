__author__ = 'srodgers'
"""
ldr.py
Display analog data from Arduino using Python (matplotlib)
Author: Mahesh Venkitachalam
Website: electronut.in
"""

import sys, serial, argparse
import numpy as np
from time import sleep
from collections import deque

import matplotlib.pyplot as plt
import matplotlib.animation as animation


# plot class
class AnalogPlot:
  # constr
  def __init__(self, strPort, maxLen):
      # open serial port
      self.ser = serial.Serial(strPort, 9600)

      #self.ax = deque([0.0]*maxLen)
      self.ay = deque([0.0]*maxLen)
      self.maxLen = maxLen
      self.state = 'Ready'

  # add to buffer
  def addToBuf(self, buf, val):
      if len(buf) < self.maxLen:
          buf.append(val)
      else:
          buf.pop()
          buf.appendleft(val)

  # add data
  def add(self, data):
      assert(len(data) == 2)
      #self.addToBuf(self.ax, data[0])
      self.addToBuf(self.ay, data[1])

  # update plot
  def update(self, frameNum, a0, a1):
      try:
          records = []

          while(True):
            records = self.ser.readline().split(':')
            if records[0] == 'S':
                self.state = records[1]
            if records[0] == 'T' and self.state  != 'Ready':
                break

          data = [int(records[1]), float(records[2])]
          # print data
          if(len(data) == 2):
              self.add(data)
              #a0.set_data(range(self.maxLen), self.ax)
              a1.set_data(range(self.maxLen), self.ay)
      except KeyboardInterrupt:
          print('exiting')

      return a0,

  # clean up
  def close(self):
      # close serial
      self.ser.flush()
      self.ser.close()

# main() function
def main():
  # create parser
  parser = argparse.ArgumentParser(description="LDR serial")
  # add expected arguments
  parser.add_argument('--port', dest='port', required=False)

  # parse args
  args = parser.parse_args()

  strPort = '/dev/ttyUSB0'
  #strPort = args.port

  print('reading from serial port %s...' % strPort)

  # plot parameters
  analogPlot = AnalogPlot(strPort, 2000)

  print('plotting data...')

  # set up animation
  fig = plt.figure()
  fig.suptitle('Reflow profile', fontsize=20)

  ax = plt.axes(xlim=(0, 500), ylim=(0, 300))
  plt.xlabel('Time', fontsize=12)
  plt.ylabel('Temperature (deg. C)', fontsize=12)

  a0, = ax.plot([], [])
  a1, = ax.plot([], [])
  anim = animation.FuncAnimation(fig, analogPlot.update,
                                 fargs=(a0,a1),
                                 interval=50)

  # show plot
  plt.show()

  # clean up
  analogPlot.close()

  print('exiting.')


# call main
if __name__ == '__main__':
  main()