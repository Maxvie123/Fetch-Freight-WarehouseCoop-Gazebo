#!/usr/bin/env python
"""   This module is used to read and write Portable GrayMap(PGM) files.
  PGM file format: http://netpbm.sourceforge.net/doc/pgm.html 
  Module          : pgm
  Created by      : Vijay
  Creation Date   : Jan 6, 2011
  Current version : 1.02
  -------
  Changes:
  -------
  1.0  | Jan 6, 2011  | Creation (pgmread, pgmwrite)
  1.01 | Jan 9, 2011  | Changed pgmread output and pgmwrite input 
                        from python list to numpy array
  1.02 | Jan 11, 2011 | Instead of importing full numpy,
                        imported array and int32  only
  """

from numpy import array, int32

def pgmread(filename):
  """  This function reads Portable GrayMap (PGM) image files and returns
  a numpy array. Image needs to have P2 or P5 header number.
  Line1 : MagicNum
  Line2 : Width Height
  Line3 : Max Gray level
  Lines starting with # are ignored """
  f = open(filename,'r')
  # Read header information
  count = 0
  while count < 3:
    line = f.readline()
    if line[0] == '#': # Ignore comments
      continue
    count = count + 1
    if count == 1: # Magic num info
      magicNum = line.strip()
      if magicNum != 'P2' and magicNum != 'P5':
        f.close()
        print 'Not a valid PGM file'
        exit()
    elif count == 2: # Width and Height
      [width, height] = (line.strip()).split()
      width = int(width)
      height = int(height)
    elif count == 3: # Max gray level
      maxVal = int(line.strip())
  # Read pixels information
  img = []
  buf = f.read()
  elem = buf.split()
  if len(elem) != width*height:
    print 'Error in number of pixels'
    exit()
  for i in range(height):
    tmpList = []
    for j in range(width):
      tmpList.append(elem[i*width+j])
    img.append(tmpList)
  return (array(img), width, height)

def pgmwrite(img, filename, maxVal=255, magicNum='P2'):
  """  This function writes a numpy array to a Portable GrayMap (PGM) 
  image file. By default, header number P2 and max gray level 255 are 
  written. Width and height are same as the size of the given list.
  Line1 : MagicNum
  Line2 : Width Height
  Line3 : Max Gray level
  Image Row 1
  Image Row 2 etc. """
  img = int32(img).tolist()
  f = open(filename,'w')
  width = 0
  height = 0
  for row in img:
    height = height + 1
    width = len(row)
  f.write(magicNum + '\n')
  f.write(str(width) + ' ' + str(height) + '\n')
  f.write(str(maxVal) + '\n')
  for i in range(height):
    count = 1
    for j in range(width):
      f.write(str(img[i][j]) + ' ')
      if count >= 17:
        # No line should contain gt 70 chars (17*4=68)
        # Max three chars for pixel plus one space
        count = 1
        f.write('\n')
      else:
        count = count + 1
    f.write('\n')
  f.close()
