#!/usr/bin/python3

################################################################################
# Copyright 2019 Andreas Brokalakis, Technical University of Crete, Greece
# Code is heavily dependent on the code provided by Supranational in the 
# VDF FPGA contest
#
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#   http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
################################################################################

import sys
import getopt

################################################################################
# Parameters to set
################################################################################
REDUNDANT_ELEMENTS    = 2
NONREDUNDANT_ELEMENTS = 64
NUM_SEGMENTS          = 1
WORD_LEN              = 16
EXTRA_ELEMENTS        = 2
NUM_URAM              = 0

# TODO - we probably don't need these hardcoded values anymore
if (NONREDUNDANT_ELEMENTS == 128):
   M = 6314466083072888893799357126131292332363298818330841375588990772701957128924885547308446055753206513618346628848948088663500368480396588171361987660521897267810162280557475393838308261759713218926668611776954526391570120690939973680089721274464666423319187806830552067951253070082020241246233982410737753705127344494169501180975241890667963858754856319805507273709904397119733614666701543905360152543373982524579313575317653646331989064651402133985265800341991903982192844710212464887459388853582070318084289023209710907032396934919962778995323320184064522476463966355937367009369212758092086293198727008292431243681
else:
   M = 302934307671667531413257853548643485645

try:
   opts, args = getopt.getopt(sys.argv[1:],"hM:r:n:w:",           \
                              ["modulus=","redundant=",           \
                               "nonredundant=", "wordlen=", "urams="])
except getopt.GetoptError:
   print ('gen_reduction_lut.py -M <modulus> -r <num redundant>', \
         '-nr <num nonredundant> -wl <word length> -u <num_uram>')
   sys.exit(2)

for opt, arg in opts:
   if opt == '-h':
      print ('gen_reduction_lut.py -M <modulus> -r <num redundant>', \
            '-nr <num nonredundant> -wl <word length> -u <num_uram>')
      sys.exit()
   elif opt in ("-M", "--modulus"):
      M = int(arg)
   elif opt in ("-r", "--redundant"):
      REDUNDANT_ELEMENTS = int(arg)
   elif opt in ("-n", "--nonredundant"):
      NONREDUNDANT_ELEMENTS = int(arg)
   elif opt in ("-w", "--wordlen"):
      WORD_LEN = int(arg)
   elif opt in ("-u", "--urams"):
      NUM_URAM = int(arg)



################################################################################
# Calculated parameters
################################################################################
SEGMENT_ELEMENTS      = (NONREDUNDANT_ELEMENTS // NUM_SEGMENTS)
#LUT_NUM_ELEMENTS      = REDUNDANT_ELEMENTS + (SEGMENT_ELEMENTS*2) + \
#                        EXTRA_ELEMENTS
LUT_NUM_ELEMENTS	      = 33

LOOK_UP_WIDTH         = WORD_LEN // 2
LUT8_SIZE              = 2**LOOK_UP_WIDTH
LUT9_SIZE             =  2**(LOOK_UP_WIDTH+1)		
LUT_WIDTH             = WORD_LEN * NONREDUNDANT_ELEMENTS;



################################################################################
# Compute the reduction tables
################################################################################

print ('Creating', LUT_NUM_ELEMENTS, 'files')
print ('precompute_lut_{0:03d}.dat'.format(0))
print ('         ...          ')
print ('precompute_lut_{0:03d}.dat'.format(LUT_NUM_ELEMENTS-1))

for i in range (LUT_NUM_ELEMENTS):
   

   # Polynomial degree offset for V7V6
   offset = 8

   # Compute base reduction value for the coefficient degree
   t_lut9 = (2**(((i + NONREDUNDANT_ELEMENTS) * WORD_LEN) + offset)) % M
   t_lut8 = (2**((i + NONREDUNDANT_ELEMENTS) * WORD_LEN)) % M

   # Each address represents a different value stored in the coefficient
   Filename = list('precompute_lut8_{0:03d}.dat'.format(i))
   f = open(''.join(Filename), 'w')
   for j in range (LUT8_SIZE):
      cur_lut8 = (t_lut8 * j) % M
      f.write(hex(cur_lut8)[2:].zfill(LUT_WIDTH // 4))
      f.write('\n')
   #f.close()

   i = i+1
   for j in range (LUT8_SIZE):
      cur_lut8 = (t_lut8 * j) % M
      f.write(hex(cur_lut8)[2:].zfill(LUT_WIDTH // 4))
      f.write('\n')
   f.close() 

   i = i-1  


   Filename = list('precompute_lut9_{0:03d}.dat'.format(i))
   f = open(''.join(Filename), 'w')

   for j in range (LUT9_SIZE):
      cur_lut9 = (t_lut9 * j) % M
      f.write(hex(cur_lut9)[2:].zfill(LUT_WIDTH // 4))
      f.write('\n')
  
   i = i+1
   for j in range (LUT9_SIZE):
      cur_lut9 = (t_lut9 * j) % M
      f.write(hex(cur_lut9)[2:].zfill(LUT_WIDTH // 4))
      f.write('\n')

   f.close()


