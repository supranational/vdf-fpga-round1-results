# Submission for the VDF FPGA Competition 

This repository contains my submission for the VDF low latency multiplier FPGA competition. 

My code is based on the baseline code as provided by the organisers. This code is available in [this GitHub repository](https://github.com/supranational/vdf-fpga) by Supranational.


## Description of Work / Documentation

The implementation provided in this repository is based on the low latency algorithm provided by Erdinc Ozturk of Sabanci University. However it replaces the main implementation provided in a number of ways in an effort to produce a more optimized (in terms of latency) final FPGA implementation.

In the following subsections, I provide the different ways this can be achieved.

** Attacking the generality of the multiplier implementation **

   INITIAL CONCEPT
   
   The following were the initial idea behind the multiply that was provided by
   vdf alliance.
   
   Multiply two arrays element by element
   The products are split into low (L) and high (H) values
   The products in each column are summed using compressor trees
   Leave results in carry/sum format

   Example A*B 4x4 element multiply results in 8 carry/sum values 
```
                                           |----------------------------------|
                                           |   B3   |   B2   |   B1   |   B0  |
                                           |----------------------------------|
                                           |----------------------------------|
                                     x     |   A3   |   A2   |   A1   |   A0  |
                                           |----------------------------------|
  ------------------------------------------------------------------------------    
   Column    
   Row     7        6        5        4        3        2        1        0
    0                                       A00B03L  A00B02L  A00B01L  A00B00L 
    1                              A00B03H  A00B02H  A00B01H  A00B00H          
    2                              A01B03L  A01B02L  A01B01L  A01B00L          
    3                     A01B03H  A01B02H  A01B01H  A01B00H                   
    4                     A02B03L  A02B02L  A02B01L  A02B00L                   
    5            A02B03H  A02B02H  A02B01H  A02B00H                            
    6            A03B03L  A03B02L  A03B01L  A03B00L                            
    7 + A03B03H  A03B02H  A03B01H  A03B00H
      -------------------------------------------------------------------------
         C7,S7    C6,S6    C5,S5    C4,S4    C3,S3    C2,S2    C1,S1    C0,S0
   ```      
      
  IMPROVED VERSION
  
  Since we actually want to do a squaring, therefore it is AxA rather than AxB. So
  then we can make a lot of improvements.
  
  1.
  Basic observation: since A and B are the same, in the picture above notice that
  for example A01B00 will be the same as B01A00. Therefore we can seriously reduce
  all multipliers required from n^2 to (n^2+n)/2
  
  2. 
  Based on the previous observation, in each column the terms, for example,
  A00B01L and A01B00L are the same. As a result, we can shorten significantly the depth 
  of the CSA tree, by using only the upper half of the grid and using all numbers multiplied
  by 2 (which is not a true multiplication but a shift and in actual implementation just a 
  wiring issue), except for the numbers in the main diagonal which are unique. The result is that 
  in the tree now we have to add at most n numbers instead of 2n-1
  
  3.
  Reducing the tree depth by as much, means that we can follow a much wider multiply strategy
  without sacrificing clock frequency as the critical path becomes significantly smaller.
  
  4.
  Problem: the above works only for squaring and NOT for general multiplications. This means
  that the strategy described cannot be generally applied in the reference source code provided by 
  vdfalliance. 
  Solution: It is now possible to complete the whole squaring operation (for all elements + 
  redundant elemements, i.e. no segmentation) in 2 cycles and the synthesis/implementation
  prove that this can be done safely @250MHz. 
  
  Note: the drawback is that significantly more DSPs are now required.        
  
 
 ** Simpler pipeline ** 

By using the aforementioned approach, the whole NUM_ELEMENTS*NUM_ELEMENTS squaring can be carried out
in two (2) cycles and synthesis and implementation results demonstrate that this can be achieved with
a 250MHz clock. The result is that the code becomes much simpler significantly reducing wiring and improving
routability despite the fact that much more resources are required (DSPs kinds explode). The circuit is more
regular and the wiring simpler. This way the routing-to-logic delay becomes smaller (routing delay is approx.
2x the logic delay while in reference implementation it is far worse).

 ** Stressing the LUTs for reduction **

The shortcoming of the aforementioned approach is that significantly more LUTs are required, as the optimizations
of the reference code cannot be applied in this implementation. 

A simple trick has been devised in order to reduce the stress on memory resources: since blockRAMs as primitives have a 
fixed address width (the data stored per memory slot can be expanded by parallel cascading more BRAMs) then it becomes 
"free" to use a memory with 512 elements instaed of 256 (for example for the case of LUT8 as described in Ozturk's paper).
As such, in each instantiated LUT, we use all available address space and load the precomputed values for two elements instead of one.

 ** Simpler pipeline leads to shorter pipeline **
 
 Since all previous operations are carried for the whole number that we have to deal with, this means that the pipeline
 can be reduced from 8 to 6 cycles following this structure:
 
 CYCLE 0:
 	Core multiplications
  		
  	These are all carried out in parallel using 17x17 multipliers implemented in DSP resources
  
 CYCLE 1:
 	CSA adder tree
 	
 	Using the construction mentioned above (using a grid of elements multiplied by 2), the Carry Save Adder
 	as provided by the reference code (compressor tree) can be used directly without modifications.
 
 CYCLE 2: 
 	Carry propagate addition
 	
 	Using DSP resources again, the carry propagate addition can be realized in a single cycle.
 
 CYCLE 3: 
 	LUT-based reduction
 	
 	The upper part of the results of the CPA are reduced through the look-up tables. Notice that we do not employ
 	the lookup tables provided by the reference code, but we generate the ones thata are appropriate. Also we implement two modules that handle the LUT8 and LUT9 lookup tables.
 	The result of this process requires a single cycle.
 
 CYCLE 4:
 	CSA adder tree
 	
 	The results are accumulated in a single cycle.
 	
 CYCLE 5: 
 	Carry propagate addition
 	
 	This is practically the same as Cycle 3.
 
 Cycle 5 completes the process providing the final result.
 
 	
 ## Performance Expectations
 
 As mentioned above the implementation is designed to take 6 cycles to complete. 
 
 Each pipeline stage is constructed with a 250MHz clock constraint. This clock frequency is one of the supported by the AWS F1 and therefore has been deemed as a valid target. 
 
 Synthesis and implementation results demonstrate that this clock can be achieved by the design.
 
 As such, the expected latency for modular square can be expected to be: 6 cycles * 4ns/cycles = 24ns.
 
 This improves the baseline model by 50ns-24ns = 26ns.
 
 

## Repository organization

The repository structure is based on the reference repository that was provided by the competition, with some notable differences:

- <i>primitives</i> directory does not have the multiply.sv file that contained the original multiply structure. Instead it contains the square_multiply.sv file that has the square multiplier employed in this implementation. 

- <i>modular_square</i> directory contains two subdirectories: rtl and lut_generator. In the first one you can find the my_modular_square.sv , reduction_lut8.sv and reduction_lut9.sv SystemVerilog files that contain the implementation of the modular square and the reduction luts. Also included are the *.dat files that contain the precomputed luts. These are named precompute_lut{8,9}_0{00..32}.dat that are required fot the internal ROMs. The python script (modified from the one supplied in the reference code) that calculates those LUTs can be found in the lut_generator directory.

- <i>msu</i> directory contains the modified modular_square_wrapper.sv file. The only change with the original file is that the module name of the modular square is now that of my modules instead of modular_square_8_cycles.


 ## Important notices when trying to simulate/synthesize/implement the design
 
 Although certain scripts (based on the original ones of the reference code) are provided, these do not work reliably for some peculiar reason (most probably something is left in the cache of my system). However since the implementation does not change the interfaces (as required by the rules), one needs simply to replace the modules that have changed and indicate that the autogenerated files of the original code should not be used. Instead the provided files have to be used.
 

## Contact

Please reach out with any questions, comments, or feedback :

- E-mail: abrokalakis@mhl.tuc.gr
