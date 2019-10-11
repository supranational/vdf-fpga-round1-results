/*******************************************************************************
  Copyright 2019 Kurt Baty

  Licensed under the Apache License, Version 2.0 (the "License");
  you may not use this file except in compliance with the License.
  You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  See the License for the specific language governing permissions and
  limitations under the License.

*******************************************************************************/

//
// compressor 6 to 3
//
//  9/25/2019
//
//  by Kurt Baty
//

/*    
  A compressor 6:3 Carry Two bit
*/

module compressor_6_to_3_C2_bit
   (
    input  logic I0,
    input  logic I1,
    input  logic I2,
    input  logic I3,
    input  logic I4,
    input  logic I5,
    output logic C2
   );

   // this maps to a LUT6 in Xilinx FPGAs

   always_comb begin
       case ({I5,I4,I3,I2,I1,I0})
           6'b00_00_00: C2 = 1'b0;
           6'b00_00_01: C2 = 1'b0;
           6'b00_00_10: C2 = 1'b0;
           6'b00_00_11: C2 = 1'b1;
           6'b00_01_00: C2 = 1'b0;
           6'b00_01_01: C2 = 1'b1;
           6'b00_01_10: C2 = 1'b1;
           6'b00_01_11: C2 = 1'b1;
           6'b00_10_00: C2 = 1'b0;
           6'b00_10_01: C2 = 1'b1;
           6'b00_10_10: C2 = 1'b1;
           6'b00_10_11: C2 = 1'b1;
           6'b00_11_00: C2 = 1'b1;
           6'b00_11_01: C2 = 1'b1;
           6'b00_11_10: C2 = 1'b1;
           6'b00_11_11: C2 = 1'b0;
           6'b01_00_00: C2 = 1'b0;
           6'b01_00_01: C2 = 1'b1;
           6'b01_00_10: C2 = 1'b1;
           6'b01_00_11: C2 = 1'b1;
           6'b01_01_00: C2 = 1'b1;
           6'b01_01_01: C2 = 1'b1;
           6'b01_01_10: C2 = 1'b1;
           6'b01_01_11: C2 = 1'b0;
           6'b01_10_00: C2 = 1'b1;
           6'b01_10_01: C2 = 1'b1;
           6'b01_10_10: C2 = 1'b1;
           6'b01_10_11: C2 = 1'b0;
           6'b01_11_00: C2 = 1'b1;
           6'b01_11_01: C2 = 1'b0;
           6'b01_11_10: C2 = 1'b0;
           6'b01_11_11: C2 = 1'b0;
           6'b10_00_00: C2 = 1'b0;
           6'b10_00_01: C2 = 1'b1;
           6'b10_00_10: C2 = 1'b1;
           6'b10_00_11: C2 = 1'b1;
           6'b10_01_00: C2 = 1'b1;
           6'b10_01_01: C2 = 1'b1;
           6'b10_01_10: C2 = 1'b1;
           6'b10_01_11: C2 = 1'b0;
           6'b10_10_00: C2 = 1'b1;
           6'b10_10_01: C2 = 1'b1;
           6'b10_10_10: C2 = 1'b1;
           6'b10_10_11: C2 = 1'b0;
           6'b10_11_00: C2 = 1'b1;
           6'b10_11_01: C2 = 1'b0;
           6'b10_11_10: C2 = 1'b0;
           6'b10_11_11: C2 = 1'b0;
           6'b11_00_00: C2 = 1'b1;
           6'b11_00_01: C2 = 1'b1;
           6'b11_00_10: C2 = 1'b1;
           6'b11_00_11: C2 = 1'b0;
           6'b11_01_00: C2 = 1'b1;
           6'b11_01_01: C2 = 1'b0;
           6'b11_01_10: C2 = 1'b0;
           6'b11_01_11: C2 = 1'b0;
           6'b11_10_00: C2 = 1'b1;
           6'b11_10_01: C2 = 1'b0;
           6'b11_10_10: C2 = 1'b0;
           6'b11_10_11: C2 = 1'b0;
           6'b11_11_00: C2 = 1'b0;
           6'b11_11_01: C2 = 1'b0;
           6'b11_11_10: C2 = 1'b0;
           6'b11_11_11: C2 = 1'b1;
       endcase
   end

endmodule

