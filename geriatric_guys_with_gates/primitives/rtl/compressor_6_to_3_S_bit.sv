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
  A compressor 6:3 Sum bit
*/

module compressor_6_to_3_S_bit
   (
    input  logic I0,
    input  logic I1,
    input  logic I2,
    input  logic I3,
    input  logic I4,
    input  logic I5,
    output logic S
   );

   // this maps to a LUT6 in Xilinx FPGAs

   always_comb begin
       S = ^{I5,I4,I3,I2,I1,I0};
   end

endmodule

