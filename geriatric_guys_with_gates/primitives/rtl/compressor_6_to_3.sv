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
//  9/5/2019
//
//  by Kurt Baty
//

/*    
  A compressor 6:3
  Loops through each input bit and
  feeds three LUT6 one for each output
*/

module compressor_6_to_3
   #(
     parameter BIT_LEN = 19
   )
   (
    input  logic [BIT_LEN-1:0] I0,
    input  logic [BIT_LEN-1:0] I1,
    input  logic [BIT_LEN-1:0] I2,
    input  logic [BIT_LEN-1:0] I3,
    input  logic [BIT_LEN-1:0] I4,
    input  logic [BIT_LEN-1:0] I5,
    output wire  [BIT_LEN-1:0] C4,
    output wire  [BIT_LEN-1:0] C2,
    output wire  [BIT_LEN-1:0] S
   );

   genvar i;
   generate
      for (i=0; i<BIT_LEN; i++) begin : LUT6_insts

         compressor_6_to_3_C4_bit
         C4_bit_inst
	 (
	    .C4(C4[i]),  //1-bit output: LUT
	    .I0(I0[i]), //1-bit input:  LUT
	    .I1(I1[i]), //1-bit input:  LUT
	    .I2(I2[i]), //1-bit input:  LUT
	    .I3(I3[i]), //1-bit input:  LUT
	    .I4(I4[i]), //1-bit input:  LUT
	    .I5(I5[i])  //1-bit input:  LUT
	 );


         compressor_6_to_3_C2_bit
         C2_bit_inst
	 (
	    .C2(C2[i]),  //1-bit output: LUT
	    .I0(I0[i]), //1-bit input:  LUT
	    .I1(I1[i]), //1-bit input:  LUT
	    .I2(I2[i]), //1-bit input:  LUT
	    .I3(I3[i]), //1-bit input:  LUT
	    .I4(I4[i]), //1-bit input:  LUT
	    .I5(I5[i])  //1-bit input:  LUT
	 );


         compressor_6_to_3_S_bit
         S_bit_inst
	 (
	    .S(S[i]),   //1-bit output: LUT
	    .I0(I0[i]), //1-bit input:  LUT
	    .I1(I1[i]), //1-bit input:  LUT
	    .I2(I2[i]), //1-bit input:  LUT
	    .I3(I3[i]), //1-bit input:  LUT
	    .I4(I4[i]), //1-bit input:  LUT
	    .I5(I5[i])  //1-bit input:  LUT
	 );

      end
   endgenerate
endmodule

