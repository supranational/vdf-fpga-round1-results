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
// A faster carry_save_adder_tree_level
//
//  9/12/2019
//
//  by Kurt Baty
//

module faster_carry_save_adder_tree_level
   #(
     parameter int NUM_ELEMENTS = 3,
     parameter int BIT_LEN      = 19,

     parameter int NUM_RESULTS  = (integer'(NUM_ELEMENTS/6) * 3) + 
                                   ((NUM_ELEMENTS%6 > 3)? 3 :
                                    ((NUM_ELEMENTS%6 > 1)? 2 : NUM_ELEMENTS%6))

    )
   (
    input  logic [BIT_LEN-1:0] terms[NUM_ELEMENTS],
    output logic [BIT_LEN-1:0] results[NUM_RESULTS]
   );

   localparam NEXT_ELEMENT  = (NUM_ELEMENTS/6) * 6;

   wire [BIT_LEN-1:0] local_results[NUM_RESULTS];

   genvar i,j;
   generate
      for (i=0; i<(NUM_ELEMENTS / 6); i++) begin : compress_6_3_insts
         // Add six consecutive terms 
         compressor_6_to_3 #(.BIT_LEN(BIT_LEN))
            compressor_6_to_3 (
                              .I0(terms[i*6]),
                              .I1(terms[(i*6)+1]),
                              .I2(terms[(i*6)+2]),
                              .I3(terms[(i*6)+3]),
                              .I4(terms[(i*6)+4]),
                              .I5(terms[(i*6)+5]),
                              .C4(local_results[ i*3   ][BIT_LEN-1:0]),
                              .C2(local_results[(i*3)+1][BIT_LEN-1:0]),
                              .S( results[(i*3)+2][BIT_LEN-1:0])
                             );

         assign results[i*3  ] = {local_results[i*3  ],2'b0};
         assign results[i*3+1] = {local_results[i*3+1],1'b0};

      end

      if (NUM_ELEMENTS%6 == 5) begin
         // Add five consecutive terms 
         compressor_6_to_3 #(.BIT_LEN(BIT_LEN))
            compressor_6_to_3 (
                              .I0(terms[NEXT_ELEMENT]),
                              .I1(terms[NEXT_ELEMENT+1]),
                              .I2(terms[NEXT_ELEMENT+2]),
                              .I3(terms[NEXT_ELEMENT+3]),
                              .I4(terms[NEXT_ELEMENT+4]),
                              .I5({BIT_LEN{1'b0}}),
                              .C4(local_results[NUM_RESULTS-3][BIT_LEN-1:0]),
                              .C2(local_results[NUM_RESULTS-2][BIT_LEN-1:0]),
                              .S( results[NUM_RESULTS-1][BIT_LEN-1:0])
                             );

         assign results[NUM_RESULTS-3] = {local_results[NUM_RESULTS-3],2'b0};
         assign results[NUM_RESULTS-2] = {local_results[NUM_RESULTS-2],1'b0};
      end

      else if (NUM_ELEMENTS%6 == 4) begin
         // Add four consecutive terms 
         compressor_6_to_3 #(.BIT_LEN(BIT_LEN))
            compressor_6_to_3 (
                              .I0(terms[NEXT_ELEMENT]),
                              .I1(terms[NEXT_ELEMENT+1]),
                              .I2(terms[NEXT_ELEMENT+2]),
                              .I3(terms[NEXT_ELEMENT+3]),
                              .I4({BIT_LEN{1'b0}}),
                              .I5({BIT_LEN{1'b0}}),
                              .C4(local_results[NUM_RESULTS-3][BIT_LEN-1:0]),
                              .C2(local_results[NUM_RESULTS-2][BIT_LEN-1:0]),
                              .S( results[NUM_RESULTS-1][BIT_LEN-1:0])
                             );

         assign results[NUM_RESULTS-3] = {local_results[NUM_RESULTS-3],2'b0};
         assign results[NUM_RESULTS-2] = {local_results[NUM_RESULTS-2],1'b0};
      end

      else if (NUM_ELEMENTS%6 == 3) begin
         // Add three consecutive terms 
         carry_save_adder #(.BIT_LEN(BIT_LEN))
            carry_save_adder (
                              .A(  terms[NEXT_ELEMENT]),
                              .B(  terms[NEXT_ELEMENT+1]),
                              .Cin(terms[NEXT_ELEMENT+2]),
                              .Cout(local_results[NUM_RESULTS-2][BIT_LEN-1:0]),
                              .S(   results[NUM_RESULTS-1][BIT_LEN-1:0])
                             );

         assign results[NUM_RESULTS-2] = {local_results[NUM_RESULTS-2],1'b0};

      end

      else if (NUM_ELEMENTS%6 == 2) begin
         // Save any unused terms for the next level 
         assign results[NUM_RESULTS -2][BIT_LEN-1:0] = 
                  terms[NUM_ELEMENTS-2][BIT_LEN-1:0];
         assign results[NUM_RESULTS -1][BIT_LEN-1:0] = 
                  terms[NUM_ELEMENTS-1][BIT_LEN-1:0];
      end

      else if (NUM_ELEMENTS%6 == 1) begin
         // Save any unused terms for the next level 
         assign results[NUM_RESULTS -1][BIT_LEN-1:0] = 
                  terms[NUM_ELEMENTS-1][BIT_LEN-1:0];
      end

   endgenerate
endmodule


