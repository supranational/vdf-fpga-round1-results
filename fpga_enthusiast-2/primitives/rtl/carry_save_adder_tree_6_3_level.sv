/*******************************************************************************
  Copyright 2019 Supranational LLC

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

/*
  Group the input terms into sets of three for input into a carry save adder
  Shift the CSA carry output by 1 for use in the next level
  The sum already has the correct weight, therefore we only pad for consistency
  Any leftover terms that did not fit into a set are returned padded
*/

module carry_save_adder_tree_6_3_level
   #(
     parameter int NUM_ELEMENTS = 6,
     parameter int BIT_LEN      = 19,
     parameter int NUM_RESULTS  = (integer'(NUM_ELEMENTS/6)*3) + (NUM_ELEMENTS%6))
   (
    input  logic [BIT_LEN-1:0] terms[NUM_ELEMENTS],
    output logic [BIT_LEN-1:0] results[NUM_RESULTS]
   );

   genvar i;
   generate
      for (i=0; i<(NUM_ELEMENTS / 6); i++) begin : csa_insts
         // Add three consecutive terms 
         carry_save_adder_6_3 #(.BIT_LEN(BIT_LEN))
            carry_save_adder_6_3 (
                              .A(terms[i*6]),
                              .B(terms[(i*6)+1]),
                              .C(terms[(i*6)+2]),
                              .D(terms[(i*6)+3]),
                              .E(terms[(i*6)+4]),
                              .F(terms[(i*6)+5]),
                              .Cout1({results[i*3][1:0],results[i*3][BIT_LEN-1:2]}),                        
                              .Cout({results[i*3+1][0],results[i*3+1][BIT_LEN-1:1]}),
                              .S(results[(i*3)+2][BIT_LEN-1:0])
                             );
      end
      // Save any unused terms for the next level 
      for (i=0; i<(NUM_ELEMENTS % 6); i++) begin : csa_level_extras
         always_comb begin
            results[(NUM_RESULTS - 1) - i][BIT_LEN-1:0] = terms[(NUM_ELEMENTS- 1) - i][BIT_LEN-1:0];
         end
      end
   endgenerate
endmodule
