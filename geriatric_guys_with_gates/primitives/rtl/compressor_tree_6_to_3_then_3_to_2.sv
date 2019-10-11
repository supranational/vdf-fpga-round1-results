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
// compressor_tree_6_to_3_then_3_to_2
//
//  9/12/2019
//
//  by Kurt Baty
//

/*
  Tree built out of 6:3 compressors.  
  Then a final row of 3:2 compressors.  
  Parameterized to take any number of inputs, each of a common size
*/

module compressor_tree_6_to_3_then_3_to_2
   #(
     parameter int NUM_ELEMENTS      = 9,
     parameter int BIT_LEN           = 16
    )
   (
    input  logic [BIT_LEN-1:0] terms[NUM_ELEMENTS],
    output logic [BIT_LEN-1:0] C4, C2,
    output logic [BIT_LEN-1:0] S
   );

//`ifdef FASTSIM
   // This is intended for simulation only to improve compile and run time
//   always_comb begin
//      C = 0;
//      S = 0;
//      for(int k = 0; k < NUM_ELEMENTS; k++) begin
//         S += terms[k];
//      end
//   end
   
//`else

   // If there is only one or two elements, then return the input (no tree)
   // If there are three elements, this is the last level in the tree
   // For greater than three elements:
   //   Instantiate a set of carry save adders to process this level's terms
   //   Recursive instantiate this module to complete the rest of the tree
   generate
      if (NUM_ELEMENTS == 1) begin // Return value
         always_comb begin
            C4[BIT_LEN-1:0] = '0;
            C2[BIT_LEN-1:0] = '0;
            S[BIT_LEN-1:0] = terms[0];
         end
      end
      else if (NUM_ELEMENTS == 2) begin // Return value
         always_comb begin
            C4[BIT_LEN-1:0] = '0;
            C2[BIT_LEN-1:0] = terms[1];
            S[BIT_LEN-1:0] = terms[0];
         end
      end
      else if (NUM_ELEMENTS == 3) begin // last level
         /* verilator lint_off UNUSED */
        logic [BIT_LEN-1:0] Cout;
         /* verilator lint_on UNUSED */
         
         carry_save_adder #(.BIT_LEN(BIT_LEN))
            carry_save_adder (
                              .A(terms[0]),
                              .B(terms[1]),
                              .Cin(terms[2]),
                              .Cout(Cout),
                              .S(S[BIT_LEN-1:0])
                             );
         always_comb begin
            C4[BIT_LEN-1:0] = 'b0;
            C2[BIT_LEN-1:0] = {Cout[BIT_LEN-2:0], 1'b0};
         end
      end
      else if ((NUM_ELEMENTS > 3) && (NUM_ELEMENTS <= 6)) begin

         logic [BIT_LEN-1:0] local_terms[6];

         genvar m;
         for (m=0;m<6;m=m+1) begin
           if (m < NUM_ELEMENTS) begin
             assign local_terms[m] = terms[m];
           end
           else begin
             assign local_terms[m] = 'b0;
           end
         end
         logic [BIT_LEN-1:0] next_level_terms[3];

         faster_carry_save_adder_tree_level #(.NUM_ELEMENTS(6),
                                       .BIT_LEN(BIT_LEN)
                                      )
            faster_carry_save_adder_tree_level (
                                         .terms(local_terms),
                                         .results(next_level_terms)
                                        );

         compressor_tree_6_to_3_then_3_to_2 #(.NUM_ELEMENTS(3),
                                  .BIT_LEN(BIT_LEN)
                                 )
            compressor_tree_6_to_3_then_3_to_2 (
                                    .terms(next_level_terms),
                                    .C4(C4),
                                    .C2(C2),
                                    .S(S)
                                   );
      end
      else if (NUM_ELEMENTS > 6) begin

         localparam integer NUM_RESULTS = (integer'(NUM_ELEMENTS/6) * 3) + 
                                          ((NUM_ELEMENTS%6 > 3)? 3 :
                                           ((NUM_ELEMENTS%6 > 1)? 2 : NUM_ELEMENTS%6));

         logic [BIT_LEN-1:0] next_level_terms[NUM_RESULTS];

         faster_carry_save_adder_tree_level #(.NUM_ELEMENTS(NUM_ELEMENTS),
                                       .BIT_LEN(BIT_LEN)
                                      )
            faster_carry_save_adder_tree_level (
                                         .terms(terms),
                                         .results(next_level_terms)
                                        );

         compressor_tree_6_to_3_then_3_to_2 #(.NUM_ELEMENTS(NUM_RESULTS),
                                  .BIT_LEN(BIT_LEN)
                                 )
            compressor_tree_6_to_3_then_3_to_2 (
                                    .terms(next_level_terms),
                                    .C4(C4),
                                    .C2(C2),
                                    .S(S)
                                   );
      end
   endgenerate
//`endif
endmodule

