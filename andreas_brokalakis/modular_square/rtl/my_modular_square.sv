/*******************************************************************************
  Copyright 2019 Andreas Brokalakis
  School of Electrical and Computer Engineering 
  Technical University of Crete

  Licensed under the Apache License, Version 2.0 (the "License");
  you may not use this file except in compliance with the License.
  You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  See the License for the specific language governing permissions and
  limitations under the License.

  ACKNOWLEDGEMENTS
  
  1. Some parts/ideas for the code written below were taken from the source code
  provided by Supranational here: https://github.com/supranational/vdf-fpga
  2. This code has been written as part of my submission for the FPGA Contest 
  organized by VDFalliance: https://www.vdfalliance.org/contest 

*******************************************************************************/

module my_modular_square
   #(
     parameter int REDUNDANT_ELEMENTS    = 2,
     parameter int NONREDUNDANT_ELEMENTS = 64,
     parameter int NUM_SEGMENTS          = 1,
     parameter int BIT_LEN               = 17,
     parameter int WORD_LEN              = 16,

     parameter int NUM_ELEMENTS          = REDUNDANT_ELEMENTS +
                                           NONREDUNDANT_ELEMENTS
    )
   (
    input logic                   clk,
    input logic                   reset,
    input logic                   start,
    input logic [BIT_LEN-1:0]     sq_in[NUM_ELEMENTS],
    output logic [BIT_LEN-1:0]    sq_out[NUM_ELEMENTS],
    output logic                  valid
   );
 
  localparam int ACC_EXTRA_BIT_LEN   = $clog2(2*NUM_ELEMENTS+1);
  localparam int ACC_BIT_LEN         = BIT_LEN + ACC_EXTRA_BIT_LEN;
  localparam int ADDR_LUT	     = int'(WORD_LEN / 2);
  
  localparam int IDLE                = 0,
                  CYCLE_0             = 1,
                  CYCLE_1             = 2,
                  CYCLE_2             = 3,
                  CYCLE_3             = 4,
                  CYCLE_4             = 5,
                  CYCLE_5             = 6,
                  NUM_CYCLES          = 7;
 
 //SIGNALS declarations!!!
 // Cycle number state machine
   logic [NUM_CYCLES-1:0]    next_cycle;
   logic [NUM_CYCLES-1:0]    curr_cycle;
   
    logic [BIT_LEN-1:0]       sq_in_d1[NUM_ELEMENTS];
   logic                     start_d1;
   logic [BIT_LEN-1:0]       sq_out_d1[NUM_ELEMENTS];

   // Input to square (start of phase 1)
   logic [BIT_LEN-1:0]       curr_sq_in[NUM_ELEMENTS];
   
   logic                     out_valid;
 
 logic [BIT_LEN-1:0]	mult_out[NUM_ELEMENTS*2];
 logic [BIT_LEN-1:0]	multout_firsthalf[NUM_ELEMENTS];
 logic [BIT_LEN-1:0]	multout_firsthalf_sc[NUM_ELEMENTS];
 logic [ADDR_LUT-1:0]	lut8_addr[NUM_ELEMENTS];
 logic [ADDR_LUT:0]	lut9_addr[NUM_ELEMENTS];
 logic [BIT_LEN-1:0]	lut8_data[NUM_ELEMENTS][NUM_ELEMENTS];
 logic [BIT_LEN-1:0]	lut9_data[NUM_ELEMENTS][NUM_ELEMENTS];
 logic [ACC_BIT_LEN-1:0]    acc_data[NUM_ELEMENTS][(2*NUM_ELEMENTS)+1];
 
 logic [ACC_BIT_LEN-1:0] acc_C[NUM_ELEMENTS];
 logic [ACC_BIT_LEN-1:0] acc_S[NUM_ELEMENTS];
 logic [ACC_BIT_LEN-1:0] C_reg[NUM_ELEMENTS];
 logic [ACC_BIT_LEN-1:0] S_reg[NUM_ELEMENTS];
 logic [ACC_BIT_LEN:0]	 acc_sum[NUM_ELEMENTS];
 logic [BIT_LEN-1:0]	 acc_sum_out[NUM_ELEMENTS];
 
 
  
 //SIGNALS declarations end
 
   // State machine setting values based on current cycle
   always_comb begin
      next_cycle                  = '0;
      out_valid                   = 1'b0;
      

      if (reset) begin
         next_cycle               = '0;
         next_cycle[IDLE]         = 1'b1;
         out_valid                = 1'b0;
      end
      else begin
         unique case(1'b1)
            curr_cycle[IDLE]: begin
               if (start) begin
                  next_cycle[CYCLE_0]      = 1'b1;
               end
               else begin
                  next_cycle[IDLE]         = 1'b1;
               end
            end
            curr_cycle[CYCLE_0]: begin
               next_cycle[CYCLE_1]         = 1'b1;
            end
            curr_cycle[CYCLE_1]: begin

               next_cycle[CYCLE_2]         = 1'b1;
            end
            curr_cycle[CYCLE_2]: begin
               
               next_cycle[CYCLE_3]         = 1'b1;
            end
            curr_cycle[CYCLE_3]: begin
              
               next_cycle[CYCLE_4]         = 1'b1;
            end
            curr_cycle[CYCLE_4]: begin
               
               next_cycle[CYCLE_5]         = 1'b1;
            end
            curr_cycle[CYCLE_5]: begin
              next_cycle[CYCLE_0]      = 1'b1;
              out_valid                = 1'b1;  
            end
         endcase
      end
   end

 
 always_ff @(posedge clk) begin
       if (reset) begin
          valid                       <= 1'b0;
          start_d1                    <= 1'b0;
       end
       else begin
          valid                       <= out_valid;
 
          // Keep start high once set until sq_out is valid for loopback
          start_d1                    <= start || (start_d1 && ~out_valid);
       end
 
       curr_cycle                     <= next_cycle;
 
       if (start) begin
          for (int k=0; k<NUM_ELEMENTS; k=k+1) begin
             sq_in_d1[k][BIT_LEN-1:0] <= sq_in[k][BIT_LEN-1:0];
          end 
       end
   end
   
   always_ff @(posedge clk) begin
         for (int k=0; k<NUM_ELEMENTS; k=k+1) begin
            sq_out_d1[k] <= sq_out[k];
         end
   end
   // Mux square input from external or loopback
   // When looping back use the flopped lower half coefficients
   always_comb begin
      for (int k=0; k < NUM_ELEMENTS; k=k+1) begin
         curr_sq_in[k][BIT_LEN-1:0]    = sq_out_d1[k][BIT_LEN-1:0];

         if (start_d1) begin
            curr_sq_in[k][BIT_LEN-1:0] = sq_in_d1[k][BIT_LEN-1:0];
         end
      end
   end
   
 
 
 
 
 //Cycle 0 - MULTIPLY using my square_multiply
 //------------------------------------------------------------------------------------
 
 //square_multiply takes three cycles to complete and returns the results
 //Beware: Cycle 0 : core multipliers
 //Cycle 1: adder tree
 //Cycle 2: Carry Propagation addition
 
 multiply
    #(
    	//Use default MODULE PARAMETERS
    )
   square_multiplier   (
    	//MODULE INPUTS
 	.clk (clk),
 	.A (curr_sq_in),
 	.B (curr_sq_in),	//squaring therefore B = A
    	//END MODULE INPUTS
    	//MODULE OUTPUTS   
 	.reduced_output(mult_out)
 	
 	//END MODULE OUTPUTS
   );
 
 //the output of multiply is registered so I dont have to use FFs here
 
 //Cycle 3 - REDUCTION: use the precomputed LUTs for the reduction
 //---------------------------------------------------------------------
 
 //I need to use the upper half of the mult_out[NUM_ELEMENTS*2] elements
 //to address the luts and get the NUM_ELEMENTS results in redundant form
 
 //store the part of mult_out that is not going to be used in this cycle
 always_ff @(posedge clk)
 begin
 	for (int k = 0; k < NUM_ELEMENTS; k = k+1)
 	begin
 		multout_firsthalf[k] <= mult_out[k];
 	end
 end
 
 //prepare the inputs for the two LUTs (LUT8 and LUT9)
 always_comb begin
 	//we need to drive to the reduction_lut8 the lower part
 	for (int k = 0; k < NUM_ELEMENTS; k = k+1)
 	begin
 		lut8_addr[k] = mult_out[k+NUM_ELEMENTS][7:0];
 	end
 	//we need to drive to the reduction_lut9 the upper part
 	for (int k = 0; k < NUM_ELEMENTS; k = k+1)
	begin
		lut9_addr[k] = mult_out[k+NUM_ELEMENTS][16:8];
 	end
	
 end
 
 reduction_lut8 #(
  		//Use default MODULE PARAMETERS
                    )
        reduction_lut8 (
        	       //MODULE INPUTS
                       .clk(clk),
                       .lut8_addr(lut8_addr),
        	       //MODULE OUTPUTS
        	       .lut8_data(lut8_data)
                    ); 
 
 reduction_lut9 #(
 		//Use default MODULE PARAMETERS
                   )
       reduction_lut9 (
       		    //MODULE INPUTS
                      .clk(clk),
                      .lut9_addr(lut9_addr),
       	            //MODULE OUTPUTS
       		      .lut9_data(lut9_data)
                    );
 
 //Cycle 4: CSA addition
 //-------------------------------------------------------------------------
 
 //Form adder trees to accumulate the outputs coming from the LUTs. Remember 
 //to include the outputs 0 to NUM_ELEMENTS from the square multiplication.

 //make properly each input for the compressor tree
 //The compressor has to compress 133 values (2*NUM_ELEMENTS+1)
 always_comb
 begin
 	for (int k=0; k < NUM_ELEMENTS; k = k+1) 
 	begin
	    for (int l = 0; l < ((2*NUM_ELEMENTS)+1); l = l+1) begin
	       acc_data[k][l] = 0;
	    end
         end
 	for (int k = 0; k < NUM_ELEMENTS; k = k+1)
 	begin
 		for (int l = 0; l < NUM_ELEMENTS; l = l+1)
 		begin
 			acc_data[k][l][BIT_LEN-1:0] = lut9_data[k][l];
 		end
 		for (int l = NUM_ELEMENTS; l < 2*NUM_ELEMENTS; l = l+1)
		begin
			acc_data[k][l][BIT_LEN-1:0] = lut8_data[k][l-NUM_ELEMENTS];
 		end
 		acc_data[k][2*NUM_ELEMENTS][BIT_LEN-1:0] = multout_firsthalf[k];
 	end
 end
 
  // Instantiate compressor trees to accumulate over accumulator columns
  genvar i;
  generate
	  for (i=0; i<NUM_ELEMENTS; i=i+1) begin : final_acc
		compressor_tree_3_to_2 #(.NUM_ELEMENTS((2*NUM_ELEMENTS)+1),
					 .BIT_LEN(ACC_BIT_LEN)
				  )
			compressor_tree_3_to_2 (
				     .terms(acc_data[i]),
				     .C(acc_C[i]),
				     .S(acc_S[i])
				    );
	  end	  
  endgenerate
 
  always_ff @(posedge clk)
  begin
  	for (int k = 0; k < NUM_ELEMENTS; k = k+1)
  	begin
  		C_reg[k] <= acc_C[k];
  		S_reg[k] <= acc_S[k];
  	end
  
  end
 
 //Cycle 5: CPA addition and finish
 //-------------------------------------------------------------------------
	// Carry propogate add each column 
	// Partially reduce adding neighbor carries
	always_comb begin
		for (int k = 0; k < NUM_ELEMENTS; k = k+1) begin
			//CPA each Cout and S of the CSA to get final result
			//remember each Cout and S are ACC_BIT_LEN bits long and therefore each sum is ACC_BIT_LEN+1 bits long
			acc_sum[k] = C_reg[k] + S_reg[k];
		end
		
		acc_sum_out[0] = {{(BIT_LEN-WORD_LEN){1'b0}}, acc_sum[0][WORD_LEN-1:0]};

		for (int k=1; k < NUM_ELEMENTS-1; k=k+1) begin
			//Consider each coefficient is BIT_LEN bits long.
			//Add the carry from the previous coefficient.
			acc_sum_out[k] = {{(BIT_LEN-WORD_LEN){1'b0}}, acc_sum[k][WORD_LEN-1:0]} + {{(BIT_LEN-(ACC_BIT_LEN-WORD_LEN))-1{1'b0}}, acc_sum[k-1][ACC_BIT_LEN:WORD_LEN]};
		end
		acc_sum_out[NUM_ELEMENTS-1] = acc_sum[NUM_ELEMENTS-1][BIT_LEN-1:0] +  {{(BIT_LEN-(ACC_BIT_LEN-WORD_LEN))-1{1'b0}}, acc_sum[NUM_ELEMENTS-2][ACC_BIT_LEN:WORD_LEN]};
	end
 	
 	//register outputs
 	always_ff @(posedge clk) begin
    		for (int k = 0; k < NUM_ELEMENTS; k = k+1) begin
    			sq_out[k] <= acc_sum_out[k];
    		end
 	end
 
 
 
   
endmodule
