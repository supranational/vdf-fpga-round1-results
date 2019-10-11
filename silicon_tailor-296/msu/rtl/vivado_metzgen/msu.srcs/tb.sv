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

`include "msuconfig.vh"


`ifndef MOD_LEN_DEF
`define MOD_LEN_DEF 1024
`endif
`ifndef MODULUS_DEF
 `define MODULUS_DEF 1024'd124066695684124741398798927404814432744698427125735684128131855064976895337309138910015071214657674309443149407457493434579063840841220334555160125016331040933690674569571217337630239191517205721310197608387239846364360850220896772964978569683229449266819903414117058030106528073928633017118689826625594484331
`endif


module tb();
   localparam integer MOD_LEN = 1024;
   //localparam integer MOD_LEN = 128;

   // Ozturk parameters
   localparam integer WORD_LEN         = 16;
   localparam integer BIT_LEN          = 17;
   localparam integer AXI_LEN          = 32;
   localparam integer SQ_IN_BITS       = 1024;
   localparam integer SQ_OUT_BITS      = 1024;
   localparam         MODULUS          = `MODULUS_DEF;
   
   
   logic                   clk;
   logic                   reset;
   logic                   start;
   logic                   valid;
   logic [MOD_LEN-1:0]     modulus;
   logic [SQ_IN_BITS-1:0]  sq_in;
   logic [SQ_OUT_BITS-1:0] sq_out;
   logic [MOD_LEN-1:0]     sq_out_expected;
   logic [SQ_OUT_BITS-1:0] sq_out_actual;
   logic [MOD_LEN-1:0]     sq_out_reduced;

   integer                 t_start;
   integer                 t_final;
   integer                 t_curr;
   
   integer                 test_file;
   integer                 i, ret;
   integer                 cycle_count;
   integer                 error_count;
   
   integer                 total_cycle_count;
   integer                 total_squarings;
   
    //modular_square_metzgen_wrapper      // Uses Clock-domain crossing
	modular_square_metzgen                 // Single Clock
    #(
 		.MOD_LEN  (MOD_LEN),
		.MODULUS  (MODULUS),
		.USE_DENSE_ADDERS (1)
    ) uut (
        clk,
        reset,
        start,
        sq_in,
        sq_out,
        valid
    );

   initial begin
      test_file = $fopen("../../../../../test.txt", "r");
      if(test_file == 0) begin
         $display("test_file handle was NULL");
         $finish;
      end
   end
                
   always begin
      #4ns clk = ~clk;
   end
    
   initial begin
      // Reset the design
      clk           = 1'b0;
      reset         = 1'b1;
      sq_in         = 0;
      start         = 1'b0;
      t_start       = 0;
      t_curr        = 0;

      //#500us;      // Wait for Fifos to initialize

      @(negedge clk);
      @(negedge clk);
      @(negedge clk);
      @(negedge clk);

      reset      = 1'b0;

      @(negedge clk);
      @(negedge clk);
      @(negedge clk);
      @(negedge clk);

      // Scan in the modulus and initial value
      $fscanf(test_file, "%x\n", sq_in); 
      @(negedge clk);

      start         = 1'b1;
      @(negedge clk);
      start         = 1'b0;

      // Run the squarer and periodically check results
      error_count   = 0;
      total_cycle_count          = 0;
      total_squarings            = 0;
      while(1) begin
         ret = $fscanf(test_file, "%d, %x\n", t_final, sq_out_expected);
         if(ret != 2) begin
            break;
         end 

         // Run to the next checkpoint specified in the test file
         cycle_count   = 1;
         t_start       = t_curr;
         while(t_curr < t_final) begin
            if(valid == 1'b1) begin
               t_curr        = t_curr + 1;
               sq_out_actual = sq_out;
               total_squarings   = total_squarings + 1;
            end

            @(negedge clk);
            cycle_count = cycle_count + 1;
            total_cycle_count    = total_cycle_count + 1;
         end

         // Reduce the result from polynomial form
         sq_out_reduced = sq_out_actual % MODULUS;

         $display("%5d %0.2f %x", t_final, 
                  real'(cycle_count) / real'(t_final - t_start), 
                  sq_out_reduced);

         // Check correctness
         if(sq_out_reduced !== sq_out_expected) begin
            $display("MISMATCH expected %x", sq_out_expected);
            $display("           actual %x", sq_out_reduced);
            error_count = error_count + 1;
            break;
         end
         // Removed wait for negedge clock so that the next valid data on the next clock cycle is not missed 
         //@(negedge clk);
         //total_cycle_count       = total_cycle_count + 1;
      end
      $display("Overall %d cycles, %d squarings, %0.2f cyc/sq", 
               total_cycle_count, total_squarings,
               real'(total_cycle_count) / real'(total_squarings)); 
      if(error_count == 0) begin
         $display("SUCCESS!");
         $finish();
      end
      @(negedge clk);
      @(negedge clk);
      @(negedge clk);
      @(negedge clk);
      $error("FAILURE %d mismatches", error_count);
      $finish();
   end
endmodule

