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

module reduction_lut
   #(
     parameter int REDUNDANT_ELEMENTS    = 2,
     parameter int NONREDUNDANT_ELEMENTS = 8,
     parameter int NUM_SEGMENTS          = 4,
     parameter int WORD_LEN              = 16,
     parameter int BIT_LEN               = 17,
     parameter int DIN_LEN               = 8,

     parameter int NUM_ELEMENTS          = REDUNDANT_ELEMENTS+
                                           NONREDUNDANT_ELEMENTS,
     parameter int LOOK_UP_WIDTH         = int'(WORD_LEN / 2),
     parameter int SEGMENT_ELEMENTS      = int'(NONREDUNDANT_ELEMENTS /
                                                 NUM_SEGMENTS),
     parameter int EXTRA_ELEMENTS        = 2,
     parameter int LUT_NUM_ELEMENTS      = REDUNDANT_ELEMENTS+EXTRA_ELEMENTS+
                                           (SEGMENT_ELEMENTS*2)

    )
   (
    input  logic                    clk,
    input  logic [LOOK_UP_WIDTH:0]  lut_addr[LUT_NUM_ELEMENTS],
    input  logic                    shift_high,
    input  logic                    shift_overflow,
    output logic [BIT_LEN-1:0]      lut_data[NUM_ELEMENTS][LUT_NUM_ELEMENTS],
/* verilator lint_off UNUSED */
    input                           we,
    input [DIN_LEN-1:0]             din,
    input                           din_valid
/* verilator lint_on UNUSED */
   );

   // There is twice as many entries due to low and high values
   localparam int NUM_LUT_ENTRIES   = 2**(LOOK_UP_WIDTH+1);
   localparam int LUT_WIDTH         = WORD_LEN * NONREDUNDANT_ELEMENTS;

   localparam int NUM_URAM          = 0;
   localparam int NUM_BRAM          = LUT_NUM_ELEMENTS-NUM_URAM;
   localparam int XFERS_PER_URAM    = (LUT_WIDTH*NUM_LUT_ENTRIES)/DIN_LEN;

   logic [LUT_WIDTH-1:0]  lut_read_data[LUT_NUM_ELEMENTS];
   logic [LUT_WIDTH-1:0]  lut_read_data_bram[NUM_BRAM];
   logic [BIT_LEN-1:0]    lut_output[NUM_ELEMENTS][LUT_NUM_ELEMENTS];

   // Delay to align with data from memory
   logic shift_high_1d;
   logic shift_overflow_1d;

   always_ff @(posedge clk) begin
      shift_high_1d     <= shift_high;
      shift_overflow_1d <= shift_overflow;
   end
   (* rom_style = "block" *) logic [LUT_WIDTH-1:0] lut_000[NUM_LUT_ENTRIES];
   (* rom_style = "block" *) logic [LUT_WIDTH-1:0] lut_001[NUM_LUT_ENTRIES];
   (* rom_style = "block" *) logic [LUT_WIDTH-1:0] lut_002[NUM_LUT_ENTRIES];
   (* rom_style = "block" *) logic [LUT_WIDTH-1:0] lut_003[NUM_LUT_ENTRIES];
   (* rom_style = "block" *) logic [LUT_WIDTH-1:0] lut_004[NUM_LUT_ENTRIES];
   (* rom_style = "block" *) logic [LUT_WIDTH-1:0] lut_005[NUM_LUT_ENTRIES];
   (* rom_style = "block" *) logic [LUT_WIDTH-1:0] lut_006[NUM_LUT_ENTRIES];
   (* rom_style = "block" *) logic [LUT_WIDTH-1:0] lut_007[NUM_LUT_ENTRIES];
   (* rom_style = "block" *) logic [LUT_WIDTH-1:0] lut_008[NUM_LUT_ENTRIES];
   (* rom_style = "block" *) logic [LUT_WIDTH-1:0] lut_009[NUM_LUT_ENTRIES];
   (* rom_style = "block" *) logic [LUT_WIDTH-1:0] lut_010[NUM_LUT_ENTRIES];
   (* rom_style = "block" *) logic [LUT_WIDTH-1:0] lut_011[NUM_LUT_ENTRIES];
   (* rom_style = "block" *) logic [LUT_WIDTH-1:0] lut_012[NUM_LUT_ENTRIES];
   (* rom_style = "block" *) logic [LUT_WIDTH-1:0] lut_013[NUM_LUT_ENTRIES];
   (* rom_style = "block" *) logic [LUT_WIDTH-1:0] lut_014[NUM_LUT_ENTRIES];
   (* rom_style = "block" *) logic [LUT_WIDTH-1:0] lut_015[NUM_LUT_ENTRIES];
   (* rom_style = "block" *) logic [LUT_WIDTH-1:0] lut_016[NUM_LUT_ENTRIES];
   (* rom_style = "block" *) logic [LUT_WIDTH-1:0] lut_017[NUM_LUT_ENTRIES];
   (* rom_style = "block" *) logic [LUT_WIDTH-1:0] lut_018[NUM_LUT_ENTRIES];
   (* rom_style = "block" *) logic [LUT_WIDTH-1:0] lut_019[NUM_LUT_ENTRIES];
   (* rom_style = "block" *) logic [LUT_WIDTH-1:0] lut_020[NUM_LUT_ENTRIES];
   (* rom_style = "block" *) logic [LUT_WIDTH-1:0] lut_021[NUM_LUT_ENTRIES];
   (* rom_style = "block" *) logic [LUT_WIDTH-1:0] lut_022[NUM_LUT_ENTRIES];
   (* rom_style = "block" *) logic [LUT_WIDTH-1:0] lut_023[NUM_LUT_ENTRIES];
   (* rom_style = "block" *) logic [LUT_WIDTH-1:0] lut_024[NUM_LUT_ENTRIES];
   (* rom_style = "block" *) logic [LUT_WIDTH-1:0] lut_025[NUM_LUT_ENTRIES];
   (* rom_style = "block" *) logic [LUT_WIDTH-1:0] lut_026[NUM_LUT_ENTRIES];
   (* rom_style = "block" *) logic [LUT_WIDTH-1:0] lut_027[NUM_LUT_ENTRIES];
   (* rom_style = "block" *) logic [LUT_WIDTH-1:0] lut_028[NUM_LUT_ENTRIES];
   (* rom_style = "block" *) logic [LUT_WIDTH-1:0] lut_029[NUM_LUT_ENTRIES];
   (* rom_style = "block" *) logic [LUT_WIDTH-1:0] lut_030[NUM_LUT_ENTRIES];
   (* rom_style = "block" *) logic [LUT_WIDTH-1:0] lut_031[NUM_LUT_ENTRIES];
   (* rom_style = "block" *) logic [LUT_WIDTH-1:0] lut_032[NUM_LUT_ENTRIES];
   (* rom_style = "block" *) logic [LUT_WIDTH-1:0] lut_033[NUM_LUT_ENTRIES];
   (* rom_style = "block" *) logic [LUT_WIDTH-1:0] lut_034[NUM_LUT_ENTRIES];
   (* rom_style = "block" *) logic [LUT_WIDTH-1:0] lut_035[NUM_LUT_ENTRIES];

   initial begin
      $readmemh("reduction_lut_000.dat", lut_000);
      $readmemh("reduction_lut_001.dat", lut_001);
      $readmemh("reduction_lut_002.dat", lut_002);
      $readmemh("reduction_lut_003.dat", lut_003);
      $readmemh("reduction_lut_004.dat", lut_004);
      $readmemh("reduction_lut_005.dat", lut_005);
      $readmemh("reduction_lut_006.dat", lut_006);
      $readmemh("reduction_lut_007.dat", lut_007);
      $readmemh("reduction_lut_008.dat", lut_008);
      $readmemh("reduction_lut_009.dat", lut_009);
      $readmemh("reduction_lut_010.dat", lut_010);
      $readmemh("reduction_lut_011.dat", lut_011);
      $readmemh("reduction_lut_012.dat", lut_012);
      $readmemh("reduction_lut_013.dat", lut_013);
      $readmemh("reduction_lut_014.dat", lut_014);
      $readmemh("reduction_lut_015.dat", lut_015);
      $readmemh("reduction_lut_016.dat", lut_016);
      $readmemh("reduction_lut_017.dat", lut_017);
      $readmemh("reduction_lut_018.dat", lut_018);
      $readmemh("reduction_lut_019.dat", lut_019);
      $readmemh("reduction_lut_020.dat", lut_020);
      $readmemh("reduction_lut_021.dat", lut_021);
      $readmemh("reduction_lut_022.dat", lut_022);
      $readmemh("reduction_lut_023.dat", lut_023);
      $readmemh("reduction_lut_024.dat", lut_024);
      $readmemh("reduction_lut_025.dat", lut_025);
      $readmemh("reduction_lut_026.dat", lut_026);
      $readmemh("reduction_lut_027.dat", lut_027);
      $readmemh("reduction_lut_028.dat", lut_028);
      $readmemh("reduction_lut_029.dat", lut_029);
      $readmemh("reduction_lut_030.dat", lut_030);
      $readmemh("reduction_lut_031.dat", lut_031);
      $readmemh("reduction_lut_032.dat", lut_032);
      $readmemh("reduction_lut_033.dat", lut_033);
      $readmemh("reduction_lut_034.dat", lut_034);
      $readmemh("reduction_lut_035.dat", lut_035);
   end

   always_ff @(posedge clk) begin
      lut_read_data_bram[0] <= lut_000[lut_addr[0]];
      lut_read_data_bram[1] <= lut_001[lut_addr[1]];
      lut_read_data_bram[2] <= lut_002[lut_addr[2]];
      lut_read_data_bram[3] <= lut_003[lut_addr[3]];
      lut_read_data_bram[4] <= lut_004[lut_addr[4]];
      lut_read_data_bram[5] <= lut_005[lut_addr[5]];
      lut_read_data_bram[6] <= lut_006[lut_addr[6]];
      lut_read_data_bram[7] <= lut_007[lut_addr[7]];
      lut_read_data_bram[8] <= lut_008[lut_addr[8]];
      lut_read_data_bram[9] <= lut_009[lut_addr[9]];
      lut_read_data_bram[10] <= lut_010[lut_addr[10]];
      lut_read_data_bram[11] <= lut_011[lut_addr[11]];
      lut_read_data_bram[12] <= lut_012[lut_addr[12]];
      lut_read_data_bram[13] <= lut_013[lut_addr[13]];
      lut_read_data_bram[14] <= lut_014[lut_addr[14]];
      lut_read_data_bram[15] <= lut_015[lut_addr[15]];
      lut_read_data_bram[16] <= lut_016[lut_addr[16]];
      lut_read_data_bram[17] <= lut_017[lut_addr[17]];
      lut_read_data_bram[18] <= lut_018[lut_addr[18]];
      lut_read_data_bram[19] <= lut_019[lut_addr[19]];
      lut_read_data_bram[20] <= lut_020[lut_addr[20]];
      lut_read_data_bram[21] <= lut_021[lut_addr[21]];
      lut_read_data_bram[22] <= lut_022[lut_addr[22]];
      lut_read_data_bram[23] <= lut_023[lut_addr[23]];
      lut_read_data_bram[24] <= lut_024[lut_addr[24]];
      lut_read_data_bram[25] <= lut_025[lut_addr[25]];
      lut_read_data_bram[26] <= lut_026[lut_addr[26]];
      lut_read_data_bram[27] <= lut_027[lut_addr[27]];
      lut_read_data_bram[28] <= lut_028[lut_addr[28]];
      lut_read_data_bram[29] <= lut_029[lut_addr[29]];
      lut_read_data_bram[30] <= lut_030[lut_addr[30]];
      lut_read_data_bram[31] <= lut_031[lut_addr[31]];
      lut_read_data_bram[32] <= lut_032[lut_addr[32]];
      lut_read_data_bram[33] <= lut_033[lut_addr[33]];
      lut_read_data_bram[34] <= lut_034[lut_addr[34]];
      lut_read_data_bram[35] <= lut_035[lut_addr[35]];
   end


   // Read data out of the memories
   always_comb begin

      for (int k=0; k<NUM_BRAM; k=k+1) begin
         lut_read_data[k+NUM_URAM] = lut_read_data_bram[k];
      end      

   end

   always_comb begin
      for (int k=0; k<LUT_NUM_ELEMENTS; k=k+1) begin
         for (int l=NONREDUNDANT_ELEMENTS; l<NUM_ELEMENTS; l=l+1) begin
            lut_output[l][k] = '0;
         end
      end
      for (int k=0; k<LUT_NUM_ELEMENTS; k=k+1) begin
         for (int l=0; l<NONREDUNDANT_ELEMENTS+1; l=l+1) begin
            // TODO - should be unique, fails when in reset
            if (shift_high_1d) begin
               if (l < NONREDUNDANT_ELEMENTS) begin
                  lut_output[l][k][BIT_LEN-1:LOOK_UP_WIDTH] =
                     {{(BIT_LEN-WORD_LEN){1'b0}},
                      lut_read_data[k][(l*WORD_LEN)+:LOOK_UP_WIDTH]};
               end

               if (l == 0) begin
                  lut_output[l][k][LOOK_UP_WIDTH-1:0] = '0;
               end
               else begin
                  lut_output[l][k][LOOK_UP_WIDTH-1:0] =
                lut_read_data[k][((l-1)*WORD_LEN)+LOOK_UP_WIDTH+:LOOK_UP_WIDTH];
               end
            end
            else if (shift_overflow_1d) begin
               if (l == 0) begin
                  lut_output[l][k] = '0;
               end
               else begin
                  lut_output[l][k] =
                     {{(BIT_LEN-WORD_LEN){1'b0}},
                      lut_read_data[k][((l-1)*WORD_LEN)+:WORD_LEN]};
               end
            end
            else begin
               if (l < NONREDUNDANT_ELEMENTS) begin
                  lut_output[l][k] =
                     {{(BIT_LEN-WORD_LEN){1'b0}},
                      lut_read_data[k][(l*WORD_LEN)+:WORD_LEN]};
               end
            end
         end
      end
   end

   // Need above loops in combo block for Verilator to process
   always_comb begin
      lut_data = lut_output;
   end
endmodule
