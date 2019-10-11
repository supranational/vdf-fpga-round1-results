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

module reduction_lut9
   #(
     parameter int REDUNDANT_ELEMENTS    = 2,
     parameter int NONREDUNDANT_ELEMENTS = 64,
     parameter int WORD_LEN              = 16,
     parameter int BIT_LEN               = 17,
     parameter int ADDR_LEN               = 9,

     parameter int NUM_ELEMENTS          = REDUNDANT_ELEMENTS+NONREDUNDANT_ELEMENTS,
     
     parameter int LOOK_UP_WIDTH         = int'(WORD_LEN / 2)
    )
   (
    input  logic                    clk,
    input  logic [ADDR_LEN-1:0]     lut9_addr[NUM_ELEMENTS],
    output logic [BIT_LEN-1:0]      lut9_data[NUM_ELEMENTS][NUM_ELEMENTS]
   );

   // There is twice as many entries due to low and high values
   localparam int NUM_LUT_ENTRIES   = 2**(LOOK_UP_WIDTH+2);
   localparam int LUT_WIDTH         = WORD_LEN * NONREDUNDANT_ELEMENTS;

   logic [LUT_WIDTH-1:0]  lut_read_data[NUM_ELEMENTS];

   //instruct Vivado to use blockRAM for the ROMs
   //Here we need to create NUM_ELEMENTS ROMs
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

   initial begin
      $readmemh("precompute_lut9_000.dat", lut_000);
      $readmemh("precompute_lut9_001.dat", lut_001);
      $readmemh("precompute_lut9_002.dat", lut_002);
      $readmemh("precompute_lut9_003.dat", lut_003);
      $readmemh("precompute_lut9_004.dat", lut_004);
      $readmemh("precompute_lut9_005.dat", lut_005);
      $readmemh("precompute_lut9_006.dat", lut_006);
      $readmemh("precompute_lut9_007.dat", lut_007);
      $readmemh("precompute_lut9_008.dat", lut_008);
      $readmemh("precompute_lut9_009.dat", lut_009);
      $readmemh("precompute_lut9_010.dat", lut_010);
      $readmemh("precompute_lut9_011.dat", lut_011);
      $readmemh("precompute_lut9_012.dat", lut_012);
      $readmemh("precompute_lut9_013.dat", lut_013);
      $readmemh("precompute_lut9_014.dat", lut_014);
      $readmemh("precompute_lut9_015.dat", lut_015);
      $readmemh("precompute_lut9_016.dat", lut_016);
      $readmemh("precompute_lut9_017.dat", lut_017);
      $readmemh("precompute_lut9_018.dat", lut_018);
      $readmemh("precompute_lut9_019.dat", lut_019);
      $readmemh("precompute_lut9_020.dat", lut_020);
      $readmemh("precompute_lut9_021.dat", lut_021);
      $readmemh("precompute_lut9_022.dat", lut_022);
      $readmemh("precompute_lut9_023.dat", lut_023);
      $readmemh("precompute_lut9_024.dat", lut_024);
      $readmemh("precompute_lut9_025.dat", lut_025);
      $readmemh("precompute_lut9_026.dat", lut_026);
      $readmemh("precompute_lut9_027.dat", lut_027);
      $readmemh("precompute_lut9_028.dat", lut_028);
      $readmemh("precompute_lut9_029.dat", lut_029);
      $readmemh("precompute_lut9_030.dat", lut_030);
      $readmemh("precompute_lut9_031.dat", lut_031);
      $readmemh("precompute_lut9_032.dat", lut_032);
   end



//Read from ROMs
   always_ff @(posedge clk) begin
      lut_read_data[0] <= lut_000[{1'b0,lut9_addr[0]}];
      lut_read_data[1] <= lut_000[{1'b1,lut9_addr[1]}];
      lut_read_data[2] <= lut_001[{1'b0,lut9_addr[2]}];
      lut_read_data[3] <= lut_001[{1'b1,lut9_addr[3]}];
      lut_read_data[4] <= lut_002[{1'b0,lut9_addr[4]}];
      lut_read_data[5] <= lut_002[{1'b1,lut9_addr[5]}];
      lut_read_data[6] <= lut_003[{1'b0,lut9_addr[6]}];
      lut_read_data[7] <= lut_003[{1'b1,lut9_addr[7]}];
      lut_read_data[8] <= lut_004[{1'b0,lut9_addr[8]}];
      lut_read_data[9] <= lut_004[{1'b1,lut9_addr[9]}];
      lut_read_data[10] <= lut_005[{1'b0,lut9_addr[10]}];
      lut_read_data[11] <= lut_005[{1'b1,lut9_addr[11]}];
      lut_read_data[12] <= lut_006[{1'b0,lut9_addr[12]}];
      lut_read_data[13] <= lut_006[{1'b1,lut9_addr[13]}];
      lut_read_data[14] <= lut_007[{1'b0,lut9_addr[14]}];
      lut_read_data[15] <= lut_007[{1'b1,lut9_addr[15]}];
      lut_read_data[16] <= lut_008[{1'b0,lut9_addr[16]}];
      lut_read_data[17] <= lut_008[{1'b1,lut9_addr[17]}];
      lut_read_data[18] <= lut_009[{1'b0,lut9_addr[18]}];
      lut_read_data[19] <= lut_009[{1'b1,lut9_addr[19]}];
      lut_read_data[20] <= lut_010[{1'b0,lut9_addr[20]}];
      lut_read_data[21] <= lut_010[{1'b1,lut9_addr[21]}];
      lut_read_data[22] <= lut_011[{1'b0,lut9_addr[22]}];
      lut_read_data[23] <= lut_011[{1'b1,lut9_addr[23]}];
      lut_read_data[24] <= lut_012[{1'b0,lut9_addr[24]}];
      lut_read_data[25] <= lut_012[{1'b1,lut9_addr[25]}];
      lut_read_data[26] <= lut_013[{1'b0,lut9_addr[26]}];
      lut_read_data[27] <= lut_013[{1'b1,lut9_addr[27]}];
      lut_read_data[28] <= lut_014[{1'b0,lut9_addr[28]}];
      lut_read_data[29] <= lut_014[{1'b1,lut9_addr[29]}];
      lut_read_data[30] <= lut_015[{1'b0,lut9_addr[30]}];
      lut_read_data[31] <= lut_015[{1'b1,lut9_addr[31]}];
      lut_read_data[32] <= lut_016[{1'b0,lut9_addr[32]}];
      lut_read_data[33] <= lut_016[{1'b1,lut9_addr[33]}];
      lut_read_data[34] <= lut_017[{1'b0,lut9_addr[34]}];
      lut_read_data[35] <= lut_017[{1'b1,lut9_addr[35]}];
      lut_read_data[36] <= lut_018[{1'b0,lut9_addr[36]}];
      lut_read_data[37] <= lut_018[{1'b1,lut9_addr[37]}];
      lut_read_data[38] <= lut_019[{1'b0,lut9_addr[38]}];
      lut_read_data[39] <= lut_019[{1'b1,lut9_addr[39]}];
      lut_read_data[40] <= lut_020[{1'b0,lut9_addr[40]}];
      lut_read_data[41] <= lut_020[{1'b1,lut9_addr[41]}];
      lut_read_data[42] <= lut_021[{1'b0,lut9_addr[42]}];
      lut_read_data[43] <= lut_021[{1'b1,lut9_addr[43]}];
      lut_read_data[44] <= lut_022[{1'b0,lut9_addr[44]}];
      lut_read_data[45] <= lut_022[{1'b1,lut9_addr[45]}];
      lut_read_data[46] <= lut_023[{1'b0,lut9_addr[46]}];
      lut_read_data[47] <= lut_023[{1'b1,lut9_addr[47]}];
      lut_read_data[48] <= lut_024[{1'b0,lut9_addr[48]}];
      lut_read_data[49] <= lut_024[{1'b1,lut9_addr[49]}];
      lut_read_data[50] <= lut_025[{1'b0,lut9_addr[50]}];
      lut_read_data[51] <= lut_025[{1'b1,lut9_addr[51]}];
      lut_read_data[52] <= lut_026[{1'b0,lut9_addr[52]}];
      lut_read_data[53] <= lut_026[{1'b1,lut9_addr[53]}];
      lut_read_data[54] <= lut_027[{1'b0,lut9_addr[54]}];
      lut_read_data[55] <= lut_027[{1'b1,lut9_addr[55]}];
      lut_read_data[56] <= lut_028[{1'b0,lut9_addr[56]}];
      lut_read_data[57] <= lut_028[{1'b1,lut9_addr[57]}];
      lut_read_data[58] <= lut_029[{1'b0,lut9_addr[58]}];
      lut_read_data[59] <= lut_029[{1'b1,lut9_addr[59]}];
      lut_read_data[60] <= lut_030[{1'b0,lut9_addr[60]}];
      lut_read_data[61] <= lut_030[{1'b1,lut9_addr[61]}];
      lut_read_data[62] <= lut_031[{1'b0,lut9_addr[62]}];
      lut_read_data[63] <= lut_031[{1'b1,lut9_addr[63]}];
      lut_read_data[64] <= lut_032[{1'b0,lut9_addr[64]}];
      lut_read_data[65] <= lut_032[{1'b1,lut9_addr[65]}];
   end

//let's make the data read redundant again (i.e. from WORD_LEN go to BIT_LEN)
//by adding a zero at the MSB position
   always_comb 
   begin
        for (int k=0; k < NUM_ELEMENTS; k=k+1)
        begin   
            for (int l = NONREDUNDANT_ELEMENTS; l < NUM_ELEMENTS; l=l+1) 
            begin
                lut9_data[l][k] = '0;
            end
        end
        for (int k = 0; k < NUM_ELEMENTS; k = k+2)
        begin
            for (int l = 0; l < NONREDUNDANT_ELEMENTS; l = l+1)
            begin
                lut9_data[k][l] = {{(BIT_LEN-WORD_LEN){1'b0}}, lut_read_data[k][(l*WORD_LEN)+:WORD_LEN]};
            end
        end
   end
   
  
endmodule
