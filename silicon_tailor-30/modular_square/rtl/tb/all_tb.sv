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

`timescale  1 ns / 1 ns


`ifndef	MODULUS_DEF
 `define	MODULUS_DEF	1024'd124066695684124741398798927404814432744698427125735684128131855064976895337309138910015071214657674309443149407457493434579063840841220334555160125016331040933690674569571217337630239191517205721310197608387239846364360850220896772964978569683229449266819903414117058030106528073928633017118689826625594484331
`endif



module all_tb 
(
);

     
import vdfpackage::bigmod;


//squarer_tb #( .NUMBITS(18) ) i_squarer18_tb ( .result() );
//squarer_tb #( .NUMBITS(34) ) i_squarer34_tb ( .result() );
//squarer_tb #( .NUMBITS(42) ) i_squarer42_tb ( .result() );


//uconvert_tb #( .LOGNUMSYMBOLS(2), .LOGRADIX (8) ) i_uconvert_tb ( .result() );
//sconvert_tb #( .LOGNUMSYMBOLS(2), .LOGRADIX (8) ) i_sconvert_tb ( .result() );

//unsignedcarrycorrection_tb #( .LOGNUMSYMBOLS(2), .LOGRADIX (8) ) i_unsignedcarrycorrection_tb ( .result() );


//addertree_tb #( .NUMBITS(4) ) i_addertree_tb ( .result() );

//multisymbolsquarer_tb #( .LOGNUMSYMBOLS(0), .LOGRADIX(33) ) i_multisymbolsquarerA_tb ( .result() );
//multisymbolsquarer_tb #( .LOGNUMSYMBOLS(1), .LOGRADIX(17) ) i_multisymbolsquarerB_tb ( .result() );
//multisymbolsquarer_tb #( .LOGNUMSYMBOLS(1), .LOGRADIX(33) ) i_multisymbolsquarerC_tb ( .result() );
multisymbolsquarer_tb #( .LOGNUMSYMBOLS(5), .LOGRADIX(33) ) i_multisymbolsquarerD_tb ( .result() );


//modulo_tb #( .LOGNUMSYMBOLS(0), .LOGRADIX(4), .MODULUS(13) ) i_modulo_tb ( .result() );
//modulo_tb #( .LOGNUMSYMBOLS(1), .LOGRADIX(4), .MODULUS(131) ) i_modulo_tb ( .result() );
//modulo_tb #( .LOGNUMSYMBOLS(2), .LOGRADIX(4), .MODULUS(32771) ) i_modulo_tb ( .result() );
//modulo_tb #( .LOGNUMSYMBOLS(5), .LOGRADIX(33), .MODULUS(`MODULUS_DEF) ) i_modulo_tb ( .result() );

logic [31:0] testval = 32'h7fe0100;
logic [16:0] testmod = 16'h8003;
logic [63:0] testresult;

assign testresult = bigmod({ testmod, 32'h0 } - testval, testmod);



endmodule
