#!/usr/bin/python3

from random import getrandbits


N = 124066695684124741398798927404814432744698427125735684128131855064976895337309138910015071214657674309443149407457493434579063840841220334555160125016331040933690674569571217337630239191517205721310197608387239846364360850220896772964978569683229449266819903414117058030106528073928633017118689826625594484331


###############################################
# Define these parameters
LOGNUMSYMBOLS = 5;
LOGRADIX = 33;
MODULUS = N;
###############################################

####################################################################################
# Specify Number to Modulo
#    Number is concatenation of (2 << LOGNUMSYMBOLS) symbols
#    Each symbol is a sign-bit followed by (1 + LOGRADIX) bits
#################################################################
#x = getrandbits((2 << LOGNUMSYMBOLS) * (LOGRADIX + 2))
x = 4;
####################################################################################





MODBITWIDTH = (LOGRADIX)*(1 << LOGNUMSYMBOLS);
print("MODBITWIDTH = ", MODBITWIDTH);
#print("MODULUS = ", hex(MODULUS));
print("x = ", hex(x));


# signsymbol has a single '1' in the bit location of the sign-bit
signsymbol = 1 << (LOGRADIX+1);
#print("signsymbol = ", hex(signsymbol));

# generate mask for all sign bit positions
ALLSIGNBITS = 0;
for i in range(0, 2 << LOGNUMSYMBOLS, 1):
	ALLSIGNBITS = (ALLSIGNBITS << LOGRADIX) + signsymbol;
print("ALLSIGNBITS = ", hex(ALLSIGNBITS));


adderterms0 = (MODULUS << (MODBITWIDTH * 3));
adderterms0 -= ALLSIGNBITS;
adderterms0 = adderterms0 % MODULUS;
print("adderterms0 = ", hex(adderterms0));

adderterms1 = 0;

adderterms2 = 0;
for i in range(1, 1 << LOGNUMSYMBOLS, 1):
	adderterms2 = (adderterms2 << LOGRADIX) + signsymbol;
print("adderterms2 = ", hex(adderterms2));

sumofadderterms = adderterms0 + adderterms1 + adderterms2;

for i in range(3, 250, 1):
	iter = (i-3)*6 + (2+LOGRADIX)*(1 << LOGNUMSYMBOLS) - 2;
	term = 0;
	for j in range(0, 6, 1):
		SYM = (iter+j)//(2+LOGRADIX);
		BIT = (iter+j)%(2+LOGRADIX);
		if (SYM < (2 << LOGNUMSYMBOLS)):
			MOD = (1 << ((SYM * LOGRADIX) + BIT)) % MODULUS;
			x_iter = (x >> (iter+j)) & 1;
			if (BIT == (1+LOGRADIX)):
				x_iter = 1 - x_iter;		# flip sign bit
			#print("i:", i, " iter:", iter + j, " SYM=", SYM, " BIT=", BIT, " x[iter]=", x_iter, " MOD=", hex(MOD));
			if (x_iter):
				term += MOD;
	sumofadderterms += term;
	print("adderterm[", i, "] = ", hex(term));
				
#print("sumofadderterms = ", hex(sumofadderterms));
print("sumofadderterms % MODULUS = ", hex(sumofadderterms % MODULUS));


if False: '''
// Place holder
genvar i;
generate
	// The first adderterm is the constant correction needed for all the negative symbols
    assign adderterms[0] = bigmod({ MODULUS, (ALLSIGNBITS ^ ALLSIGNBITS) } - ALLSIGNBITS, MODULUS);

	// The second adderterm is for all the symbols below the modulo-bitwidth (excluding the upper 2 bits per symbol)
	for (i = 0; i < (1 << LOGNUMSYMBOLS); i = i + 1) begin : secondterm
		assign adderterms[1][i * LOGRADIX +: LOGRADIX] = data_in[i][LOGRADIX-1:0];
	end

	// The third adderterm is for all the upper bits of each symbol below the modulo-bitwidth
	assign adderterms[2][0 +: LOGRADIX] = 0;
	for (i = 1; i < (1 << LOGNUMSYMBOLS); i = i + 1) begin : thirdterm
		assign adderterms[2][i * LOGRADIX +: LOGRADIX] = (data_in[i-1] ^ signsymbol) >> LOGRADIX;
	end

	// The remaining adderterms are for all the bits above the modulo-bitwidth
	for (i = 3; i < 200; i = i + 1) begin : modulolutterm
		
		localparam iter = (i-3)*6 + (2+LOGRADIX)*(1 << LOGNUMSYMBOLS) - 2;
		
		localparam SYM0 = (iter+0)/(2+LOGRADIX);
		localparam BIT0 = (iter+0)%(2+LOGRADIX);
		if (SYM0 < (2 << LOGNUMSYMBOLS)) begin
		
			localparam SYM1 = (iter+1)/(2+LOGRADIX);
			localparam BIT1 = (iter+1)%(2+LOGRADIX);
			localparam SYM2 = (iter+2)/(2+LOGRADIX);
			localparam BIT2 = (iter+2)%(2+LOGRADIX);
			localparam SYM3 = (iter+3)/(2+LOGRADIX);
			localparam BIT3 = (iter+3)%(2+LOGRADIX);
			localparam SYM4 = (iter+4)/(2+LOGRADIX);
			localparam BIT4 = (iter+4)%(2+LOGRADIX);
			localparam SYM5 = (iter+5)/(2+LOGRADIX);
			localparam BIT5 = (iter+5)%(2+LOGRADIX);
		
			logic x0, x1, x2, x3, x4, x5;
			
			assign x0 = (SYM0 < (2 << LOGNUMSYMBOLS) ? (data_in[SYM0] ^ signsymbol) : 0) >> BIT0;
			assign x1 = (SYM1 < (2 << LOGNUMSYMBOLS) ? (data_in[SYM1] ^ signsymbol) : 0) >> BIT1;
			assign x2 = (SYM2 < (2 << LOGNUMSYMBOLS) ? (data_in[SYM2] ^ signsymbol) : 0) >> BIT2;
			assign x3 = (SYM3 < (2 << LOGNUMSYMBOLS) ? (data_in[SYM3] ^ signsymbol) : 0) >> BIT3;
			assign x4 = (SYM4 < (2 << LOGNUMSYMBOLS) ? (data_in[SYM4] ^ signsymbol) : 0) >> BIT4;
			assign x5 = (SYM5 < (2 << LOGNUMSYMBOLS) ? (data_in[SYM5] ^ signsymbol) : 0) >> BIT5;
			
			localparam MOD0 = bigmod(4096'h1 << ((SYM0 * LOGRADIX) + BIT0), MODULUS);
			localparam MOD1 = bigmod(4096'h1 << ((SYM1 * LOGRADIX) + BIT1), MODULUS);
			localparam MOD2 = bigmod(4096'h1 << ((SYM2 * LOGRADIX) + BIT2), MODULUS);
			localparam MOD3 = bigmod(4096'h1 << ((SYM3 * LOGRADIX) + BIT3), MODULUS);
			localparam MOD4 = bigmod(4096'h1 << ((SYM4 * LOGRADIX) + BIT4), MODULUS);
			localparam MOD5 = bigmod(4096'h1 << ((SYM5 * LOGRADIX) + BIT5), MODULUS);
			
			modulolut6 #(
				.MODULUS		(MODULUS),
				.MODBITWIDTH	(MODBITWIDTH),
				.MOD0			(MOD0),
				.MOD1			(MOD1),
				.MOD2			(MOD2),
				.MOD3			(MOD3),
				.MOD4			(MOD4),
				.MOD5			(MOD5)
			) i_modulolut6 (
				.x0_in			(x0),
				.x1_in			(x1),
				.x2_in			(x2),
				.x3_in			(x3),
				.x4_in			(x4),	
				.x5_in			(x5),	
				.value_out		(adderterms[i])
			);
			
		end else begin
		
			assign adderterms[i] = 0;
		
		end
'''
