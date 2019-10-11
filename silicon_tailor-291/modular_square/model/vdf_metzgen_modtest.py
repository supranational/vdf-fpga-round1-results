#!/usr/bin/python3

from random import getrandbits

# Competition is for 1024 bits
NUM_BITS       = 1024

NUM_ITERATIONS = 1000

# Rather than being random each time, we will provide randomly generated values
x = getrandbits(NUM_BITS)
N = 124066695684124741398798927404814432744698427125735684128131855064976895337309138910015071214657674309443149407457493434579063840841220334555160125016331040933690674569571217337630239191517205721310197608387239846364360850220896772964978569683229449266819903414117058030106528073928633017118689826625594484331


# Dump all 2^i mod M values
for i in range(1020, 2048, 1):
	print(i, " 0x", (hex(pow(2, i) % N))[2:].zfill(256), sep='')


# t should be small for testing purposes.  
# For the final FPGA runs, t will be 2^30
t = NUM_ITERATIONS

# Iterative modular squaring t times
# This is the function that needs to be optimized on FPGA
for _ in range(t):
   x = (x * x) % N

# Final result is a 1024b value
h = x
print(h)


###############################################
# Define these parameters
LOGNUMSYMBOLS = 5;
LOGRADIX = 33;
MODULUS = N;
###############################################


MODBITWIDTH = (LOGRADIX)*(1 << LOGNUMSYMBOLS);
print("MODBITWIDTH = ", MODBITWIDTH);

# signsymbol has a single '1' in the bit location of the sign-bit
signsymbol = 1 << (LOGRADIX+1);
print("signsymbol = ", hex(signsymbol));

# generate mask for all sign bit positions
ALLSIGNBITS = 0;
for i in range(0, 2 << LOGNUMSYMBOLS, 1):
	ALLSIGNBITS = (ALLSIGNBITS << LOGRADIX) + 2;
ALLSIGNBITS = (ALLSIGNBITS << LOGRADIX);
print("ALLSIGNBITS = ", hex(ALLSIGNBITS));

print("MODULUS = ", hex(MODULUS));


adderterms0 = (MODULUS << (MODBITWIDTH * 3));
adderterms0 -= ALLSIGNBITS;
adderterms0 = adderterms0 % MODULUS;
print("adderterms0 = ", hex(adderterms0));

adderterms1 = 0;

adderterms2 = 0;
for i in range(1, 1 << LOGNUMSYMBOLS, 1):
	adderterms2 = (adderterms2 << LOGRADIX) + signsymbol;
print("adderterms2 = ", hex(adderterms2));

adderterms = adderterms0 + adderterms1 + adderterms2;

for i in range(3, 250, 1):
	iter = (i-3)*6 + (2+LOGRADIX)*(1 << LOGNUMSYMBOLS) - 2;
	for j in range(0, 6, 1):
		SYM = (iter+j)//(2+LOGRADIX);
		BIT = (iter+j)%(2+LOGRADIX);
		if (SYM < (2 << LOGNUMSYMBOLS)):
			if (BIT == (1+LOGRADIX)):
				MOD = (1 << ((SYM * LOGRADIX) + BIT)) % MODULUS;
				print("i:", i, " SYM=", SYM, " BIT=", BIT, " MOD=", hex(MOD));
				adderterms += MOD;
				
				
print("adderterms = ", hex(adderterms));
print("adderterms'= ", hex(adderterms % MODULUS));
	
	
print("adderterms0 = ", hex(adderterms0));


dbgTERM0 = 0x0000000000000000000000008206692d70993c64d7ba8fd6e8a05758e96db6b6b1c7b5df725ec9a361381680b37b0235bc48f2ea381c4b3dea47114dc9364b14b78f85e034ffdf50a97cd3965b3ff938d852ecc24887dc74bff9058107fe18ccd2e7e35fb2746c528f90552e5476df920f1cc5a138a7bb81ed33adbcfcff2aba640a0797b9f692e302595df2;
dbgTERM1 = 0x0000000000000001615a8aab83dc6991960aefae20b48ea2e2ec0661aaef40eefbb972ab66055a100690faf1944cfd1d3ebc7e8dc6bc2194c8344fcc4565a097613e7ebc75a4e962e89e695d5f21fa8a2534054531b78861e809f331190c5a216b1923f5745545245e0f24536191f11b0d7fcdbdefa04c529db3bdc4a096ba608cdef60911c4ccd600000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000;
dbgTERM2 = 0x0000000000000001615a8aab83dc6991960aefae20b48ea2e2ec0661aaef40eefbb972ab66055a100690faf1944cfd1d3ebc7e8dc6bc2194c8344fcc4565a097613e7ebc75a4e962e89e695d5f21fa8a2534054531b78861e809f331190c5a216b1923f5745545245e0f24536191f11b0d7fcdbdefa04c529db3bdc4a096ba608cdef60911c4ccd5fffffffdfffffffeffffffff7fffffffbfffffffdfffffffeffffffff7fffffffbfffffffdfffffffeffffffff7fffffffbfffffffdfffffffeffffffff7fffffffbfffffffdfffffffeffffffff7fffffffbfffffffdfffffffeffffffff7fffffffbfffffffdfffffffeffffffff7fffffffbfffffffdfffffffeffffffff7fffffffbfffffffdfffffffeffffffff7fffffffbfffffffdfffffffeffffffff7fffffffbfffffffdfffffffeffffffff7fffffffbfffffffdfffffffeffffffff7fffffffbfffffffdfffffffeffffffff7fffffffbfffffffdfffffffeffffffff7fffffffbfffffffdfffffffeffffffff7fffffffbfffffffdfffffffeffffffff7fffffffc00000000;

print("dbgTERM0 = ", hex(dbgTERM0 % MODULUS));
print("dbgTERM1 = ", hex(dbgTERM1 % MODULUS));
print("dbgTERM2 = ", hex(dbgTERM2 % MODULUS));


print("MODULUS[1023:0]=", hex(MODULUS & ((1 << 1024)-1)));


def bigmodR(_x, m, _twopowerimodm):
	term = 0;
	x = _x;
	twopowerimodm = _twopowerimodm;
	while (x != 0):
		if (x & 1):
			term = term + twopowerimodm;
			if (term >= m):
				term = term - m;
		x = x >> 1;
		twopowerimodm = twopowerimodm << 1;
		if (twopowerimodm >= m):
			twopowerimodm = twopowerimodm - m;
	return term;
	

print("dbgTERM2'= ", hex(bigmodR(dbgTERM2, MODULUS, 1)));



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
