/************************************************
Copyright (c) 2016, Xilinx, Inc.
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, 
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, 
this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, 
this list of conditions and the following disclaimer in the documentation 
and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors 
may be used to endorse or promote products derived from this software 
without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND 
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, 
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. 
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, 
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, 
PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) 
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, 
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, 
EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.// Copyright (c) 2015 Xilinx, Inc.
************************************************/
#include "fsmtest.hpp"

void set_input(hls::stream<ap_uint<64> >& input, hls::stream<ap_uint<16> >& input1){
#pragma HLS PIPELINE II=1
#pragma HLS INLINE off

    static ap_uint<1> inputState = 0;
    switch(inputState){
        case 0:{
            input.write(0xdeadbeefdeadbeef);
            input1.write(32);
            inputState = 1;
            break;
        }
        case 1: {
            input.write(0xbeefdeadbeefdead);
            input1.write(48);
            inputState = 0;
            break;
        }
    }
}

void read_output(hls::stream<ap_uint<16> >& output){
#pragma HLS PIPELINE II=1
#pragma HLS INLINE off

    if(!output.empty()){
        std::cout << "out = " << output.read() << std::endl;
    }
}

int main()
{
	static hls::stream<ap_uint<64> > input("input");
	static hls::stream<ap_uint<16> > input1("input1");
	static hls::stream<ap_uint<16> > output("output");


	int count = 0;
	while (count < 20)
	{
		fsmtest(input, input1, output);
        set_input(input, input1);
        read_output(output);
        
        count++;
	}
    return 0;
}