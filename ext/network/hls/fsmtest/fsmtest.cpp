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

void test(	
    hls::stream<ap_uint<64> >& input,
    hls::stream<ap_uint<16> >& input1,
	hls::stream<ap_uint<16> >& output)
{
#pragma HLS PIPELINE II=1
#pragma HLS INLINE off

// !!! 22 iteration latency and II=1
// the time between the first input and the first output requires 22 cycles
// after the first output, every two cycles there will be a output generated, 

    static ap_uint<64> globalIn = 0;
    static ap_uint<16> idx = 0;

	static ap_uint<1> fsmState = 0;
	switch (fsmState)
	{
	case 0:
        if(!input.empty() && !input1.empty()){
            globalIn = input.read();
            idx = input1.read();
            fsmState = 1;
        }
		break;
    case 1: 
        ap_uint<16> globalIn1 = globalIn(16, 0) / idx;
        ap_uint<16> tmp = globalIn1;
        output.write(tmp);
        fsmState = 0;
        break;
	}
}

#define NUM_WAITING_Q 4
#define BITS_WAITING_Q 2

static hls::stream<ap_uint<16> > sessionWaitingQueues[NUM_WAITING_Q];
void test_in(hls::stream<ap_uint<16> >& input)
{
#pragma HLS PIPELINE II=1
#pragma HLS INLINE off

    #pragma HLS stream variable=sessionWaitingQueues[0] depth=64
    #pragma HLS stream variable=sessionWaitingQueues[1] depth=64
    #pragma HLS stream variable=sessionWaitingQueues[2] depth=64
    #pragma HLS stream variable=sessionWaitingQueues[3] depth=64
    
    if(!input.empty()){
        ap_uint<16> currSessionID = input.read();
        int idx = currSessionID & (NUM_WAITING_Q-1);
        sessionWaitingQueues[idx].write(currSessionID);
    }
}

void test_out(hls::stream<ap_uint<16> >& output){
#pragma HLS PIPELINE II=1
#pragma HLS INLINE off
    static ap_uint<BITS_WAITING_Q> startPos = 0;
    static ap_uint<4> fsmState = 0;
    static uint32_t slot = 0;
    
    switch(fsmState){
        case 0:{
            ap_uint<NUM_WAITING_Q> vlds = 0;
            for(int i = 0; i < NUM_WAITING_Q; i++){
                #pragma HLS UNROLL
                vlds[i] = (!sessionWaitingQueues[i].empty());
            }
            std::cout << "vlds = " << vlds << std::endl;

            if(vlds == 0){
                std::cout << "admission_control2: no valid fifo" << std::endl;
            }
            else{
                ap_uint<NUM_WAITING_Q*2> vldsDual = (vlds, vlds);
                vldsDual >>= startPos;
                ap_uint<NUM_WAITING_Q> vldsSingle = vldsDual;
                // std::cout << "vldsSingle = " << vldsSingle << std::endl;

                slot = __builtin_ctz(vldsSingle);
                if(slot < startPos){
                    slot = NUM_WAITING_Q - (startPos - slot);
                }
                else{
                    slot = slot - startPos;
                }
                // std::cout << "fsmState = " << fsmState << std::endl;
                // std::cout << "startPos = " << startPos << std::endl;
                // std::cout << "slot = " << slot << std::endl;
                fsmState = 1;
            }
            startPos += 1;
            break;
        }
        case 1:{
            std::cout << "-------------slot = " << slot << std::endl;
            ap_uint<16> currSessionID = sessionWaitingQueues[slot].read();
            output.write(currSessionID);
            fsmState = 0;
            break;
        }
    }
}

void test1(	
    hls::stream<ap_uint<16> >& input,
	hls::stream<ap_uint<16> >& output)
{
#pragma HLS PIPELINE II=1
#pragma HLS INLINE off

    static ap_uint<16> globalIn = 0;

    if(!input.empty()) {
        globalIn = input.read();
        globalIn += 1;
    }
    output.write(globalIn);
}

void test2(
    hls::stream<ap_uint<16> >& input,
	hls::stream<ap_uint<16> >& output
){
#pragma HLS PIPELINE II=1
#pragma HLS INLINE off

    static ap_uint<16> globalIn = 0;
    if(!input.empty()) {
        globalIn = input.read();
        output.write(globalIn);
    }
}

// void fsmtest(hls::stream<ap_uint<64> >& input, hls::stream<ap_uint<16> >& input1, hls::stream<ap_uint<16> >& output)
// {
// 	#pragma HLS DATAFLOW disable_start_propagation
// 	#pragma HLS INTERFACE ap_ctrl_none port=return

// #pragma HLS INTERFACE axis register port=input name=s_axis_input_port
// #pragma HLS INTERFACE axis register port=output name=m_axis_output_port

//     // static hls::stream<ap_uint<16> > tmpInput;
//     // #pragma HLS stream variable=tmpInput depth=64

//     // test1(input, tmpInput);
//     // test2(tmpInput, output);

//     test(input, input1, output);
// }

void fsmtest(hls::stream<ap_uint<16> >& input, hls::stream<ap_uint<16> >& output)
{
	#pragma HLS DATAFLOW disable_start_propagation
	#pragma HLS INTERFACE ap_ctrl_none port=return

#pragma HLS INTERFACE axis register port=input name=s_axis_input_port
#pragma HLS INTERFACE axis register port=output name=m_axis_output_port

    test_in(input);
    test_out(output);

}
