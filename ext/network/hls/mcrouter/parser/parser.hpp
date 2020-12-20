/************************************************
Copyright (c) 2019, Systems Group, ETH Zurich.
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
EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
************************************************/
#pragma once
#include "../../axi_utils.hpp"

#define DATA_WIDTH 512

struct msgHeader {
    ap_uint<8> magic;
    ap_uint<8> opcode;
    ap_uint<16> keyLen;
    ap_uint<8> extLen;
    ap_uint<8> dataType;
    ap_uint<16> status;
    ap_uint<32> bodyLen;
    ap_uint<32> opaque;
    ap_uint<64> cas;
    msgHeader() {
        magic = 0;
        opcode = 0;
        keyLen = 0;
        extLen = 0;
        dataType = 0;
        status = 0;
        bodyLen = 0;
        opaque = 0;
        cas = 0;
    }
    void consume_word(ap_uint<24*8>& w){
        magic = w(191, 184);
        opcode = w(183, 176);
        keyLen = w(175, 160);
        extLen = w(159, 152);
        dataType = w(151, 144);
        status = w(143, 128);
        bodyLen = w(127, 96);
        opaque = w(95, 64);
        cas = w(63, 0);
    }
    ap_uint<24*8> output_word(){
        return (magic, opcode, keyLen, extLen, dataType, status, bodyLen, opaque, cas);
    }
    ap_uint<32> val_len(){
        return bodyLen-extLen-keyLen;
    }
    void display(){
        std::cout << "magic " << std::hex << magic << std::endl;
        std::cout << "opcode " << std::hex << opcode << std::endl;
        std::cout << "keyLen " << std::dec << keyLen << std::endl;
        std::cout << "extlen " << std::dec << extLen << std::endl;
        std::cout << "dataType " << std::dec << dataType << std::endl;
        std::cout << "status " << std::dec << status << std::endl;
        std::cout << "bodyLen " << std::dec << bodyLen << std::endl;
        std::cout << "opaque " << std::hex << opaque << std::endl;
        std::cout << "cas " << std::dec << cas << std::endl;
    }
};

#define MAX_BODY_LEN (1024-35-48*3-5-8) // 832
#define MAX_KEY_LEN (40*8)
struct msgBody {
	ap_uint<32>                 msgID;  // globally unique msgID
    ap_uint<1>                  extInl; // indicating if extra data is stored inline.
    ap_uint<1>                  keyInl; // indicating if key is stored inline. 
    ap_uint<1>                  valInl; // indicating if val is stored inline.
    ap_uint<48>                 extPtr; // ptr to the memory region managed by slab memory allocator
    ap_uint<48>			        keyPtr; // ptr to the memory region managed by slab memory allocator
    ap_uint<48>                 valPtr; // ptr to the memory region managed by slab memory allocator
    ap_uint<5>                  reserved;
    ap_uint<8>                  reserved2;
    ap_uint<MAX_BODY_LEN>       body;
    msgBody() {
        msgID = 0;
        extInl = 0;
        keyInl = 0;
        valInl = 0;
        extPtr = 0;
        keyPtr = 0;
        valPtr = 0;
        reserved = 0;
        reserved2 = 0;
        body = 0;
    }
    void consume_word(ap_uint<1024>& w){
        msgID = w(1023, 992);
        extInl = w(991, 991);
        keyInl = w(990, 990);
        valInl = w(989, 989);
        extPtr = w(988, 941);
        keyPtr = w(940, 893);
        valPtr = w(892, 845);
        reserved = w(844, 840);
        reserved2 = w(839, 832);
        body = w(831, 0);
    }
    ap_uint<1024> output_word(){
        return (msgID, extInl, keyInl, valInl, extPtr, keyPtr, valPtr, reserved, reserved2, body);
    }
    void reset(){
        msgID = 0;
        extInl = 0;
        keyInl = 0;
        valInl = 0;
        extPtr = 0;
        keyPtr = 0;
        valPtr = 0;
        reserved = 0;
        reserved2 = 0;
        body = 0;
    }

    void display(uint32_t extlen, uint32_t keylen, uint32_t vallen){
        uint32_t pos = 0;

        std::cout << "ext: ";
        for(int i = 0; i < extlen; i++){
            std::cout << char(body(MAX_BODY_LEN-pos-1, MAX_BODY_LEN-pos-8));
            pos += 8;
        }
        std::cout << std::endl << "key: ";
        for(int i = 0; i < keylen; i++){
            std::cout << char(body(MAX_BODY_LEN-pos-1, MAX_BODY_LEN-pos-8));
            pos += 8;
        }
        std::cout << std::endl << "val: ";
        for(int i = 0; i < vallen; i++){
            std::cout << char(body(MAX_BODY_LEN-pos-1, MAX_BODY_LEN-pos-8));
            pos += 8;
        }
        std::cout << std::endl;
    }
};

#define MEMCACHED_HDRLEN 24 // bytes
struct sessionState {
    ap_uint<MEMCACHED_HDRLEN*8>       msgHeaderBuff; // assembling buffer for memcached message header 
    
    ap_uint<1>          parsingHeaderState; // indicate whether parsing header is done
    ap_uint<1>          parsingBodyState; // indicating whether parsing body is done. 

    ap_uint<32>         requiredLen; // required length of header+body 
    ap_uint<8>			currHdrLen;
    ap_uint<32>			currBodyLen;
    
    sessionState() {
        msgHeaderBuff = 0;
        parsingHeaderState = 0;
        parsingBodyState = 0;
        requiredLen = 0;
        currHdrLen = 0;
        currBodyLen = 0;
    }
    void reset() {
        // msgHeaderBuff = 0;
        
        parsingHeaderState = 0;
        parsingBodyState = 0;

        requiredLen = 0;
        currHdrLen = 0;
        currBodyLen = 0;
    }

    void display(){
        std::cout << "parsingHeaderState " << std::dec << parsingHeaderState << std::endl;
        std::cout << "parsingBodyState " << std::dec << parsingBodyState << std::endl;
        std::cout << "requiredLen " << std::dec << requiredLen << std::endl;
        std::cout << "currHdrLen " << std::dec << currHdrLen << std::endl;
        std::cout << "currBodyLen " << std::dec << currBodyLen << std::endl;
    }
};

void parser(
    hls::stream<net_axis<DATA_WIDTH> >&     currWordFifo,
    hls::stream<sessionState>&              currSessionStateFifo,
    hls::stream<msgBody>&                   currMsgBodyFifo,
    hls::stream<sessionState>&              currSessionStateOutFifo,
    hls::stream<msgBody>&                   currMsgBodyStateOutFifo,
    hls::stream<msgHeader>&                 msgHeaderOutFifo,
    hls::stream<msgBody>&                   msgBodyOutFifo
    );