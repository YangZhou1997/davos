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
    msgBody() {}
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
    
    sessionState() {}
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

const uint32_t PARSER_MAX_NUMBER_OF_ENTRIES = 1000;
#include <math.h>

//Copied from hlslib by Johannes de Fine Licht https://github.com/definelicht/hlslib/blob/master/include/hlslib/xilinx/Utility.h
constexpr unsigned long parser_ConstLog2(unsigned long val) {
  return val == 1 ? 0 : 1 + parser_ConstLog2(val >> 1);
}

const uint32_t PARSER_MAX_KEY_SIZE = 64;
const uint32_t PARSER_MAX_ADDRESS_BITS = 16;

const uint32_t PARSER_NUM_TABLES = 9;
const uint32_t PARSER_TABLE_ADDRESS_BITS = parser_ConstLog2(PARSER_MAX_NUMBER_OF_ENTRIES/(PARSER_NUM_TABLES-1));
const uint32_t PARSER_TABLE_SIZE = (1 << PARSER_TABLE_ADDRESS_BITS);

const uint32_t PARSER_KEY_SIZE = 16;
const uint32_t PARSER_MAX_TRIALS = 12;

//The hash table can easily support NUM_TABLES-1 * TABLE_SIZE
//for NUM_TABLES = 9 -> this equals to a load factor of 0.88


struct parser_htLookupReq
{
   ap_uint<PARSER_KEY_SIZE>  key;
   ap_uint<1>  source;
   parser_htLookupReq() {}
   parser_htLookupReq(ap_uint<PARSER_KEY_SIZE> key, ap_uint<1> source)
      :key(key), source(source) {}
};

struct parser_htLookupResp
{
   ap_uint<PARSER_KEY_SIZE>  key;
   sessionState value1;
   msgBody value2;
   bool        hit;
   ap_uint<1>  source;
};

struct parser_htUpdateReq
{
   ap_uint<PARSER_KEY_SIZE>  key;
   sessionState value1;
   msgBody value2;
   ap_uint<1>  source;
   parser_htUpdateReq() {}
   parser_htUpdateReq(ap_uint<PARSER_KEY_SIZE> key, sessionState value1, msgBody value2, ap_uint<1> source)
      :key(key), value1(value1), value2(value2), source(source) {}
};

struct parser_htUpdateResp
{
   ap_uint<PARSER_KEY_SIZE>  key;
   sessionState value1;
   msgBody value2;
   bool        success;
   ap_uint<1>  source;
};

struct parser_htEntry
{
   ap_uint<PARSER_KEY_SIZE>  key;
   sessionState value1;
   msgBody value2;
   bool        valid;
   parser_htEntry() {}
   parser_htEntry(ap_uint<PARSER_KEY_SIZE> key, sessionState value1, msgBody value2)
      :key(key), value1(value1), value2(value2), valid(true) {}
};

void parser_stateman(
    hls::stream<parser_htLookupReq>&       s_axis_lup_req,
    hls::stream<parser_htUpdateReq>&       s_axis_upd_req,
    hls::stream<parser_htLookupResp>&      m_axis_lup_rsp,
    hls::stream<parser_htUpdateResp>&      m_axis_upd_rsp, 
    hls::stream<ap_uint<16> >&      regInsertFailureCount);

void parser_stateman_top(
    hls::stream<parser_htLookupReq>&       s_axis_lup_req,
    hls::stream<parser_htUpdateReq>&       s_axis_upd_req,
    hls::stream<parser_htLookupResp>&      m_axis_lup_rsp,
    hls::stream<parser_htUpdateResp>&      m_axis_upd_rsp, 
    hls::stream<ap_uint<16> >&      regInsertFailureCount);

