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
#include "../mcrouter.hpp"

// #include <hash_table_config.hpp>
// namespace parser_stateman_np {

const uint32_t MAX_NUMBER_OF_ENTRIES = 1000;
#include <math.h>

//Copied from hlslib by Johannes de Fine Licht https://github.com/definelicht/hlslib/blob/master/include/hlslib/xilinx/Utility.h
constexpr unsigned long ConstLog2(unsigned long val) {
  return val == 1 ? 0 : 1 + ConstLog2(val >> 1);
}

const uint32_t MAX_KEY_SIZE = 64;
const uint32_t MAX_ADDRESS_BITS = 16;

const uint32_t NUM_TABLES = 9;
const uint32_t TABLE_ADDRESS_BITS = ConstLog2(MAX_NUMBER_OF_ENTRIES/(NUM_TABLES-1));
const uint32_t TABLE_SIZE = (1 << TABLE_ADDRESS_BITS);

const uint32_t KEY_SIZE = 16;
const uint32_t MAX_TRIALS = 12;

const uint32_t STASH_SIZE = 8;
//The hash table can easily support NUM_TABLES-1 * TABLE_SIZE
//for NUM_TABLES = 9 -> this equals to a load factor of 0.88


struct htLookupReq
{
   ap_uint<KEY_SIZE>  key;
   ap_uint<1>  source;
   htLookupReq() {}
   htLookupReq(ap_uint<KEY_SIZE> key, ap_uint<1> source)
      :key(key), source(source) {}
};

struct htLookupResp
{
   ap_uint<KEY_SIZE>  key;
   sessionState value1;
   msgBody value2;
   bool        hit;
   ap_uint<1>  source;
};

struct htUpdateReq
{
   ap_uint<KEY_SIZE>  key;
   sessionState value1;
   msgBody value2;
   ap_uint<1>  source;
   htUpdateReq() {}
   htUpdateReq(ap_uint<KEY_SIZE> key, sessionState value1, msgBody value2, ap_uint<1> source)
      :key(key), value1(value1), value2(value2), source(source) {}
};

struct htUpdateResp
{
   ap_uint<KEY_SIZE>  key;
   sessionState value1;
   msgBody value2;
   bool        success;
   ap_uint<1>  source;
};

struct htEntry
{
   ap_uint<KEY_SIZE>  key;
   sessionState value1;
   msgBody value2;
   bool        valid;
   htEntry() {}
   htEntry(ap_uint<KEY_SIZE> key, sessionState value1, msgBody value2)
      :key(key), value1(value1), value2(value2), valid(true) {}
};

struct stashEntry
{
    htUpdateReq request;
    bool        valid;
    stashEntry() {}
    stashEntry(htUpdateReq request, bool valid)
        : request(request), valid(valid) {}
};

void ht_insert(
    hls::stream<htUpdateReq>&   requestFifo, 
    hls::stream<htUpdateResp>&  responseFifo, 
    hls::stream<ap_uint<16> >&  regInsertFailureCount);

void parser_stateman_running(
    hls::stream<htLookupReq>&       s_axis_lup_req,
    hls::stream<htUpdateReq>&       s_axis_upd_req,
    hls::stream<htLookupResp>&      m_axis_lup_rsp,
    hls::stream<htUpdateResp>&      m_axis_upd_rsp, 
    hls::stream<htUpdateReq>&       requestFifo, 
    hls::stream<htUpdateResp>&      responseFifo);

void parser_stateman(
    hls::stream<htLookupReq>&     s_axis_lup_req,
    hls::stream<htUpdateReq>&     s_axis_upd_req,
    hls::stream<htLookupResp>&    m_axis_lup_rsp,
    hls::stream<htUpdateResp>&    m_axis_upd_rsp,
    hls::stream<ap_uint<16> >&    regInsertFailureCount);

// }
