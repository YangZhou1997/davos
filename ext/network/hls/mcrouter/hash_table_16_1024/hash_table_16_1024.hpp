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
// #include <hash_table_config.hpp>
namespace hash_table_16_1024 {

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
const uint32_t VALUE_SIZE = 1024;
const uint32_t RET_VALUE_SIZE = 32;
const uint32_t MAX_TRIALS = 12;

//The hash table can easily support NUM_TABLES-1 * TABLE_SIZE
//for NUM_TABLES = 9 -> this equals to a load factor of 0.88


template <int K, int RV>
struct mqPushReq
{
   ap_uint<K>  key;
   ap_uint<RV>  value;
   mqPushReq<K,RV>() {}
   mqPushReq<K,RV>(ap_uint<K> key, ap_uint<RV> value)
      :key(key), value(value){}
};

template <int K, int RV>
struct mqPushResp
{
   ap_uint<K>  key;
   ap_uint<RV>  value;
   bool        hit;
};

template <int K>
struct mqPopReq
{
   ap_uint<K>  key;
   mqPopReq<K>() {}
   mqPopReq<K>(ap_uint<K> key)
      :key(key){}
};

template <int K, int RV>
struct mqPopResp
{
   ap_uint<K>  key;
   ap_uint<RV>  value;
   bool        success;
};

template <int K, int V>
struct htEntry
{
   ap_uint<K>  key;
   ap_uint<V>  value;
   bool        valid;
   htEntry<K,V>() {}
   htEntry<K,V>(ap_uint<K> key, ap_uint<V> value)
      :key(key), value(value), valid(true) {}
};


template <int K, int V, int RV>
void hash_table(hls::stream<mqPushReq<K,RV> >&    mq_push_req,
               hls::stream<mqPopReq<K> >&         mq_pop_req,
               hls::stream<mqPushResp<K,RV> >&    mq_push_rsp,
               hls::stream<mqPopResp<K,RV> >&     mq_pop_rsp,
               hls::stream<ap_uint<16> >&         regInsertFailureCount);

void hash_table_top(hls::stream<mqPushReq<KEY_SIZE,RET_VALUE_SIZE> >&   mq_push_req,
               hls::stream<mqPopReq<KEY_SIZE> >&                        mq_pop_req,
               hls::stream<mqPushResp<KEY_SIZE,RET_VALUE_SIZE> >&       mq_push_rsp,
               hls::stream<mqPopResp<KEY_SIZE,RET_VALUE_SIZE> >&        mq_pop_rsp,
               hls::stream<ap_uint<16> >&                               regInsertFailureCount);

}
