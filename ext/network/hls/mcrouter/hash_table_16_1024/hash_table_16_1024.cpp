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
#include "hash_table_16_1024.hpp"

// using namespace hls;
namespace hash_table_16_1024 {

const ap_uint<MAX_ADDRESS_BITS> tabulation_table[NUM_TABLES][2][MAX_KEY_SIZE] = {
   #include "tabulation_table.txt"
};

static htEntry<KEY_SIZE, VALUE_SIZE> cuckooTables[NUM_TABLES][TABLE_SIZE];

void calculate_hashes(ap_uint<KEY_SIZE> key, ap_uint<TABLE_ADDRESS_BITS>   hashes[NUM_TABLES])
{
#pragma HLS INLINE
#pragma HLS ARRAY_PARTITION variable=hashes complete dim=1

   for (int i = 0; i < NUM_TABLES; i++)
   {
      #pragma HLS UNROLL

      ap_uint<TABLE_ADDRESS_BITS> hash = 0;
      for (int k = 0; k < KEY_SIZE; k++)
      {
         #pragma HLS UNROLL
         ap_uint<MAX_ADDRESS_BITS> randomValue = tabulation_table[i][key[k]][k];
         hash ^= randomValue(TABLE_ADDRESS_BITS-1, 0); 
      }
      hashes[i] = hash;
   }
}

template <int V, int RV>
void push_val(ap_uint<V>& ht_val, ap_uint<RV>& val){
#pragma HLS INLINE
    ap_uint<RV> idx = ht_val(RV-1, 0);
#ifndef __SYNTHESIS__
    if(idx >= V/RV){
        std::cout << "[ERROR] push_val overflow" << std::endl;
        return;
    }
#endif
    ht_val(V-idx*RV-1, V-idx*RV-RV) = val;
    ht_val(RV-1, 0) = idx+1;
}

template <int V, int RV>
void pop_val(ap_uint<V>& ht_val, ap_uint<RV>& val){
#pragma HLS INLINE
    ap_uint<RV> idx = ht_val(RV-1, 0);
#ifndef __SYNTHESIS__
    if(idx == 0){
        std::cout << "[ERROR] pop_val underflow" << std::endl;
        return;
    }
#endif
    val = ht_val(V-1, V-RV);
    ht_val <<= RV;
    ht_val(RV-1, 0) = idx-1;
}

template <int K, int V, int RV>
mqPushResp<K,RV> push(mqPushReq<K,RV> request, hls::stream<ap_uint<16> >& regInsertFailureCount)
{
#pragma HLS INLINE

   htEntry<K,V> currentEntries[NUM_TABLES];
   #pragma HLS ARRAY_PARTITION variable=currentEntries complete
   ap_uint<TABLE_ADDRESS_BITS> hashes[NUM_TABLES];
   mqPushResp<K,RV> response;
   response.key = request.key;
   response.value = request.value;
   response.hit = false;

   calculate_hashes(request.key, hashes);
   //Look for matching key
   for (int i = 0; i < NUM_TABLES; i++)
   {
      #pragma HLS UNROLL
      currentEntries[i] = cuckooTables[i][hashes[i]];
      if(currentEntries[i].valid && currentEntries[i].key == request.key)
      {
         std::cout << "push currentEntries[i].key = " << currentEntries[i].key << std::endl;
        //  currentEntries[i].value = request.value;
         push_val<V,RV>(currentEntries[i].value, request.value);
         response.hit = true;
      }
      cuckooTables[i][hashes[i]] = currentEntries[i];
   }

    // update succeeds, return 
    if(response.hit)
        return response;

     std::cout << "push request.key = " << request.key << std::endl;
    
    // update fails, do inserting
    static ap_uint<8> victimIdx = 0;
    static ap_uint<1> victimBit = 0;
    static uint16_t insertFailureCounter = 0;

    ap_uint<V> init_htval = 0;
    init_htval(RV-1, 0) = 1; // setup idx
    init_htval(V-1, V-RV) = request.value; // setup the first value
    htEntry<K,V> currentEntry(request.key, init_htval);
    victimIdx = 0;
    //Try multiple times
    insertLoop: for (int j = 0; j < MAX_TRIALS; j++)
    {
       std::cout << "key " << response.key << " tries " << j << std::endl;
        // saving the first hash calculation
        if(j != 0){
           calculate_hashes(currentEntry.key, hashes);
        }
        //Look for free slot
        int slot = -1;
        for (int i = 0; i < NUM_TABLES; i++)
        {
            #pragma HLS UNROLL
            currentEntries[i] = cuckooTables[i][hashes[i]];
            if (!currentEntries[i].valid)
            {
                slot = i;
            }
        }

        //If free slot
        if (slot != -1)
        {
            currentEntries[slot] = currentEntry;
            response.hit = true;
        }
        else
        {
            //Evict existing entry and try to re-insert
            int victimPos = (hashes[victimIdx] % (NUM_TABLES-1)) + victimBit;
            htEntry<K,V> victimEntry = currentEntries[victimPos];//cuckooTables[victimPos][hashes[victimPos]];
            currentEntries[victimPos] = currentEntry;
            currentEntry = victimEntry;
            victimIdx++;
            if (victimIdx == NUM_TABLES)
                victimIdx = 0;
        }
        //Write currentEntries back
        for (int i = 0; i < NUM_TABLES; i++)
        {
            #pragma HLS UNROLL
            cuckooTables[i][hashes[i]] = currentEntries[i];
        }

        victimBit++;
        if (response.hit)
            break;
    }//for
    if (!response.hit)
    {
        std::cout << "REACHED MAX TRIALS: " << request.key << " " << currentEntry.key << std::endl;
        insertFailureCounter++;
        regInsertFailureCount.write(insertFailureCounter);
    }
    return response;
}

template <int K, int V, int RV>
mqPopResp<K,RV> pop(mqPopReq<K> request)
{
#pragma HLS INLINE
   
   htEntry<K,V> currentEntries[NUM_TABLES];
   #pragma HLS ARRAY_PARTITION variable=currentEntries complete
   ap_uint<TABLE_ADDRESS_BITS> hashes[NUM_TABLES];
   mqPopResp<K,RV> response;
   response.key = request.key;
   response.success = false;
   
   calculate_hashes(request.key, hashes);
   //Look for matching key
   int slot = -1;
   for (int i = 0; i < NUM_TABLES; i++)
   {
      #pragma HLS UNROLL
      currentEntries[i] = cuckooTables[i][hashes[i]];
      if (currentEntries[i].valid && currentEntries[i].key == request.key)
      {
          std::cout << "lookup currentEntries[i].key = " << currentEntries[i].key << std::endl;
         slot = i;
      }
   }
   //Check if key found
   if (slot != -1)
   {
    //   response.value = currentEntries[slot].value;
      pop_val<V,RV>(currentEntries[slot].value, response.value);
      cuckooTables[slot][hashes[slot]].value = currentEntries[slot].value;
      response.success = true;
   }

   return response;
}

template <int K, int V, int RV>
void hash_table(hls::stream<mqPushReq<K,RV> >&    mq_push_req,
               hls::stream<mqPopReq<K> >&         mq_pop_req,
               hls::stream<mqPushResp<K,RV> >&    mq_push_rsp,
               hls::stream<mqPopResp<K,RV> >&     mq_pop_rsp,
               hls::stream<ap_uint<16> >&        regInsertFailureCount)
{
// #pragma HLS PIPELINE II=1
// #pragma HLS INLINE off
#pragma HLS INLINE

   //Global arrays
   #pragma HLS ARRAY_PARTITION variable=tabulation_table complete dim=1
   #pragma HLS RESOURCE variable=cuckooTables core=RAM_2P_BRAM
   #pragma HLS ARRAY_PARTITION variable=cuckooTables complete dim=1

   if (!mq_push_req.empty())
   {
      mqPushReq<K,RV> request = mq_push_req.read();
      mqPushResp<K,RV> response = push<K,V,RV>(request, regInsertFailureCount);
      mq_push_rsp.write(response);
   }
   else if (!mq_pop_req.empty())
   {
      mqPopReq<K> request = mq_pop_req.read();
      mqPopResp<K,RV> response = pop<K,V,RV>(request);
      mq_pop_rsp.write(response);
   }
}

void hash_table_top(hls::stream<mqPushReq<KEY_SIZE,RET_VALUE_SIZE> >&   mq_push_req,
               hls::stream<mqPopReq<KEY_SIZE> >&                        mq_pop_req,
               hls::stream<mqPushResp<KEY_SIZE,RET_VALUE_SIZE> >&       mq_push_rsp,
               hls::stream<mqPopResp<KEY_SIZE,RET_VALUE_SIZE> >&        mq_pop_rsp,
               hls::stream<ap_uint<16> >&                               regInsertFailureCount)
{
// #pragma HLS PIPELINE II=1
#pragma HLS INLINE

    hash_table<KEY_SIZE, VALUE_SIZE, RET_VALUE_SIZE>(mq_push_req, mq_pop_req, mq_push_rsp, mq_pop_rsp, regInsertFailureCount);
}

}