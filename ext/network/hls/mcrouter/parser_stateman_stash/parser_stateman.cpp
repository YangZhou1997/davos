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
#include "parser_stateman.hpp"

// using namespace hls;
// namespace parser_stateman_np {

const ap_uint<MAX_ADDRESS_BITS> tabulation_table[NUM_TABLES][2][MAX_KEY_SIZE] = {
    #include "tabulation_table.txt"
};

static htEntry cuckooTables[NUM_TABLES][TABLE_SIZE];

void calculate_hashes(ap_uint<KEY_SIZE> key, ap_uint<TABLE_ADDRESS_BITS>   hashes[NUM_TABLES])
{
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

htLookupResp ht_lookup(htLookupReq request)
{
#pragma HLS INLINE
   
    htEntry currentEntries[NUM_TABLES];
    #pragma HLS ARRAY_PARTITION variable=currentEntries complete
    ap_uint<TABLE_ADDRESS_BITS> hashes[NUM_TABLES];
    htLookupResp response;
    response.key = request.key;
    response.source = request.source;
    response.hit = false;

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
        response.value1 = currentEntries[slot].value1;
        response.value2 = currentEntries[slot].value2;
        response.hit = true;
    }

    return response;
}

htUpdateResp ht_update(htUpdateReq request)
{
#pragma HLS INLINE
   
    htEntry currentEntries[NUM_TABLES];
    #pragma HLS ARRAY_PARTITION variable=currentEntries complete
    ap_uint<TABLE_ADDRESS_BITS> hashes[NUM_TABLES];
    htUpdateResp response;
    response.key = request.key;
    response.value1 = request.value1;
    response.value2 = request.value2;
    response.source = request.source;
    response.success = false;

    calculate_hashes(request.key, hashes);
    //Look for matching key
    for (int i = 0; i < NUM_TABLES; i++)
    {
        #pragma HLS UNROLL
        currentEntries[i] = cuckooTables[i][hashes[i]];
        if(currentEntries[i].valid && currentEntries[i].key == request.key)
        {
            std::cout << "update currentEntries[i].key = " << currentEntries[i].key << std::endl;
            currentEntries[i].value1 = request.value1;
            currentEntries[i].value2 = request.value2;
            response.success = true;
        }
        cuckooTables[i][hashes[i]] = currentEntries[i];
    }
    return response;
}

void ht_insert(
    hls::stream<htUpdateReq>& requestFifo_update, 
    hls::stream<htUpdateReq>& requestFifo, 
    hls::stream<htUpdateResp>& responseFifo, 
    hls::stream<ap_uint<16> >& regInsertFailureCount)
{
// #pragma HLS PIPELINE II=1
#pragma HLS INLINE off
// #pragma HLS INLINE
    if(!requestFifo_update.empty()){
        htUpdateReq request = requestFifo.read();
        // This is guaranteed to be existing. 
        ht_update(request);
    }
    else if(!requestFifo.empty()){
        htUpdateReq request = requestFifo.read();

        // do we need to make it static?? 
        htEntry currentEntries[NUM_TABLES];
        #pragma HLS ARRAY_PARTITION variable=currentEntries complete
        ap_uint<TABLE_ADDRESS_BITS> hashes[NUM_TABLES];
        htUpdateResp response;
        response.key = request.key;
        response.value1 = request.value1;
        response.value2 = request.value2;
        response.source = request.source;
        response.success = false;

        // update fails, do inserting
        static ap_uint<8> victimIdx = 0;
        static ap_uint<1> victimBit = 0;
        static uint16_t insertFailureCounter = 0;

        htEntry currentEntry(request.key, request.value1, request.value2);
        victimIdx = 0;
        //Try multiple times
        insertLoop: for (int j = 0; j < MAX_TRIALS; j++)
        {
            std::cout << "key " << response.key << " tries " << j << std::endl;
            // saving the first hash calculation
            calculate_hashes(currentEntry.key, hashes);
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
                response.success = true;
            }
            else
            {
                //Evict existing entry and try to re-insert
                int victimPos = (hashes[victimIdx] % (NUM_TABLES-1)) + victimBit;
                htEntry victimEntry = currentEntries[victimPos];//cuckooTables[victimPos][hashes[victimPos]];
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
            if (response.success)
                break;
        }//for
        if (!response.success)
        {
            std::cout << "REACHED MAX TRIALS: " << request.key << " " << currentEntry.key << std::endl;
            insertFailureCounter++;
            regInsertFailureCount.write(insertFailureCounter);
        }
        responseFifo.write(response);
    }
}

static stashEntry stashTable[STASH_SIZE];

bool stash_insert(htUpdateReq request){
#pragma HLS INLINE
    bool response = false;

    int slot = -1;
    for (int i = 0; i < STASH_SIZE; i++)
    {
        #pragma HLS UNROLL
        if(!stashTable[i].valid)
        {
            slot = i;
        }
    }
    std::cout << "stash_insert slot = " << slot << std::endl;
    if(slot != -1){
        response = true;
        stashTable[slot].request = request;
        stashTable[slot].valid = true;
    }
    return response;
}

htLookupResp stash_lookup(ap_uint<KEY_SIZE> key){
#pragma HLS INLINE
    htLookupResp response;
    response.key = key;
    response.source = 0;
    response.hit = false;

    int slot = -1;
    for (int i = 0; i < STASH_SIZE; i++)
    {
        #pragma HLS UNROLL
        if(stashTable[i].valid && stashTable[i].request.key == key)
        {
            std::cout << "stash_lookup i = " << i << std::endl;
            slot = i;
        }
    }
    if(slot != -1){
        response.value1 = stashTable[slot].request.value1;
        response.value2 = stashTable[slot].request.value2;
        response.hit = true;
    }
    return response;
}

bool stash_remove(ap_uint<KEY_SIZE> key){
#pragma HLS INLINE
    bool response = false;

    int slot = -1;
    for (int i = 0; i < STASH_SIZE; i++)
    {
        #pragma HLS UNROLL
        if(stashTable[i].valid && stashTable[i].request.key == key)
        {
            std::cout << "stash_remove i = " << i << std::endl;
            slot = i;
        }
    }
    if(slot != -1){
        stashTable[slot].valid = false;
        response = true;
    }
    return response;
}

void parser_stateman_running(
    hls::stream<htLookupReq>&       s_axis_lup_req,
    hls::stream<htUpdateReq>&       s_axis_upd_req,
    hls::stream<htLookupResp>&      m_axis_lup_rsp,
    hls::stream<htUpdateResp>&      m_axis_upd_rsp, 
    hls::stream<htUpdateReq>&       requestFifo_update, 
    hls::stream<htUpdateReq>&       requestFifo, 
    hls::stream<htUpdateResp>&      responseFifo
)
{
#pragma HLS PIPELINE II=1
#pragma HLS INLINE off
    if(!responseFifo.empty()){
        htUpdateResp response = responseFifo.read();
        // delete entry from the stash. 
        bool ret = stash_remove(response.key);
        if(!ret){
            std::cout << "[ERROR]: parser_stateman stash delete not found!!!" << std::endl;
        }
    }

    if (!s_axis_upd_req.empty())
    {
        std::cout << "update_insert" << std::endl;
        htUpdateReq request = s_axis_upd_req.read();
        htUpdateResp response;
        response.key = request.key;
        response.value1 = request.value1;
        response.value2 = request.value2;
        response.success = true;
        
        // try to update hash table in case this is the not a new session
        // !!! this breaks srsw in HLS
        // htUpdateResp response = ht_update(request);

        htLookupReq request2(request.key, 0);
        htLookupResp response2 = ht_lookup(request2);
        
        if(response2.hit){
            std::cout << "update_insert update succeeds" << std::endl;
            requestFifo_update.write(request);

            m_axis_upd_rsp.write(response);
        }
        else{
            std::cout << "update_insert update fails; calling ht_insert()" << std::endl;
            // first write request into a stash
            bool ret = stash_insert(request);
            if(!ret){
                std::cout << "[ERROR]: parser_stateman stash is full!!!" << std::endl;
            }
            // submit request to insert module to do real insert. 
            requestFifo.write(request);
            
            m_axis_upd_rsp.write(response);
        }
    }
    else if (!s_axis_lup_req.empty())
    {
        htLookupReq request = s_axis_lup_req.read();
        // first lookup the stash
        htLookupResp response = stash_lookup(request.key);
        // concurrently lookup the hash table
        htLookupResp response2 = ht_lookup(request);
        m_axis_lup_rsp.write(response.hit ? response : response2);
    }
}

void parser_stateman(
    hls::stream<htLookupReq>&     s_axis_lup_req,
    hls::stream<htUpdateReq>&     s_axis_upd_req,
    hls::stream<htLookupResp>&    m_axis_lup_rsp,
    hls::stream<htUpdateResp>&    m_axis_upd_rsp,
    hls::stream<ap_uint<16> >&    regInsertFailureCount
)
{
#pragma HLS DATAFLOW disable_start_propagation
#pragma HLS INTERFACE ap_ctrl_none port=return

#pragma HLS INTERFACE axis register port=s_axis_lup_req
#pragma HLS INTERFACE axis register port=s_axis_upd_req
#pragma HLS INTERFACE axis register port=m_axis_lup_rsp
#pragma HLS INTERFACE axis register port=m_axis_upd_rsp
#pragma HLS INTERFACE axis register port=regInsertFailureCount
#pragma HLS DATA_PACK variable=s_axis_lup_req
#pragma HLS DATA_PACK variable=s_axis_upd_req
#pragma HLS DATA_PACK variable=m_axis_lup_rsp
#pragma HLS DATA_PACK variable=m_axis_upd_rsp

    //Global arrays
    #pragma HLS ARRAY_PARTITION variable=tabulation_table complete dim=1

    //!!! guarantee read and write in the same cycle. 
    #pragma HLS RESOURCE variable=cuckooTables core=RAM_T2P_BRAM
    #pragma HLS ARRAY_PARTITION variable=cuckooTables complete dim=1

    // hls will infer it as registers
    #pragma HLS ARRAY_PARTITION variable=stashTable complete

    static hls::stream<htUpdateReq>       requestFifo;
    #pragma HLS stream variable=requestFifo depth=4
    #pragma HLS DATA_PACK variable=requestFifo
    
    static hls::stream<htUpdateReq>       requestFifo_update;
    #pragma HLS stream variable=requestFifo_update depth=4
    #pragma HLS DATA_PACK variable=requestFifo_update
    
    static hls::stream<htUpdateResp>       responseFifo;
    #pragma HLS stream variable=responseFifo depth=4
    #pragma HLS DATA_PACK variable=responseFifo

    ht_insert(requestFifo_update, requestFifo, responseFifo, regInsertFailureCount);
    parser_stateman_running(s_axis_lup_req, s_axis_upd_req, m_axis_lup_rsp, m_axis_upd_rsp, \
        requestFifo_update, requestFifo, responseFifo);
}
// }