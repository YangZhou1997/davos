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

const ap_uint<PARSER_MAX_ADDRESS_BITS> parser_tabulation_table[PARSER_NUM_TABLES][2][PARSER_KEY_SIZE] = {
    #include "tabulation_table.txt"
};

static parser_htEntry parser_cuckooTables[PARSER_NUM_TABLES][PARSER_TABLE_SIZE];

void parser_calculate_hashes(ap_uint<PARSER_KEY_SIZE> key, ap_uint<PARSER_TABLE_ADDRESS_BITS>   hashes[PARSER_NUM_TABLES])
{
#pragma HLS ARRAY_PARTITION variable=hashes complete dim=1

    for (int i = 0; i < PARSER_NUM_TABLES; i++)
    {
        #pragma HLS UNROLL

        ap_uint<PARSER_TABLE_ADDRESS_BITS> hash = 0;
        for (int k = 0; k < PARSER_KEY_SIZE; k++)
        {
            #pragma HLS UNROLL
            ap_uint<PARSER_MAX_ADDRESS_BITS> randomValue = parser_tabulation_table[i][key[k]][k];
            hash ^= randomValue(PARSER_TABLE_ADDRESS_BITS-1, 0); 
        }
        hashes[i] = hash;
    }
}

parser_htLookupResp parser_lookup(parser_htLookupReq request)
{
#pragma HLS INLINE
   
    parser_htEntry currentEntries[PARSER_NUM_TABLES];
    #pragma HLS ARRAY_PARTITION variable=currentEntries complete
    ap_uint<PARSER_TABLE_ADDRESS_BITS> hashes[PARSER_NUM_TABLES];
    parser_htLookupResp response;
    response.key = request.key;
    response.source = request.source;
    response.hit = false;

    parser_calculate_hashes(request.key, hashes);
    //Look for matching key
    int slot = -1;
    for (int i = 0; i < PARSER_NUM_TABLES; i++)
    {
        #pragma HLS UNROLL
        currentEntries[i] = parser_cuckooTables[i][hashes[i]];
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

parser_htUpdateResp parser_update_insert(parser_htUpdateReq request,
                     hls::stream<ap_uint<16> >& regInsertFailureCount)
{
#pragma HLS INLINE

    parser_htEntry currentEntries[PARSER_NUM_TABLES];
    #pragma HLS ARRAY_PARTITION variable=currentEntries complete
    ap_uint<PARSER_TABLE_ADDRESS_BITS> hashes[PARSER_NUM_TABLES];
    parser_htUpdateResp response;
    response.key = request.key;
    response.value1 = request.value1;
    response.value2 = request.value2;
    response.source = request.source;
    response.success = false;

    parser_calculate_hashes(request.key, hashes);
    //Look for matching key
    for (int i = 0; i < PARSER_NUM_TABLES; i++)
    {
        #pragma HLS UNROLL
        currentEntries[i] = parser_cuckooTables[i][hashes[i]];
        if(currentEntries[i].valid && currentEntries[i].key == request.key)
        {
            std::cout << "update currentEntries[i].key = " << currentEntries[i].key << std::endl;
            currentEntries[i].value1 = request.value1;
            currentEntries[i].value2 = request.value2;
            response.success = true;
        }
        parser_cuckooTables[i][hashes[i]] = currentEntries[i];
    }
    // update succeeds, return 
    if(response.success)
        return response;

     std::cout << "update_insert request.key = " << request.key << std::endl;

    // update fails, do inserting
    static ap_uint<8> victimIdx = 0;
    static ap_uint<1> victimBit = 0;
    static uint16_t insertFailureCounter = 0;

    parser_htEntry currentEntry(request.key, request.value1, request.value2);
    victimIdx = 0;
    //Try multiple times
    insertLoop: for (int j = 0; j < PARSER_MAX_TRIALS; j++)
    {
        std::cout << "key " << response.key << " tries " << j << std::endl;
        // saving the first hash calculation
        if(j != 0){
           parser_calculate_hashes(currentEntry.key, hashes);
        }
        //Look for free slot
        int slot = -1;
        for (int i = 0; i < PARSER_NUM_TABLES; i++)
        {
            #pragma HLS UNROLL
            currentEntries[i] = parser_cuckooTables[i][hashes[i]];
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
            int victimPos = (hashes[victimIdx] % (PARSER_NUM_TABLES-1)) + victimBit;
            parser_htEntry victimEntry = currentEntries[victimPos];//parser_cuckooTables[victimPos][hashes[victimPos]];
            currentEntries[victimPos] = currentEntry;
            currentEntry = victimEntry;
            victimIdx++;
            if (victimIdx == PARSER_NUM_TABLES)
                victimIdx = 0;
        }
        //Write currentEntries back
        for (int i = 0; i < PARSER_NUM_TABLES; i++)
        {
            #pragma HLS UNROLL
            parser_cuckooTables[i][hashes[i]] = currentEntries[i];
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
    return response;
}

void parser_stateman(
    hls::stream<parser_htLookupReq>&       s_axis_lup_req,
    hls::stream<parser_htUpdateReq>&       s_axis_upd_req,
    hls::stream<parser_htLookupResp>&      m_axis_lup_rsp,
    hls::stream<parser_htUpdateResp>&      m_axis_upd_rsp, 
    hls::stream<ap_uint<16> >&      regInsertFailureCount
)
{
// #pragma HLS PIPELINE II=1
#pragma HLS INLINE off

//Global arrays
#pragma HLS ARRAY_PARTITION variable=parser_tabulation_table complete dim=1
#pragma HLS RESOURCE variable=parser_cuckooTables core=RAM_2P_BRAM
#pragma HLS ARRAY_PARTITION variable=parser_cuckooTables complete dim=1
    
    if (!s_axis_upd_req.empty())
    {
        parser_htUpdateReq request = s_axis_upd_req.read();
        parser_htUpdateResp response = parser_update_insert(request, regInsertFailureCount);
        m_axis_upd_rsp.write(response);
    }
    else if (!s_axis_lup_req.empty())
    {
        parser_htLookupReq request = s_axis_lup_req.read();
        parser_htLookupResp response = parser_lookup(request);
        m_axis_lup_rsp.write(response);
    }
}

void parser_stateman_top(
    hls::stream<parser_htLookupReq>&     s_axis_lup_req,
    hls::stream<parser_htUpdateReq>&     s_axis_upd_req,
    hls::stream<parser_htLookupResp>&    m_axis_lup_rsp,
    hls::stream<parser_htUpdateResp>&    m_axis_upd_rsp,
    hls::stream<ap_uint<16> >&    regInsertFailureCount
)
{
#pragma HLS INLINE

// #pragma HLS DATAFLOW disable_start_propagation
// #pragma HLS INTERFACE ap_ctrl_none port=return

// #pragma HLS INTERFACE axis register port=s_axis_lup_req
// #pragma HLS INTERFACE axis register port=s_axis_upd_req
// #pragma HLS INTERFACE axis register port=m_axis_lup_rsp
// #pragma HLS INTERFACE axis register port=m_axis_upd_rsp
// #pragma HLS INTERFACE axis register port=regInsertFailureCount
// #pragma HLS DATA_PACK variable=s_axis_lup_req
// #pragma HLS DATA_PACK variable=s_axis_upd_req
// #pragma HLS DATA_PACK variable=m_axis_lup_rsp
// #pragma HLS DATA_PACK variable=m_axis_upd_rsp

    parser_stateman(s_axis_lup_req, s_axis_upd_req, m_axis_lup_rsp, m_axis_upd_rsp, regInsertFailureCount);
}