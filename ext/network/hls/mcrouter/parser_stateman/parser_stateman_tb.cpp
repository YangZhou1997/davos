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
#include <map>
#include <vector>
#include <stdlib.h>

using namespace std;
using namespace hls;
// using namespace parser_stateman_np;

int main()
{
    srand(0xdeafbeaf);

    hls::stream<parser_htLookupReq>    s_axis_lup_req;
    hls::stream<parser_htUpdateReq>    s_axis_upd_req;
    hls::stream<parser_htLookupResp>   m_axis_lup_rsp;
    hls::stream<parser_htUpdateResp>   m_axis_upd_rsp;
    hls::stream<ap_uint<16> >   regInsertFailureCount;

    std::map<uint16_t, uint32_t> expectedKV;
    std::vector<uint16_t> keys;
    uint32_t numElements = (PARSER_NUM_TABLES-1) * PARSER_TABLE_SIZE;
    std::cout << "Testing with " << numElements << ", total number of entries: " << PARSER_NUM_TABLES * PARSER_TABLE_SIZE << ", load factor: " << (double) (PARSER_NUM_TABLES-1) / (double) PARSER_NUM_TABLES << std::endl;

    //insert elements
    int i =0;
    while (i < numElements)
    {
        uint16_t newKey = rand() & 0xFFFF;
        uint32_t newValue = i;

        //Check if key unique
        if (expectedKV.find(newKey) == expectedKV.end())
        {           
            expectedKV[newKey] = newValue;
            keys.push_back(newKey);

            sessionState value1;
            value1.currBodyLen = newValue;
            msgBody value2;
            value2.msgID = newValue;
            s_axis_upd_req.write(parser_htUpdateReq(newKey, value1, value2, 0));
            std::cout << "insert key " << newKey << " " << newValue << std::endl;

            i++;
        }
    }

    bool success = true;
    int count = 0;
    //Execute Inserts
    int successCounter = 0;
    while (count < numElements+100)
    {
        parser_stateman(s_axis_lup_req,
                    s_axis_upd_req,
                    m_axis_lup_rsp,
                    m_axis_upd_rsp,
                    regInsertFailureCount);

        if (!m_axis_upd_rsp.empty())
        {
            parser_htUpdateResp response = m_axis_upd_rsp.read();
            if (!response.success)
            {
                success = false;
                std::cerr << "[ERROR] insert failed" << std::endl;
            }
            else
            {
                successCounter++;
            }
        }
        if(!regInsertFailureCount.empty()){
            uint16_t tmp = regInsertFailureCount.read();
            std::cout << "[ERROR] regInsertFailureCount: " << tmp << std::endl;
        }
        count++;
    }
    if (successCounter != numElements)
    {
        success = false;
        std::cerr << "[ERROR] not all elements inserted successfully, expected: " << numElements << ", received: " << successCounter << std::endl;
    }


    //Issue lookups
    std::map<uint16_t, uint32_t>::const_iterator it;
    for(it = expectedKV.begin(); it != expectedKV.end(); ++it)
    {
        uint64_t key = it->first;
        s_axis_lup_req.write(parser_htLookupReq(key, 0));
    }

    //Execute lookups
    count = 0;
    int lookupCounter = 0;
    it = expectedKV.begin();
    while (count < numElements+100)
    {
        parser_stateman(s_axis_lup_req,
                    s_axis_upd_req,
                    m_axis_lup_rsp,
                    m_axis_upd_rsp,
                    regInsertFailureCount);
        
        if (!m_axis_lup_rsp.empty())
        {
            parser_htLookupResp response = m_axis_lup_rsp.read();
            if (!response.hit)
            {
                success = false;
                std::cerr << "[ERROR] lookup did not return hit" << std::endl;
            }
            else
            {
                if (response.key == it->first && response.value1.currBodyLen == it->second && response.value2.msgID == it->second)
                {
                    lookupCounter++;
                }
                else
                {
                    success = false;
                    std::cerr << "[ERROR] lookup returned wrong key or value" << std::endl;
                }

            }
            it++;
        }
        if(!regInsertFailureCount.empty()){
            uint16_t tmp = regInsertFailureCount.read();
            std::cout << "[ERROR] regInsertFailureCount: " << tmp << std::endl;
        }
        count++;
    }
    if (lookupCounter != numElements)
    {
        success = false;
        std::cerr << "[ERROR] not all lookups successfully, expected: " << numElements << ", received: " << lookupCounter << std::endl;
    }

    return (success) ? 0 : -1;
}