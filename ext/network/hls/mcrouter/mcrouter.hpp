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
#pragma once

#include "../axi_utils.hpp"
#include "../toe/toe.hpp"
#include "parser_stateman/parser_stateman.hpp"
#include "parser/parser.hpp"
#include "hash_table_32_32/hash_table_32_32.hpp"
#include "multi_queue/multi_queue.hpp"
#include "common.hpp"

#define MAX_CONNECTED_SESSIONS 512

enum lockOP { LOCK_ACQUIRE, LOCK_RELEASE};
struct lockReq{
    ap_uint<32> msgID;
    lockOP opcode;
    lockReq() {}
    lockReq(ap_uint<32> m, lockOP o): msgID(m), opcode(o) {}
};

struct msgContext {
    ap_uint<16> numRsp;
    ap_uint<16> srcSessionID;
    msgContext() {}
    msgContext(ap_uint<16> _numRsp, ap_uint<16> _srcSessionID) :
        numRsp(_numRsp), 
        srcSessionID(_srcSessionID)
    {}
    void consume_word(ap_uint<32>& w){
        numRsp = w(31, 16);
        srcSessionID = w(15, 0);
    }
    ap_uint<32> output_word(){
        return (numRsp, srcSessionID);
    }
};

// const uint32_t AC_STASH_SIZE = 34;
const uint32_t AC_STASH_SIZE = 8;
static ap_uint<AC_STASH_SIZE> ac_valid_stashTable = ~(uint64_t)0;
static ap_uint<16> ac_sessionID_stashTable[AC_STASH_SIZE];
static ap_uint<2> ac_parsingState_rsp[AC_STASH_SIZE];
static sessionState ac_parsingState1[AC_STASH_SIZE];
static msgBody ac_parsingState2[AC_STASH_SIZE];

int ac_stash_insert(ap_uint<16> sessionID);
int ac_stash_lookup(ap_uint<16> sessionID);
bool ac_stash_remove(ap_uint<16> sessionID);

/** @defgroup mcrouter Echo Server Application
 *
 */
void mcrouter(hls::stream<ap_uint<16> >& listenPort, hls::stream<bool>& listenPortStatus,
			hls::stream<appNotification>& notifications, hls::stream<appReadRequest>& readRequest,
			hls::stream<ap_uint<16> >& rxMetaData, hls::stream<net_axis<DATA_WIDTH> >& rxData,
			hls::stream<ipTuple>& openConnection, hls::stream<openStatus>& openConStatus, hls::stream<ap_uint<16> >& closeConnection,
			hls::stream<appTxMeta>& txMetaData, hls::stream<net_axis<DATA_WIDTH> >& txData, hls::stream<appTxRsp>& txStatus, 
            ap_uint<14>		useConn,
        	ap_uint<32>		regIpAddress0,
        	ap_uint<32>		regIpAddress1,
        	ap_uint<32>		regIpAddress2,
        	ap_uint<32>		regIpAddress3,
        	ap_uint<32>		regIpAddress4,
        	ap_uint<32>		regIpAddress5,
        	ap_uint<32>		regIpAddress6,
        	ap_uint<32>		regIpAddress7,
        	ap_uint<32>		regIpAddress8,
        	ap_uint<32>		regIpAddress9,
            ap_uint<16>		regIpPort0,
        	ap_uint<16>		regIpPort1,
        	ap_uint<16>		regIpPort2,
        	ap_uint<16>		regIpPort3,
        	ap_uint<16>		regIpPort4,
        	ap_uint<16>		regIpPort5,
        	ap_uint<16>		regIpPort6,
        	ap_uint<16>		regIpPort7,
        	ap_uint<16>		regIpPort8,
        	ap_uint<16>		regIpPort9);
