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
#include "../common.hpp"

struct bodyExtState{
    ap_uint<16> currSessionID;
    ap_uint<DATA_WIDTH> data;
    ap_uint<32> currWord_parsingPos;
    ap_uint<32> currWordValidLen;
    ap_uint<32> currBodyLen;
    ap_uint<32> requiredBodyLen;
    msgHeader   currMsgHeader;
    ap_uint<4>  lastMsgIndicator;
    bodyExtState(){
        currSessionID = 0;
        data = 0;
        currWord_parsingPos = 0;
        currWordValidLen = 0;
        currBodyLen = 0;
        requiredBodyLen = 0;
        currMsgHeader.reset();
        lastMsgIndicator = 0;
    }
    bodyExtState(ap_uint<16> currSessionID, ap_uint<DATA_WIDTH> data, ap_uint<32> currWord_parsingPos, ap_uint<32> currWordValidLen, 
        ap_uint<32> currBodyLen, ap_uint<32> requiredBodyLen, msgHeader currMsgHeader, ap_uint<4> lastMsgIndicator): 
        currSessionID(currSessionID), data(data), currWord_parsingPos(currWord_parsingPos), currWordValidLen(currWordValidLen), 
        currBodyLen(currBodyLen), requiredBodyLen(requiredBodyLen), currMsgHeader(currMsgHeader), lastMsgIndicator(lastMsgIndicator) {}
};

struct bodyExtState2{
    msgHeader   currMsgHeader;
    ap_uint<4>  lastMsgIndicator;
    bodyExtState2(){
        currMsgHeader.reset();
        lastMsgIndicator = 0;
    }
    bodyExtState2(msgHeader currMsgHeader, ap_uint<4> lastMsgIndicator): 
        currMsgHeader(currMsgHeader), lastMsgIndicator(lastMsgIndicator) {}
};

struct bodyMergeState{
    ap_uint<16> currSessionID;
    msgHeader   currMsgHeader;
    ap_uint<4>  lastMsgIndicator;
    ap_uint<32> startPos;
    ap_uint<32> length;
    ap_uint<1>  endOfBody;
    bodyMergeState(){
        currSessionID = 0;
        currMsgHeader.reset();
        lastMsgIndicator = 0;
        startPos = 0;
        length = 0;
        endOfBody = 0;
    }
    bodyMergeState(ap_uint<16> currSessionID, msgHeader currMsgHeader, ap_uint<4> lastMsgIndicator, 
        ap_uint<32> startPos, ap_uint<32> length, ap_uint<1> endOfBody): 
        currSessionID(currSessionID), currMsgHeader(currMsgHeader), lastMsgIndicator(lastMsgIndicator), 
        startPos(startPos), length(length), endOfBody(endOfBody) {}
};

const uint32_t PARSER_STASH_SIZE = 34;

static ap_uint<4> fsmState_stashTable[PARSER_STASH_SIZE];
static ap_uint<16> sessionID_stashTable[PARSER_STASH_SIZE];
static msgBody msgbody_stashTable[PARSER_STASH_SIZE];
// 1 means available slot, 0 means occupied slot
static ap_uint<PARSER_STASH_SIZE> valid_stashTable = ~(uint64_t)0;

int parser_stash_insert(ap_uint<16> sessionID, msgBody msgbody);
int parser_stash_lookup(ap_uint<16> sessionID);
bool parser_stash_remove(ap_uint<16> sessionID);

void parser(
    hls::stream<net_axis<DATA_WIDTH> >&     currWordFifo,
    hls::stream<sessionState>&              currSessionStateFifo,
    hls::stream<msgBody>&                   currMsgBodyFifo,
    hls::stream<sessionState>&              currSessionStateOutFifo,
    hls::stream<msgBody>&                   currMsgBodyStateOutFifo,
    hls::stream<msgHeader>&                 msgHeaderOutFifo,
    hls::stream<msgBody>&                   msgBodyOutFifo
    );