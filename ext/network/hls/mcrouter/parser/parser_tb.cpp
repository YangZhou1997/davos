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
// #include "parser.hpp"
#include "parser_v1_9cycles.hpp"
#include "../common.hpp"
#include <string.h>
#include <stdlib.h>
#include <iostream>
#include <arpa/inet.h>
#include <string>
#include <map>
#include <queue>
#include <vector>
#include <stdlib.h>

using namespace std;
using namespace hls;

#define DATA_LEN (DATA_WIDTH/8)

// len <= 64
void str2bits(uint8_t* str, uint32_t len, ap_uint<DATA_WIDTH>& bits){
    for(int i = 0; i < len; i++){
        bits(DATA_WIDTH-1-i*8, DATA_WIDTH-(i+1)*8) = str[i];
    }
}

// len <= 64
void bits2str(uint8_t* str, uint32_t len, ap_uint<DATA_WIDTH> bits){
    for(int i = 0; i < len; i++){
        str[i] = bits(DATA_WIDTH-1-i*8, DATA_WIDTH-(i+1)*8);
    }
}

void byteArray2axisWord(uint8_t* data, uint32_t len, vector<net_axis<DATA_WIDTH>>& words){
    uint32_t numWords = len / DATA_LEN;
    uint32_t remainBytes = len - numWords * DATA_LEN;
    
    //!!! it gets randomized initially, since it is not static. 
    net_axis<DATA_WIDTH> currWord;
    currWord.last = 0;
    for(int i = 0; i < numWords; i++){
        str2bits(data + i*DATA_LEN, DATA_LEN, currWord.data);
        if(remainBytes == 0 && i == numWords-1){
            currWord.last = 1;
        }
        currWord.keep = lenToKeep(DATA_LEN);
        words.push_back(currWord);
    }

    if(remainBytes != 0){
        str2bits(data + numWords*DATA_LEN, remainBytes, currWord.data);
        currWord.last = 1;
        currWord.keep = lenToKeep(remainBytes);
        words.push_back(currWord);
    }
}

void axisWord2byteArray(uint8_t* data, uint32_t& len, vector<net_axis<DATA_WIDTH>>& words){
    uint32_t numWords = words.size();

    for(int i = 0; i < numWords; i++){
        net_axis<DATA_WIDTH> currWord = words[i];
        uint32_t wordLen = keepToLen(currWord.keep);
        bits2str(data + len, wordLen, currWord.data);
        len += wordLen;
    }
}

#define NUM_KV 1024
static vector<string> keys;
static vector<string> vals;
static vector<bool> ops;
static map<string, string> kvMap;
static vector<net_axis<DATA_WIDTH>> globalWords;

// 62 chars
static const char* baseStr = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz";

string getStr(uint32_t startPos, uint32_t length){
    static char buf[128];
    if(startPos + length <= strlen(baseStr)){
        memcpy(buf, baseStr+startPos, length);
    }
    else{
        uint32_t remainLen = startPos + length - strlen(baseStr);
        memcpy(buf, baseStr+startPos, length-remainLen);
        memcpy(buf+length-remainLen, baseStr, remainLen);
    }
    buf[length] = '\0';
    return string(buf);
}

void workload_gen(){    
    srand(0xdeadbeef);
    // key: 1-40bytes
    // value: 1-59bytes
    for(int i = 0; i < NUM_KV; i++){
        uint32_t startPos = rand() % strlen(baseStr);
        uint32_t length = rand() % (40-1) + 1;
        string key = getStr(startPos, length);

        startPos = rand() % strlen(baseStr);
        length = rand() % (59-1) + 1;
        string val = getStr(startPos, length);
        if(kvMap.find(key) != kvMap.end()){
            i--;
            continue;
        }
        keys.push_back(key);
        vals.push_back(val);
        ops.push_back(rand() % 2);
        // ops.push_back(0);

        kvMap[key] = val;
    }

    uint8_t* buf = new uint8_t[NUM_KV*(24+40+59)];
    int currPos = 0;
    for(int i = 0; i < NUM_KV; i++){
        bool op = ops[i];
        string key = keys[i];
        string val = vals[i];
        
        static protocol_binary_request_header req; 
        req.request = {
            .magic = PROTOCOL_BINARY_REQ, 
            .opcode = PROTOCOL_BINARY_CMD_GET, 
            .keylen = htons(0),
            .extlen = 0,
            .datatype = 0,
            .reserved = htons(0),
            .bodylen = htonl(0),
            .opaque = htonl(i), // used as request id
            .cas = 0
        };
        if(!op){
            req.request.keylen = htons((uint16_t)key.length());
            req.request.bodylen = htonl(key.length());
            memcpy(buf + currPos, req.bytes, 24);
            currPos += 24;
            memcpy(buf + currPos, key.c_str(), key.length());
            currPos += key.length();
        }
        else{
            req.request.opcode = PROTOCOL_BINARY_CMD_SET;
            req.request.keylen = htons((uint16_t)key.length());
            req.request.bodylen = htonl(key.length() + val.length());
            memcpy(buf + currPos, req.bytes, 24);
            currPos += 24;
            memcpy(buf + currPos, key.c_str(), key.length());
            currPos += key.length();
            memcpy(buf + currPos, val.c_str(), val.length());
            currPos += val.length();
        }
    }
    // generated all words; 
    int currPos2 = 0;
    while(currPos2 < currPos){
        uint32_t pktlen = rand() % 128 + 128;
        if(currPos2 + pktlen > currPos){
            pktlen = currPos - currPos2;
        }
        vector<net_axis<DATA_WIDTH>> tmpWords;
        tmpWords.clear();
        byteArray2axisWord(buf + currPos2, pktlen, tmpWords);
        currPos2 += pktlen;

        for(int j = 0; j < tmpWords.size(); j++){
            globalWords.push_back(tmpWords[j]);
        }
    }
    delete buf;
}

const uint16_t sessionID1 = 34567;
const uint16_t sessionID2 = 45678;
const uint16_t sessionID3 = 56789;

int get_idx(uint16_t sessionID){
    if(sessionID == sessionID1){
        return 0;
    }if(sessionID == sessionID2){
        return 1;
    }if(sessionID == sessionID3){
        return 2;
    }
    else{
        std::cout << "ERROR: get_idx" << std::endl;
        return 0;
    }
}

template <int W>
void traffic_gen(
    stream<net_axis<DATA_WIDTH> >&  currWordFifo,
    stream<msgSessionState>&           currSessionStateFifo,
    stream<msgBody>&                currMsgBodyFifo,
    stream<msgSessionState>&           currSessionStateOutFifo, 
    stream<msgBody>&                currMsgBodyStateOutFifo, 
    stream<ap_uint<16> >&           sessionIDOutFifo, 
    stream<msgHeader>&              msgHeaderOutFifo, 
    stream<msgBody>&                msgBodyOutFifo
){
    static msgSessionState currSessionState[3] = {msgSessionState(sessionID1), msgSessionState(sessionID2), msgSessionState(sessionID3)};
    static msgBody currMsgBody[3] = {msgBody(sessionID1), msgBody(sessionID2), msgBody(sessionID3)};

    static int msgCnt[3] = {0, 0, 0};
    static bool allow_new_word[3] = {true, true, true};
    static uint32_t wordCnt[3] = {0, 0, 0};
    

    int sidx = rand() % 3;
    // workload empty
    if(wordCnt[sidx] < globalWords.size() && allow_new_word[sidx]){
        net_axis<DATA_WIDTH> currWord = globalWords[wordCnt[sidx]++];

        currWordFifo.write(currWord);
        currSessionStateFifo.write(currSessionState[sidx]);
        currMsgBodyFifo.write(currMsgBody[sidx]);
        allow_new_word[sidx] = false;

        cout << "==============================sending a word===================================" << endl;
    }

    if(!currSessionStateOutFifo.empty() && !currMsgBodyStateOutFifo.empty()){
        msgSessionState tmp = currSessionStateOutFifo.read();
        int sidx = get_idx(tmp.currSessionID);
        currSessionState[sidx] = tmp;
        currMsgBody[sidx] = currMsgBodyStateOutFifo.read();
        allow_new_word[sidx] = true;
    }

    if(!sessionIDOutFifo.empty() && !msgHeaderOutFifo.empty() && !msgBodyOutFifo.empty()){
        ap_uint<16> sessionID = sessionIDOutFifo.read();
        msgHeader outMsgHeader = msgHeaderOutFifo.read();
        msgBody outMsgBody = msgBodyOutFifo.read();
        int sidx = get_idx(outMsgBody.currSessionID);

        outMsgHeader.display();
        outMsgBody.display(outMsgHeader.extLen, outMsgHeader.keyLen, outMsgHeader.val_len());
        cout << "correct msg: " << endl;
        cout << "key: " << keys[msgCnt[sidx]] << endl;
        if(ops[msgCnt[sidx]]){
            cout << "val: " << vals[msgCnt[sidx]] << endl;
        }
        cout << "==============================receive a msg===================================" << endl;

        msgCnt[sidx] ++;
    }
}

int main()
{
    stream<net_axis<DATA_WIDTH> >     currWordFifo;
    stream<msgSessionState>              currSessionStateFifo;
    stream<msgBody>                   currMsgBodyFifo;
    stream<msgSessionState>              currSessionStateOutFifo;
    stream<msgBody>                   currMsgBodyStateOutFifo;
    stream<ap_uint<16> >              sessionIDOutFifo;
    stream<msgHeader>                 msgHeaderOutFifo;
    stream<msgBody>                   msgBodyOutFifo;
    
    workload_gen();

    int cycleCount = 0;

    
    while(cycleCount < 1000){
        parser(currWordFifo, currSessionStateFifo, currMsgBodyFifo, 
            currSessionStateOutFifo, currMsgBodyStateOutFifo, sessionIDOutFifo, msgHeaderOutFifo, msgBodyOutFifo);
        traffic_gen<1>(currWordFifo, currSessionStateFifo, currMsgBodyFifo, 
            currSessionStateOutFifo, currMsgBodyStateOutFifo, sessionIDOutFifo, msgHeaderOutFifo, msgBodyOutFifo);
        
        cycleCount ++;
    }
}