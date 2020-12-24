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
#include "parser.hpp"
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
static queue<net_axis<DATA_WIDTH>> globalWords;

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
            globalWords.push(tmpWords[j]);
        }
    }
    delete buf;
}

int main()
{
    stream<net_axis<DATA_WIDTH> >     currWordFifo;
    stream<sessionState>              currSessionStateFifo;
    stream<msgBody>                   currMsgBodyFifo;
    stream<sessionState>              currSessionStateOutFifo;
    stream<msgBody>                   currMsgBodyStateOutFifo;
    stream<msgHeader>                 msgHeaderOutFifo;
    stream<msgBody>                   msgBodyOutFifo;
    
    workload_gen();

    net_axis<DATA_WIDTH> currWord;
    sessionState currSessionState = sessionState();
    msgBody currMsgBody = msgBody();

    msgHeader outMsgHeader;
    msgBody outMsgBody;

    int cycleCount = 0;
    int msgCnt = 0;
    bool allow_new_word = true;

    while(cycleCount < 1000){
        parser(currWordFifo, currSessionStateFifo, currMsgBodyFifo, 
            currSessionStateOutFifo, currMsgBodyStateOutFifo, msgHeaderOutFifo, msgBodyOutFifo);

        // workload empty
        if(globalWords.size() != 0 && allow_new_word){
            net_axis<DATA_WIDTH> currWord = globalWords.front();
            globalWords.pop();

            currWordFifo.write(currWord);
            currSessionStateFifo.write(currSessionState);
            currMsgBodyFifo.write(currMsgBody);
            allow_new_word = false;

            cout << "==============================sending a word===================================" << endl;
        }

        if(!currSessionStateOutFifo.empty() && !currMsgBodyStateOutFifo.empty()){
            currSessionState = currSessionStateOutFifo.read();
            currMsgBody = currMsgBodyStateOutFifo.read();
            allow_new_word = true;
        }

        if(!msgHeaderOutFifo.empty() && !msgBodyOutFifo.empty()){
            outMsgHeader = msgHeaderOutFifo.read();
            outMsgBody = msgBodyOutFifo.read();
            outMsgHeader.display();
            outMsgBody.display(outMsgHeader.extLen, outMsgHeader.keyLen, outMsgHeader.val_len());
            cout << "correct msg: " << endl;
            cout << "key: " << keys[msgCnt] << endl;
            if(ops[msgCnt]){
                cout << "val: " << vals[msgCnt] << endl;
            }
            cout << "==============================receive a msg===================================" << endl;

            msgCnt ++;
        }
        cycleCount ++;
    }
}