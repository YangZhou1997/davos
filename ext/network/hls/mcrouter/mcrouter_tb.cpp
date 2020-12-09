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
#include "mcrouter.hpp"
#include <string.h>
#include <stdlib.h>
#include <iostream>
#include <vector>
#include <queue>
#include <arpa/inet.h>
#include <string>
#include <map>

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

void displayMsg(uint8_t* data, uint32_t len, bool check_res, uint32_t opaque){
    protocol_binary_request_header req;
    memcpy(req.bytes, data, 24);
    uint16_t keylen = htons(req.request.keylen);
    uint16_t extlen = req.request.extlen;
    uint32_t bodylen = htonl(req.request.bodylen);
    uint32_t vallen = bodylen - keylen - extlen;

    switch(req.request.opcode){
        case PROTOCOL_BINARY_CMD_GET: {
            req.display();

            char* key = new char[keylen + 1];
            memcpy(key, data+24, keylen);
            key[keylen] = '\0';
            cout << "key: " << key << endl;
            
            if(check_res){
                cout << "==================GET request check==================" << endl;
                cout << "received key: " << key << endl;
                cout << "-correct key: " << keys[opaque] << endl;
                cout << "==================GET request end===================" << endl;
            }

            delete key;
            break;
        }
        case PROTOCOL_BINARY_CMD_SET: {
            req.display();

            char* key = new char[keylen + 1];
            memcpy(key, data+24, keylen);
            key[keylen] = '\0';
            cout << "key: " << key << endl;

            char* val = new char[vallen + 1];
            memcpy(val, data+24+keylen, vallen);
            val[vallen] = '\0';
            cout << "val: " << val << endl;

            if(check_res){
                cout << "==================SET request check==================" << endl;
                cout << "received key: " << key << endl;
                cout << "-correct key: " << keys[opaque] << endl;

                cout << "received val: " << val << endl;
                cout << "-correct val: " << vals[opaque] << endl;
                cout << "==================SET request end===================" << endl;
            }

            delete key, val;
            break;
        }
        case PROTOCOL_BINARY_CMD_RGET: {
            req.display();
            
            char* val = new char[vallen + 1];
            memcpy(val, data+24, vallen);
            val[vallen] = '\0';

            if(check_res){
                cout << "==================GET response check==================" << endl;
                cout << "received val: " << val << endl;
                cout << "-correct val: " << vals[opaque] << endl;
                cout << "==================GET response end===================" << endl;
            }
            delete val;
            break;
        }
        case PROTOCOL_BINARY_CMD_RSET: {
            req.display();
            if(check_res){
                cout << "==================SET response check==================" << endl;
                cout << "==================SET response end===================" << endl;
            }
            break;
        }
    }
}

uint16_t clientSessionID = 32245;
uint16_t memcachedSessionID = 41235;

void client(
    stream<appNotification>& notifications,
    stream<appReadRequest>& readRequest, 
    stream<ap_uint<16> >& rxMetaData,
    stream<net_axis<DATA_WIDTH> >& rxData,
    stream<appTxMeta>& txMetaData,
    stream<net_axis<DATA_WIDTH> >& txData,
    stream<appTxRsp>& txStatus
){
    static ap_uint<4> clientState = 0;
    static vector<net_axis<DATA_WIDTH>> rxWords;
    static vector<net_axis<DATA_WIDTH>> txWords;
    static uint32_t currLen = 0;

    switch(clientState){
        case 0: {
            // the start of a key-value request. 
            rxWords.clear();
            currLen = 0;
            
            // workload empty
            if(globalWords.size() == 0){
                clientState = 1;
                break;
            }

            while(true){
                net_axis<DATA_WIDTH> currWord = globalWords.front();
                rxWords.push_back(currWord);
                currLen += keepToLen(currWord.keep);
                globalWords.pop();
                if(currWord.last){
                    break;
                }
            }
            appNotification notific(clientSessionID, currLen, 0, 0, false);
            notifications.write(notific);
            clientState = 1;
            break;
        }
        case 1: {
            // client writing memcached request to mcrouter. 
            if(!readRequest.empty()){
                appReadRequest appReq = readRequest.read();
                cout << "client sends data: session " << appReq.sessionID << ", length " << appReq.length << endl;

                rxMetaData.write(clientSessionID);            
                cout << "rxWords.size() = " << rxWords.size() << endl;

                for(int i = 0; i < rxWords.size(); i++){
                    rxData.write(rxWords[i]);
                }
            }
            // client receives final response from mcrouter. 
            else if(!txMetaData.empty()){
                appTxMeta txMeta = txMetaData.read();
                if(txMeta.sessionID == clientSessionID){
                    uint32_t length = txMeta.length;
                    cout << "client prepares receiving data: len=" << dec << length << endl;
                    txStatus.write(appTxRsp(clientSessionID, length, 64<<10, 0));
                    txWords.clear();
                    clientState = 2;
                }
            }
            break;
        }
        case 2: {
            // client actually receives final response from mcrouter. 
            if(!txData.empty()){
                net_axis<DATA_WIDTH> currWord = txData.read();
                txWords.push_back(currWord);
                if(currWord.last){
                    uint8_t* data = new uint8_t[64 * txWords.size()];
                    uint32_t len = 0;
                    axisWord2byteArray(data, len, txWords);

                    protocol_binary_response_header rsp;
                    memcpy(rsp.bytes, data, 24);
                    uint32_t opaque = htonl(rsp.response.opaque);
                    
                    cout << "client receives data: len=" << dec << len << endl;
                    displayMsg(data, len, true, opaque);
                    // clientState = 1;
                    clientState = 0; // initiating a new key-value request. 
                    delete data;
                    break;
                }
            }
        }
    }    
}

void memcached(
    stream<appNotification>& notifications,
    stream<appReadRequest>& readRequest, 
    stream<ap_uint<16> >& rxMetaData,
    stream<net_axis<DATA_WIDTH> >& rxData,
    stream<appTxMeta>& txMetaData,
    stream<net_axis<DATA_WIDTH> >& txData,
    stream<appTxRsp>& txStatus
){
    static ap_uint<4> mcState = 0;
    static vector<net_axis<DATA_WIDTH>> txWords;
    static vector<net_axis<DATA_WIDTH>> rxWords;
    
    switch(mcState){
        case 0: {
            // memcached waiting to receive request from mcrouter. 
            if(!txMetaData.empty()){
                appTxMeta txMeta = txMetaData.read();
                if(txMeta.sessionID == memcachedSessionID){
                    uint32_t length = txMeta.length;
                    cout << "memcached prepares receiving data: " << dec << length << endl;
                    txStatus.write(appTxRsp(memcachedSessionID, length, 64<<10, 0));
                    txWords.clear();
                    mcState = 1;
                }
            }
            // memcached sends response to mcrouter
            else if(!readRequest.empty()){
                appReadRequest appReq = readRequest.read();
                cout << "memcached sends data: session " << dec << appReq.sessionID << ", length" << appReq.length << endl;
                rxMetaData.write(memcachedSessionID);

                for(int i = 0; i < rxWords.size(); i++){
                    rxData.write(rxWords[i]);
                }
            }
            break;
        }
        case 1: {
            // memcached actually receives request data from mcrouter. 
            if(!txData.empty()){
                net_axis<DATA_WIDTH> currWord = txData.read();
                txWords.push_back(currWord);
                if(currWord.last){
                    // cout << txWords.size() << endl;
                    uint8_t* data = new uint8_t[64 * txWords.size()];
                    uint32_t len = 0;
                    axisWord2byteArray(data, len, txWords);
                    
                    protocol_binary_request_header req;
                    memcpy(req.bytes, data, 24);
                    uint32_t opaque = htonl(req.request.opaque);
                    
                    cout << "memcached receives data: " << dec << len << endl;
                    displayMsg(data, len, true, opaque);
                    
                    delete data;

                    // generating response; 
                    protocol_binary_response_header rsp;
                    rsp.response = {
                        .magic = PROTOCOL_BINARY_RES, 
                        .opcode = PROTOCOL_BINARY_CMD_RGET, 
                        .keylen = htons(0),
                        .extlen = 0,
                        .datatype = 0,
                        .status = htons(0),
                        .bodylen = htonl(0),
                        .opaque = htonl(opaque),
                        .cas = 0
                    };
                    rxWords.clear();
                    
                    bool op = (req.request.opcode != PROTOCOL_BINARY_CMD_GET);
                    string val = vals[opaque];
                    uint32_t vallen = val.length();
                    uint8_t* buf = new uint8_t[24+vallen];
                    if(!op){
                        rsp.response.magic = PROTOCOL_BINARY_RES;
                        rsp.response.opcode = PROTOCOL_BINARY_CMD_RGET;
                        rsp.response.bodylen = htonl(vallen);
                        memcpy(buf, rsp.bytes, 24);
                        memcpy(buf + 24, val.c_str(), vallen);
                        byteArray2axisWord(buf, 24+vallen, rxWords);
                        // initiating the process of memcached sending response back
                        appNotification notific(memcachedSessionID, 24+vallen, 0, 0, false);
                        notifications.write(notific);
                    }
                    else{
                        rsp.response.magic = PROTOCOL_BINARY_RES;
                        rsp.response.opcode = PROTOCOL_BINARY_CMD_RSET;
                        memcpy(buf, rsp.bytes, 24);
                        byteArray2axisWord(buf, 24, rxWords);
                        // initiating the process of memcached sending response back
                        appNotification notific(memcachedSessionID, 24, 0, 0, false);
                        notifications.write(notific);
                    }
                    
                    delete buf;
                    mcState = 0;
                }
            }
            break;
        }
    }
}

// static const char* globalKey = "hellohellohellohellohellohellohellohello";
// static const char* globalVal = "worldworldworldworldworldworldworldworldworldworldworld";
    
void client2(
    stream<appNotification>& notifications,
    stream<appReadRequest>& readRequest, 
    stream<ap_uint<16> >& rxMetaData,
    stream<net_axis<DATA_WIDTH> >& rxData,
    stream<appTxMeta>& txMetaData,
    stream<net_axis<DATA_WIDTH> >& txData,
    stream<appTxRsp>& txStatus
){
    static ap_uint<4> clientState = 0;    
    static uint32_t idx = 0;
    static vector<net_axis<DATA_WIDTH>> words;
    
    int nMsg = 4;

    switch(clientState){
        case 0: {
            // the start of a key-value request. 
            uint16_t keylen = keys[idx].length(); 
            uint16_t vallen = vals[idx].length(); 
            if(!ops[idx]){
                appNotification notific(clientSessionID, (24+keylen)*nMsg, 0, 0, false);
                notifications.write(notific);
            }
            else{
                appNotification notific(clientSessionID, (24+keylen+vallen)*nMsg, 0, 0, false);
                notifications.write(notific);
            }
            clientState = 1;
            break;
        }
        case 1: {
            // client writing memcached request to mcrouter. 
            if(!readRequest.empty()){
                appReadRequest appReq = readRequest.read();
                cout << "client sends data: session " << appReq.sessionID << ", length " << appReq.length << endl;

                rxMetaData.write(clientSessionID);            
                words.clear();

                if(!ops[idx]){
                    const char* globalKey = keys[idx].c_str();
                    uint16_t keylen = strlen(globalKey); 
                    protocol_binary_request_header req; 
                    req.request = {
                            .magic = PROTOCOL_BINARY_REQ, 
                            .opcode = PROTOCOL_BINARY_CMD_GET, 
                            .keylen = htons(keylen),
                            .extlen = 0,
                            .datatype = 0,
                            .reserved = htons(0),
                            .bodylen = htonl(keylen),
                            .opaque = htonl(idx),
                            .cas = 0
                        };
                    idx = (idx + 1) % NUM_KV;

                    uint8_t* data = new uint8_t[(24+keylen)*nMsg];
                    for(int i = 0; i < nMsg; i++){
                        memcpy(data+(24+keylen)*i, req.bytes, 24);
                        memcpy(data+(24+keylen)*i+24, globalKey, keylen);
                    }
                    
                    byteArray2axisWord(data, (24+keylen)*nMsg, words);
                    cout << "words.size() = " << words.size() << endl;
                    delete data;
                }
                else{
                    const char* globalKey = keys[idx].c_str();
                    const char* globalVal = vals[idx].c_str();
                    uint16_t keylen = strlen(globalKey); 
                    uint16_t vallen = strlen(globalVal); 
                    protocol_binary_request_header req; 
                    req.request = {
                            .magic = PROTOCOL_BINARY_REQ, 
                            .opcode = PROTOCOL_BINARY_CMD_SET, 
                            .keylen = htons(keylen),
                            .extlen = 0,
                            .datatype = 0,
                            .reserved = htons(0),
                            .bodylen = htonl(keylen + vallen),
                            .opaque = htonl(idx),
                            .cas = 0
                        };
                    idx = (idx + 1) % NUM_KV;

                    uint8_t* data = new uint8_t[(24+keylen+vallen)*nMsg];
                    for(int i = 0; i < nMsg; i++){
                        memcpy(data+(24+keylen+vallen)*i, req.bytes, 24);
                        memcpy(data+(24+keylen+vallen)*i+24, globalKey, keylen);
                        memcpy(data+(24+keylen+vallen)*i+24+keylen, globalVal, vallen);
                    }
                    
                    byteArray2axisWord(data, (24+keylen+vallen)*nMsg, words);
                    cout << "words.size() = " << words.size() << endl;
                    delete data;
                }

                for(int i = 0; i < words.size(); i++){
                    rxData.write(words[i]);
                }
            }
            // client receives final response from mcrouter. 
            else if(!txMetaData.empty()){
                appTxMeta txMeta = txMetaData.read();
                if(txMeta.sessionID == clientSessionID){
                    uint32_t length = txMeta.length;
                    cout << "client prepares receivomg data: len=" << dec << length << endl;
                    txStatus.write(appTxRsp(clientSessionID, length, 64<<10, 0));
                    words.clear();
                    clientState = 2;
                }
            }
            break;
        }
        case 2: {
            // client actually receives final response from mcrouter. 
            if(!txData.empty()){
                net_axis<DATA_WIDTH> currWord = txData.read();
                words.push_back(currWord);
                if(currWord.last){
                    uint8_t* data = new uint8_t[64 * words.size()];
                    uint32_t len = 0;
                    axisWord2byteArray(data, len, words);

                    protocol_binary_response_header rsp;
                    memcpy(rsp.bytes, data, 24);
                    uint32_t opaque = htonl(rsp.response.opaque);
                    
                    cout << "client receives data: len=" << dec << len << endl;
                    displayMsg(data, len, true, opaque);
                    // clientState = 1;
                    clientState = 0; // initiating a new key-value request. 
                    delete data;
                    break;
                }
            }
        }
    }    
}

void memcached2(
    stream<appNotification>& notifications,
    stream<appReadRequest>& readRequest, 
    stream<ap_uint<16> >& rxMetaData,
    stream<net_axis<DATA_WIDTH> >& rxData,
    stream<appTxMeta>& txMetaData,
    stream<net_axis<DATA_WIDTH> >& txData,
    stream<appTxRsp>& txStatus
){
    static ap_uint<4> mcState = 0;
    static vector<net_axis<DATA_WIDTH>> words;
    static uint32_t opaque = 0;
    
    switch(mcState){
        case 0: {
            // memcached waiting to receive request from mcrouter. 
            if(!txMetaData.empty()){
                appTxMeta txMeta = txMetaData.read();
                if(txMeta.sessionID == memcachedSessionID){
                    uint32_t length = txMeta.length;
                    cout << "memcached prepares receiving data: " << dec << length << endl;
                    txStatus.write(appTxRsp(memcachedSessionID, length, 64<<10, 0));
                    words.clear();
                    mcState = 1;
                }
            }
            // memcached sends response to mcrouter
            else if(!readRequest.empty()){
                appReadRequest appReq = readRequest.read();
                cout << "memcached sends data: session " << dec << appReq.sessionID << ", length" << appReq.length << endl;

                rxMetaData.write(memcachedSessionID);
                words.clear();
                
                if(!ops[opaque]){
                    const char* globalVal = vals[opaque].c_str();
                    uint16_t vallen = strlen(globalVal);
                    protocol_binary_response_header rsp;
                    rsp.response = {
                            .magic = PROTOCOL_BINARY_RES, 
                            .opcode = PROTOCOL_BINARY_CMD_RGET, 
                            .keylen = htons(0),
                            .extlen = 0,
                            .datatype = 0,
                            .status = htons(0),
                            .bodylen = htonl(vallen),
                            .opaque = htonl(opaque),
                            .cas = 0
                        };
                    
                    uint8_t* data = new uint8_t[24+vallen];
                    memcpy(data, rsp.bytes, 24);
                    memcpy(data+24, globalVal, vallen);
                    byteArray2axisWord(data, 24+vallen, words);
                    delete data;
                }
                else{
                    protocol_binary_response_header rsp;
                    rsp.response = {
                            .magic = PROTOCOL_BINARY_RES, 
                            .opcode = PROTOCOL_BINARY_CMD_RSET, 
                            .keylen = htons(0),
                            .extlen = 0,
                            .datatype = 0,
                            .status = htons(0),
                            .bodylen = htonl(0),
                            .opaque = htonl(opaque),
                            .cas = 0
                        };
                    
                    uint8_t* data = new uint8_t[24];
                    memcpy(data, rsp.bytes, 24);
                    byteArray2axisWord(data, 24, words);
                    delete data;
                }

                for(int i = 0; i < words.size(); i++){
                    rxData.write(words[i]);
                }
            }
            break;
        }
        case 1: {
            // memcached actually receives request data from mcrouter. 
            if(!txData.empty()){
                net_axis<DATA_WIDTH> currWord = txData.read();
                words.push_back(currWord);
                if(currWord.last){
                    uint8_t* data = new uint8_t[64 * words.size()];
                    uint32_t len = 0;
                    axisWord2byteArray(data, len, words);
                    
                    protocol_binary_request_header req;
                    memcpy(req.bytes, data, 24);
                    opaque = htonl(req.request.opaque);
                    uint16_t vallen = vals[opaque].length();

                    cout << "memcached receives data: " << dec << len << endl;
                    displayMsg(data, len, true, opaque);
                    delete data;
                    
                    // initiating the process of memcached sending response back
                    appNotification notific(memcachedSessionID, 24+vallen, 0, 0, false);
                    notifications.write(notific);
                    
                    mcState = 0;
                }
            }
            break;
        }
    }
}
template <class T>
void mux(stream<T>& in1, stream<T>& in2, stream<T>& out){
    if(!in1.empty()){
        T tmp = in1.read();
        out.write(tmp);
    }
    else if(!in2.empty()){
        T tmp = in2.read();
        out.write(tmp);
    }
}

void appReqMux(stream<appReadRequest>& readRequest, stream<appReadRequest>& readRequest1, stream<appReadRequest>& readRequest2){
    if(!readRequest.empty()){
        appReadRequest appReq = readRequest.read();
        if(appReq.sessionID == clientSessionID){
            readRequest1.write(appReq);
        }
        else if(appReq.sessionID == memcachedSessionID){
            readRequest2.write(appReq);
        }
        else{
        }
    }
}

void txMetaMux(stream<appTxMeta>& txMetaData, stream<appTxMeta>& txMetaData1, stream<appTxMeta>& txMetaData2, 
    stream<net_axis<DATA_WIDTH> >& txData, stream<net_axis<DATA_WIDTH> >& txData1, stream<net_axis<DATA_WIDTH> >& txData2){
    static ap_uint<4> muxState = 0;
    switch(muxState){
        case 0: {
            if(!txMetaData.empty()){
                appTxMeta txMeta = txMetaData.read();
                if(txMeta.sessionID == clientSessionID){
                    txMetaData1.write(txMeta);
                    muxState = 1;
                }
                else if(txMeta.sessionID == memcachedSessionID){
                    txMetaData2.write(txMeta);
                    muxState = 2;
                }
                else{
                    cout << "txMetaData unknown sessionID: " << txMeta.sessionID << endl;
                }
            }
            break;
        }
        case 1: {
            if(!txData.empty()){
                net_axis<DATA_WIDTH> currWord = txData.read();
                txData1.write(currWord);
                if(currWord.last){
                    muxState = 0;
                }
            }
            break;
        }
        case 2: {
            if(!txData.empty()){
                net_axis<DATA_WIDTH> currWord = txData.read();
                txData2.write(currWord);
                if(currWord.last){
                    muxState = 0;
                }
            }
            break;
        }
    }
}

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

int main()
{
	stream<ap_uint<16> > listenPort("listenPort");
	stream<bool> listenPortStatus("listenPortStatus");
	stream<ipTuple> openTuples;
	stream<ipTuple> openConnection;
	stream<openStatus> openConStatus;
	stream<ap_uint<16> > closeConnection;

	stream<appNotification> notifications;
	stream<appReadRequest> readRequest;
	stream<ap_uint<16> > rxMetaData;
	stream<net_axis<DATA_WIDTH> > rxData;
	stream<appTxMeta> txMetaData;
	stream<net_axis<DATA_WIDTH> > txData;
	stream<appTxRsp> txStatus;

	stream<appNotification> notifications1;
	stream<appReadRequest> readRequest1;
	stream<ap_uint<16> > rxMetaData1;
	stream<net_axis<DATA_WIDTH> > rxData1;
	stream<appTxMeta> txMetaData1;
	stream<net_axis<DATA_WIDTH> > txData1;
	stream<appTxRsp> txStatus1;
    
	stream<appNotification> notifications2;
	stream<appReadRequest> readRequest2;
	stream<ap_uint<16> > rxMetaData2;
	stream<net_axis<DATA_WIDTH> > rxData2;
	stream<appTxMeta> txMetaData2;
	stream<net_axis<DATA_WIDTH> > txData2;
	stream<appTxRsp> txStatus2;


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

	int cycleCount = 0;
	int portOpened = -1;

    static ap_uint<4> tbState = 0;
   
	while (cycleCount < 1000)
	{
        mcrouter(listenPort, listenPortStatus, notifications, readRequest,
		    rxMetaData, rxData, openTuples, openConnection, openConStatus,
			closeConnection, txMetaData, txData, txStatus);

#define TEST 1

#if TEST == 1
        client(notifications1, readRequest1, rxMetaData1, rxData1, txMetaData1, txData1, txStatus1);
        memcached(notifications2, readRequest2, rxMetaData2, rxData2, txMetaData2, txData2, txStatus2);
#elif TEST == 2
        client2(notifications1, readRequest1, rxMetaData1, rxData1, txMetaData1, txData1, txStatus1);
        memcached2(notifications2, readRequest2, rxMetaData2, rxData2, txMetaData2, txData2, txStatus2);
#endif

        mux(notifications1, notifications2, notifications);
        appReqMux(readRequest, readRequest1, readRequest2);
        mux(rxMetaData1, rxMetaData2, rxMetaData);
        mux(rxData1, rxData2, rxData);
        txMetaMux(txMetaData, txMetaData1, txMetaData2, txData, txData1, txData2);
        mux(txStatus1, txStatus2, txStatus);
        
		switch(tbState){
            case 0: {
                // let mcrouter try to connect remote memcached; 
        	    ipTuple tuple;
                tuple.ip_address = 0x0a010101;
        		tuple.ip_port = 0x3412;
                openTuples.write(tuple);
                tbState = 1;
                break;
            }
            case 1: {
                // read mcrouter connection request to memcached. 
                if (!openConnection.empty()){
                    ipTuple tuple = openConnection.read();
                    openConStatus.write(openStatus(memcachedSessionID, true));
        		    cout << "mcrouter connecting to: " << hex << tuple.ip_address << ":" << dec << tuple.ip_port << endl;
                }
                tbState = 2;
                break;
            }
            case 2: {
                // let mcrouter listen on port 
        		if (!listenPort.empty())
        		{
        			ap_uint<16> port = listenPort.read();
        			listenPortStatus.write(true);
        			cout << "mcrouter listens on port: " << dec << port << endl;
        			portOpened = 0;
        		}
                tbState = 1;
                break;
            }
        }
   
		cycleCount++;
	}
	return portOpened;
}
