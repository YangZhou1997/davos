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
#include <arpa/inet.h>
#include <string>

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
    
    net_axis<DATA_WIDTH> currWord;
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

void axisWord2byteArray(uint8_t* data, uint32_t& len, vector<net_axis<DATA_WIDTH>> words){
    uint32_t numWords = words.size();

    for(int i = 0; i < numWords; i++){
        net_axis<DATA_WIDTH> currWord = words[i];
        uint32_t wordLen = keepToLen(currWord.keep);
        bits2str(data + len, wordLen, currWord.data);
        len += wordLen;
    }
}

void displayMsg(uint8_t* data, uint32_t len){
    protocol_binary_request_header req;
    memcpy(req.bytes, data, 24);
    uint16_t keylen = htons(req.request.keylen);
    uint16_t extlen = req.request.extlen;
    uint32_t bodylen = htonl(req.request.bodylen);
    uint32_t vallen = bodylen - keylen - extlen;

    switch(req.request.opcode){
        case PROTOCOL_BINARY_CMD_GET: {
            cout << "GET request: keylen "  << dec << keylen << ", magic " << hex << req.request.magic << endl;
            cout << "GET request: bodylen " << dec << bodylen << ", opaque " << hex << req.request.opaque << endl;
            
            char* key = new char[keylen + 1];
            memcpy(key, data+24, keylen);
            key[keylen] = '\0';
            cout << "key: " << string(key) << endl;
            break;
        }
        case PROTOCOL_BINARY_CMD_SET: {
            cout << "SET request: keylen " << keylen << " vallen " << vallen << endl;
            
            char* key = new char[keylen + 1];
            memcpy(key, data+24, keylen);
            key[keylen] = '\0';
            cout << "key: " << key << endl;

            char* val = new char[vallen + 1];
            memcpy(val, data+24+keylen, vallen);
            val[vallen] = '\0';
            cout << "val: " << val << endl;
            break;
        }
        case PROTOCOL_BINARY_CMD_RGET: {
            cout << "GET response: vallen " << vallen << endl;
            
            char* val = new char[vallen + 1];
            memcpy(val, data+24, vallen);
            val[vallen] = '\0';
            cout << "val: " << val << endl;
            break;
        }
        case PROTOCOL_BINARY_CMD_RSET: {
            cout << "SET response" << endl;
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
    static char* key = "hello";
    static uint16_t keylen = strlen(key); 
    
    static protocol_binary_request_header req; 
    req.request = {
            .magic = PROTOCOL_BINARY_REQ, 
            .opcode = PROTOCOL_BINARY_CMD_GET, 
            .keylen = htons(keylen),
            .extlen = 0,
            .datatype = 0,
            .reserved = htons(0),
            .bodylen = htonl(keylen),
            .opaque = 0xdeadbeef,
            .cas = 0
        };
    static vector<net_axis<DATA_WIDTH>> words;

    switch(clientState){
        case 0: {
            cout << "client0" << endl;
            appNotification notific(clientSessionID, 24+keylen, 0, 0, false);
            notifications.write(notific);
            clientState = 1;
            break;
        }
        case 1: {
            cout << "client1" << endl;
            // client writing memcached request to mcrouter. 
            if(!readRequest.empty()){
                cout << "client11" << endl;
                appReadRequest appReq = readRequest.read();
                cout << "client sends data: session " << appReq.sessionID << ", length " << appReq.length << endl;

                rxMetaData.write(clientSessionID);            
                words.clear();
                
                uint8_t* data = new uint8_t[24+keylen];
                memcpy(data, req.bytes, 24);
                memcpy(data+24, key, keylen);
                byteArray2axisWord(data, 24+keylen, words);
                cout << "words.size() = " << words.size() << endl;

                for(int i = 0; i < words.size(); i++){
                    rxData.write(words[i]);
                }
            }
            // client receives final response from mcrouter. 
            else if(!txMetaData.empty()){
                cout << "client12" << endl;
                appTxMeta txMeta = txMetaData.read();
                if(txMeta.sessionID == clientSessionID){
                    uint32_t length = txMeta.length;
                    cout << "client receives data: " << dec << length << endl;

                    txStatus.write(appTxRsp(clientSessionID, length, 64<<10, 0));

                    words.clear();
                    clientState = 2;
                }
            }
            break;
        }
        case 2: {
            cout << "client2" << endl;
            if(!txData.empty()){
                net_axis<DATA_WIDTH> currWord = txData.read();
                words.push_back(currWord);
                if(currWord.last){
                    uint8_t* data = new uint8_t[64 * words.size()];
                    uint32_t len = 0;
                    axisWord2byteArray(data, len, words);
                    cout << "client2 len: " << len << endl;
                    
                    displayMsg(data, len);
                    clientState = 1;
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

    static char* val = "world";
    static uint16_t vallen = strlen(val);
    
    static protocol_binary_response_header rsp;
    rsp.response = {
            .magic = PROTOCOL_BINARY_REQ, 
            .opcode = PROTOCOL_BINARY_CMD_RGET, 
            .keylen = htons(0),
            .extlen = 0,
            .datatype = 0,
            .status = htons(0),
            .bodylen = htonl(vallen),
            .opaque = 0xdeadbeef,
            .cas = 0
        };
    
    static vector<net_axis<DATA_WIDTH>> words;
    
    switch(mcState){
        case 0: {
            cout << "memcached0" << endl;
            if(!txMetaData.empty()){
                cout << "memcached01" << endl;
                appTxMeta txMeta = txMetaData.read();
                if(txMeta.sessionID == memcachedSessionID){
                    uint32_t length = txMeta.length;
                    cout << "memcached receives data: " << dec << length << endl;
                    txStatus.write(appTxRsp(memcachedSessionID, length, 64<<10, 0));
                    mcState = 1;
                    words.clear();
                }
            }
            else if(!readRequest.empty()){
                cout << "memcached02" << endl;
                appReadRequest appReq = readRequest.read();
                cout << "memcached sends data: session " << dec << appReq.sessionID << ", length" << appReq.length << endl;

                rxMetaData.write(memcachedSessionID);
            
                words.clear();
                
                uint8_t* data = new uint8_t[24+vallen];
                memcpy(data, rsp.bytes, 24);
                memcpy(data+24, val, vallen);
                byteArray2axisWord(data, 24+vallen, words);

                for(int i = 0; i < words.size(); i++){
                    rxData.write(words[i]);
                }
            }
            break;
        }
        case 1: {
            cout << "memcached1" << endl;
            if(!txData.empty()){
                net_axis<DATA_WIDTH> currWord = txData.read();
                words.push_back(currWord);
                if(currWord.last){
                    uint8_t* data = new uint8_t[64 * words.size()];
                    uint32_t len = 0;
                    axisWord2byteArray(data, len, words);
                    cout << "memcached1 len: " << len << endl;
                    
                    appNotification notific(memcachedSessionID, 24+vallen, 0, 0, false);
                    notifications.write(notific);
                    
                    displayMsg(data, len);
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
                cout << "txMeta.sessionID " << txMeta.sessionID << endl;
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


	int count = 0;
	int portOpened = -1;

    static ap_uint<4> tbState = 0;
   
	while (count < 50)
	{
        mcrouter(listenPort, listenPortStatus, notifications, readRequest,
		    rxMetaData, rxData, openTuples, openConnection, openConStatus,
			closeConnection, txMetaData, txData, txStatus);
        client(notifications1, readRequest1, rxMetaData1, rxData1, txMetaData1, txData1, txStatus1);
        memcached(notifications2, readRequest2, rxMetaData2, rxData2, txMetaData2, txData2, txStatus2);
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
   
		count++;
	}
	return portOpened;
}
