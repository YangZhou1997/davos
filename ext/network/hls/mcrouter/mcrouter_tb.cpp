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
    switch(req.request.opcode){
        case PROTOCOL_BINARY_CMD_GET: {
            cout << "GET request: keylen " << req.request.keylen << endl;
            
            char* key = new char[req.request.keylen + 1];
            memcpy(key, data+24, req.request.keylen);
            key[req.request.keylen] = '\0';
            cout << "key: " << key << endl;
            break;
        }
        case PROTOCOL_BINARY_CMD_SET: {
            uint32_t vallen = req.request.bodylen - req.request.extlen - req.request.keylen;
            cout << "SET request: keylen " << req.request.keylen << " vallen " << vallen << endl;
            
            char* key = new char[req.request.keylen + 1];
            memcpy(key, data+24, req.request.keylen);
            key[req.request.keylen] = '\0';
            cout << "key: " << key << endl;

            char* val = new char[vallen + 1];
            memcpy(val, data+24+req.request.keylen, vallen);
            val[vallen] = '\0';
            cout << "val: " << val << endl;
            break;
        }
        case PROTOCOL_BINARY_CMD_RGET: {
            uint32_t vallen = req.request.bodylen - req.request.extlen - req.request.keylen;
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

uint32_t clientSessionID = 32245;
uint32_t memcachedSessionID = 41235;

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
    char* key = "hello";
    uint32_t keylen = strlen(key); 
    
    protocol_binary_request_header req; 
    req.request = {
            .magic = PROTOCOL_BINARY_REQ, 
            .opcode = PROTOCOL_BINARY_CMD_GET, 
            .keylen = keylen,
            .extlen = 0,
            .datatype = 0,
            .reserved = 0,
            .bodylen = 24 + keylen,
            .opaque = 0xdeadbeef,
            .cas = 0
        };
    
    appNotification notific(clientSessionID, 24+keylen, 0, 0, false);
    notifications.write(notific);

    switch(clientState){
        case 0: {
            // client writing memcached request to mcrouter. 
            if(!readRequest.empty()){
                appReadRequest appReq = readRequest.read();
                cout << "client sends data: session " << appReq.sessionID << ", length" << appReq.length << endl;

                rxMetaData.write(clientSessionID);
            
                vector<net_axis<DATA_WIDTH>> words;
                words.clear();
                
                uint8_t* data = new uint8_t[24+keylen];
                memcpy(data, req.bytes, 24);
                memcpy(data+24, key, keylen);
                byteArray2axisWord(data, 24+keylen, words);

                for(int i = 0; i < words.size(); i++){
                    rxData.write(words[i]);
                }
                clientState = 1;
            }
            break;
        }
        case 1: {
            // client receives final response from mcrouter. 
            if(!txMetaData.empty()){
                appTxMeta txMeta = txMetaData.read();
                if(txMeta.sessionID == clientSessionID){
                    uint32_t length = txMeta.length;
                    cout << "client receives data: " << dec << length << endl;

                    txStatus.write(appTxRsp(clientSessionID, length, 64<<10, 0));

                    vector<net_axis<DATA_WIDTH>> words;
                    words.clear();

                    while(true){
                        net_axis<DATA_WIDTH> currWord = txData.read();
                        words.push_back(currWord);
                        if(currWord.last){
                            uint8_t* data = new uint8_t[64 * words.size()];
                            uint32_t len = 0;
                            byteArray2axisWord(data, len, words);
                            
                            displayMsg(data, len);
                            break;
                        }
                    }
                }
                clientState = 0;
            }
            break;
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

    char* val = "world";
    uint32_t vallen = strlen(val);
    
    protocol_binary_response_header rsp;
    rsp.response = {
            .magic = PROTOCOL_BINARY_REQ, 
            .opcode = PROTOCOL_BINARY_CMD_RGET, 
            .keylen = 0,
            .extlen = 0,
            .datatype = 0,
            .status = 0,
            .bodylen = 24 + strlen(val),
            .opaque = 0xdeadbeef,
            .cas = 0
        };

    switch (mcState){
        case 0: {
            if(!txMetaData.empty()){
                appTxMeta txMeta = txMetaData.read();
                if(txMeta.sessionID == memcachedSessionID){
                    uint32_t length = txMeta.length;
                    cout << "memcached receives data: " << dec << length << endl;

                    txStatus.write(appTxRsp(memcachedSessionID, length, 64<<10, 0));

                    vector<net_axis<DATA_WIDTH>> words;
                    words.clear();

                    while(true){
                        net_axis<DATA_WIDTH> currWord = txData.read();
                        words.push_back(currWord);
                        if(currWord.last){
                            uint8_t* data = new uint8_t[64 * words.size()];
                            uint32_t len = 0;
                            byteArray2axisWord(data, len, words);
                            
                            displayMsg(data, len);
                            break;
                        }
                    }

                    appNotification notific(clientSessionID, 24+vallen, 0, 0, false);
                    notifications.write(notific);
                    mcState = 1;
                }
            }
            break;
        }
        case 1: {
            if(!readRequest.empty()){
                appReadRequest appReq = readRequest.read();
                cout << "memcached sends data: session " << appReq.sessionID << ", length" << appReq.length << endl;

                rxMetaData.write(clientSessionID);
            
                vector<net_axis<DATA_WIDTH>> words;
                words.clear();
                
                uint8_t* data = new uint8_t[24+vallen];
                memcpy(data, rsp.bytes, 24);
                memcpy(data+24, val, vallen);
                byteArray2axisWord(data, 24+vallen, words);

                for(int i = 0; i < words.size(); i++){
                    rxData.write(words[i]);
                }
                mcState = 0;
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
    else if(!in1.empty()){
        T tmp = in2.read();
        out.write(tmp);
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
	while (count < 50)
	{
		mcrouter(listenPort, listenPortStatus,
					notifications, readRequest,
					rxMetaData, rxData,
					openTuples, openConnection, openConStatus,
					closeConnection,
					txMetaData, txData,
					txStatus);
        client(notifications1, readRequest1, rxMetaData1, rxData1, txMetaData1, txData1, txStatus1);
        memcached(notifications2, readRequest2, rxMetaData2, rxData2, txMetaData2, txData2, txStatus2);

        // let mcrouter try to connect remote memcached; 
	    ipTuple tuple;
        tuple.ip_address = 0x0a010101;
		tuple.ip_port = 0x3412 + count;
        openTuples.write(tuple);
        
        if (!openConnection.empty()){
            ipTuple tuple = openConnection.read();
		    cout << "mcrouter connecting to: " << hex << tuple.ip_address << ":" << dec << tuple.ip_port << endl;
        }
        openConStatus.write(openStatus(memcachedSessionID, true));

        // let mcrouter listen on port 
		if (!listenPort.empty())
		{
			ap_uint<16> port = listenPort.read();
			cout << "mcrouter listens on port: " << dec << port << endl;
			listenPortStatus.write(true);
			portOpened = 0;
		}

        if(!txMetaData.empty()){
            appTxMeta txMeta = txMetaData.read();
            if(txMeta.sessionID == clientSessionID){
                txMetaData1.write(txMeta);
                while(true){
                    net_axis<DATA_WIDTH> currWord = txData.read();
                    txData1.write(currWord);
                    if(currWord.last){
                        break;
                    }
                }
            }
            else if(txMeta.sessionID == memcachedSessionID){
                txMetaData2.write(txMeta);
                while(true){
                    net_axis<DATA_WIDTH> currWord = txData.read();
                    txData2.write(currWord);
                    if(currWord.last){
                        break;
                    }
                }
            }
            else{
                cout << "txMetaData unknown sessionID: " << txMeta.sessionID << endl;
            }
        }
        if(!readRequest.empty()){
            appReadRequest appReq = readRequest.read();
            if(appReq.sessionID == clientSessionID){
                readRequest1.write(appReq);
            }
            else if(appReq.sessionID == memcachedSessionID){
                readRequest2.write(appReq);
            }
            else{
                cout << "readRequest unknown sessionID: " << appReq.sessionID << endl;
            }
        }
        mux(notifications1, notifications2, notifications);
        mux(rxMetaData1, rxMetaData2, rxMetaData);
        mux(rxData1, rxData2, rxData);
        mux(txStatus1, txStatus2, txStatus);

		count++;
	}
	return portOpened;
}
