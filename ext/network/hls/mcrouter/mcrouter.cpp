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

void open_port(	
    hls::stream<ap_uint<16> >&		listenPort,
	hls::stream<bool>&				listenSts
){
#pragma HLS PIPELINE II=1
#pragma HLS INLINE off

	static ap_uint<2> state = 0;
	#pragma HLS reset variable=state

	bool listenDone = false;

	switch (state){
    	case 0:
            // mcrouter uses 5001 port
    		listenPort.write(5001);
    		state = 1;
    		break;
    	case 1:
    		if (!listenSts.empty()){
    			listenSts.read(listenDone);
    			if (listenDone){
    				state = 2;
    			}
    			else{
    				state = 0;
    			}
    		}
    		break;
    	case 2:
    		//IDLE
    		break;
	}//switch
}

void connect_memcached(
    hls::stream<ipTuple>& openTuples, //input
    hls::stream<openStatus>& openConStatus, //input
    hls::stream<ipTuple>& openConnection, //output 
    // you should separate the following fifos, as their rspFifo are not handled by a single function. 
    hls::stream<ap_uint<2> >& cmdFifo, //input
    hls::stream<ap_uint<16> >& sessionCountFifo, //output
    hls::stream<ap_uint<32> >& hashValFifo, //input
    hls::stream<ap_uint<16> >& sessionIdFifo, //output
    hls::stream<ap_uint<16> >& idxFifo, //input
	hls::stream<ap_uint<16> >& sessionIdFifo3, //output
	hls::stream<ap_uint<16> >& closedSessionIdFifo //input
){
#pragma HLS PIPELINE II=1
#pragma HLS INLINE off

static ap_uint<16> connectedSessions[MAX_CONNECTED_SESSIONS];
static ap_uint<1> connectedSessionsSts[MAX_CONNECTED_SESSIONS];
#pragma HLS RESOURCE variable=connectedSessions core=RAM_T2P_BRAM
#pragma HLS ARRAY_PARTITION variable=connectedSessions complete
#pragma HLS DEPENDENCE variable=connectedSessions inter false
#pragma HLS RESOURCE variable=connectedSessionsSts core=RAM_T2P_BRAM
#pragma HLS ARRAY_PARTITION variable=connectedSessionsSts complete
#pragma HLS DEPENDENCE variable=connectedSessionsSts inter false

static ap_uint<16> sessionCount = 0;

    if (!openTuples.empty()){
        ipTuple tuple = openTuples.read();
		openConnection.write(tuple);
    }
	else if (!openConStatus.empty())
	{
		openStatus newConn = openConStatus.read();
		if (newConn.success)
		{
            connectedSessions[sessionCount] = newConn.sessionID;
            connectedSessionsSts[sessionCount] = 1;
            sessionCount += 1;
		}
	}
    // you should use "else if"; 
    // if you just use "if", hls will think of you want to parallel the five block
    //, and check the data dependency, and reduce the II. 
    else if(!cmdFifo.empty()){
        ap_uint<2> cmd = cmdFifo.read();
        switch(cmd) {
            case 0:{
                sessionCountFifo.write(sessionCount);
                break;
            }
        }
    }
    else if(!hashValFifo.empty()){
        ap_uint<32> hashVal = hashValFifo.read();
        // TODO: handling closed connections, or in policy engine
        ap_uint<16> idx = (hashVal&(MAX_CONNECTED_SESSIONS-1)) % sessionCount;
        sessionIdFifo.write(connectedSessions[idx]);
    }
    else if(!idxFifo.empty()){
        ap_uint<16> idx = idxFifo.read();
        // TODO: handling closed connections, or in policy engine
        sessionIdFifo3.write(connectedSessions[idx]);
    }
    // if(!closedSessionIdFifo.empty()){
    //     ap_uint<16> closedSessionID = closedSessionIdFifo.read();
    //     for(int i = 0; i < MAX_CONNECTED_SESSIONS; i++){
    //         #pragma HLS unroll
    //         if(connectedSessions[i] == closedSessionID){
    //             connectedSessionsSts[i] = 0;
    //             break;
    //         }
    //     }
    // }
}

void notification_handler(	
    hls::stream<appNotification>&	notific, // input from toe
	hls::stream<appReadRequest>&	readReq,  // output to toe
	hls::stream<ap_uint<16> >& closedSessionIdFifo //output
){
#pragma HLS PIPELINE II=1
#pragma HLS INLINE off

	appNotification notification;

	// Receive notifications, about new data which is available
	if (!notific.empty())
	{
		notific.read(notification);
		// std::cout << notification.ipAddress << "\t" << notification.dstPort << std::endl;
		if (notification.length != 0)
		{
            std::cout << "notification: sessionID " <<  notification.sessionID << " len " << notification.length << std::endl;
            // requesting the data from this session
			readReq.write(appReadRequest(notification.sessionID, notification.length));
		}
        // else if(notification.closed) {
        //     closedSessionIdFifo.write(notification.sessionID);
        // }
	}
}

#define MAX_SESSION_NUM ((1 << 16) - 1)

void parsingMsgBody(sessionState& currSessionState, msgBody& currMsgBody, msgHeader& currMsgHeader, \
    net_axis<DATA_WIDTH>& currWord, ap_uint<16>& currValidLen, ap_uint<16>& initValidLen, hls::stream<msgBody>& msgBodyFifo){

#pragma HLS INLINE

    ap_uint<16> wordPos = (initValidLen - currValidLen)*8;

    switch(currSessionState.parsingBody){
        case 0: 
            if(currMsgHeader.extLen != 0){
                ap_uint<16> currExtLen = currSessionState.currExtLen;
                ap_uint<16> requiredExtLen = currMsgHeader.extLen - currExtLen;
                if(currValidLen >= requiredExtLen){
                    currMsgBody.ext(KV_MAX_EXT_SIZE-1-currExtLen*8, KV_MAX_EXT_SIZE-currExtLen*8-requiredExtLen*8) = currWord.data(DATA_WIDTH-1-wordPos, DATA_WIDTH-requiredExtLen*8-wordPos);
                    currValidLen -= requiredExtLen;
                    currSessionState.currLen += requiredExtLen;
                    currSessionState.currExtLen = 0;
                    currSessionState.parsingBody = 1; // ext is parsed
                }
                else{
                    currMsgBody.ext(KV_MAX_EXT_SIZE-1-currExtLen*8, KV_MAX_EXT_SIZE-currExtLen*8-currValidLen*8) = currWord.data(DATA_WIDTH-1-wordPos, DATA_WIDTH-currValidLen*8-wordPos);
                    currSessionState.currExtLen += currValidLen;
                    currSessionState.currLen += currValidLen;
                    currValidLen = 0;
                }
            }
            else{
                currSessionState.parsingBody = 1;
            }
            break;
        case 1:
            if(currMsgHeader.keyLen != 0){
                ap_uint<16> currKeyLen = currSessionState.currKeyLen;
                std::cout << currMsgHeader.keyLen << " " << currKeyLen << " wordPos " << wordPos << std::endl;
                ap_uint<16> requiredKeyLen = currMsgHeader.keyLen - currKeyLen;
                if(currValidLen >= requiredKeyLen){
                    currMsgBody.key(KV_MAX_KEY_SIZE-1-currKeyLen*8, KV_MAX_KEY_SIZE-currKeyLen*8-requiredKeyLen*8) = currWord.data(DATA_WIDTH-1-wordPos, DATA_WIDTH-requiredKeyLen*8-wordPos);
                    std::cout << "currWord.data(DATA_WIDTH-1-wordPos, DATA_WIDTH-8-wordPos) " 
                        << char(currWord.data(DATA_WIDTH-1-wordPos, DATA_WIDTH-8-wordPos)) << std::endl;
                    currValidLen -= requiredKeyLen;
                    currSessionState.currLen += requiredKeyLen;
                    currSessionState.currKeyLen = 0;
                    currSessionState.parsingBody = 2; // key is parsed
                }
                else{
                    currMsgBody.key(KV_MAX_KEY_SIZE-1-currKeyLen*8, KV_MAX_KEY_SIZE-currKeyLen*8-currValidLen*8) = currWord.data(DATA_WIDTH-1-wordPos, DATA_WIDTH-currValidLen*8-wordPos);
                    currSessionState.currKeyLen += currValidLen;
                    currSessionState.currLen += currValidLen;
                    currValidLen = 0;
                }
            }
            else{
                currSessionState.parsingBody = 2;
            }
            break;
        case 2: 
            if(currMsgHeader.val_len() != 0){
                ap_uint<16> currValLen = currSessionState.currValLen;
                std::cout << currMsgHeader.val_len() << " " << currValLen << " wordPos " << wordPos << std::endl;
                ap_uint<16> requiredValLen = currMsgHeader.val_len() - currValLen;
                if(currValidLen >= requiredValLen){
                    currMsgBody.val(KV_MAX_VAL_SIZE-1-currValLen*8, KV_MAX_VAL_SIZE-currValLen*8-requiredValLen*8) = currWord.data(DATA_WIDTH-1-wordPos, DATA_WIDTH-requiredValLen*8-wordPos);
                    std::cout << "currWord.data(DATA_WIDTH-1-wordPos, DATA_WIDTH-8-wordPos) " 
                        << char(currWord.data(DATA_WIDTH-1-wordPos, DATA_WIDTH-8-wordPos)) << std::endl;
                    currValidLen -= requiredValLen;
                    currSessionState.currLen += requiredValLen;
                    currSessionState.currValLen = 0;
                    currSessionState.parsingBody = 0; // val is parsed
                }
                else{
                    currMsgBody.val(KV_MAX_VAL_SIZE-1-currValLen*8, KV_MAX_VAL_SIZE-currValLen*8-currValidLen*8) = currWord.data(DATA_WIDTH-1-wordPos, DATA_WIDTH-currValidLen*8-wordPos);
                    currSessionState.currValLen += currValidLen;
                    currSessionState.currLen += currValidLen;
                    currValidLen = 0;
                }
            }
            else{
                currSessionState.parsingBody = 0;
            }
            break;
        case 3: 
            std::cerr << "error" << std::endl;
    }
}

void extract_msg(
    hls::stream<ap_uint<16> >& rxMetaData, // input
	hls::stream<net_axis<DATA_WIDTH> >& rxData, // intput 
    hls::stream<ap_uint<16> >& sessionIdFifo, // output
    hls::stream<msgHeader>& msgHeaderFifo, // output
    hls::stream<msgBody>& msgBodyFifo, // output
    hls::stream<hash_table_16_1024::htLookupReq<16> >&       s_axis_lup_req, 
    hls::stream<hash_table_16_1024::htUpdateReq<16,1024> >&    s_axis_upd_req, 
    hls::stream<hash_table_16_1024::htLookupResp<16,1024> >&   m_axis_lup_rsp
){
#pragma HLS PIPELINE II=3
#pragma HLS INLINE off
    // generating globally unique msgID. 
    static ap_uint<32> currentMsgID = 0;

    // static value gets inited to zero by default. 
    static sessionState sessionStateTable[MAX_SESSION_NUM];
    #pragma HLS RESOURCE variable=sessionStateTable core=RAM_T2P_BRAM
    #pragma HLS DEPENDENCE variable=sessionStateTable inter false

    enum axisFsmType {IDLE, RECOVER_STATE, READ_WORD, PARSE_WORD};
    char* axisFsmName[4] = {"IDLE", "RECOVER_STATE", "READ_WORD", "PARSE_WORD"};
    static axisFsmType currAxisState = IDLE;

	static ap_uint<16>      currSessionID; // valid when currAxisState becomes on-going
    static sessionState     currSessionState; // valid when currAxisState becomes on-going
    static msgBody          currMsgBody;
    static msgHeader        currMsgHeader;
	static net_axis<DATA_WIDTH> currWord;
    static ap_uint<16>       currValidLen;
    static ap_uint<16>       initValidLen;
    static ap_uint<16>       totalLen;
    
    std::cout << "mcrouter::extract_msg fsmState " << axisFsmName[(int)currAxisState] << std::endl;
    switch (currAxisState){
        case IDLE: { // a new AXIS transaction 
            if(!rxMetaData.empty()){
                rxMetaData.read(currSessionID);
                std::cout << "mcrouter::extract_msg rxMetaData.sessionID " << currSessionID << std::endl;
                currSessionState = sessionStateTable[currSessionID];
                currAxisState = READ_WORD;

                s_axis_lup_req.write(hash_table_16_1024::htLookupReq<16>(currSessionID, 0));
                currAxisState = RECOVER_STATE;
            }
            break;
        }
        case RECOVER_STATE: {
            if(!m_axis_lup_rsp.empty()){
                hash_table_16_1024::htLookupResp<16, 1024> response = m_axis_lup_rsp.read();
                if(response.hit){
                    currMsgBody.consume_word(response.value);
                }
                else{
                    s_axis_upd_req.write(hash_table_16_1024::htUpdateReq<16, 1024>(hash_table_16_1024::KV_INSERT, currSessionID, currMsgBody.output_word(), 0));
                }
                // std::cout << response.hit << std::endl;
                // we should expect the hash table is enough to handle all active connections; 
                currAxisState = READ_WORD;
            }
            break;
        }
        case READ_WORD: { // continue the AXIS transaction
            if(!rxData.empty()){
                rxData.read(currWord);
                currValidLen = keepToLen(currWord.keep);
                initValidLen = currValidLen;
                std::cout << "currValidLen=" << currValidLen << std::endl;
                currAxisState = PARSE_WORD;
            }
            break;
        }
        // consuming currWord 
        case PARSE_WORD: {
            ap_uint<16> wordPos = (initValidLen - currValidLen)*8;
            if(currValidLen > 0){
                if(currSessionState.parsingHeader == 0){
                    ap_uint<5> requiredHeaderLen = 24 - currSessionState.currHdrLen;
                    if(currValidLen >= requiredHeaderLen){
                        currSessionState.msgHeaderBuff(requiredHeaderLen*8-1, 0) = currWord.data(DATA_WIDTH-1-wordPos, DATA_WIDTH-requiredHeaderLen*8-wordPos);
                        currMsgHeader.consume_word(currSessionState.msgHeaderBuff);
                        
                        std::cout << "extract_msg receives data" << std::endl;
                        currMsgHeader.display();

                        msgHeaderFifo.write(currMsgHeader);
                        sessionIdFifo.write(currSessionID);
                        currValidLen -= requiredHeaderLen;
                        currSessionState.currHdrLen = 0;
                        currSessionState.currLen += requiredHeaderLen;
                        currSessionState.parsingHeader = 1; // header is parsed. 

                        totalLen = currMsgHeader.bodyLen + 24;

                        parsingMsgBody(currSessionState, currMsgBody, currMsgHeader, currWord, currValidLen, initValidLen, msgBodyFifo);
                    }
                    else{
                        currSessionState.msgHeaderBuff(requiredHeaderLen*8-1, requiredHeaderLen*8-currValidLen*8) = currWord.data(DATA_WIDTH-1-wordPos, DATA_WIDTH-currValidLen*8-wordPos);
                        currSessionState.currHdrLen += currValidLen;
                        currSessionState.currLen += currValidLen;
                        currValidLen = 0;
                    }
                }
                else{
                    currMsgHeader.consume_word(currSessionState.msgHeaderBuff);
                    parsingMsgBody(currSessionState, currMsgBody, currMsgHeader, currWord, currValidLen, initValidLen, msgBodyFifo);
                }
            }
            std::cout << "currValidLen=" << currValidLen << std::endl;

            if(currSessionState.currLen == totalLen){
                currMsgBody.msgID = currentMsgID;
                currentMsgID += 1;
                currSessionState.reset(); // ready to parse next message
                std::cout << "writing currMsgBody for currMsgBody.msgID " << currMsgBody.msgID << std::endl;
                msgBodyFifo.write(currMsgBody);
                // removing currMsgBody from the hash table. 
                s_axis_upd_req.write(hash_table_16_1024::htUpdateReq<16, 1024>(hash_table_16_1024::KV_DELETE, currSessionID, currMsgBody.output_word(), 0));
            }
            
            // check whether we have consumed current word. 
            currAxisState = currValidLen > 0 ? PARSE_WORD : READ_WORD;

            // check whether this is the last word of this ASIX transaction. 
            if(currWord.last == 1 && currValidLen == 0){ // the AXIS transaction ends
                currAxisState = IDLE;
                // In the end of each AXIS, store sessionState and currMsgBody back. 
                // msgHeader already written out or can be recovered from sessionStateTable. 
                sessionStateTable[currSessionID] = currSessionState;
                s_axis_upd_req.write(hash_table_16_1024::htUpdateReq<16, 1024>(hash_table_16_1024::KV_UPDATE, currSessionID, currMsgBody.output_word(), 0));
                // TODO: optimization: implementing hash table with a stash, such that read and write can finish in one cycle. 
                // TODO: or store currMsgBody in URAM. 
            }
            break;
        }
    }
}

void simple_hash(
    hls::stream<ap_uint<KV_MAX_KEY_SIZE> >& keyFifo, 
    hls::stream<ap_uint<32> >& hashFifo
){
#pragma HLS PIPELINE II=1
#pragma HLS INLINE off

    ap_uint<KV_MAX_KEY_SIZE> key;
    if(!keyFifo.empty()){
        keyFifo.read(key);

        ap_uint<32> res = 0;
        for(int i = 0; i < KV_MAX_KEY_SIZE/32; i++){
            #pragma HLS unroll
            res ^= key(KV_MAX_KEY_SIZE-i*32-1, KV_MAX_KEY_SIZE-i*32-32);
        }
        hashFifo.write(res);
    }
}

void proxy(
    hls::stream<ap_uint<16> >&              sessionIdFifo, // input
    hls::stream<msgHeader>&                 msgHeaderFifo, // input
    hls::stream<msgBody>&                   msgBodyFifo, // input
    hls::stream<ap_uint<16> >&              sessionIdFifo_dst, // output
    hls::stream<msgHeader>&                 msgHeaderFifo_dst, // output
    hls::stream<msgBody>&                   msgBodyFifo_dst, // output
    hls::stream<ap_uint<KV_MAX_KEY_SIZE> >& keyFifo, 
    hls::stream<ap_uint<32> >&              hashFifo,
    hls::stream<mqInsertReq<ap_uint<32>> >& multiQueue_push, 
    hls::stream<mqPopReq>&				    multiQueue_pop_req, 
    hls::stream<ap_uint<32>>&				multiQueue_rsp,
    hls::stream<hash_table_32_32::htLookupReq<32> >&          s_axis_lup_req, 
    hls::stream<hash_table_32_32::htUpdateReq<32,32> >&       s_axis_upd_req, 
    hls::stream<hash_table_32_32::htLookupResp<32,32> >&      m_axis_lup_rsp,
    hls::stream<hash_table_32_32::htUpdateResp<32,32> >&      m_axis_upd_rsp,
    hls::stream<ap_uint<2> >& cmdFifo,
    hls::stream<ap_uint<16> >& sessionCountFifo, 
    hls::stream<ap_uint<32> >& mc_hashValFifo,
    hls::stream<ap_uint<16> >& sessionIdFifo2,
    hls::stream<ap_uint<16> >& idxFifo,
	hls::stream<ap_uint<16> >& sessionIdFifo3
){
#pragma HLS PIPELINE II=1
#pragma HLS INLINE off

	static ap_uint<16> currSessionID;
    static msgHeader currMsgHeader;
    static msgBody currMsgBody;
    static ap_uint<16> currSessionID_dst;
    static ap_uint<16> currSessionCount;
    static ap_uint<16> currSessionCount_idx;
    
    enum proxy_fsmType {IDLE=0, GET_HASH, GET_DEST, GET_WRITEOUT, SET_RANGE, SET_CHECKHT, SET_WRITEOUT, RSP_MQ, RSP_HT, RSP_CHECKHT};
    char * proxy_fsmName[10] = {"IDLE", "GET_HASH", "GET_DEST", "GET_WRITEOUT", "SET_RANGE", "SET_CHECKHT", "SET_WRITEOUT", "RSP_MQ", "RSP_HT", "RSP_CHECKHT"};

    static proxy_fsmType proxyFsmState = IDLE;

    std::cout << "mcrouter::proxy fsmState " << proxy_fsmName[(int)proxyFsmState] << std::endl;
    switch (proxyFsmState){
        case IDLE:{
            if(!sessionIdFifo.empty() && !msgHeaderFifo.empty() && !msgBodyFifo.empty()){
                sessionIdFifo.read(currSessionID);
                msgHeaderFifo.read(currMsgHeader);
                msgBodyFifo.read(currMsgBody);
                switch(currMsgHeader.opcode){
                    case PROTOCOL_BINARY_CMD_GET: { // GET -> picks one memcached based on key hashing. 
                        keyFifo.write(currMsgBody.key); // calculating hashing
                        proxyFsmState = GET_HASH;
                        break;
                    }
                    case PROTOCOL_BINARY_CMD_SET: { // SET -> sets all memcached
                        cmdFifo.write(0); // get current total session count
                        proxyFsmState = SET_RANGE;
                        break;
                    }
                    case PROTOCOL_BINARY_CMD_RGET: { // RGET
        				multiQueue_pop_req.write(mqPopReq(POP, currSessionID)); // get request msg ID;
                        proxyFsmState = RSP_MQ;
                        break;
                    }
                    case PROTOCOL_BINARY_CMD_RSET: { // RSet
        				multiQueue_pop_req.write(mqPopReq(POP, currSessionID));
                        proxyFsmState = RSP_MQ;
                        break;
                    }
                }
            }
            break;
        }
        case GET_HASH:{
            if(!hashFifo.empty()){
                ap_uint<32> hashVal = hashFifo.read();
                mc_hashValFifo.write(hashVal);
                proxyFsmState = GET_DEST;
            }
            break;
        }
        case GET_DEST:{
            if(!sessionIdFifo2.empty()){
                currSessionID_dst = sessionIdFifo2.read();
                mqInsertReq<ap_uint<32> > insertReq(currSessionID_dst, currMsgBody.msgID);
        		multiQueue_push.write(insertReq);
                msgContext srcMsgContext(1, currSessionID);

                std::cout << "GET_DEST currMsgBody.msgID " << currMsgBody.msgID << std::endl;
                s_axis_upd_req.write(hash_table_32_32::htUpdateReq<32, 32>(hash_table_32_32::KV_INSERT, currMsgBody.msgID, srcMsgContext.output_word(), 0));
                proxyFsmState = GET_WRITEOUT;
            }
            break;
        }
        case GET_WRITEOUT:{
            if(!m_axis_upd_rsp.empty()){
                hash_table_32_32::htUpdateResp<32,32> response = m_axis_upd_rsp.read();
                if (response.success){
                    sessionIdFifo_dst.write(currSessionID_dst);
                    msgHeaderFifo_dst.write(currMsgHeader);
                    msgBodyFifo_dst.write(currMsgBody);
                    proxyFsmState = IDLE;
                }
                else{
                    std::cout << "response.key " << response.key << std::endl;
                    std::cout << "response.op " << response.op << std::endl;
                    std::cerr << "[ERROR] proxy GET HT insert failed" << std::endl;
                    proxyFsmState = IDLE;
                }
            }
            break;
        }
        case SET_RANGE:{
            if(!sessionCountFifo.empty()){
                currSessionCount = sessionCountFifo.read();
                currSessionCount_idx = 0;

                // update msg context table
                msgContext srcMsgContext(currSessionCount, currSessionID);
                s_axis_upd_req.write(hash_table_32_32::htUpdateReq<32, 32>(hash_table_32_32::KV_INSERT, currMsgBody.msgID, srcMsgContext.output_word(), 0));
                proxyFsmState = SET_CHECKHT;
            }
            break;
        }
        case SET_CHECKHT: {
            if(!m_axis_upd_rsp.empty()){
                hash_table_32_32::htUpdateResp<32,32> response = m_axis_upd_rsp.read();
                if (response.success){
                    idxFifo.write(currSessionCount_idx);
                    proxyFsmState = SET_WRITEOUT;
                }
                else{
                    std::cerr << "[ERROR] proxy GET HT insert failed" << std::endl;
                    proxyFsmState = IDLE;
                }
            }
            break;
        }
        case SET_WRITEOUT:{
            if(!sessionIdFifo3.empty()){
                ap_uint<16> sessionID_dst = sessionIdFifo3.read();
                mqInsertReq<ap_uint<32> > insertReq(sessionID_dst, currMsgBody.msgID);
        		multiQueue_push.write(insertReq);
                
                sessionIdFifo_dst.write(sessionID_dst);
                msgHeaderFifo_dst.write(currMsgHeader);
                msgBodyFifo_dst.write(currMsgBody);  
                currSessionCount_idx += 1;
                if(currSessionCount_idx < currSessionCount){
                    idxFifo.write(currSessionCount_idx);
                    proxyFsmState = SET_WRITEOUT;
                }
                else{
                    proxyFsmState = IDLE;
                }
            }
            break;
        }
        case RSP_MQ:{
            if(!multiQueue_rsp.empty())
        	{
        		ap_uint<32> srcMsgID = multiQueue_rsp.read();
                s_axis_lup_req.write(hash_table_32_32::htLookupReq<32>(srcMsgID, 0));
                proxyFsmState = RSP_HT;
            }
            break;
        }
        case RSP_HT:{
            if(!m_axis_lup_rsp.empty()){
                hash_table_32_32::htLookupResp<32, 32> response = m_axis_lup_rsp.read();
                msgContext srcMsgContext;
                srcMsgContext.consume_word(response.value);

                ap_uint<16> numRsp = srcMsgContext.numRsp;
                static ap_uint<16> srcSessionID = srcMsgContext.srcSessionID;
                ap_uint<32> srcMsgID = response.key;
                
                if(numRsp == 1){
                    srcMsgContext = msgContext(numRsp-1, srcSessionID);
                    s_axis_upd_req.write(hash_table_32_32::htUpdateReq<32, 32>(hash_table_32_32::KV_DELETE, srcMsgID, srcMsgContext.output_word(), 0));
                    sessionIdFifo_dst.write(srcSessionID);
                    msgHeaderFifo_dst.write(currMsgHeader);
                    msgBodyFifo_dst.write(currMsgBody);
                    proxyFsmState = RSP_CHECKHT;
                }
                else{
                    // update hash table numRsp; 
                    srcMsgContext = msgContext(numRsp-1, srcSessionID);
                    s_axis_upd_req.write(hash_table_32_32::htUpdateReq<32, 32>(hash_table_32_32::KV_UPDATE, srcMsgID, srcMsgContext.output_word(), 0));
                    proxyFsmState = RSP_CHECKHT;
                }
            }   
            break;
        }
        case RSP_CHECKHT: {
            if(!m_axis_upd_rsp.empty()){
                hash_table_32_32::htUpdateResp<32,32> response = m_axis_upd_rsp.read();
                if (response.success){
                    proxyFsmState = IDLE;
                }
                else{
                    std::cerr << "[ERROR] proxy GET HT delete/update failed" << std::endl;
                    proxyFsmState = IDLE;
                }
            }
            break;
        }
    }
}

void deparser(
    hls::stream<ap_uint<16> >& sessionIdFifo, // input
    hls::stream<msgHeader>& msgHeaderFifo, // input
    hls::stream<msgBody>& msgBodyFifo, // input
	hls::stream<appTxMeta>& txMetaData, 
    hls::stream<net_axis<DATA_WIDTH> >& txData
){
#pragma HLS PIPELINE II=2
#pragma HLS INLINE off

	ap_uint<16> currSessionID;
    msgHeader currMsgHeader;
    msgBody currMsgBody;
	net_axis<DATA_WIDTH> currWord;

#define SENDBUF_LEN (24*8+KV_MAX_EXT_SIZE+KV_MAX_KEY_SIZE+KV_MAX_VAL_SIZE)
    static ap_uint<SENDBUF_LEN> sendBuf;
    static ap_uint<32> totalSendLen;
    static ap_uint<32> currSendLen;
    static ap_uint<4> esac_fsmState = 0;

    static ap_uint<24*8>    hdr;
    static ap_uint<KV_MAX_EXT_SIZE>    ext;
    static ap_uint<KV_MAX_KEY_SIZE>    key;
    static ap_uint<KV_MAX_VAL_SIZE>    val;

    static ap_uint<8> extLen;
    static ap_uint<16> keyLen;
    static ap_uint<32> valLen;
    
    static ap_uint<16> loc0;
    static ap_uint<16> loc1;
        
    char* deparser_fsmName[9] = {"txMetaData", "ext1", "ext2", "key1", "key2", "val1", "val2", "txData1", "txData2"};
    std::cout << "mcrouter::deparser fsmState " << deparser_fsmName[(int)esac_fsmState] << std::endl;

	switch (esac_fsmState)
	{
	case 0:
		if (!txMetaData.full() && !sessionIdFifo.empty() && !msgHeaderFifo.empty() && !msgBodyFifo.empty())
		{
            sessionIdFifo.read(currSessionID);
            msgHeaderFifo.read(currMsgHeader);
            msgBodyFifo.read(currMsgBody);
            totalSendLen = 24*8+currMsgHeader.bodyLen*8;
            currSendLen = 0;

            std::cout << "currMsgHeader.keyLen " << currMsgHeader.keyLen << std::endl;
            std::cout << "currMsgHeader.bodyLen " << currMsgHeader.bodyLen << std::endl;

            hdr = currMsgHeader.output_word();
            
            std::cout << "deparser sends data" << std::endl;
            currMsgHeader.display();

            ext = currMsgBody.ext;
            key = currMsgBody.key;
            val = currMsgBody.val;

            extLen = currMsgHeader.extLen*8;
            keyLen = currMsgHeader.keyLen*8;
            valLen = currMsgHeader.val_len()*8;

            loc0 = SENDBUF_LEN-1;
            loc1 = SENDBUF_LEN-24*8;
            sendBuf(loc0, loc1) = hdr;

            txMetaData.write(appTxMeta(currSessionID, totalSendLen));
			esac_fsmState = 1;
		}
		break;
        // sendBuf = (hdr, ext(KV_MAX_EXT_SIZE-1, KV_MAX_EXT_SIZE-extLen), key(KV_MAX_KEY_SIZE-1, KV_MAX_KEY_SIZE-keyLen), val(KV_MAX_VAL_SIZE-1, KV_MAX_VAL_SIZE-valLen) );
    case 1: 
        if(extLen != 0){
            loc0 = loc1-1;
            loc1 -= extLen;
            esac_fsmState = 2;
        }
        else{
            esac_fsmState = 3;
        }
        break;
    case 2: 
        sendBuf(loc0, loc1) = ext(KV_MAX_EXT_SIZE-1,KV_MAX_EXT_SIZE-extLen);
        esac_fsmState = 3;
        break;
    case 3:
        if(keyLen != 0){
            loc0 = loc1-1;
            loc1 -= keyLen;
            esac_fsmState = 4;
        }
        else{
            esac_fsmState = 5;
        }
        break;
    case 4:
        sendBuf(loc0, loc1) = key(KV_MAX_KEY_SIZE-1,KV_MAX_KEY_SIZE-keyLen);
        esac_fsmState = 5;
        break;
    case 5: 
        if(valLen != 0){
            loc0 = loc1-1;
            loc1 -= valLen;
            esac_fsmState = 6;
        }
        else{
            esac_fsmState = 7;
        }
        break;
    case 6: 
        sendBuf(loc0, loc1) = val(KV_MAX_VAL_SIZE-1,KV_MAX_VAL_SIZE-valLen);
        esac_fsmState = 7;
        break;
    case 7:
		if (!txData.full())
		{
            ap_uint<32> remainLen = totalSendLen - currSendLen;
            std::cout << "remainLen " << remainLen << std::endl;

            if(remainLen > DATA_WIDTH){
                loc0 = SENDBUF_LEN-currSendLen-1;
                loc1 = SENDBUF_LEN-currSendLen-DATA_WIDTH;
                esac_fsmState = 8;
            }
            else{
                if(currSendLen >= DATA_WIDTH){
                    currSendLen -= DATA_WIDTH;
                }
                currWord.data(DATA_WIDTH-currSendLen-1, DATA_WIDTH-currSendLen-remainLen) = sendBuf(SENDBUF_LEN-currSendLen-1, SENDBUF_LEN-currSendLen-remainLen);
                currWord.keep = lenToKeep(remainLen/8);
                currWord.last = 1;
                currSendLen += remainLen;
    			txData.write(currWord);
				esac_fsmState = 0;
            }
		}
		break;
    case 8: {
        currWord.data = sendBuf(loc0, loc1);
        currWord.keep = lenToKeep(DATA_WIDTH/8);
        currSendLen += DATA_WIDTH;
		txData.write(currWord);
        esac_fsmState = 7;
        break;
    }
	}
}

void dummy(	
    hls::stream<ap_uint<16> >& closeConnection,
	hls::stream<appTxRsp>& txStatus,
    hls::stream<hash_table_16_1024::htUpdateResp<16,1024> >&  m_axis_upd_rsp,
    hls::stream<ap_uint<16> >& regInsertFailureCount,
    hls::stream<ap_uint<16> >& regInsertFailureCount2
){
#pragma HLS PIPELINE II=1
#pragma HLS INLINE off
	
    if (!txStatus.empty()) //Make Checks
	{
		txStatus.read();
	}
    
    if(!m_axis_upd_rsp.empty()){
        hash_table_16_1024::htUpdateResp<16,1024> response = m_axis_upd_rsp.read();
        if (!response.success){
            std::cerr << "[ERROR] update failed" << std::endl;
        }
    }   
    if(!regInsertFailureCount.empty()){
        ap_uint<16> cnt = regInsertFailureCount.read();
        std::cerr << "[ERROR] insert failed cnt1 " << cnt << std::endl;
    }
    if(!regInsertFailureCount2.empty()){
        ap_uint<16> cnt = regInsertFailureCount2.read();
        std::cerr << "[ERROR] insert failed cnt2 " << cnt << std::endl;
    }
}


// @yang, this implements a TCP interface 
void mcrouter(	
    // for mcrouter listening on a TCP port
    hls::stream<ap_uint<16> >&		listenPort,
	hls::stream<bool>&				listenPortStatus,
    // for mcrouter receiving notifications: either new data available or connection closed
	hls::stream<appNotification>&	notifications,
    // for mcrouter requesting and receiving data
	hls::stream<appReadRequest>&	readRequest,
	hls::stream<ap_uint<16> >&		rxMetaData,
	hls::stream<net_axis<DATA_WIDTH> >&		rxData,
    // for mcrouter openning a connection
    hls::stream<ipTuple>&           openTuples, // user-provided memcached tuples. 
	hls::stream<ipTuple>&			openConnection,
	hls::stream<openStatus>&		openConStatus,
    // for mcrouter closing a connection
	hls::stream<ap_uint<16> >&		closeConnection,
    // for mcrouter sending data out
	hls::stream<appTxMeta>&			txMetaData,
	hls::stream<net_axis<DATA_WIDTH> >&	txData,
	hls::stream<appTxRsp>&			txStatus
){
#pragma HLS DATAFLOW disable_start_propagation
#pragma HLS INTERFACE ap_ctrl_none port=return


#pragma HLS INTERFACE axis register port=listenPort name=m_axis_listen_port
#pragma HLS INTERFACE axis register port=listenPortStatus name=s_axis_listen_port_status
//#pragma HLS INTERFACE axis register port=closePort name=m_axis_close_port

#pragma HLS INTERFACE axis register port=notifications name=s_axis_notifications
#pragma HLS INTERFACE axis register port=readRequest name=m_axis_read_package
#pragma HLS DATA_PACK variable=notifications
#pragma HLS DATA_PACK variable=readRequest

#pragma HLS INTERFACE axis register port=rxMetaData name=s_axis_rx_metadata
#pragma HLS INTERFACE axis register port=rxData name=s_axis_rx_data
//#pragma HLS DATA_PACK variable=rxMetaData

#pragma HLS INTERFACE axis register port=openConnection name=m_axis_open_connection
#pragma HLS INTERFACE axis register port=openConStatus name=s_axis_open_status
#pragma HLS INTERFACE axis register port=openTuples name=s_axis_open_tuples
#pragma HLS DATA_PACK variable=openConnection
#pragma HLS DATA_PACK variable=openConStatus
#pragma HLS DATA_PACK variable=openTuples

#pragma HLS INTERFACE axis register port=closeConnection name=m_axis_close_connection

#pragma HLS INTERFACE axis register port=txMetaData name=m_axis_tx_metadata
#pragma HLS INTERFACE axis register port=txData name=m_axis_tx_data
#pragma HLS INTERFACE axis register port=txStatus name=s_axis_tx_status
#pragma HLS DATA_PACK variable=txMetaData
#pragma HLS DATA_PACK variable=txStatus


	static hls::stream<ap_uint<16> >		mc_sessionIdFifo0("mc_sessionIdFifo0");
	static hls::stream<ap_uint<16> >		mc_sessionIdFifo1("mc_sessionIdFifo1");
    static hls::stream<msgHeader>		    mc_msgHeaderFifo0("mc_msgHeaderFifo0");
    static hls::stream<msgHeader>		    mc_msgHeaderFifo1("mc_msgHeaderFifo1");
	static hls::stream<msgBody>		        mc_msgBodyFifo0("mc_msgBodyFifo0");
	static hls::stream<msgBody>		        mc_msgBodyFifo1("mc_msgBodyFifo1");
    #pragma HLS stream variable=mc_sessionIdFifo0 depth=64
    #pragma HLS stream variable=mc_sessionIdFifo1 depth=64
    #pragma HLS stream variable=mc_msgHeaderFifo0 depth=64
    #pragma HLS stream variable=mc_msgHeaderFifo1 depth=64
    #pragma HLS stream variable=mc_msgBodyFifo0 depth=64
    #pragma HLS stream variable=mc_msgBodyFifo1 depth=64


	static hls::stream<ap_uint<KV_MAX_KEY_SIZE> >		mc_keyFifo("mc_keyFifo");
	static hls::stream<ap_uint<32> >		mc_hashFifo("mc_hashFifo");
    #pragma HLS stream variable=mc_keyFifo depth=64
    #pragma HLS stream variable=mc_hashFifo depth=64


    static hls::stream<hash_table_16_1024::htLookupReq<16> >         s_axis_lup_req;
    static hls::stream<hash_table_16_1024::htUpdateReq<16,1024> >    s_axis_upd_req;
    static hls::stream<hash_table_16_1024::htLookupResp<16,1024> >   m_axis_lup_rsp;
    static hls::stream<hash_table_16_1024::htUpdateResp<16,1024> >   m_axis_upd_rsp;
    static hls::stream<ap_uint<16> > regInsertFailureCount;
    #pragma HLS stream variable=s_axis_lup_req depth=64
    #pragma HLS stream variable=s_axis_upd_req depth=64
    #pragma HLS stream variable=m_axis_lup_rsp depth=64
    #pragma HLS stream variable=m_axis_upd_rsp depth=64
    #pragma HLS stream variable=regInsertFailureCount depth=64
    #pragma HLS DATA_PACK variable=s_axis_lup_req
    #pragma HLS DATA_PACK variable=s_axis_upd_req
    #pragma HLS DATA_PACK variable=m_axis_lup_rsp
    #pragma HLS DATA_PACK variable=m_axis_upd_rsp


    static hls::stream<hash_table_32_32::htLookupReq<32> >       s_axis_lup_req1;
    static hls::stream<hash_table_32_32::htUpdateReq<32,32> >    s_axis_upd_req1;
    static hls::stream<hash_table_32_32::htLookupResp<32,32> >   m_axis_lup_rsp1;
    static hls::stream<hash_table_32_32::htUpdateResp<32,32> >   m_axis_upd_rsp1;
    static hls::stream<ap_uint<16> > regInsertFailureCount1;
    #pragma HLS stream variable=s_axis_lup_req1 depth=64
    #pragma HLS stream variable=s_axis_upd_req1 depth=64
    #pragma HLS stream variable=m_axis_lup_rsp1 depth=64
    #pragma HLS stream variable=m_axis_upd_rsp1 depth=64
    #pragma HLS stream variable=regInsertFailureCount1 depth=64
    #pragma HLS DATA_PACK variable=s_axis_lup_req1
    #pragma HLS DATA_PACK variable=s_axis_upd_req1
    #pragma HLS DATA_PACK variable=m_axis_lup_rsp1
    #pragma HLS DATA_PACK variable=m_axis_upd_rsp1


	static hls::stream<mqInsertReq<ap_uint<32>> >	multiQueue_push("multiQueue_push");
	static hls::stream<mqPopReq>					multiQueue_pop_req("multiQueue_pop_req");
	static hls::stream<ap_uint<32>>					multiQueue_rsp("multiQueue_rsp");
    #pragma HLS stream variable=multiQueue_push depth=64
    #pragma HLS stream variable=multiQueue_pop_req depth=64
    #pragma HLS stream variable=multiQueue_rsp depth=64
    #pragma HLS DATA_PACK variable=multiQueue_push
    #pragma HLS DATA_PACK variable=multiQueue_pop_req
    #pragma HLS DATA_PACK variable=multiQueue_rsp
    

	static hls::stream<ap_uint<2> >	    mc_cmdFifo("mc_cmdFifo");
	static hls::stream<ap_uint<16> >	mc_sessionCountFifo("mc_sessionCountFifo");
	static hls::stream<ap_uint<32> >	mc_hashValFifo("mc_hashValFifo");
	static hls::stream<ap_uint<16> >	mc_sessionIdFifo2("mc_sessionIdFifo2");
	static hls::stream<ap_uint<16> >	mc_idxFifo("mc_idxFifo");
	static hls::stream<ap_uint<16> >	mc_sessionIdFifo3("mc_sessionIdFifo3");
	static hls::stream<ap_uint<16> >    mc_closedSessionIdFifo("mc_closedSessionIdFifo");
    #pragma HLS stream variable=mc_cmdFifo depth=64
    #pragma HLS stream variable=mc_sessionCountFifo depth=64
    #pragma HLS stream variable=mc_hashValFifo depth=64
    #pragma HLS stream variable=mc_sessionIdFifo2 depth=64
    #pragma HLS stream variable=mc_idxFifo depth=64
    #pragma HLS stream variable=mc_sessionIdFifo3 depth=64
    #pragma HLS stream variable=mc_closedSessionIdFifo depth=64


    // initing a hash table block
    hash_table_16_1024::hash_table_top(s_axis_lup_req, s_axis_upd_req, m_axis_lup_rsp, m_axis_upd_rsp, regInsertFailureCount);
    hash_table_32_32::hash_table_top(s_axis_lup_req1, s_axis_upd_req1, m_axis_lup_rsp1, m_axis_upd_rsp1, regInsertFailureCount1);

    // initing a multi queue block
    // multi_queue<ap_uint<32>, MAX_CONNECTED_SESSIONS, MAX_CONNECTED_SESSIONS*16>(multiQueue_push, multiQueue_pop_req, multiQueue_rsp);
    multi_queue<ap_uint<32>, 65535, 65535*16>(multiQueue_push, multiQueue_pop_req, multiQueue_rsp);

    // opening the mcrouter listening port
	open_port(listenPort, listenPortStatus);

    // connecting to remote memcached;
    connect_memcached(openTuples, openConStatus, openConnection, \
            mc_cmdFifo, mc_sessionCountFifo, mc_hashValFifo, mc_sessionIdFifo2, mc_idxFifo, mc_sessionIdFifo3, \
            mc_closedSessionIdFifo);
    
    // handling new data arriving (it might come from a new session), and connection closed
	notification_handler(notifications, readRequest, mc_closedSessionIdFifo);
    
    // read data from network and parse them into reqFifo
    extract_msg(rxMetaData, rxData, mc_sessionIdFifo0, mc_msgHeaderFifo0, mc_msgBodyFifo0, \
            s_axis_lup_req, s_axis_upd_req, m_axis_lup_rsp);

    // simple hash function 
    simple_hash(mc_keyFifo, mc_hashFifo);

	// 1) read request from fifo, determine the destination mc, query mc
    // 2) read response from fifo, forward response to client. 
    proxy(mc_sessionIdFifo0, mc_msgHeaderFifo0, mc_msgBodyFifo0, mc_sessionIdFifo1, mc_msgHeaderFifo1, mc_msgBodyFifo1, \
            mc_keyFifo, mc_hashFifo, multiQueue_push, multiQueue_pop_req, multiQueue_rsp, \
            s_axis_lup_req1, s_axis_upd_req1, m_axis_lup_rsp1, m_axis_upd_rsp1,\
            mc_cmdFifo, mc_sessionCountFifo, mc_hashValFifo, mc_sessionIdFifo2, mc_idxFifo, mc_sessionIdFifo3);

    // deparsing the res and resp, sending to destination (client or memcached)
    deparser(mc_sessionIdFifo1, mc_msgHeaderFifo1, mc_msgBodyFifo1, txMetaData, txData);

	dummy(closeConnection, txStatus, m_axis_upd_rsp, regInsertFailureCount, regInsertFailureCount1);

}
