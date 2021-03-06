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

struct sessionID_stream{
    ap_uint<16> sessionID;
    ap_uint<1> last;
    sessionID_stream() {}
    sessionID_stream(ap_uint<16> s, ap_uint<1> l): sessionID(s), last(l) {}
};

// TODO: 
// return a list of sessionID. 
// EOF, SOF. 
// return a specific sessionID
void conn_manager(
    hls::stream<ipTuple>& openTuples, //input
    hls::stream<openStatus>& openConStatus, //input
    hls::stream<ipTuple>& openConnection, //output 
    // you should separate the following fifos, as their rspFifo are not handled by a single function. 
    hls::stream<ap_uint<2> >& cmdFifo, //input
    hls::stream<ap_uint<16> >& sessionCountFifo, //output
    hls::stream<sessionID_stream>& sessionIdStreamFifo, //output
    hls::stream<ap_uint<32> >& hashValFifo, //input
    hls::stream<ap_uint<16> >& sessionIdFifo2, //output
    hls::stream<ap_uint<16> >& idxFifo, //input
	hls::stream<ap_uint<16> >& sessionIdFifo3, //output
	hls::stream<ap_uint<16> >& closedSessionIdFifo //input
    // softTkoFIFO () 
){
#pragma HLS PIPELINE II=1
#pragma HLS INLINE off

static ap_uint<16> connectedSessions[MAX_CONNECTED_SESSIONS];
static ap_uint<1> connectedSessionsSts[MAX_CONNECTED_SESSIONS];

// static SoftTkoCount sessionState[];
#pragma HLS RESOURCE variable=connectedSessions core=RAM_T2P_BRAM
#pragma HLS ARRAY_PARTITION variable=connectedSessions complete
#pragma HLS DEPENDENCE variable=connectedSessions inter false
#pragma HLS RESOURCE variable=connectedSessionsSts core=RAM_T2P_BRAM
#pragma HLS ARRAY_PARTITION variable=connectedSessionsSts complete
#pragma HLS DEPENDENCE variable=connectedSessionsSts inter false

    static ap_uint<16> sessionCount = 0;

    static ap_uint<2> sessionOutState = 0;
    static ap_uint<16> sessionIdx = 0;

    // this actually causes II=1 violation 
    switch(sessionOutState) {
        case 0: {
            if(!cmdFifo.empty()){
                ap_uint<2> cmd = cmdFifo.read();
                switch(cmd) {
                    case 0:{
                        sessionCountFifo.write(sessionCount);
                        
                        if(sessionCount > 1){// needs consective writeout;
                            sessionOutState = 1;
                            sessionIdx = sessionCount-2;
                            sessionIdStreamFifo.write(sessionID_stream(connectedSessions[sessionCount-1], 0));
                        }
                        else{
                            sessionIdStreamFifo.write(sessionID_stream(connectedSessions[sessionCount-1], 1));
                        }
                        break;
                    }
                }
            }
            break;
        }
        case 1:{
            if(sessionIdx != 0){
                sessionIdStreamFifo.write(sessionID_stream(connectedSessions[sessionIdx], 0));
                sessionIdx -= 1;
            }
            else{
                sessionIdStreamFifo.write(sessionID_stream(connectedSessions[sessionIdx], 1));
                sessionOutState = 0;
            }
            break;
        }
    }

    // !!!??? you should use "else if"; 
    // if you just use "if", hls will think of you want to parallel the five block
    //, and check the data dependency, and reduce the II. 
    if (!openTuples.empty()){
        ipTuple tuple = openTuples.read();
		openConnection.write(tuple);
    }
	if (!openConStatus.empty())
	{
		openStatus newConn = openConStatus.read();
		if (newConn.success)
		{
            connectedSessions[sessionCount] = newConn.sessionID;
            connectedSessionsSts[sessionCount] = 1;
            sessionCount += 1;
		}
	}
    if(!hashValFifo.empty()){
        ap_uint<32> hashVal = hashValFifo.read();
        // TODO: handling closed connections, or in policy engine
        ap_uint<16> idx = (hashVal&(MAX_CONNECTED_SESSIONS-1)) % sessionCount;
        // ap_uint<16> idx = (hashVal&(MAX_CONNECTED_SESSIONS-1));
        sessionIdFifo2.write(connectedSessions[idx]);
    }
    if(!idxFifo.empty()){
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
    net_axis<DATA_WIDTH>& currWord, ap_uint<16>& currWordValidLen, ap_uint<16>& currWordValidLen_init){
#pragma HLS INLINE

    if(currWordValidLen > 0){
        ap_uint<16> currWord_parsingPos = DATA_WIDTH - (currWordValidLen_init - currWordValidLen)*8;
        if(currSessionState.parsingBodyState == 0){
            ap_uint<32> currBodyLen = currSessionState.currBodyLen;
            ap_uint<32> requiredBodyLen = currSessionState.requiredLen - MEMCACHED_HDRLEN - currBodyLen;

            if(currMsgBody.msgID != 0)std::cout << "currMsgBody.msgID = " << currMsgBody.msgID << ": ";
            std::cout << "currWordValidLen=" << currWordValidLen << std::endl;
            std::cout << "currBodyLen=" << currBodyLen << " requiredBodyLen=" << requiredBodyLen << std::endl;
            
            if(currWordValidLen >= requiredBodyLen){
                currMsgBody.body(MAX_BODY_LEN-1-currBodyLen*8, MAX_BODY_LEN-currBodyLen*8-requiredBodyLen*8) = currWord.data(currWord_parsingPos-1, currWord_parsingPos-requiredBodyLen*8);
                currWordValidLen -= requiredBodyLen;
                currSessionState.currBodyLen += requiredBodyLen;
                currSessionState.parsingBodyState = 1; // body is parsed
            }
            else{
                currMsgBody.body(MAX_BODY_LEN-1-currBodyLen*8, MAX_BODY_LEN-currBodyLen*8-currWordValidLen*8) = currWord.data(currWord_parsingPos-1, currWord_parsingPos-currWordValidLen*8);
                currSessionState.currBodyLen += currWordValidLen;
                currWordValidLen = 0;
            }
        }
    }
}

void rxLock(
    hls::stream<lockReq>& lockReqFifo, // input
    hls::stream<ap_uint<1> >& grantFifo // output
){
#pragma HLS PIPELINE II=1
#pragma HLS INLINE off

    static ap_uint<1> is_locked = 0;
    static ap_uint<32> currMsgID = 0;
    if(!lockReqFifo.empty()){
        lockReq req = lockReqFifo.read();
        if(req.opcode == LOCK_ACQUIRE){
            if(is_locked){
                if(currMsgID == req.msgID){
                    grantFifo.write(1);
                    std::cout << "rxlock acquiring okay: " << req.msgID << std::endl;
                }
                else{
                    grantFifo.write(0);
                    std::cout << "rxlock acquiring fails: " << req.msgID << std::endl;
                }
            }
            else{
                is_locked = 1;
                currMsgID = req.msgID;
                grantFifo.write(1);
                std::cout << "rxlock acquiring okay: " << req.msgID << std::endl;
            }
        }
        else{
            if(is_locked && currMsgID == req.msgID){
                is_locked = 0;
                std::cout << "rxlock releasing okay: " << req.msgID << std::endl;
            }
        }
    }
}

void parser(
    hls::stream<ap_uint<16> >& rxMetaData, // input
	hls::stream<net_axis<DATA_WIDTH> >& rxData, // intput 
    hls::stream<ap_uint<16> >& sessionIdFifo, // output
    hls::stream<msgHeader>& msgHeaderFifo, // output
    hls::stream<msgBody>& msgBodyFifo, // output
    hls::stream<hash_table_16_1024::htLookupReq<16> >&       s_axis_lup_req, 
    hls::stream<hash_table_16_1024::htUpdateReq<16,1024> >&    s_axis_upd_req, 
    hls::stream<hash_table_16_1024::htLookupResp<16,1024> >&   m_axis_lup_rsp,
    hls::stream<lockReq>& lockReqFifo, // output
    hls::stream<ap_uint<1> >& grantFifo // input
){
#pragma HLS PIPELINE II=1
#pragma HLS INLINE off
    // generating globally unique msgID. 
    static ap_uint<32> currentMsgID = 1;
    // this ASIX transaction should be locked. 
    // rxTransID is not the same as currentMsgID, as one transaction might include multiple msg. 
    static ap_uint<32> rxTransID = 1;
 
    static ap_uint<32> currRxTransID = 1;

    // static value gets inited to zero by default. 
    static sessionState sessionStateTable[MAX_SESSION_NUM];
    #pragma HLS RESOURCE variable=sessionStateTable core=RAM_T2P_BRAM
    #pragma HLS DEPENDENCE variable=sessionStateTable inter false

    enum axisFsmType {IDLE, RECOVER_STATE, WAITING_LOCK, READ_WORD, PARSE_WORD};
#ifndef __SYNTHESIS__
    const char* axisFsmName[5] = {"IDLE", "RECOVER_STATE", "WAITING_LOCK", "READ_WORD", "PARSE_WORD"};
#endif
    static axisFsmType currAxisState = IDLE;

	static ap_uint<16>          currSessionID; // valid when currAxisState becomes on-going
    static sessionState         currSessionState; // valid when currAxisState becomes on-going
    
    static msgBody              currMsgBody;
    static msgHeader            currMsgHeader;

	static net_axis<DATA_WIDTH> currWord;
    static ap_uint<16>          currWordValidLen_init; // the available length of currWord
    static ap_uint<16>          currWordValidLen; // the current waiting-for-parsing length of currWord
    
#ifndef __SYNTHESIS__
    if(currMsgBody.msgID != 0)std::cout << "currMsgBody.msgID = " << currMsgBody.msgID << ": ";
    std::cout << "mcrouter::parser fsmState " << axisFsmName[(int)currAxisState] << std::endl;
#endif
    switch (currAxisState){
        case IDLE: { // a new AXIS transaction 
            if(!rxMetaData.empty()){
                rxMetaData.read(currSessionID);
                s_axis_lup_req.write(hash_table_16_1024::htLookupReq<16>(currSessionID, 0));
                currAxisState = RECOVER_STATE;
            }
            break;
        }
        // get the session-specific parsing state
        case RECOVER_STATE: {
            if(!m_axis_lup_rsp.empty()){
                hash_table_16_1024::htLookupResp<16, 1024> response = m_axis_lup_rsp.read();
                if(response.hit){
                    currMsgBody.consume_word(response.value);
                }
                else{
                    currMsgBody.reset(); // fresh new start for this sesion. 
                    currMsgBody.msgID = currentMsgID;
                    currentMsgID += 1;
                }
                if(currMsgBody.msgID != 0)std::cout << "currMsgBody.msgID = " << currMsgBody.msgID << ": ";
                std::cout << "RECOVER_STATE " << response.hit << std::endl;
                // we should expect the hash table is enough to handle all active connections; 

                currRxTransID = rxTransID;
                lockReqFifo.write(lockReq(currRxTransID, LOCK_ACQUIRE));
                rxTransID ++;

                currAxisState = WAITING_LOCK;
            }
            break;
        }
        case WAITING_LOCK: {
            if(!grantFifo.empty()){
                ap_uint<1> grant = grantFifo.read();
                if(grant){
                    currAxisState = READ_WORD;
                    if(currMsgBody.msgID != 0)std::cout << "currMsgBody.msgID = " << currMsgBody.msgID << ": ";
                        std::cout << "mcrouter::parser rxMetaData.sessionID " << currSessionID << std::endl;
                    // recoverying sessionState. Note that currSessionState also contains the currMsgHeader 
                    currSessionState = sessionStateTable[currSessionID];
                    currSessionState.display();
                }
                else{
                    lockReqFifo.write(lockReq(currRxTransID, LOCK_ACQUIRE));
                }
            }
            break;
        }
        case READ_WORD: { // continue the AXIS transaction
            if(!rxData.empty()){
                rxData.read(currWord);
                currWordValidLen_init = keepToLen(currWord.keep);
                currWordValidLen = currWordValidLen_init;

                if(currMsgBody.msgID != 0)std::cout << "currMsgBody.msgID = " << currMsgBody.msgID << ": ";
                std::cout << "currWordValidLen_init=" << currWordValidLen_init << std::endl;
                currAxisState = PARSE_WORD;
            }
            break;
        }
        // case LSHIFT_WORD: {
            
        //     break;
        // }
        // consuming currWord 
        case PARSE_WORD: {
            if(currWordValidLen > 0){
                ap_uint<16> currWord_parsingPos = DATA_WIDTH - (currWordValidLen_init - currWordValidLen)*8;
                if(currSessionState.parsingHeaderState == 0){
                    ap_uint<5> requiredHeaderLen = MEMCACHED_HDRLEN - currSessionState.currHdrLen;
                    if(currWordValidLen >= requiredHeaderLen){
                        currSessionState.msgHeaderBuff(requiredHeaderLen*8-1, 0) = currWord.data(currWord_parsingPos-1, currWord_parsingPos-requiredHeaderLen*8);
                        
                        currWordValidLen -= requiredHeaderLen;
                        currSessionState.currHdrLen += requiredHeaderLen;
                        currSessionState.parsingHeaderState = 1; // header is parsed. 
                        
                        currMsgHeader.consume_word(currSessionState.msgHeaderBuff);
                        
                        if(currMsgBody.msgID != 0)std::cout << "currMsgBody.msgID = " << currMsgBody.msgID << ": ";
                        std::cout << "parser receives data" << std::endl;
                        currMsgHeader.display();

                        currSessionState.requiredLen = currMsgHeader.bodyLen + MEMCACHED_HDRLEN;
                        
                        std::cout << "currSessionState.requiredLen=" << currSessionState.requiredLen 
                            << " MEMCACHED_HDRLEN + currSessionState.currBodyLen=" << MEMCACHED_HDRLEN + currSessionState.currBodyLen
                            << std::endl;
                        if(currSessionState.requiredLen > MEMCACHED_HDRLEN + currSessionState.currBodyLen){
                            parsingMsgBody(currSessionState, currMsgBody, currMsgHeader, currWord, currWordValidLen, currWordValidLen_init);
                        }
                        else{
                            currSessionState.parsingBodyState = 1;
                        }
                    }
                    else{
                        currSessionState.msgHeaderBuff(requiredHeaderLen*8-1, requiredHeaderLen*8-currWordValidLen*8) = currWord.data(currWord_parsingPos-1, currWord_parsingPos-currWordValidLen*8);
                        currSessionState.currHdrLen += currWordValidLen;
                        currWordValidLen = 0;
                    }
                }
                else{
                    currMsgHeader.consume_word(currSessionState.msgHeaderBuff);
                    std::cout << "currSessionState.requiredLen=" << currSessionState.requiredLen 
                        << " MEMCACHED_HDRLEN + currSessionState.currBodyLen=" << MEMCACHED_HDRLEN + currSessionState.currBodyLen
                        << std::endl;
                    if(currSessionState.requiredLen > MEMCACHED_HDRLEN + currSessionState.currBodyLen){
                        parsingMsgBody(currSessionState, currMsgBody, currMsgHeader, currWord, currWordValidLen, currWordValidLen_init);
                    }
                    else{
                        currSessionState.parsingBodyState = 1;
                    }
                }
            }
            if(currMsgBody.msgID != 0)std::cout << "currMsgBody.msgID = " << currMsgBody.msgID << ": ";
            std::cout << "currWordValidLen=" << currWordValidLen << std::endl;

            ap_uint<1> parsingMsgState = (currSessionState.parsingHeaderState == 1 && currSessionState.parsingBodyState == 1);
            // or (currSessionState.currBodyLen + currSessionState.currHdrLen == currSessionState.requiredLen)
            
            if(currWordValidLen > 0){
                if(parsingMsgState){
                    currMsgBody.extInl = 1;
                    currMsgBody.keyInl = 1;
                    currMsgBody.valInl = 1;
                    
                    if(currMsgBody.msgID != 0)std::cout << "currMsgBody.msgID = " << currMsgBody.msgID << ": ";
                    std::cout << "writing currMsgBody (path1) for currMsgBody.msgID " << currMsgBody.msgID << std::endl;
                    // !!! only write out when current msg is parsed
                    msgHeaderFifo.write(currMsgHeader);
                    msgBodyFifo.write(currMsgBody);
                    sessionIdFifo.write(currSessionID);

                    currMsgHeader.display();
                    currMsgBody.display(currMsgHeader.extLen, currMsgHeader.keyLen, currMsgHeader.val_len());

                    // the AXIS for current session still has data to parse; we do not need to store currSessionState back.  
                    // sessionStateTable[currSessionID] = currSessionState;

                    currSessionState.reset(); // ready to parse next message

                    currMsgBody.reset(); // fresh new start for this sesion. 
                    currMsgBody.msgID = currentMsgID;
                    currentMsgID++;

                    // removing currMsgBody from the hash table. 
                    s_axis_upd_req.write(hash_table_16_1024::htUpdateReq<16, 1024>(hash_table_16_1024::KV_DELETE, currSessionID, currMsgBody.output_word(), 0));
                }
                currAxisState = PARSE_WORD;
            }
            else{
                if(currWord.last == 1){
                    if(parsingMsgState){
                        currMsgBody.extInl = 1;
                        currMsgBody.keyInl = 1;
                        currMsgBody.valInl = 1;
                        
                        if(currMsgBody.msgID != 0)std::cout << "currMsgBody.msgID = " << currMsgBody.msgID << ": ";
                        std::cout << "writing currMsgBody for (path2) currMsgBody.msgID " << currMsgBody.msgID << std::endl;
                        // !!! only write out when current msg is parsed
                        msgHeaderFifo.write(currMsgHeader);
                        msgBodyFifo.write(currMsgBody);
                        sessionIdFifo.write(currSessionID);
                        
                        currMsgHeader.display();
                        currMsgBody.display(currMsgHeader.extLen, currMsgHeader.keyLen, currMsgHeader.val_len());

                        currSessionState.reset(); // ready to parse next message
                        sessionStateTable[currSessionID] = currSessionState;
                        sessionStateTable[currSessionID].display();
                        // removing currMsgBody from the hash table. 
                        s_axis_upd_req.write(hash_table_16_1024::htUpdateReq<16, 1024>(hash_table_16_1024::KV_DELETE, currSessionID, currMsgBody.output_word(), 0));
                    }
                    else{
                        if(currMsgBody.msgID != 0)std::cout << "currMsgBody.msgID = " << currMsgBody.msgID << ": ";
                        std::cout << "storing sessionState and currMsgBody back: currSessionID " << currSessionID << " currMsgBody.msgID " << currMsgBody.msgID << std::endl;
                        // In the end of each AXIS, store sessionState and currMsgBody back. 
                        // msgHeader already written out or can be recovered from sessionStateTable. 
                        sessionStateTable[currSessionID] = currSessionState;
                        currSessionState.reset();
                        s_axis_upd_req.write(hash_table_16_1024::htUpdateReq<16, 1024>(hash_table_16_1024::KV_UPDATE_INSERT, currSessionID, currMsgBody.output_word(), 0));
                        // TODO: optimization: implementing hash table with a stash, such that read and write can finish in one cycle. 
                        // TODO: or store currMsgBody in URAM. 
                    }
                    currMsgBody.reset(); // fresh new start for this sesion. 
                    lockReqFifo.write(lockReq(currRxTransID, LOCK_RELEASE));
                    currAxisState = IDLE;
                }
                else{
                    if(parsingMsgState){
                        currMsgBody.extInl = 1;
                        currMsgBody.keyInl = 1;
                        currMsgBody.valInl = 1;
                        
                        if(currMsgBody.msgID != 0)std::cout << "currMsgBody.msgID = " << currMsgBody.msgID << ": ";
                        std::cout << "writing currMsgBody (path3) for currMsgBody.msgID " << currMsgBody.msgID << std::endl;
                        // !!! only write out when current msg is parsed
                        msgHeaderFifo.write(currMsgHeader);
                        msgBodyFifo.write(currMsgBody);
                        sessionIdFifo.write(currSessionID);

                        currMsgHeader.display();
                        currMsgBody.display(currMsgHeader.extLen, currMsgHeader.keyLen, currMsgHeader.val_len());

                        // the AXIS for current session still has data to parse; we do not need to store currSessionState back.  
                        // sessionStateTable[currSessionID] = currSessionState;

                        currSessionState.reset(); // ready to parse next message

                        currMsgBody.reset(); // fresh new start for this sesion. 
                        currMsgBody.msgID = currentMsgID;
                        currentMsgID++;

                        // removing currMsgBody from the hash table. 
                        s_axis_upd_req.write(hash_table_16_1024::htUpdateReq<16, 1024>(hash_table_16_1024::KV_DELETE, currSessionID, currMsgBody.output_word(), 0));
                    }
                    currAxisState = READ_WORD;
                }
            }
            break;
        }
    }
}

// void parser2(
//     ap_uint<16>          currSessionID, // valid when currAxisState becomes on-going
//     sessionState         currSessionState, // valid when currAxisState becomes on-going
//     msgBody              currMsgBody,
//     msgHeader            currMsgHeader,
// 	net_axis<DATA_WIDTH> currWord,
//     ap_uint<16>          currWordValidLen_init, // the available length of currWord
//     ap_uint<16>          currWordValidLen, // the current waiting-for-parsing length of currWord
// ){

// }

void simple_hash(
    hls::stream<ap_uint<MAX_KEY_LEN> >& keyFifo, 
    hls::stream<ap_uint<32> >& hashFifo
){
#pragma HLS PIPELINE II=1
#pragma HLS INLINE off

    ap_uint<MAX_KEY_LEN> key;
    if(!keyFifo.empty()){
        keyFifo.read(key);

        ap_uint<32> res = 0;
        for(int i = 0; i < MAX_KEY_LEN/32; i++){
            #pragma HLS unroll
            res ^= key(MAX_KEY_LEN-i*32-1, MAX_KEY_LEN-i*32-32);
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
    hls::stream<ap_uint<MAX_KEY_LEN> >&     keyFifo, 
    hls::stream<ap_uint<32> >&              hashFifo,
    hls::stream<mqInsertReq<ap_uint<32>> >& multiQueue_push, // output to mq
    hls::stream<mqPopReq>&				    multiQueue_pop_req, // output to mq
    hls::stream<ap_uint<32>>&				multiQueue_rsp, // input from mq
    hls::stream<hash_table_32_32::htLookupReq<32> >&          s_axis_lup_req, 
    hls::stream<hash_table_32_32::htUpdateReq<32,32> >&       s_axis_upd_req, 
    hls::stream<hash_table_32_32::htLookupResp<32,32> >&      m_axis_lup_rsp,
    hls::stream<hash_table_32_32::htUpdateResp<32,32> >&      m_axis_upd_rsp,
    hls::stream<ap_uint<2> >&  cmdFifo,
    hls::stream<ap_uint<16> >& sessionCountFifo, //input
    hls::stream<sessionID_stream >& sessionIdStreamFifo, 
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
    
    enum proxy_fsmType {IDLE=0, GET_HASH, GET_DEST, GET_WRITEOUT, SET_RANGE, SET_CHECKHT, SET_BROADCAST, RSP_MQ, RSP_HT, RSP_CHECKHT};

#ifndef __SYNTHESIS__
    const char * proxy_fsmName[10] = {"IDLE", "GET_HASH", "GET_DEST", "GET_WRITEOUT", "SET_RANGE", "SET_CHECKHT", "SET_BROADCAST", "RSP_MQ", "RSP_HT", "RSP_CHECKHT"};
#endif

    static proxy_fsmType proxyFsmState = IDLE;

#ifndef __SYNTHESIS__
    if(currMsgBody.msgID != 0)std::cout << "currMsgBody.msgID = " << currMsgBody.msgID << ": ";
    std::cout << "mcrouter::proxy fsmState " << proxy_fsmName[(int)proxyFsmState] << std::endl;
#endif
    switch (proxyFsmState){
        case IDLE:{
            if(!sessionIdFifo.empty() && !msgHeaderFifo.empty() && !msgBodyFifo.empty()){
                sessionIdFifo.read(currSessionID);
                msgHeaderFifo.read(currMsgHeader);
                msgBodyFifo.read(currMsgBody);
                switch(currMsgHeader.opcode){
                    case PROTOCOL_BINARY_CMD_GET: { // GET -> picks one memcached based on key hashing. 
                        ap_uint<MAX_KEY_LEN> key;
                        ap_uint<16> keyLen = currMsgHeader.keyLen;
                        key(MAX_KEY_LEN-1, MAX_KEY_LEN-keyLen*8) = currMsgBody.body(MAX_BODY_LEN-1, MAX_BODY_LEN-keyLen*8);
                        keyFifo.write(key); // calculating hashing
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
                        std::cout << "mcrouter::proxy IDLE POP currSessionID " << currSessionID << std::endl;
                        proxyFsmState = RSP_MQ;
                        break;
                    }
                    case PROTOCOL_BINARY_CMD_RSET: { // RSet
        				multiQueue_pop_req.write(mqPopReq(POP, currSessionID));
                        std::cout << "mcrouter::proxy IDLE POP currSessionID " << currSessionID << std::endl;
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

                if(currMsgBody.msgID != 0)std::cout << "currMsgBody.msgID = " << currMsgBody.msgID << ": ";
                std::cout << "mcrouter::proxy GET_DEST currMsgBody.msgID " << currMsgBody.msgID << " currSessionID_dst " << currSessionID_dst << std::endl;
                s_axis_upd_req.write(hash_table_32_32::htUpdateReq<32, 32>(hash_table_32_32::KV_INSERT, currMsgBody.msgID, srcMsgContext.output_word(), 0));
     
                sessionIdFifo_dst.write(currSessionID_dst);
                msgHeaderFifo_dst.write(currMsgHeader);
                msgBodyFifo_dst.write(currMsgBody);
                currMsgBody.reset();
        
                proxyFsmState = GET_WRITEOUT;
            }
            break;
        }
        // !!! just for simultion happy; can be removed 
        case GET_WRITEOUT:{
            if(!m_axis_upd_rsp.empty()){
                // let's assume ht is not gonna full ever. 
                hash_table_32_32::htUpdateResp<32,32> response = m_axis_upd_rsp.read();
                if (response.success){
                    proxyFsmState = IDLE;
                }
                else{
                    if(currMsgBody.msgID != 0)std::cout << "currMsgBody.msgID = " << currMsgBody.msgID << ": ";
                    std::cout << "response.key " << response.key << std::endl;
                    std::cout << "response.op " << response.op << std::endl;
                    std::cout << "[ERROR] proxy GET HT insert failed" << std::endl;
                    proxyFsmState = IDLE;
                }
            }
            break;
        }
        case SET_RANGE:{
            if(!sessionCountFifo.empty()){
                ap_uint<16> currSessionCount = sessionCountFifo.read();

                // update msg context table
                msgContext srcMsgContext(currSessionCount, currSessionID);
                s_axis_upd_req.write(hash_table_32_32::htUpdateReq<32, 32>(hash_table_32_32::KV_INSERT, currMsgBody.msgID, srcMsgContext.output_word(), 0));
                proxyFsmState = SET_CHECKHT;
            }
            break;
        }
        // !!! just for simultion happy; can be removed 
        case SET_CHECKHT: {
            if(!m_axis_upd_rsp.empty()){
                hash_table_32_32::htUpdateResp<32,32> response = m_axis_upd_rsp.read();
                if (response.success){
                    proxyFsmState = SET_BROADCAST;
                }
                else{
                    if(currMsgBody.msgID != 0)std::cout << "currMsgBody.msgID = " << currMsgBody.msgID << ": ";
                    std::cout << "[ERROR] proxy SET HT insert failed" << std::endl;
                    proxyFsmState = IDLE;
                }
            }
            break;
        }
        case SET_BROADCAST:{
            if(!sessionIdStreamFifo.empty()){
                sessionID_stream sessionID_dst = sessionIdStreamFifo.read();
                mqInsertReq<ap_uint<32> > insertReq(sessionID_dst.sessionID, currMsgBody.msgID);
        		multiQueue_push.write(insertReq);
                
                if(currMsgBody.msgID != 0)std::cout << "currMsgBody.msgID = " << currMsgBody.msgID << ": ";
                std::cout << "mcrouter::proxy SET_BROADCAST sessionID_dst.sessionID " << sessionID_dst.sessionID << std::endl;
                
                sessionIdFifo_dst.write(sessionID_dst.sessionID);
                msgHeaderFifo_dst.write(currMsgHeader);
                msgBodyFifo_dst.write(currMsgBody);  
                currMsgBody.reset();

                if(sessionID_dst.last){
                    proxyFsmState = IDLE;
                }
                else{
                    proxyFsmState = SET_BROADCAST;
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
                    if(currMsgBody.msgID != 0)std::cout << "currMsgBody.msgID = " << currMsgBody.msgID << ": ";
                    std::cout << "srcSessionID =" << srcSessionID << std::endl;
                    msgHeaderFifo_dst.write(currMsgHeader);
                    msgBodyFifo_dst.write(currMsgBody);
                    currMsgBody.reset();
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
        // !!! this (empty) stage is critical to make simulation work, as it let the last hash table update finish before next read. 
        case RSP_CHECKHT: {
            if(!m_axis_upd_rsp.empty()){
                hash_table_32_32::htUpdateResp<32,32> response = m_axis_upd_rsp.read();
                if (response.success){
                    proxyFsmState = IDLE;
                }
                else{
                    if(currMsgBody.msgID != 0)std::cout << "currMsgBody.msgID = " << currMsgBody.msgID << ": ";
                    std::cout << "[ERROR] proxy GET HT delete/update failed" << std::endl;
                    proxyFsmState = IDLE;
                }
            }
            // proxyFsmState = IDLE;

            break;
        }
    }
}

void txLock(
    hls::stream<lockReq>& lockReqFifo, // input
    hls::stream<ap_uint<1> >& grantFifo // output
){
#pragma HLS PIPELINE II=1
#pragma HLS INLINE off

    static ap_uint<1> is_locked = 0;
    static ap_uint<32> currMsgID = 0;
    if(!lockReqFifo.empty()){
        lockReq req = lockReqFifo.read();
        if(req.opcode == LOCK_ACQUIRE){
            if(is_locked){
                if(currMsgID == req.msgID){
                    grantFifo.write(1);
                    std::cout << "txlock acquiring okay: " << req.msgID << std::endl;
                }
                else{
                    grantFifo.write(0);
                    std::cout << "txlock acquiring fails: " << req.msgID << std::endl;
                }
            }
            else{
                is_locked = 1;
                currMsgID = req.msgID;
                grantFifo.write(1);
                std::cout << "txlock acquiring okay: " << req.msgID << std::endl;
            }
        }
        else{
            if(is_locked && currMsgID == req.msgID){
                is_locked = 0;
            }
        }
    }
}

void deparser(
    hls::stream<ap_uint<16> >& sessionIdFifo, // input
    hls::stream<msgHeader>& msgHeaderFifo, // input
    hls::stream<msgBody>& msgBodyFifo, // input
	hls::stream<appTxMeta>& txMetaData, 
    hls::stream<net_axis<DATA_WIDTH> >& txData,
    hls::stream<lockReq>& lockReqFifo, // output
    hls::stream<ap_uint<1> >& grantFifo // input
){
#pragma HLS PIPELINE II=1
#pragma HLS INLINE off

    static ap_uint<32> requiredSendLen;
    static ap_uint<32> currSendLen;
    static ap_uint<MEMCACHED_HDRLEN*8> hdr;
    static ap_uint<MAX_BODY_LEN> body;

    static ap_uint<16> currSessionID;
    static msgHeader currMsgHeader;
    static msgBody currMsgBody;

    enum deparser_fsmType{IDLE=0, SEND_HDR, SEND_OTHERS, WAITING_LOCK};
#ifndef __SYNTHESIS__
    const char* deparser_fsmName[4] = {"IDLE", "SEND_HDR", "SEND_OTHERS", "WAITING_LOCK"};
#endif
    static deparser_fsmType esac_fsmState = IDLE;

#ifndef __SYNTHESIS__
    if(currMsgBody.msgID != 0)std::cout << "currMsgBody.msgID = " << currMsgBody.msgID << ": ";
    std::cout << "mcrouter::deparser fsmState " << deparser_fsmName[(int)esac_fsmState] << std::endl;
#endif
	switch (esac_fsmState){
    	case IDLE:{
    		if (!txMetaData.full() && !sessionIdFifo.empty() && !msgHeaderFifo.empty() && !msgBodyFifo.empty())
    		{
                currSessionID = sessionIdFifo.read();
                currMsgHeader = msgHeaderFifo.read();
                currMsgBody = msgBodyFifo.read();

                requiredSendLen = MEMCACHED_HDRLEN*8 + currMsgHeader.bodyLen*8;
                currSendLen = 0;

                if(currMsgBody.msgID != 0)std::cout << "currMsgBody.msgID = " << currMsgBody.msgID << ": ";
                std::cout << "deparser currMsgHeader.keyLen " << currMsgHeader.keyLen << std::endl;
                std::cout << "deparser currMsgHeader.bodyLen " << currMsgHeader.bodyLen << std::endl;

                currMsgHeader.display();
                currMsgBody.display(currMsgHeader.extLen, currMsgHeader.keyLen, currMsgHeader.val_len());
                hdr = currMsgHeader.output_word();
                body = currMsgBody.body;

                txMetaData.write(appTxMeta(currSessionID, requiredSendLen));

                lockReqFifo.write(lockReq(currMsgBody.msgID, LOCK_ACQUIRE));
    			esac_fsmState = WAITING_LOCK;
    		}
    		break;
        }
        // TODO: need to check txStatus. 
        case WAITING_LOCK: {
            if(!grantFifo.empty()){
                ap_uint<1> grant = grantFifo.read();
                if(grant){
                    esac_fsmState = SEND_HDR;
                }
                else{
                    lockReqFifo.write(lockReq(currMsgBody.msgID, LOCK_ACQUIRE));
                }
            }
            break;
        }
        case SEND_HDR: {
    		if (!txData.full())
    		{
    	        net_axis<DATA_WIDTH> currWord;
                if(DATA_WIDTH >= requiredSendLen){
                    ap_uint<32> secondPartLen = requiredSendLen - MEMCACHED_HDRLEN*8;
                    if(currMsgBody.msgID != 0)std::cout << "currMsgBody.msgID = " << currMsgBody.msgID << ": ";
                    std::cout << "secondPartLen " << secondPartLen << std::endl;
                    std::cout << "requiredSendLen " << requiredSendLen << std::endl;
                    
                    if(secondPartLen == 0){
                        currWord.data(DATA_WIDTH-1, DATA_WIDTH-requiredSendLen) = hdr(MEMCACHED_HDRLEN*8-1, 0);
                    }
                    else{
                        // this is not supported in HLS -- causing currWord.data to be all zero, not sure why
                        // currWord.data(DATA_WIDTH-1, DATA_WIDTH-requiredSendLen) = (hdr(MEMCACHED_HDRLEN*8-1, 0), body(MAX_BODY_LEN-1, MAX_BODY_LEN-secondPartLen));
                        currWord.data(DATA_WIDTH-1, DATA_WIDTH-MEMCACHED_HDRLEN*8) = hdr;
                        currWord.data(DATA_WIDTH-MEMCACHED_HDRLEN*8-1, DATA_WIDTH-requiredSendLen) = body(MAX_BODY_LEN-1, MAX_BODY_LEN-secondPartLen);
                    }
                    
                    currWord.keep = lenToKeep(requiredSendLen/8);
                    currWord.last = 1;
            		txData.write(currWord);

                    currSendLen += requiredSendLen;
                    
                    lockReqFifo.write(lockReq(currMsgBody.msgID, LOCK_RELEASE));
                    esac_fsmState = IDLE;
                }
                else{
                    ap_uint<32> secondPartLen = DATA_WIDTH - MEMCACHED_HDRLEN*8;
                    if(currMsgBody.msgID != 0)std::cout << "currMsgBody.msgID = " << currMsgBody.msgID << ": ";
                    std::cout << "secondPartLen " << secondPartLen << std::endl;
                    
                    // currWord.data = (hdr, body(MAX_BODY_LEN-1, MAX_BODY_LEN-secondPartLen));
                    currWord.data(DATA_WIDTH-1, DATA_WIDTH-MEMCACHED_HDRLEN*8) = hdr;
                    currWord.data(DATA_WIDTH-MEMCACHED_HDRLEN*8-1, 0) = body(MAX_BODY_LEN-1, MAX_BODY_LEN-secondPartLen);
                    currWord.keep = lenToKeep(DATA_WIDTH/8);
                    currWord.last = 0;
                    txData.write(currWord);

                    currSendLen += DATA_WIDTH;
    				esac_fsmState = SEND_OTHERS;
                }
    		}
    		break;
    	}
        case SEND_OTHERS: {
            if (!txData.full())
    		{
    	        net_axis<DATA_WIDTH> currWord;
                if(currSendLen < requiredSendLen){
                    ap_uint<16> remainLen = requiredSendLen - currSendLen;
                    ap_uint<16> body_sendingPos = (MAX_BODY_LEN + MEMCACHED_HDRLEN*8 - currSendLen);
                    if(currMsgBody.msgID != 0)std::cout << "currMsgBody.msgID = " << currMsgBody.msgID << ": ";
                    std::cout << "remainLen = " << remainLen << std::endl;

                    if(DATA_WIDTH >= remainLen){
                        currWord.data(DATA_WIDTH-1, DATA_WIDTH-remainLen) = body(body_sendingPos-1, body_sendingPos-remainLen);
                        currWord.keep = lenToKeep(remainLen/8);
                        currWord.last = 1;
                        txData.write(currWord);
        
                        currSendLen += remainLen;
                        lockReqFifo.write(lockReq(currMsgBody.msgID, LOCK_RELEASE));
                        esac_fsmState = IDLE;
                    }
                    else{
                        currWord.data = body(body_sendingPos-1, body_sendingPos-DATA_WIDTH);
                        currWord.keep = lenToKeep(DATA_WIDTH/8);
                        currWord.last = 0;
                        txData.write(currWord);
        
                        currSendLen += DATA_WIDTH;
                        esac_fsmState = SEND_OTHERS;
                    }
                }
                else{
                    lockReqFifo.write(lockReq(currMsgBody.msgID, LOCK_RELEASE));
                    esac_fsmState = IDLE;
                }
            }
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
    // you must use each FIFO in order to make it get synthesized into IP
    if(!closeConnection.full()){
    	closeConnection.write(0);
    }

    if (!txStatus.empty()) //Make Checks
	{
		txStatus.read();
	}
    
    if(!m_axis_upd_rsp.empty()){
        hash_table_16_1024::htUpdateResp<16,1024> response = m_axis_upd_rsp.read();
        if (!response.success){
            std::cout << "[ERROR] update failed" << std::endl;
        }
    }   
    if(!regInsertFailureCount.empty()){
        ap_uint<16> cnt = regInsertFailureCount.read();
        std::cout << "[ERROR] insert failed cnt1 " << cnt << std::endl;
    }
    if(!regInsertFailureCount2.empty()){
        ap_uint<16> cnt = regInsertFailureCount2.read();
        std::cout << "[ERROR] insert failed cnt2 " << cnt << std::endl;
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


	static hls::stream<ap_uint<MAX_KEY_LEN> >		mc_keyFifo("mc_keyFifo");
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
	static hls::stream<sessionID_stream>	mc_sessionIdStreamFifo("mc_sessionIdStreamFifo");
	static hls::stream<ap_uint<32> >	mc_hashValFifo("mc_hashValFifo");
	static hls::stream<ap_uint<16> >	mc_sessionIdFifo2("mc_sessionIdFifo2");
	static hls::stream<ap_uint<16> >	mc_idxFifo("mc_idxFifo");
	static hls::stream<ap_uint<16> >	mc_sessionIdFifo3("mc_sessionIdFifo3");
	static hls::stream<ap_uint<16> >    mc_closedSessionIdFifo("mc_closedSessionIdFifo");
    #pragma HLS stream variable=mc_cmdFifo depth=64
    #pragma HLS stream variable=mc_sessionCountFifo depth=64
    #pragma HLS stream variable=mc_sessionIdStreamFifo depth=64
    #pragma HLS stream variable=mc_hashValFifo depth=64
    #pragma HLS stream variable=mc_sessionIdFifo2 depth=64
    #pragma HLS stream variable=mc_idxFifo depth=64
    #pragma HLS stream variable=mc_sessionIdFifo3 depth=64
    #pragma HLS stream variable=mc_closedSessionIdFifo depth=64

    static hls::stream<lockReq> m_lockReqFifo_rx("m_lockReqFifo_rx");
    static hls::stream<ap_uint<1> > m_grantFifo_rx("m_grantFifo_rx");
    #pragma HLS stream variable=m_lockReqFifo_rx depth=64
    #pragma HLS DATA_PACK variable=m_lockReqFifo_rx
    #pragma HLS stream variable=m_grantFifo_rx depth=64

    static hls::stream<lockReq> m_lockReqFifo_tx("m_lockReqFifo_tx");
    static hls::stream<ap_uint<1> > m_grantFifo_tx("m_grantFifo_tx");
    #pragma HLS stream variable=m_lockReqFifo_tx depth=64
    #pragma HLS DATA_PACK variable=m_lockReqFifo_tx
    #pragma HLS stream variable=m_grantFifo_tx depth=64
    
    // initing a hash table block
    hash_table_16_1024::hash_table_top(s_axis_lup_req, s_axis_upd_req, m_axis_lup_rsp, m_axis_upd_rsp, regInsertFailureCount);
    hash_table_32_32::hash_table_top(s_axis_lup_req1, s_axis_upd_req1, m_axis_lup_rsp1, m_axis_upd_rsp1, regInsertFailureCount1);

    // initing a multi queue block
    // multi_queue<ap_uint<32>, MAX_CONNECTED_SESSIONS, MAX_CONNECTED_SESSIONS*16>(multiQueue_push, multiQueue_pop_req, multiQueue_rsp);
    multi_queue<ap_uint<32>, 65535, 65535*8>(multiQueue_push, multiQueue_pop_req, multiQueue_rsp);

    // opening the mcrouter listening port
	open_port(listenPort, listenPortStatus);

    // connecting to remote memcached;
    conn_manager(openTuples, openConStatus, openConnection, mc_cmdFifo, mc_sessionCountFifo, mc_sessionIdStreamFifo, \
            mc_hashValFifo, mc_sessionIdFifo2, mc_idxFifo, mc_sessionIdFifo3, mc_closedSessionIdFifo);
    
    // handling new data arriving (it might come from a new session), and connection closed
	notification_handler(notifications, readRequest, mc_closedSessionIdFifo);
    
    // read data from network and parse them into lockReqFifo
    parser(rxMetaData, rxData, mc_sessionIdFifo0, mc_msgHeaderFifo0, mc_msgBodyFifo0, \
            s_axis_lup_req, s_axis_upd_req, m_axis_lup_rsp, m_lockReqFifo_rx, m_grantFifo_rx);
    // locking data tx. 
    rxLock(m_lockReqFifo_rx, m_grantFifo_rx);

    // simple hash function 
    simple_hash(mc_keyFifo, mc_hashFifo);

	// 1) read request from fifo, determine the destination mc, query mc
    // 2) read response from fifo, forward response to client. 
    proxy(mc_sessionIdFifo0, mc_msgHeaderFifo0, mc_msgBodyFifo0, mc_sessionIdFifo1, mc_msgHeaderFifo1, mc_msgBodyFifo1, \
            mc_keyFifo, mc_hashFifo, multiQueue_push, multiQueue_pop_req, multiQueue_rsp, \
            s_axis_lup_req1, s_axis_upd_req1, m_axis_lup_rsp1, m_axis_upd_rsp1,\
            mc_cmdFifo, mc_sessionCountFifo, mc_sessionIdStreamFifo, mc_hashValFifo, mc_sessionIdFifo2, mc_idxFifo, mc_sessionIdFifo3);

    // deparsing the res and resp, sending to destination (client or memcached)
    deparser(mc_sessionIdFifo1, mc_msgHeaderFifo1, mc_msgBodyFifo1, txMetaData, txData, m_lockReqFifo_tx, m_grantFifo_tx);
    // locking data tx. 
    txLock(m_lockReqFifo_tx, m_grantFifo_tx);

	dummy(closeConnection, txStatus, m_axis_upd_rsp, regInsertFailureCount, regInsertFailureCount1);

}
