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

// !!! do not specify RAM_T2P_BRAM -- let HLS automatically use register with mux. 
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
    else if(!hashValFifo.empty()){
        ap_uint<32> hashVal = hashValFifo.read();
        // TODO: handling closed connections, or in policy engine
        ap_uint<16> idx = (hashVal&(MAX_CONNECTED_SESSIONS-1)) % sessionCount;
        // ap_uint<16> idx = (hashVal&(MAX_CONNECTED_SESSIONS-1));
        sessionIdFifo2.write(connectedSessions[idx]);
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

void state_recovery(
	hls::stream<net_axis<DATA_WIDTH> >& rxData, // intput 
    hls::stream<htUpdateReq>&     s_axis_upd_req, // output to parser_stateman
    hls::stream<htLookupResp>&    m_axis_lup_rsp, // input from parser_stateman
    hls::stream<htUpdateResp>&    m_axis_upd_rsp, // input from parser_stateman
    hls::stream<ap_uint<16> >& sessionIdFifo // output to notify admission_control
){
    
}

#define NUM_WAITING_Q 2
void admission_control(
    hls::stream<ap_uint<16> >&          rxMetaData, // input
	hls::stream<net_axis<DATA_WIDTH> >& rxData, // intput 
    hls::stream<htLookupReq>&           s_axis_lup_req, // output
    hls::stream<ap_uint<16> >&          sessionIdFifo, // input
	hls::stream<net_axis<DATA_WIDTH> >& rxData_out // output
){
#pragma HLS PIPELINE II=1
#pragma HLS INLINE off
    // static value gets inited to zero by default. 
    // multile queues 
    static std::stream<ap_uint<16> > sessionWaitingQueues[NUM_WAITING_Q];
    #pragma HLS stream variable=sessionWaitingQueues[0] depth=64
    #pragma HLS stream variable=sessionWaitingQueues[1] depth=64
    
    static std::stream<net_axis<DATA_WIDTH> > wordWaitingQueues[NUM_WAITING_Q];
    #pragma HLS stream variable=wordWaitingQueues[0] depth=128
    #pragma HLS stream variable=wordWaitingQueues[1] depth=128
    
    static ap_uint<4> fsmState = 0;
    static ap_uint<16> hashIdx;

    switch(fsmState){
        case 0:{
            if(!sessionIdFifo.empty()){
                ap_uint<16> sessionID = sessionIdFifo.read();
                bool ret = stash_remove(sessionID);
                std::cout << "remove sessionID = " << sessionID << std::endl;
                if(!ret){
                    std::cout << "remove sessionID error" << std::endl;
                }
            }
            else if(!rxMetaData.empty()){
                ap_uint<16> sessionID = rxMetaData.read();
                bool ret = stash_lookup(sessionID);
                if(ret){
                    hashIdx = sessionID & (NUM_WAITING_Q-1);
                    sessionWaitingQueues[hashIdx].write(sessionID);
                    fsmState = 1;
                }
                else{
                    fsmState = 2;
                    // forward sessionId and words to parser. 
                }
            }
            break;
        }
        case 1:{
            if(!rxData.empty()){
                net_axis<DATA_WIDTH> currWord = rxData.read();
                wordWaitingQueues[hashIdx].write(currWord);
                if(currWord.last){
                    fsmState = 0;
                }
            }
            break;
        }
        case 2:{
            if(!rxData.empty()){
                net_axis<DATA_WIDTH> currWord = rxData.read();
                rxData_out.write(currWord);
                if(currWord.last){
                    fsmState = 0;
                }
            }
            break;
        }
    }

}

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
                currMsgBody.extInl = 1;
                currMsgBody.keyInl = 1;
                currMsgBody.valInl = 1;
                
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


void parser(
    hls::stream<ap_uint<16> >& rxMetaData, // input
	hls::stream<net_axis<DATA_WIDTH> >& rxData, // intput 
    hls::stream<ap_uint<16> >& sessionIdFifo, // output
    hls::stream<msgHeader>& msgHeaderFifo, // output
    hls::stream<msgBody>& msgBodyFifo, // output
    hls::stream<parser_htLookupReq>&    s_axis_lup_req, 
    hls::stream<parser_htUpdateReq>&    s_axis_upd_req, 
    hls::stream<parser_htLookupResp>&   m_axis_lup_rsp
){
#pragma HLS PIPELINE II=2
#pragma HLS INLINE off
    // this ASIX transaction should be locked. 
    // rxTransID is not the same as currentMsgID, as one transaction might include multiple msg. 
    static ap_uint<32> rxTransID = 1;
 
    static ap_uint<32> currRxTransID = 1;

    enum axisFsmType {IDLE, RECOVER_STATE, READ_WORD, PARSE_WORD};
#ifndef __SYNTHESIS__
    const char* axisFsmName[4] = {"IDLE", "RECOVER_STATE", "READ_WORD", "PARSE_WORD"};
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
                s_axis_lup_req.write(parser_htLookupReq(currSessionID, 0));
                currAxisState = RECOVER_STATE;
            }
            break;
        }
        // get the session-specific parsing state
        case RECOVER_STATE: {
            if(!m_axis_lup_rsp.empty()){
                parser_htLookupResp response = m_axis_lup_rsp.read();
                if(response.hit){
                    currSessionState = response.value1;
                    currMsgBody = response.value2;
                }
                else{ // this is the first word for this session. 
                    currSessionState.reset();
                    currMsgBody.reset();
                }
                if(currMsgBody.msgID != 0)std::cout << "currMsgBody.msgID = " << currMsgBody.msgID << ": ";
                std::cout << "RECOVER_STATE " << response.hit << std::endl;
                // we should expect the hash table is enough to handle all active connections; 
                
                std::cout << "currSessionState: " << std::endl;
                currSessionState.display();
            
                currAxisState = READ_WORD;
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
        // consuming currWord 
        case PARSE_WORD: {
            ap_uint<16> currWord_parsingPos = DATA_WIDTH - (currWordValidLen_init - currWordValidLen)*8;
            // parse partial header + body; 
            if(currSessionState.parsingHeaderState == 0){
                ap_uint<5> requiredHeaderLen = MEMCACHED_HDRLEN - currSessionState.currHdrLen;
                if(currWordValidLen < requiredHeaderLen){
                    currSessionState.msgHeaderBuff(requiredHeaderLen*8-1, requiredHeaderLen*8-currWordValidLen*8) = currWord.data(currWord_parsingPos-1, currWord_parsingPos-currWordValidLen*8);
                    currSessionState.currHdrLen += currWordValidLen;
                    currWordValidLen = 0;
                }
                else if(currWordValidLen == requiredHeaderLen){
                    currSessionState.msgHeaderBuff(requiredHeaderLen*8-1, 0) = currWord.data(currWord_parsingPos-1, currWord_parsingPos-requiredHeaderLen*8);
                    currMsgHeader.consume_word(currSessionState.msgHeaderBuff);

                    if(currMsgBody.msgID != 0)std::cout << "currMsgBody.msgID = " << currMsgBody.msgID << ": ";
                    std::cout << "parser receives data" << std::endl;
                    currMsgHeader.display();

                    currSessionState.currHdrLen = MEMCACHED_HDRLEN;
                    currSessionState.parsingHeaderState = 1; // header is parsed. 
                    if(currMsgHeader.bodyLen > 0){
                        currSessionState.requiredLen = currMsgHeader.bodyLen + MEMCACHED_HDRLEN;
                    }
                    else{
                        currSessionState.parsingBodyState = 1;
                    }
                    currWordValidLen = 0;
                }
                else{
                    currSessionState.msgHeaderBuff(requiredHeaderLen*8-1, 0) = currWord.data(currWord_parsingPos-1, currWord_parsingPos-requiredHeaderLen*8);
                    currMsgHeader.consume_word(currSessionState.msgHeaderBuff);

                    if(currMsgBody.msgID != 0)std::cout << "currMsgBody.msgID = " << currMsgBody.msgID << ": ";
                    std::cout << "parser receives data" << std::endl;
                    currMsgHeader.display();

                    currWordValidLen -= requiredHeaderLen;
                    currSessionState.currHdrLen += requiredHeaderLen;
                    currSessionState.parsingHeaderState = 1; // header is parsed. 
                    
                    if(currMsgHeader.bodyLen > 0){
                        currSessionState.requiredLen = currMsgHeader.bodyLen + MEMCACHED_HDRLEN;
                        std::cout << "currSessionState.requiredLen=" << currSessionState.requiredLen 
                            << " MEMCACHED_HDRLEN + currSessionState.currBodyLen=" << MEMCACHED_HDRLEN + currSessionState.currBodyLen << std::endl;
                        
                        // parsingMsgBody(currSessionState, currMsgBody, currMsgHeader, currWord, currWordValidLen, currWordValidLen_init);
                        ap_uint<16> currWord_parsingPos = DATA_WIDTH - (currWordValidLen_init - currWordValidLen)*8;
                        ap_uint<32> requiredBodyLen = currMsgHeader.bodyLen;

                        if(currMsgBody.msgID != 0)std::cout << "currMsgBody.msgID = " << currMsgBody.msgID << ": ";
                        std::cout << "currWordValidLen=" << currWordValidLen << std::endl;
                        std::cout << "currBodyLen=" << 0 << " requiredBodyLen=" << requiredBodyLen << std::endl;
                        
                        if(currWordValidLen >= requiredBodyLen){
                            currMsgBody.body(MAX_BODY_LEN-1, MAX_BODY_LEN-requiredBodyLen*8) = currWord.data(currWord_parsingPos-1, currWord_parsingPos-requiredBodyLen*8);
                            currMsgBody.extInl = 1;
                            currMsgBody.keyInl = 1;
                            currMsgBody.valInl = 1;
                            
                            currSessionState.parsingBodyState = 1; // body is parsed
                            currWordValidLen -= requiredBodyLen;
                        }
                        else{
                            currMsgBody.body(MAX_BODY_LEN-1, MAX_BODY_LEN-currWordValidLen*8) = currWord.data(currWord_parsingPos-1, currWord_parsingPos-currWordValidLen*8);
                            currSessionState.currBodyLen += currWordValidLen;
                            currWordValidLen = 0;
                        }
                    }
                    else{
                        currSessionState.parsingBodyState = 1;
                    }
                }
            }
            // parse partial body; 
            else{
                currMsgHeader.consume_word(currSessionState.msgHeaderBuff);
                std::cout << "currSessionState.requiredLen=" << currSessionState.requiredLen 
                    << " MEMCACHED_HDRLEN + currSessionState.currBodyLen=" << MEMCACHED_HDRLEN + currSessionState.currBodyLen << std::endl;

                // parsingMsgBody(currSessionState, currMsgBody, currMsgHeader, currWord, currWordValidLen, currWordValidLen_init);
                ap_uint<16> currWord_parsingPos = DATA_WIDTH - (currWordValidLen_init - currWordValidLen)*8;
                ap_uint<32> currBodyLen = currSessionState.currBodyLen;
                ap_uint<32> requiredBodyLen = currSessionState.requiredLen - MEMCACHED_HDRLEN - currBodyLen;

                if(currMsgBody.msgID != 0)std::cout << "currMsgBody.msgID = " << currMsgBody.msgID << ": ";
                std::cout << "currWordValidLen=" << currWordValidLen << std::endl;
                std::cout << "currBodyLen=" << currBodyLen << " requiredBodyLen=" << requiredBodyLen << std::endl;
                
                if(currWordValidLen >= requiredBodyLen){
                    currMsgBody.body(MAX_BODY_LEN-1-currBodyLen*8, MAX_BODY_LEN-currBodyLen*8-requiredBodyLen*8) = currWord.data(currWord_parsingPos-1, currWord_parsingPos-requiredBodyLen*8);
                    currMsgBody.extInl = 1;
                    currMsgBody.keyInl = 1;
                    currMsgBody.valInl = 1;
                    
                    currSessionState.parsingBodyState = 1; // body is parsed
                    currWordValidLen -= requiredBodyLen;
                }
                else{
                    currMsgBody.body(MAX_BODY_LEN-1-currBodyLen*8, MAX_BODY_LEN-currBodyLen*8-currWordValidLen*8) = currWord.data(currWord_parsingPos-1, currWord_parsingPos-currWordValidLen*8);
                    currSessionState.currBodyLen += currWordValidLen;
                    currWordValidLen = 0;
                }
            }

            if(currMsgBody.msgID != 0)std::cout << "currMsgBody.msgID = " << currMsgBody.msgID << ": ";
            std::cout << "currWordValidLen=" << currWordValidLen << std::endl;

            ap_uint<1> parsingMsgState = (currSessionState.parsingHeaderState == 1 && currSessionState.parsingBodyState == 1);
            
            if(currWordValidLen > 0 || (currWordValidLen == 0 && currWord.last != 1)){
                if(parsingMsgState){
                    if(currMsgBody.msgID != 0)std::cout << "currMsgBody.msgID = " << currMsgBody.msgID << ": ";
                    std::cout << "writing currMsgBody (path1) for currMsgBody.msgID " << currMsgBody.msgID << std::endl;

                    // !!! only write out when current msg is parsed
                    msgHeaderFifo.write(currMsgHeader);
                    msgBodyFifo.write(currMsgBody);
                    sessionIdFifo.write(currSessionID);

                    currMsgHeader.display();
                    currMsgBody.display(currMsgHeader.extLen, currMsgHeader.keyLen, currMsgHeader.val_len());

                    currSessionState.reset(); // ready to parse next message
                    // currMsgBody.msgID = currentMsgID;
                    // currentMsgID++;
                }
                if(currWordValidLen > 0){
                    currAxisState = PARSE_WORD;
                }
                else{
                    currAxisState = READ_WORD;
                }
            }
            else if(currWord.last == 1){
                if(parsingMsgState){
                    if(currMsgBody.msgID != 0)std::cout << "currMsgBody.msgID = " << currMsgBody.msgID << ": ";
                    std::cout << "writing currMsgBody for (path2) currMsgBody.msgID " << currMsgBody.msgID << std::endl;
                    
                    // !!! only write out when current msg is parsed
                    msgHeaderFifo.write(currMsgHeader);
                    msgBodyFifo.write(currMsgBody);
                    sessionIdFifo.write(currSessionID);
                    
                    currMsgHeader.display();
                    currMsgBody.display(currMsgHeader.extLen, currMsgHeader.keyLen, currMsgHeader.val_len());

                    // In the end of each AXIS, store sessionState and currMsgBody back. 
                    currSessionState.reset();
                    currMsgBody.reset();
                    // currMsgBody.msgID = currentMsgID;
                    // currentMsgID++;
                    // !!! you could also delete the current session context, but it will increase the possibility of KV_UPDATE_INSERT. 
                    s_axis_upd_req.write(parser_htUpdateReq(currSessionID, currSessionState, currMsgBody, 0));
                }
                else{
                    if(currMsgBody.msgID != 0)std::cout << "currMsgBody.msgID = " << currMsgBody.msgID << ": ";
                    std::cout << "storing sessionState and currMsgBody back: currSessionID " << currSessionID << " currMsgBody.msgID " << currMsgBody.msgID << std::endl;
                    
                    // In the end of each AXIS, store sessionState and currMsgBody back. 
                    s_axis_upd_req.write(parser_htUpdateReq(currSessionID, currSessionState,currMsgBody, 0));
                    // TODO: optimization: implementing hash table with a stash, such that read and write can finish in one cycle. 
                    // TODO: or store currMsgBody in URAM. 
                }
                currAxisState = IDLE;
            }
            break;
        }
    }
}

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
    
    // generating globally unique msgID. 
    static ap_uint<32> currentMsgID = 1;

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

                // generating msgID for each parsed msg. 
                currMsgBody.msgID = currentMsgID;
                currentMsgID += 1;

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

void deparser(
    hls::stream<ap_uint<16> >& sessionIdFifo, // input
    hls::stream<msgHeader>& msgHeaderFifo, // input
    hls::stream<msgBody>& msgBodyFifo, // input
    hls::stream<appTxMeta>& txMetaData, // ouput
	hls::stream<appTxRsp>& txStatus, // ouput
    hls::stream<net_axis<DATA_WIDTH> >& txData // ouput
){
#pragma HLS PIPELINE II=1
#pragma HLS INLINE off

    static ap_uint<32> requiredSendLen;
    static ap_uint<32> currSendLen;
    static ap_uint<MEMCACHED_HDRLEN*8+MAX_BODY_LEN> sendBuf; // 1024
    
    static ap_uint<16> currSessionID;
    static msgHeader currMsgHeader;
    static msgBody currMsgBody;

    enum deparser_fsmType{IDLE=0, WAIT_TX_META, WAIT_TX_STATUS, SEND_WORD};
#ifndef __SYNTHESIS__
    const char* deparser_fsmName[4] = {"IDLE", "WAIT_TX_META", "WAIT_TX_STATUS", "SEND_WORD"};
#endif
    static deparser_fsmType fsmState = IDLE;

#ifndef __SYNTHESIS__
    if(currMsgBody.msgID != 0)std::cout << "currMsgBody.msgID = " << currMsgBody.msgID << ": ";
    std::cout << "mcrouter::deparser fsmState " << deparser_fsmName[(int)fsmState] << std::endl;
#endif
	switch (fsmState){
    	case IDLE:{
    		if (!sessionIdFifo.empty() && !msgHeaderFifo.empty() && !msgBodyFifo.empty())
    		{
                currSessionID = sessionIdFifo.read();
                currMsgHeader = msgHeaderFifo.read();;
                currMsgBody = msgBodyFifo.read();;

                requiredSendLen = MEMCACHED_HDRLEN*8 + currMsgHeader.bodyLen*8;
                currSendLen = 0;

                if(currMsgBody.msgID != 0)std::cout << "currMsgBody.msgID = " << currMsgBody.msgID << ": ";
                std::cout << "deparser currMsgHeader.keyLen " << currMsgHeader.keyLen << std::endl;
                std::cout << "deparser currMsgHeader.bodyLen " << currMsgHeader.bodyLen << std::endl;

                currMsgHeader.display();
                currMsgBody.display(currMsgHeader.extLen, currMsgHeader.keyLen, currMsgHeader.val_len());
                sendBuf(MAX_BODY_LEN + MEMCACHED_HDRLEN*8-1, MAX_BODY_LEN) = currMsgHeader.output_word();
                sendBuf(MAX_BODY_LEN -1, 0) = currMsgBody.body;

                fsmState = WAIT_TX_META;       
    		}
    		break;
        }
        // ??? why putting to to IDLE will cause simulation fail? 
        case WAIT_TX_META: {
            // if(!txMetaData.full()){ 
            // !!! for AXIS interface, we do not need to check whether it is full(), but we do need to check whether it is empty() (ie, valid); 
            std::cout << "deparser_send: preparing to send" << std::endl;
            txMetaData.write(appTxMeta(currSessionID, requiredSendLen));
            fsmState = WAIT_TX_STATUS;
            break;
            // }
        }
        // !!! txMetaData.write() must follow txStatus.read() immediately, as they are AXIS interface not FIFO -- data will be overwritten. 
        case WAIT_TX_STATUS: {
            // TODO: retries later if no available space
            // TODO: solving HOL blocking. 
            if(!txStatus.empty()){
                appTxRsp txRsp = txStatus.read();
                std::cout << "txRsp.remaining_space = " << txRsp.remaining_space << std::endl;
                if(txRsp.remaining_space >= requiredSendLen){
                    fsmState = SEND_WORD;
                }
            }
            break;
        }
        case SEND_WORD:{
            if (!txData.full())
    		{
                net_axis<DATA_WIDTH> currWord;
                
                ap_uint<16> remainLen = requiredSendLen - currSendLen;
                ap_uint<16> body_sendingPos = (MAX_BODY_LEN + MEMCACHED_HDRLEN*8 - currSendLen);
                if(currMsgBody.msgID != 0)std::cout << "currMsgBody.msgID = " << currMsgBody.msgID << ": ";
                std::cout << "remainLen = " << remainLen << std::endl;

                if(DATA_WIDTH >= remainLen){
                    // currWord.data(DATA_WIDTH-1, DATA_WIDTH-remainLen) = sendBuf(body_sendingPos-1, body_sendingPos-remainLen);
                    // currWord.data = sendBuf(body_sendingPos-1, body_sendingPos-DATA_WIDTH);
                    currWord.data = sendBuf(MAX_BODY_LEN + MEMCACHED_HDRLEN*8-1, MAX_BODY_LEN + MEMCACHED_HDRLEN*8-DATA_WIDTH);
                    currWord.keep = lenToKeep(remainLen/8);
                    currWord.last = 1;
    
                    currSendLen += remainLen;
                    fsmState = IDLE;
                }
                else{
                    // currWord.data = sendBuf(body_sendingPos-1, body_sendingPos-DATA_WIDTH);
                    currWord.data = sendBuf(MAX_BODY_LEN + MEMCACHED_HDRLEN*8-1, MAX_BODY_LEN + MEMCACHED_HDRLEN*8-DATA_WIDTH);
                    currWord.keep = lenToKeep(DATA_WIDTH/8);
                    currWord.last = 0;
                    sendBuf <<= DATA_WIDTH;
                    currSendLen += DATA_WIDTH;
                    fsmState = SEND_WORD;
                }
                txData.write(currWord);
    		}
    		break;
        }
    }
}

void dummy(	
    hls::stream<ap_uint<16> >& closeConnection,
    hls::stream<parser_htUpdateResp>&  m_axis_upd_rsp,
    hls::stream<ap_uint<16> >& regInsertFailureCount,
    hls::stream<ap_uint<16> >& regInsertFailureCount2
){
#pragma HLS PIPELINE II=1
#pragma HLS INLINE off
    // you must use each FIFO in order to make it get synthesized into IP
    if(!closeConnection.full()){
    	closeConnection.write(0);
    }

    if(!m_axis_upd_rsp.empty()){
        parser_htUpdateResp response = m_axis_upd_rsp.read();
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

    // hls will infer it as registers
    #pragma HLS ARRAY_PARTITION variable=stashTable complete

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

	static hls::stream<ap_uint<16> >		mc_sessionIdFifo4("mc_sessionIdFifo4");
	static hls::stream<ap_uint<16> >		mc_sessionIdFifo5("mc_sessionIdFifo5");
    static hls::stream<ap_uint<16> >		mc_sessionIdFifo6("mc_sessionIdFifo6");
	static hls::stream<ap_uint<16> >		mc_sessionIdFifo7("mc_sessionIdFifo7");
    static hls::stream<msgHeader>		    mc_msgHeaderFifo4("mc_msgHeaderFifo4");
    static hls::stream<msgHeader>		    mc_msgHeaderFifo5("mc_msgHeaderFifo5");
	static hls::stream<msgHeader>		    mc_msgHeaderFifo6("mc_msgHeaderFifo6");
    static hls::stream<msgHeader>		    mc_msgHeaderFifo7("mc_msgHeaderFifo7");
	static hls::stream<msgBody>		        mc_msgBodyFifo4("mc_msgBodyFifo4");
	static hls::stream<msgBody>		        mc_msgBodyFifo5("mc_msgBodyFifo5");
    static hls::stream<msgBody>		        mc_msgBodyFifo6("mc_msgBodyFifo6");
	static hls::stream<msgBody>		        mc_msgBodyFifo7("mc_msgBodyFifo7");
    #pragma HLS stream variable=mc_sessionIdFifo4 depth=64
    #pragma HLS stream variable=mc_sessionIdFifo5 depth=64
    #pragma HLS stream variable=mc_sessionIdFifo6 depth=64
    #pragma HLS stream variable=mc_sessionIdFifo7 depth=64
    #pragma HLS stream variable=mc_msgHeaderFifo4 depth=64
    #pragma HLS stream variable=mc_msgHeaderFifo5 depth=64
    #pragma HLS stream variable=mc_msgHeaderFifo6 depth=64
    #pragma HLS stream variable=mc_msgHeaderFifo7 depth=64
    #pragma HLS stream variable=mc_msgBodyFifo4 depth=64
    #pragma HLS stream variable=mc_msgBodyFifo5 depth=64
    #pragma HLS stream variable=mc_msgBodyFifo6 depth=64
    #pragma HLS stream variable=mc_msgBodyFifo7 depth=64


	static hls::stream<ap_uint<MAX_KEY_LEN> >		mc_keyFifo("mc_keyFifo");
	static hls::stream<ap_uint<32> >		mc_hashFifo("mc_hashFifo");
    #pragma HLS stream variable=mc_keyFifo depth=64
    #pragma HLS stream variable=mc_hashFifo depth=64


    static hls::stream<parser_htLookupReq>    s_axis_lup_req;
    static hls::stream<parser_htUpdateReq>    s_axis_upd_req;
    static hls::stream<parser_htLookupResp>   m_axis_lup_rsp;
    static hls::stream<parser_htUpdateResp>   m_axis_upd_rsp;
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


    static hls::stream<ap_uint<MEMCACHED_HDRLEN*8+MAX_BODY_LEN> > mc_sendBufFifo("mc_sendBufFifo");
    #pragma HLS stream variable=mc_sendBufFifo depth=64
    static hls::stream<ap_uint<32> > mc_lengthFifo("mc_lengthFifo");
    #pragma HLS stream variable=mc_lengthFifo depth=64
    static hls::stream<ap_uint<32> > mc_msgIDFifo("mc_msgIDFifo");
    #pragma HLS stream variable=mc_msgIDFifo depth=64
	static hls::stream<ap_uint<16> >	mc_sessionIdFifo8("mc_sessionIdFifo8");
    #pragma HLS stream variable=mc_sessionIdFifo8 depth=64

    // initing a hash table block
    parser_stateman_top(s_axis_lup_req, s_axis_upd_req, m_axis_lup_rsp, m_axis_upd_rsp, regInsertFailureCount);
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
    
    // read data from network and parse them into msgFifo
    parser(rxMetaData, rxData, mc_sessionIdFifo0, mc_msgHeaderFifo0, mc_msgBodyFifo0, \
            s_axis_lup_req, s_axis_upd_req, m_axis_lup_rsp);
    
    // simple hash function 
    simple_hash(mc_keyFifo, mc_hashFifo);

	// 1) read request from fifo, determine the destination mc, query mc
    // 2) read response from fifo, forward response to client. 
    proxy(mc_sessionIdFifo0, mc_msgHeaderFifo0, mc_msgBodyFifo0, mc_sessionIdFifo1, mc_msgHeaderFifo1, mc_msgBodyFifo1, \
            mc_keyFifo, mc_hashFifo, multiQueue_push, multiQueue_pop_req, multiQueue_rsp, \
            s_axis_lup_req1, s_axis_upd_req1, m_axis_lup_rsp1, m_axis_upd_rsp1,\
            mc_cmdFifo, mc_sessionCountFifo, mc_sessionIdStreamFifo, mc_hashValFifo, mc_sessionIdFifo2, mc_idxFifo, mc_sessionIdFifo3);

    // deparsing the res and resp, sending to destination (client or memcached)
    deparser(mc_sessionIdFifo1, mc_msgHeaderFifo1, mc_msgBodyFifo1, txMetaData, txStatus, txData);

	dummy(closeConnection, m_axis_upd_rsp, regInsertFailureCount, regInsertFailureCount1);

}
