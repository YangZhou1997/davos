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

// consume currWord;
// update currSessionState, currMsgBody, currMsgHeader, currWordValidLen, currWordParsingPos;
void parseMsgHeader(sessionState& currSessionState, msgBody& currMsgBody, msgHeader& currMsgHeader, net_axis<DATA_WIDTH>& currWord, \
    ap_uint<32>& currWordValidLen, ap_uint<32>& currWordParsingPos, ap_uint<32>& currWordValidLen1, ap_uint<32>& currWordParsingPos1, ap_uint<16>& currSessionID, \
    hls::stream<ap_uint<16> >& sessionIdFifo, hls::stream<msgHeader>& msgHeaderFifo, hls::stream<msgBody>& msgBodyFifo, \
    ap_uint<4>& ret
){
#pragma HLS INLINE
    // not enough to cover a header 
    if(currWordValidLen < MEMCACHED_HDRLEN){
        currSessionState.msgHeaderBuff(MEMCACHED_HDRLEN*8-1, MEMCACHED_HDRLEN*8-currWordValidLen*8) = 
            currWord.data(currWordParsingPos-1, currWordParsingPos-currWordValidLen*8);
        
        currSessionState.parsingHeaderState = 0;
        currSessionState.currHdrLen = currWordValidLen;
        currSessionState.currBodyLen = 0;

        currWordValidLen1 = currWordValidLen;
        currWordParsingPos1 = currWordParsingPos;

        ret = 0;
    }
    // exactly covering a header
    if(currWordValidLen == MEMCACHED_HDRLEN && currMsgHeader.bodyLen > 0){
        currSessionState.msgHeaderBuff(MEMCACHED_HDRLEN*8-1, MEMCACHED_HDRLEN*8-currWordValidLen*8) = 
            currWord.data(currWordParsingPos-1, currWordParsingPos-currWordValidLen*8);
        currMsgHeader.consume_word(currSessionState.msgHeaderBuff);

        currSessionState.parsingHeaderState = 1;
        currSessionState.requiredLen = MEMCACHED_HDRLEN + currMsgHeader.bodyLen;
        currSessionState.currHdrLen = MEMCACHED_HDRLEN;
        currSessionState.currBodyLen = 0;
        
        ret = 1;
        
        currWordValidLen1 = currWordValidLen;
        currWordParsingPos1 = currWordParsingPos;    
    }
    // exactly covering a header with body or more than a header. 
    if((currWordValidLen == MEMCACHED_HDRLEN && currMsgHeader.bodyLen == 0) || currWordValidLen > MEMCACHED_HDRLEN){
        currSessionState.msgHeaderBuff(MEMCACHED_HDRLEN*8-1, 0) = 
            currWord.data(currWordParsingPos-1, currWordParsingPos-MEMCACHED_HDRLEN*8);
        currMsgHeader.consume_word(currSessionState.msgHeaderBuff);

        if(currWordValidLen > MEMCACHED_HDRLEN){
            currSessionState.parsingHeaderState = 1;
            currSessionState.requiredLen = MEMCACHED_HDRLEN + currMsgHeader.bodyLen;
            currSessionState.currHdrLen = MEMCACHED_HDRLEN;
            currSessionState.currBodyLen = 0;
            
            currWordValidLen1 = currWordValidLen - MEMCACHED_HDRLEN;
            currWordParsingPos1 = currWordParsingPos - MEMCACHED_HDRLEN*8;
        }
        else{
            currWordValidLen1 = currWordValidLen;
            currWordParsingPos1 = currWordParsingPos;
        }

        ret = 2;
    }
    
}

// consume currMsgHeader, currWord; 
// update currSessionState, currMsgBody, currWordValidLen, currWordParsingPos;
void parseMsgBody(sessionState& currSessionState, msgBody& currMsgBody, msgHeader& currMsgHeader, net_axis<DATA_WIDTH>& currWord, \
    ap_uint<32>& currWordValidLen, ap_uint<32>& currWordParsingPos, ap_uint<32>& currWordValidLen1, ap_uint<32>& currWordParsingPos1, ap_uint<16>& currSessionID, \
    hls::stream<ap_uint<16> >& sessionIdFifo, hls::stream<msgHeader>& msgHeaderFifo, hls::stream<msgBody>& msgBodyFifo, \
    ap_uint<4>& ret
){
#pragma HLS INLINE

    // require more word to parse body
    if(currMsgHeader.bodyLen > 0 && currWordValidLen < currMsgHeader.bodyLen){
        currMsgBody.body(MAX_BODY_LEN-1, MAX_BODY_LEN-currWordValidLen*8) = 
            currWord.data(currWordParsingPos-1, currWordParsingPos-currWordValidLen*8);
        
        currSessionState.parsingHeaderState = 1;
        currSessionState.requiredLen = MEMCACHED_HDRLEN + currMsgHeader.bodyLen;
        currSessionState.currHdrLen = MEMCACHED_HDRLEN;
        currSessionState.currBodyLen = currWordValidLen;
    
        currWordValidLen1 = currWordValidLen;
        currWordParsingPos1 = currWordParsingPos;

        ret = 0;
    }
    // exactly parsing body done
    if(currMsgHeader.bodyLen > 0 && currWordValidLen == currMsgHeader.bodyLen){
        currMsgBody.body(MAX_BODY_LEN-1, MAX_BODY_LEN-currWordValidLen*8) = 
            currWord.data(currWordParsingPos-1, currWordParsingPos-currWordValidLen*8);
        
        currMsgBody.extInl = 1;
        currMsgBody.keyInl = 1;
        currMsgBody.valInl = 1;
        
        sessionIdFifo.write(currSessionID);
        msgHeaderFifo.write(currMsgHeader);
        msgBodyFifo.write(currMsgBody);

        currSessionState.reset();

        currWordValidLen1 = currWordValidLen;
        currWordParsingPos1 = currWordParsingPos;

        ret = 1;
    }
    // more than a body or no body
    if((currMsgHeader.bodyLen > 0 && currWordValidLen > currMsgHeader.bodyLen) || currMsgHeader.bodyLen == 0){
        if(currMsgHeader.bodyLen > 0){
            currMsgBody.body(MAX_BODY_LEN-1, MAX_BODY_LEN-currMsgHeader.bodyLen*8) = 
                currWord.data(currWordParsingPos-1, currWordParsingPos-currMsgHeader.bodyLen*8);
            currWordValidLen1 = currWordValidLen - currMsgHeader.bodyLen;
            currWordParsingPos1 = currWordParsingPos - currMsgHeader.bodyLen*8;
        }
        else{
            currWordValidLen1 = currWordValidLen;
            currWordParsingPos1 = currWordParsingPos;
        }
        currMsgBody.extInl = 1;
        currMsgBody.keyInl = 1;
        currMsgBody.valInl = 1;

        sessionIdFifo.write(currSessionID);
        msgHeaderFifo.write(currMsgHeader);
        msgBodyFifo.write(currMsgBody);

        currSessionState.reset();
        ret = 2;   
    }
}

void parser(
    hls::stream<ap_uint<16> >& rxMetaData, // input
	hls::stream<net_axis<DATA_WIDTH> >& rxData, // intput 
    hls::stream<ap_uint<16> >& sessionIdFifo, // output
    hls::stream<msgHeader>& msgHeaderFifo, // output
    hls::stream<msgBody>& msgBodyFifo, // output
    hls::stream<ap_uint<16> >& sessionIdFifo1, // output
    hls::stream<msgHeader>& msgHeaderFifo1, // output
    hls::stream<msgBody>& msgBodyFifo1, // output
    hls::stream<ap_uint<16> >& sessionIdFifo2, // output
    hls::stream<msgHeader>& msgHeaderFifo2, // output
    hls::stream<msgBody>& msgBodyFifo2, // output
    hls::stream<hash_table_16_1024::htLookupReq<16> >&       s_axis_lup_req, 
    hls::stream<hash_table_16_1024::htUpdateReq<16,1024> >&    s_axis_upd_req, 
    hls::stream<hash_table_16_1024::htLookupResp<16,1024> >&   m_axis_lup_rsp
){
#pragma HLS PIPELINE II=1
#pragma HLS INLINE off
    // generating globally unique msgID. 
    static ap_uint<32> currentMsgID = 1;

    // static value gets inited to zero by default. 
    static sessionState sessionStateTable[MAX_SESSION_NUM];
    #pragma HLS RESOURCE variable=sessionStateTable core=RAM_T2P_BRAM
    #pragma HLS DEPENDENCE variable=sessionStateTable inter false

    enum axisFsmType {IDLE, RECOVER_STATE, READ_WORD, PARSE_WORD};
#ifndef __SYNTHESIS__
    const char* axisFsmName[4] = {"IDLE", "RECOVER_STATE", "READ_WORD", "PARSE_WORD"};
#endif
    static axisFsmType currAxisState = IDLE;

	static ap_uint<16>          currSessionID; // 
    static sessionState         currSessionState; // storing currMsgHeader
    #pragma HLS DEPENDENCE variable=currSessionState inter false
    static msgBody              currMsgBody; // storing body, restored from ht
    #pragma HLS DEPENDENCE variable=currMsgBody inter false
    
	static net_axis<DATA_WIDTH> currWord;
    static ap_uint<32>          currWordValidLen_init; // the available length of currWord
    static ap_uint<32>          currWordValidLen; // the current waiting-for-parsing length of currWord
    
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
                    // currMsgBody.reset(); // fresh new start for this sesion. 
                    currMsgBody.msgID = currentMsgID;
                    currentMsgID += 1;
                }
                if(currMsgBody.msgID != 0)std::cout << "currMsgBody.msgID = " << currMsgBody.msgID << ": ";
                std::cout << "RECOVER_STATE " << response.hit << std::endl;
                // we should expect the hash table is enough to handle all active connections; 
                
                // recoverying sessionState. Note that currSessionState also contains the currMsgHeader 
                currSessionState = sessionStateTable[currSessionID];
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
            msgHeader            currMsgHeader;
            msgHeader            currMsgHeader1;
            msgHeader            currMsgHeader2;
            msgHeader            currMsgHeader3;

            msgBody              currMsgBody1;
            msgBody              currMsgBody2;
            msgBody              currMsgBody3;

            sessionState         currSessionState1;
            sessionState         currSessionState2;
            sessionState         currSessionState3;

            ap_uint<32> currWordParsingPos = DATA_WIDTH;

            ap_uint<32> currWordParsingPos1 = DATA_WIDTH;
            ap_uint<32> currWordParsingPos2 = DATA_WIDTH;
            ap_uint<32> currWordParsingPos3 = DATA_WIDTH;
            ap_uint<32> currWordParsingPos4 = DATA_WIDTH;
            ap_uint<32> currWordParsingPos5 = DATA_WIDTH;
            ap_uint<32> currWordParsingPos6 = DATA_WIDTH;
            ap_uint<32> currWordParsingPos7 = DATA_WIDTH;
            ap_uint<32> currWordParsingPos8 = DATA_WIDTH;

            ap_uint<32> currWordValidLen1 = currWordValidLen;
            ap_uint<32> currWordValidLen2 = currWordValidLen;
            ap_uint<32> currWordValidLen3 = currWordValidLen;
            ap_uint<32> currWordValidLen4 = currWordValidLen;
            ap_uint<32> currWordValidLen5 = currWordValidLen;
            ap_uint<32> currWordValidLen6 = currWordValidLen;
            ap_uint<32> currWordValidLen7 = currWordValidLen;
            ap_uint<32> currWordValidLen8 = currWordValidLen;


            ap_uint<32> currWordParsingPos9 = DATA_WIDTH;
            ap_uint<32> currWordParsingPos10 = DATA_WIDTH;
            ap_uint<32> currWordParsingPos11 = DATA_WIDTH;
            ap_uint<32> currWordParsingPos12 = DATA_WIDTH;
            ap_uint<32> currWordParsingPos13 = DATA_WIDTH;
            ap_uint<32> currWordParsingPos14 = DATA_WIDTH;
            ap_uint<32> currWordParsingPos15 = DATA_WIDTH;
            ap_uint<32> currWordParsingPos16 = DATA_WIDTH;

            ap_uint<32> currWordValidLen9 = currWordValidLen;
            ap_uint<32> currWordValidLen10 = currWordValidLen;
            ap_uint<32> currWordValidLen11 = currWordValidLen;
            ap_uint<32> currWordValidLen12 = currWordValidLen;
            ap_uint<32> currWordValidLen13 = currWordValidLen;
            ap_uint<32> currWordValidLen14 = currWordValidLen;
            ap_uint<32> currWordValidLen15 = currWordValidLen;
            ap_uint<32> currWordValidLen16 = currWordValidLen;

            ap_uint<4> ret, ret1, ret2, ret3, ret4, ret5, ret6, ret7, ret8, ret9, ret10;
            ap_uint<32> msgParsingState = 0;

            // !!! we do not need to care parsingBodyState
            // partial header
            if(!currSessionState.parsingHeaderState){
                if(currWordValidLen < MEMCACHED_HDRLEN - currSessionState.currHdrLen){
                    currSessionState.msgHeaderBuff(MEMCACHED_HDRLEN*8-currSessionState.currHdrLen*8-1, MEMCACHED_HDRLEN*8-currSessionState.currHdrLen*8-currWordValidLen*8) = 
                        currWord.data(currWordParsingPos-1, currWordParsingPos-currWordValidLen*8);
                    
                    currSessionState.currHdrLen += currWordValidLen;

                    msgParsingState = 0;
                }
                // exactly covering a header
                else if(currWordValidLen == MEMCACHED_HDRLEN - currSessionState.currHdrLen){
                    currSessionState.msgHeaderBuff(MEMCACHED_HDRLEN*8-currSessionState.currHdrLen*8-1, MEMCACHED_HDRLEN*8-currSessionState.currHdrLen*8-currWordValidLen*8) = 
                        currWord.data(currWordParsingPos-1, currWordParsingPos-currWordValidLen*8);
                    currMsgHeader.consume_word(currSessionState.msgHeaderBuff);

                    if(currMsgHeader.bodyLen > 0){
                        currSessionState.parsingHeaderState = 1;
                        currSessionState.requiredLen = MEMCACHED_HDRLEN + currMsgHeader.bodyLen;
                        currSessionState.currHdrLen = MEMCACHED_HDRLEN;
                        currSessionState.currBodyLen = 0;
                        
                        msgParsingState = 1;
                    }
                    // parsing msg done; 
                    else{
                        currMsgBody.extInl = 1;
                        currMsgBody.keyInl = 1;
                        currMsgBody.valInl = 1;

                        sessionIdFifo.write(currSessionID);
                        msgHeaderFifo.write(currMsgHeader);
                        msgBodyFifo.write(currMsgBody);

                        currSessionState.reset();
                        msgParsingState = 2;
                    }
                }
                // more than a header
                else{
                    currSessionState.msgHeaderBuff(MEMCACHED_HDRLEN*8-currSessionState.currHdrLen*8-1, 0) = 
                        currWord.data(currWordParsingPos-1, currWordParsingPos-(MEMCACHED_HDRLEN-currSessionState.currHdrLen)*8);
                    currMsgHeader.consume_word(currSessionState.msgHeaderBuff);
                    
                    currWordValidLen1 = currWordValidLen - (MEMCACHED_HDRLEN-currSessionState.currHdrLen);
                    currWordParsingPos1 = currWordParsingPos - (MEMCACHED_HDRLEN-currSessionState.currHdrLen)*8;
                    
                    if(currMsgHeader.bodyLen > 0){
                        // require more word to parse body
                        if(currWordValidLen1 < currMsgHeader.bodyLen){
                            currMsgBody.body(MAX_BODY_LEN-1, MAX_BODY_LEN-currWordValidLen1*8) = 
                                currWord.data(currWordParsingPos1-1, currWordParsingPos1-currWordValidLen1*8);
                            
                            currSessionState.parsingHeaderState = 1;
                            currSessionState.requiredLen = MEMCACHED_HDRLEN + currMsgHeader.bodyLen;
                            currSessionState.currHdrLen = MEMCACHED_HDRLEN;
                            currSessionState.currBodyLen = currWordValidLen1;

                            msgParsingState = 3;
                        }
                        // exactly parsing body done
                        else if(currWordValidLen1 == currMsgHeader.bodyLen){
                            currMsgBody.body(MAX_BODY_LEN-1, MAX_BODY_LEN-currWordValidLen1*8) = 
                                currWord.data(currWordParsingPos1-1, currWordParsingPos1-currWordValidLen1*8);
                            
                            currMsgBody.extInl = 1;
                            currMsgBody.keyInl = 1;
                            currMsgBody.valInl = 1;
                            
                            sessionIdFifo.write(currSessionID);
                            msgHeaderFifo.write(currMsgHeader);
                            msgBodyFifo.write(currMsgBody);

                            currSessionState.reset();

                            msgParsingState = 4;
                        }
                        // more than a body or no body
                        // else if((currWordValidLen1 > currMsgHeader.bodyLen)){
                        else{
                            currMsgBody.body(MAX_BODY_LEN-1, MAX_BODY_LEN-currMsgHeader.bodyLen*8) = 
                                currWord.data(currWordParsingPos1-1, currWordParsingPos1-currMsgHeader.bodyLen*8);
                            currWordValidLen2 = currWordValidLen1 - currMsgHeader.bodyLen;
                            currWordParsingPos2 = currWordParsingPos1 - currMsgHeader.bodyLen*8;
                            
                            currMsgBody.extInl = 1;
                            currMsgBody.keyInl = 1;
                            currMsgBody.valInl = 1;

                            sessionIdFifo.write(currSessionID);
                            msgHeaderFifo.write(currMsgHeader);
                            msgBodyFifo.write(currMsgBody);

                            // currSessionState.reset();
                            
                            // not enough to cover a header 
                            if(currWordValidLen2 < MEMCACHED_HDRLEN){
                                currSessionState1.msgHeaderBuff(MEMCACHED_HDRLEN*8-1, MEMCACHED_HDRLEN*8-currWordValidLen2*8) = 
                                    currWord.data(currWordParsingPos2-1, currWordParsingPos2-currWordValidLen2*8);
                                
                                currSessionState1.parsingHeaderState = 0;
                                currSessionState1.currHdrLen = currWordValidLen2;
                                currSessionState1.currBodyLen = 0;

                                msgParsingState = 5;
                            }
                            // exactly covering a header
                            else if(currWordValidLen2 == MEMCACHED_HDRLEN){
                                currSessionState1.msgHeaderBuff(MEMCACHED_HDRLEN*8-1, MEMCACHED_HDRLEN*8-currWordValidLen2*8) = 
                                    currWord.data(currWordParsingPos2-1, currWordParsingPos2-currWordValidLen2*8);
                                currMsgHeader1.consume_word(currSessionState1.msgHeaderBuff);

                                currSessionState1.parsingHeaderState = 1;
                                currSessionState1.requiredLen = MEMCACHED_HDRLEN + currMsgHeader1.bodyLen;
                                currSessionState1.currHdrLen = MEMCACHED_HDRLEN;
                                currSessionState1.currBodyLen = 0;
                                
                                msgParsingState = 6;
                            }
                            // exactly covering a header with body or more than a header. 
                            // else if((currWordValidLen2 == MEMCACHED_HDRLEN && currMsgHeader1.bodyLen == 0) || currWordValidLen2 > MEMCACHED_HDRLEN){
                            else{
                                currSessionState1.msgHeaderBuff(MEMCACHED_HDRLEN*8-1, 0) = 
                                    currWord.data(currWordParsingPos2-1, currWordParsingPos2-MEMCACHED_HDRLEN*8);
                                currMsgHeader1.consume_word(currSessionState1.msgHeaderBuff);

                                currWordValidLen3 = currWordValidLen2 - MEMCACHED_HDRLEN;
                                currWordParsingPos3 = currWordParsingPos2 - MEMCACHED_HDRLEN*8;

                                if(currMsgHeader1.bodyLen > 0){
                                    // currSessionState1.parsingHeaderState = 1;
                                    // currSessionState1.requiredLen = MEMCACHED_HDRLEN + currMsgHeader1.bodyLen;
                                    // currSessionState1.currHdrLen = MEMCACHED_HDRLEN;
                                    // currSessionState1.currBodyLen = 0;

                                    currMsgBody1.msgID = currentMsgID;
                                    currentMsgID += 1;
                                    // require more word to parse body
                                    if(currWordValidLen3 < currMsgHeader1.bodyLen){
                                        currMsgBody1.body(MAX_BODY_LEN-1, MAX_BODY_LEN-currWordValidLen3*8) = 
                                            currWord.data(currWordParsingPos3-1, currWordParsingPos3-currWordValidLen3*8);
                                        
                                        currSessionState1.parsingHeaderState = 1;
                                        currSessionState1.requiredLen = MEMCACHED_HDRLEN + currMsgHeader1.bodyLen;
                                        currSessionState1.currHdrLen = MEMCACHED_HDRLEN;
                                        currSessionState1.currBodyLen = currWordValidLen3;

                                        msgParsingState = 8;
                                    }
                                    // exactly parsing body done
                                    else if(currWordValidLen3 == currMsgHeader1.bodyLen){
                                        currMsgBody1.body(MAX_BODY_LEN-1, MAX_BODY_LEN-currWordValidLen3*8) = 
                                            currWord.data(currWordParsingPos3-1, currWordParsingPos3-currWordValidLen3*8);
                                        
                                        currMsgBody1.extInl = 1;
                                        currMsgBody1.keyInl = 1;
                                        currMsgBody1.valInl = 1;
                                        
                                        sessionIdFifo1.write(currSessionID);
                                        msgHeaderFifo1.write(currMsgHeader1);
                                        msgBodyFifo1.write(currMsgBody1);

                                        currSessionState1.reset();

                                        msgParsingState = 9;
                                    }
                                    // more than a body or no body
                                    // else if((currMsgHeader1.bodyLen > 0 && currWordValidLen3 > currMsgHeader1.bodyLen) || currMsgHeader1.bodyLen == 0){
                                    else{
                                        currMsgBody1.body(MAX_BODY_LEN-1, MAX_BODY_LEN-currMsgHeader1.bodyLen*8) = 
                                            currWord.data(currWordParsingPos3-1, currWordParsingPos3-currMsgHeader1.bodyLen*8);
                                        currWordValidLen4 = currWordValidLen3 - currMsgHeader1.bodyLen;
                                        currWordParsingPos4 = currWordParsingPos3 - currMsgHeader1.bodyLen*8;
                                        
                                        currMsgBody1.extInl = 1;
                                        currMsgBody1.keyInl = 1;
                                        currMsgBody1.valInl = 1;

                                        sessionIdFifo1.write(currSessionID);
                                        msgHeaderFifo1.write(currMsgHeader1);
                                        msgBodyFifo1.write(currMsgBody1);

                                        // currSessionState1.reset();
                                        
                                        // not enough to cover a header 
                                        if(currWordValidLen4 < MEMCACHED_HDRLEN){
                                            currSessionState2.msgHeaderBuff(MEMCACHED_HDRLEN*8-1, MEMCACHED_HDRLEN*8-currWordValidLen4*8) = 
                                                currWord.data(currWordParsingPos4-1, currWordParsingPos4-currWordValidLen4*8);
                                            
                                            currSessionState2.parsingHeaderState = 0;
                                            currSessionState2.currHdrLen = currWordValidLen4;
                                            currSessionState2.currBodyLen = 0;

                                            msgParsingState = 10;
                                        }
                                        // exactly covering a header
                                        else if(currWordValidLen4 == MEMCACHED_HDRLEN){
                                            currSessionState2.msgHeaderBuff(MEMCACHED_HDRLEN*8-1, MEMCACHED_HDRLEN*8-currWordValidLen4*8) = 
                                                currWord.data(currWordParsingPos4-1, currWordParsingPos4-currWordValidLen4*8);
                                            currMsgHeader2.consume_word(currSessionState2.msgHeaderBuff);

                                            currSessionState2.parsingHeaderState = 1;
                                            currSessionState2.requiredLen = MEMCACHED_HDRLEN + currMsgHeader2.bodyLen;
                                            currSessionState2.currHdrLen = MEMCACHED_HDRLEN;
                                            currSessionState2.currBodyLen = 0;
                                            
                                            msgParsingState = 11;
                                        }
                                        // exactly covering a header with body or more than a header. 
                                        else{
                                            currSessionState2.msgHeaderBuff(MEMCACHED_HDRLEN*8-1, 0) = 
                                                currWord.data(currWordParsingPos4-1, currWordParsingPos4-MEMCACHED_HDRLEN*8);
                                            currMsgHeader2.consume_word(currSessionState2.msgHeaderBuff);
                                            currWordValidLen5 = currWordValidLen4 - MEMCACHED_HDRLEN;
                                            currWordParsingPos5 = currWordParsingPos4 - MEMCACHED_HDRLEN*8;

                                            if(currMsgHeader2.bodyLen > 0){

                                                // currSessionState2.parsingHeaderState = 1;
                                                // currSessionState2.requiredLen = MEMCACHED_HDRLEN + currMsgHeader2.bodyLen;
                                                // currSessionState2.currHdrLen = MEMCACHED_HDRLEN;
                                                // currSessionState2.currBodyLen = 0;
                                             
                                                currMsgBody2.msgID = currentMsgID;
                                                currentMsgID += 1;
                                                // require more word to parse body
                                                if(currWordValidLen5 < currMsgHeader2.bodyLen){
                                                    currMsgBody2.body(MAX_BODY_LEN-1, MAX_BODY_LEN-currWordValidLen5*8) = 
                                                        currWord.data(currWordParsingPos5-1, currWordParsingPos5-currWordValidLen5*8);
                                                    
                                                    currSessionState2.parsingHeaderState = 1;
                                                    currSessionState2.requiredLen = MEMCACHED_HDRLEN + currMsgHeader2.bodyLen;
                                                    currSessionState2.currHdrLen = MEMCACHED_HDRLEN;
                                                    currSessionState2.currBodyLen = currWordValidLen5;

                                                    msgParsingState = 13;
                                                }
                                                // exactly parsing body done
                                                else if(currWordValidLen5 == currMsgHeader2.bodyLen){
                                                    currMsgBody2.body(MAX_BODY_LEN-1, MAX_BODY_LEN-currWordValidLen5*8) = 
                                                        currWord.data(currWordParsingPos5-1, currWordParsingPos5-currWordValidLen5*8);
                                                    
                                                    currMsgBody2.extInl = 1;
                                                    currMsgBody2.keyInl = 1;
                                                    currMsgBody2.valInl = 1;
                                                    
                                                    sessionIdFifo2.write(currSessionID);
                                                    msgHeaderFifo2.write(currMsgHeader2);
                                                    msgBodyFifo2.write(currMsgBody2);

                                                    currSessionState2.reset();

                                                    msgParsingState = 14;
                                                }
                                                // more than a body or no body
                                                else{
                                                    currMsgBody2.body(MAX_BODY_LEN-1, MAX_BODY_LEN-currMsgHeader2.bodyLen*8) = 
                                                        currWord.data(currWordParsingPos5-1, currWordParsingPos5-currMsgHeader2.bodyLen*8);
                                                    currWordValidLen6 = currWordValidLen5 - currMsgHeader2.bodyLen;
                                                    currWordParsingPos6 = currWordParsingPos5 - currMsgHeader2.bodyLen*8;
                                                    
                                                    currMsgBody2.extInl = 1;
                                                    currMsgBody2.keyInl = 1;
                                                    currMsgBody2.valInl = 1;

                                                    sessionIdFifo2.write(currSessionID);
                                                    msgHeaderFifo2.write(currMsgHeader2);
                                                    msgBodyFifo2.write(currMsgBody2);

                                                    // currSessionState2.reset();
                                                    
                                                    currMsgBody3.msgID = currentMsgID;
                                                    currentMsgID += 1;
                                                    // currMsgBody3, currMsgHeader3 will not be updated
                                                    if(currWordValidLen6 < MEMCACHED_HDRLEN){
                                                        currSessionState3.msgHeaderBuff(MEMCACHED_HDRLEN*8-1, MEMCACHED_HDRLEN*8-currWordValidLen6*8) = 
                                                            currWord.data(currWordParsingPos6-1, currWordParsingPos6-currWordValidLen6*8);
                                                        
                                                        currSessionState3.parsingHeaderState = 0;
                                                        currSessionState3.currHdrLen = currWordValidLen6;
                                                        currSessionState3.currBodyLen = 0;

                                                        msgParsingState = 15;
                                                    }
                                                    else{
                                                        std::cout << "[ERROR]: parsing error" << std::endl;
                                                    }
                                                }
                                            }
                                            else{
                                                currMsgBody2.extInl = 1;
                                                currMsgBody2.keyInl = 1;
                                                currMsgBody2.valInl = 1;

                                                sessionIdFifo2.write(currSessionID);
                                                msgHeaderFifo2.write(currMsgHeader2);
                                                msgBodyFifo2.write(currMsgBody2);

                                                // currSessionState2.reset();
                                                
                                                currMsgBody3.msgID = currentMsgID;
                                                currentMsgID += 1;
                                                // currMsgBody3, currMsgHeader3 will not be updated
                                                if(currWordValidLen5 < MEMCACHED_HDRLEN){
                                                    currSessionState3.msgHeaderBuff(MEMCACHED_HDRLEN*8-1, MEMCACHED_HDRLEN*8-currWordValidLen5*8) = 
                                                        currWord.data(currWordParsingPos5-1, currWordParsingPos5-currWordValidLen5*8);
                                                    
                                                    currSessionState3.parsingHeaderState = 0;
                                                    currSessionState3.currHdrLen = currWordValidLen5;
                                                    currSessionState3.currBodyLen = 0;

                                                    msgParsingState = 15;
                                                }
                                                else{
                                                    std::cout << "[ERROR]: parsing error" << std::endl;
                                                }
                                            }
                                        }
                                    }
                                }
                                else{
                                    currWordValidLen4 = currWordValidLen3;
                                    currWordParsingPos4 = currWordParsingPos3;

                                    currMsgBody1.extInl = 1;
                                    currMsgBody1.keyInl = 1;
                                    currMsgBody1.valInl = 1;

                                    sessionIdFifo1.write(currSessionID);
                                    msgHeaderFifo1.write(currMsgHeader1);
                                    msgBodyFifo1.write(currMsgBody1);

                                    currSessionState1.reset();
                                    
                                    // not enough to cover a header 
                                    if(currWordValidLen4 < MEMCACHED_HDRLEN){
                                        currSessionState2.msgHeaderBuff(MEMCACHED_HDRLEN*8-1, MEMCACHED_HDRLEN*8-currWordValidLen4*8) = 
                                            currWord.data(currWordParsingPos4-1, currWordParsingPos4-currWordValidLen4*8);
                                        
                                        currSessionState2.parsingHeaderState = 0;
                                        currSessionState2.currHdrLen = currWordValidLen4;
                                        currSessionState2.currBodyLen = 0;

                                        msgParsingState = 10;
                                    }
                                    // exactly covering a header
                                    else if(currWordValidLen4 == MEMCACHED_HDRLEN){
                                        currSessionState2.msgHeaderBuff(MEMCACHED_HDRLEN*8-1, MEMCACHED_HDRLEN*8-currWordValidLen4*8) = 
                                            currWord.data(currWordParsingPos4-1, currWordParsingPos4-currWordValidLen4*8);
                                        currMsgHeader2.consume_word(currSessionState2.msgHeaderBuff);

                                        currSessionState2.parsingHeaderState = 1;
                                        currSessionState2.requiredLen = MEMCACHED_HDRLEN + currMsgHeader2.bodyLen;
                                        currSessionState2.currHdrLen = MEMCACHED_HDRLEN;
                                        currSessionState2.currBodyLen = 0;
                                        
                                        msgParsingState = 11;
                                    }
                                    // exactly covering a header with body or more than a header. 
                                    else{
                                        currSessionState2.msgHeaderBuff(MEMCACHED_HDRLEN*8-1, 0) = 
                                            currWord.data(currWordParsingPos4-1, currWordParsingPos4-MEMCACHED_HDRLEN*8);
                                        currMsgHeader2.consume_word(currSessionState2.msgHeaderBuff);
                                        currWordValidLen5 = currWordValidLen4 - MEMCACHED_HDRLEN;
                                        currWordParsingPos5 = currWordParsingPos4 - MEMCACHED_HDRLEN*8;
                                        
                                        if(currMsgHeader2.bodyLen > 0){
                                            currMsgBody2.msgID = currentMsgID;
                                            currentMsgID += 1;
                                            // require more word to parse body
                                            if(currWordValidLen5 < currMsgHeader2.bodyLen){
                                                currMsgBody2.body(MAX_BODY_LEN-1, MAX_BODY_LEN-currWordValidLen5*8) = 
                                                    currWord.data(currWordParsingPos5-1, currWordParsingPos5-currWordValidLen5*8);
                                                
                                                currSessionState2.parsingHeaderState = 1;
                                                currSessionState2.requiredLen = MEMCACHED_HDRLEN + currMsgHeader2.bodyLen;
                                                currSessionState2.currHdrLen = MEMCACHED_HDRLEN;
                                                currSessionState2.currBodyLen = currWordValidLen5;

                                                msgParsingState = 13;
                                            }
                                            // exactly parsing body done
                                            else if(currWordValidLen5 == currMsgHeader2.bodyLen){
                                                currMsgBody2.body(MAX_BODY_LEN-1, MAX_BODY_LEN-currWordValidLen5*8) = 
                                                    currWord.data(currWordParsingPos5-1, currWordParsingPos5-currWordValidLen5*8);
                                                
                                                currMsgBody2.extInl = 1;
                                                currMsgBody2.keyInl = 1;
                                                currMsgBody2.valInl = 1;
                                                
                                                sessionIdFifo2.write(currSessionID);
                                                msgHeaderFifo2.write(currMsgHeader2);
                                                msgBodyFifo2.write(currMsgBody2);

                                                currSessionState2.reset();

                                                msgParsingState = 14;
                                            }
                                            // more than a body or no body
                                            else{
                                                currMsgBody2.body(MAX_BODY_LEN-1, MAX_BODY_LEN-currMsgHeader2.bodyLen*8) = 
                                                    currWord.data(currWordParsingPos5-1, currWordParsingPos5-currMsgHeader2.bodyLen*8);
                                                currWordValidLen6 = currWordValidLen5 - currMsgHeader2.bodyLen;
                                                currWordParsingPos6 = currWordParsingPos5 - currMsgHeader2.bodyLen*8;
                                                
                                                currMsgBody2.extInl = 1;
                                                currMsgBody2.keyInl = 1;
                                                currMsgBody2.valInl = 1;

                                                sessionIdFifo2.write(currSessionID);
                                                msgHeaderFifo2.write(currMsgHeader2);
                                                msgBodyFifo2.write(currMsgBody2);

                                                currSessionState2.reset();
                                                
                                                currMsgBody3.msgID = currentMsgID;
                                                currentMsgID += 1;
                                                // currMsgBody3, currMsgHeader3 will not be updated
                                                if(currWordValidLen6 < MEMCACHED_HDRLEN){
                                                    currSessionState3.msgHeaderBuff(MEMCACHED_HDRLEN*8-1, MEMCACHED_HDRLEN*8-currWordValidLen6*8) = 
                                                        currWord.data(currWordParsingPos6-1, currWordParsingPos6-currWordValidLen6*8);
                                                    
                                                    currSessionState3.parsingHeaderState = 0;
                                                    currSessionState3.currHdrLen = currWordValidLen6;
                                                    currSessionState3.currBodyLen = 0;

                                                    msgParsingState = 15;
                                                }
                                                else{
                                                    std::cout << "[ERROR]: parsing error" << std::endl;
                                                }
                                            }
                                        }
                                        else{
                                            currMsgBody2.extInl = 1;
                                            currMsgBody2.keyInl = 1;
                                            currMsgBody2.valInl = 1;

                                            sessionIdFifo2.write(currSessionID);
                                            msgHeaderFifo2.write(currMsgHeader2);
                                            msgBodyFifo2.write(currMsgBody2);

                                            currSessionState2.reset();
                                            
                                            currMsgBody3.msgID = currentMsgID;
                                            currentMsgID += 1;
                                            // currMsgBody3, currMsgHeader3 will not be updated
                                            if(currWordValidLen5 < MEMCACHED_HDRLEN){
                                                currSessionState3.msgHeaderBuff(MEMCACHED_HDRLEN*8-1, MEMCACHED_HDRLEN*8-currWordValidLen5*8) = 
                                                    currWord.data(currWordParsingPos5-1, currWordParsingPos5-currWordValidLen5*8);
                                                
                                                currSessionState3.parsingHeaderState = 0;
                                                currSessionState3.currHdrLen = currWordValidLen5;
                                                currSessionState3.currBodyLen = 0;

                                                msgParsingState = 15;
                                            }
                                            else{
                                                std::cout << "[ERROR]: parsing error" << std::endl;
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                    else{
                        currWordValidLen2 = currWordValidLen1;
                        currWordParsingPos2 = currWordParsingPos1;
                    
                        currMsgBody.extInl = 1;
                        currMsgBody.keyInl = 1;
                        currMsgBody.valInl = 1;

                        sessionIdFifo.write(currSessionID);
                        msgHeaderFifo.write(currMsgHeader);
                        msgBodyFifo.write(currMsgBody);

                        currSessionState.reset();
                        
                        // not enough to cover a header 
                        if(currWordValidLen2 < MEMCACHED_HDRLEN){
                            currSessionState1.msgHeaderBuff(MEMCACHED_HDRLEN*8-1, MEMCACHED_HDRLEN*8-currWordValidLen2*8) = 
                                currWord.data(currWordParsingPos2-1, currWordParsingPos2-currWordValidLen2*8);
                            
                            currSessionState1.parsingHeaderState = 0;
                            currSessionState1.currHdrLen = currWordValidLen2;
                            currSessionState1.currBodyLen = 0;

                            currWordValidLen3 = currWordValidLen2;
                            currWordParsingPos3 = currWordParsingPos2;

                            msgParsingState = 5;
                        }
                        // exactly covering a header
                        else if(currWordValidLen2 == MEMCACHED_HDRLEN){
                            currSessionState1.msgHeaderBuff(MEMCACHED_HDRLEN*8-1, MEMCACHED_HDRLEN*8-currWordValidLen2*8) = 
                                currWord.data(currWordParsingPos2-1, currWordParsingPos2-currWordValidLen2*8);
                            currMsgHeader1.consume_word(currSessionState1.msgHeaderBuff);

                            currSessionState1.parsingHeaderState = 1;
                            currSessionState1.requiredLen = MEMCACHED_HDRLEN + currMsgHeader1.bodyLen;
                            currSessionState1.currHdrLen = MEMCACHED_HDRLEN;
                            currSessionState1.currBodyLen = 0;
                            
                            msgParsingState = 6;
                            
                            currWordValidLen3 = currWordValidLen2;
                            currWordParsingPos3 = currWordParsingPos2;    
                        }
                        // exactly covering a header with body or more than a header. 
                        else{
                            currSessionState1.msgHeaderBuff(MEMCACHED_HDRLEN*8-1, 0) = 
                                currWord.data(currWordParsingPos2-1, currWordParsingPos2-MEMCACHED_HDRLEN*8);
                            currMsgHeader1.consume_word(currSessionState1.msgHeaderBuff);

                            if(currWordValidLen2 > MEMCACHED_HDRLEN){
                                currSessionState1.parsingHeaderState = 1;
                                currSessionState1.requiredLen = MEMCACHED_HDRLEN + currMsgHeader1.bodyLen;
                                currSessionState1.currHdrLen = MEMCACHED_HDRLEN;
                                currSessionState1.currBodyLen = 0;
                                
                                currWordValidLen3 = currWordValidLen2 - MEMCACHED_HDRLEN;
                                currWordParsingPos3 = currWordParsingPos2 - MEMCACHED_HDRLEN*8;
                            }
                            else{
                                currWordValidLen3 = currWordValidLen2;
                                currWordParsingPos3 = currWordParsingPos2;
                            }

                            if(currMsgHeader1.bodyLen > 0){
                                currMsgBody1.msgID = currentMsgID;
                                currentMsgID += 1;
                                // require more word to parse body
                                if(currWordValidLen3 < currMsgHeader1.bodyLen){
                                    currMsgBody1.body(MAX_BODY_LEN-1, MAX_BODY_LEN-currWordValidLen3*8) = 
                                        currWord.data(currWordParsingPos3-1, currWordParsingPos3-currWordValidLen3*8);
                                    
                                    currSessionState1.parsingHeaderState = 1;
                                    currSessionState1.requiredLen = MEMCACHED_HDRLEN + currMsgHeader1.bodyLen;
                                    currSessionState1.currHdrLen = MEMCACHED_HDRLEN;
                                    currSessionState1.currBodyLen = currWordValidLen3;

                                    msgParsingState = 8;
                                }
                                // exactly parsing body done
                                else if(currWordValidLen3 == currMsgHeader1.bodyLen){
                                    currMsgBody1.body(MAX_BODY_LEN-1, MAX_BODY_LEN-currWordValidLen3*8) = 
                                        currWord.data(currWordParsingPos3-1, currWordParsingPos3-currWordValidLen3*8);
                                    
                                    currMsgBody1.extInl = 1;
                                    currMsgBody1.keyInl = 1;
                                    currMsgBody1.valInl = 1;
                                    
                                    sessionIdFifo1.write(currSessionID);
                                    msgHeaderFifo1.write(currMsgHeader1);
                                    msgBodyFifo1.write(currMsgBody1);

                                    currSessionState1.reset();

                                    msgParsingState = 9;
                                }
                                // more than a body or no body
                                else{
                                    currMsgBody1.body(MAX_BODY_LEN-1, MAX_BODY_LEN-currMsgHeader1.bodyLen*8) = 
                                        currWord.data(currWordParsingPos3-1, currWordParsingPos3-currMsgHeader1.bodyLen*8);
                                    currWordValidLen4 = currWordValidLen3 - currMsgHeader1.bodyLen;
                                    currWordParsingPos4 = currWordParsingPos3 - currMsgHeader1.bodyLen*8;
                                    
                                    currMsgBody1.extInl = 1;
                                    currMsgBody1.keyInl = 1;
                                    currMsgBody1.valInl = 1;

                                    sessionIdFifo1.write(currSessionID);
                                    msgHeaderFifo1.write(currMsgHeader1);
                                    msgBodyFifo1.write(currMsgBody1);

                                    currSessionState1.reset();
                                    
                                    // not enough to cover a header 
                                    if(currWordValidLen4 < MEMCACHED_HDRLEN){
                                        currSessionState2.msgHeaderBuff(MEMCACHED_HDRLEN*8-1, MEMCACHED_HDRLEN*8-currWordValidLen4*8) = 
                                            currWord.data(currWordParsingPos4-1, currWordParsingPos4-currWordValidLen4*8);
                                        
                                        currSessionState2.parsingHeaderState = 0;
                                        currSessionState2.currHdrLen = currWordValidLen4;
                                        currSessionState2.currBodyLen = 0;

                                        currWordValidLen5 = currWordValidLen4;
                                        currWordParsingPos5 = currWordParsingPos4;

                                        msgParsingState = 10;
                                    }
                                    // exactly covering a header
                                    else if(currWordValidLen4 == MEMCACHED_HDRLEN && currMsgHeader2.bodyLen > 0){
                                        currSessionState2.msgHeaderBuff(MEMCACHED_HDRLEN*8-1, MEMCACHED_HDRLEN*8-currWordValidLen4*8) = 
                                            currWord.data(currWordParsingPos4-1, currWordParsingPos4-currWordValidLen4*8);
                                        currMsgHeader2.consume_word(currSessionState2.msgHeaderBuff);

                                        currSessionState2.parsingHeaderState = 1;
                                        currSessionState2.requiredLen = MEMCACHED_HDRLEN + currMsgHeader2.bodyLen;
                                        currSessionState2.currHdrLen = MEMCACHED_HDRLEN;
                                        currSessionState2.currBodyLen = 0;
                                        
                                        msgParsingState = 11;
                                    }
                                    // exactly covering a header with body or more than a header. 
                                    // else if((currWordValidLen4 == MEMCACHED_HDRLEN && currMsgHeader2.bodyLen == 0) || currWordValidLen4 > MEMCACHED_HDRLEN){
                                    else{
                                        currSessionState2.msgHeaderBuff(MEMCACHED_HDRLEN*8-1, 0) = 
                                            currWord.data(currWordParsingPos4-1, currWordParsingPos4-MEMCACHED_HDRLEN*8);
                                        currMsgHeader2.consume_word(currSessionState2.msgHeaderBuff);
    
                                        currWordValidLen5 = currWordValidLen4 - MEMCACHED_HDRLEN;
                                        currWordParsingPos5 = currWordParsingPos4 - MEMCACHED_HDRLEN*8;

                                        if(currMsgHeader2.bodyLen > 0){
                                            currMsgBody2.msgID = currentMsgID;
                                            currentMsgID += 1;
                                            // require more word to parse body
                                            if(currMsgHeader2.bodyLen > 0 && currWordValidLen5 < currMsgHeader2.bodyLen){
                                                currMsgBody2.body(MAX_BODY_LEN-1, MAX_BODY_LEN-currWordValidLen5*8) = 
                                                    currWord.data(currWordParsingPos5-1, currWordParsingPos5-currWordValidLen5*8);
                                                
                                                currSessionState2.parsingHeaderState = 1;
                                                currSessionState2.requiredLen = MEMCACHED_HDRLEN + currMsgHeader2.bodyLen;
                                                currSessionState2.currHdrLen = MEMCACHED_HDRLEN;
                                                currSessionState2.currBodyLen = currWordValidLen5;

                                                msgParsingState = 13;
                                            }
                                            // exactly parsing body done
                                            else if(currMsgHeader2.bodyLen > 0 && currWordValidLen5 == currMsgHeader2.bodyLen){
                                                currMsgBody2.body(MAX_BODY_LEN-1, MAX_BODY_LEN-currWordValidLen5*8) = 
                                                    currWord.data(currWordParsingPos5-1, currWordParsingPos5-currWordValidLen5*8);
                                                
                                                currMsgBody2.extInl = 1;
                                                currMsgBody2.keyInl = 1;
                                                currMsgBody2.valInl = 1;
                                                
                                                sessionIdFifo2.write(currSessionID);
                                                msgHeaderFifo2.write(currMsgHeader2);
                                                msgBodyFifo2.write(currMsgBody2);

                                                currSessionState2.reset();

                                                msgParsingState = 14;
                                            }
                                            // more than a body or no body
                                            else{
                                                currMsgBody2.body(MAX_BODY_LEN-1, MAX_BODY_LEN-currMsgHeader2.bodyLen*8) = 
                                                    currWord.data(currWordParsingPos5-1, currWordParsingPos5-currMsgHeader2.bodyLen*8);
                                                currWordValidLen6 = currWordValidLen5 - currMsgHeader2.bodyLen;
                                                currWordParsingPos6 = currWordParsingPos5 - currMsgHeader2.bodyLen*8;
                                                
                                                currMsgBody2.extInl = 1;
                                                currMsgBody2.keyInl = 1;
                                                currMsgBody2.valInl = 1;

                                                sessionIdFifo2.write(currSessionID);
                                                msgHeaderFifo2.write(currMsgHeader2);
                                                msgBodyFifo2.write(currMsgBody2);

                                                currSessionState2.reset();
                                                
                                                currMsgBody3.msgID = currentMsgID;
                                                currentMsgID += 1;
                                                // currMsgBody3, currMsgHeader3 will not be updated
                                                if(currWordValidLen6 < MEMCACHED_HDRLEN){
                                                    currSessionState3.msgHeaderBuff(MEMCACHED_HDRLEN*8-1, MEMCACHED_HDRLEN*8-currWordValidLen6*8) = 
                                                        currWord.data(currWordParsingPos6-1, currWordParsingPos6-currWordValidLen6*8);
                                                    
                                                    currSessionState3.parsingHeaderState = 0;
                                                    currSessionState3.currHdrLen = currWordValidLen6;
                                                    currSessionState3.currBodyLen = 0;

                                                    msgParsingState = 15;
                                                }
                                                else{
                                                    std::cout << "[ERROR]: parsing error" << std::endl;
                                                }
                                            }
                                        }
                                        else{
                                            currMsgBody2.extInl = 1;
                                            currMsgBody2.keyInl = 1;
                                            currMsgBody2.valInl = 1;

                                            sessionIdFifo2.write(currSessionID);
                                            msgHeaderFifo2.write(currMsgHeader2);
                                            msgBodyFifo2.write(currMsgBody2);

                                            currSessionState2.reset();
                                            
                                            currMsgBody3.msgID = currentMsgID;
                                            currentMsgID += 1;
                                            // currMsgBody3, currMsgHeader3 will not be updated
                                            if(currWordValidLen5 < MEMCACHED_HDRLEN){
                                                currSessionState3.msgHeaderBuff(MEMCACHED_HDRLEN*8-1, MEMCACHED_HDRLEN*8-currWordValidLen5*8) = 
                                                    currWord.data(currWordParsingPos5-1, currWordParsingPos5-currWordValidLen5*8);
                                                
                                                currSessionState3.parsingHeaderState = 0;
                                                currSessionState3.currHdrLen = currWordValidLen5;
                                                currSessionState3.currBodyLen = 0;

                                                msgParsingState = 15;
                                            }
                                            else{
                                                std::cout << "[ERROR]: parsing error" << std::endl;
                                            }
                                        }                                
                                    }
                                }
                            }
                            else{
                                currWordValidLen4 = currWordValidLen3;
                                currWordParsingPos4 = currWordParsingPos3;

                                currMsgBody1.extInl = 1;
                                currMsgBody1.keyInl = 1;
                                currMsgBody1.valInl = 1;

                                sessionIdFifo1.write(currSessionID);
                                msgHeaderFifo1.write(currMsgHeader1);
                                msgBodyFifo1.write(currMsgBody1);

                                currSessionState1.reset();
                                
                                // not enough to cover a header 
                                if(currWordValidLen4 < MEMCACHED_HDRLEN){
                                    currSessionState2.msgHeaderBuff(MEMCACHED_HDRLEN*8-1, MEMCACHED_HDRLEN*8-currWordValidLen4*8) = 
                                        currWord.data(currWordParsingPos4-1, currWordParsingPos4-currWordValidLen4*8);
                                    
                                    currSessionState2.parsingHeaderState = 0;
                                    currSessionState2.currHdrLen = currWordValidLen4;
                                    currSessionState2.currBodyLen = 0;

                                    currWordValidLen5 = currWordValidLen4;
                                    currWordParsingPos5 = currWordParsingPos4;

                                    msgParsingState = 10;
                                }
                                // exactly covering a header
                                else if(currWordValidLen4 == MEMCACHED_HDRLEN){
                                    currSessionState2.msgHeaderBuff(MEMCACHED_HDRLEN*8-1, MEMCACHED_HDRLEN*8-currWordValidLen4*8) = 
                                        currWord.data(currWordParsingPos4-1, currWordParsingPos4-currWordValidLen4*8);
                                    currMsgHeader2.consume_word(currSessionState2.msgHeaderBuff);

                                    currSessionState2.parsingHeaderState = 1;
                                    currSessionState2.requiredLen = MEMCACHED_HDRLEN + currMsgHeader2.bodyLen;
                                    currSessionState2.currHdrLen = MEMCACHED_HDRLEN;
                                    currSessionState2.currBodyLen = 0;
                                    
                                    msgParsingState = 11;
                                    
                                    currWordValidLen5 = currWordValidLen4;
                                    currWordParsingPos5 = currWordParsingPos4;    
                                }
                                // exactly covering a header with body or more than a header. 
                                else{
                                    currSessionState2.msgHeaderBuff(MEMCACHED_HDRLEN*8-1, 0) = 
                                        currWord.data(currWordParsingPos4-1, currWordParsingPos4-MEMCACHED_HDRLEN*8);
                                    currMsgHeader2.consume_word(currSessionState2.msgHeaderBuff);
                                    currWordValidLen5 = currWordValidLen4 - MEMCACHED_HDRLEN;
                                    currWordParsingPos5 = currWordParsingPos4 - MEMCACHED_HDRLEN*8;

                                    if(currMsgHeader2.bodyLen > 0){
                                        currMsgBody2.msgID = currentMsgID;
                                        currentMsgID += 1;
                                        // require more word to parse body
                                        if(currWordValidLen5 < currMsgHeader2.bodyLen){
                                            currMsgBody2.body(MAX_BODY_LEN-1, MAX_BODY_LEN-currWordValidLen5*8) = 
                                                currWord.data(currWordParsingPos5-1, currWordParsingPos5-currWordValidLen5*8);
                                            
                                            currSessionState2.parsingHeaderState = 1;
                                            currSessionState2.requiredLen = MEMCACHED_HDRLEN + currMsgHeader2.bodyLen;
                                            currSessionState2.currHdrLen = MEMCACHED_HDRLEN;
                                            currSessionState2.currBodyLen = currWordValidLen5;

                                            msgParsingState = 13;
                                        }
                                        // exactly parsing body done
                                        else if(currWordValidLen5 == currMsgHeader2.bodyLen){
                                            currMsgBody2.body(MAX_BODY_LEN-1, MAX_BODY_LEN-currWordValidLen5*8) = 
                                                currWord.data(currWordParsingPos5-1, currWordParsingPos5-currWordValidLen5*8);
                                            
                                            currMsgBody2.extInl = 1;
                                            currMsgBody2.keyInl = 1;
                                            currMsgBody2.valInl = 1;
                                            
                                            sessionIdFifo2.write(currSessionID);
                                            msgHeaderFifo2.write(currMsgHeader2);
                                            msgBodyFifo2.write(currMsgBody2);

                                            currSessionState2.reset();

                                            msgParsingState = 14;
                                        }
                                        // more than a body or no body
                                        // else if((currMsgHeader2.bodyLen > 0 && currWordValidLen5 > currMsgHeader2.bodyLen) || currMsgHeader2.bodyLen == 0){
                                        else{
                                            currMsgBody2.body(MAX_BODY_LEN-1, MAX_BODY_LEN-currMsgHeader2.bodyLen*8) = 
                                                currWord.data(currWordParsingPos5-1, currWordParsingPos5-currMsgHeader2.bodyLen*8);
                                            currWordValidLen6 = currWordValidLen5 - currMsgHeader2.bodyLen;
                                            currWordParsingPos6 = currWordParsingPos5 - currMsgHeader2.bodyLen*8;
                                            
                                            currMsgBody2.extInl = 1;
                                            currMsgBody2.keyInl = 1;
                                            currMsgBody2.valInl = 1;

                                            sessionIdFifo2.write(currSessionID);
                                            msgHeaderFifo2.write(currMsgHeader2);
                                            msgBodyFifo2.write(currMsgBody2);

                                            currSessionState2.reset();
                                            
                                            currMsgBody3.msgID = currentMsgID;
                                            currentMsgID += 1;
                                            // currMsgBody3, currMsgHeader3 will not be updated
                                            if(currWordValidLen6 < MEMCACHED_HDRLEN){
                                                currSessionState3.msgHeaderBuff(MEMCACHED_HDRLEN*8-1, MEMCACHED_HDRLEN*8-currWordValidLen6*8) = 
                                                    currWord.data(currWordParsingPos6-1, currWordParsingPos6-currWordValidLen6*8);
                                                
                                                currSessionState3.parsingHeaderState = 0;
                                                currSessionState3.currHdrLen = currWordValidLen6;
                                                currSessionState3.currBodyLen = 0;

                                                currWordValidLen7 = currWordValidLen6;
                                                currWordParsingPos7 = currWordParsingPos6;

                                                msgParsingState = 15;
                                            }
                                            else{
                                                std::cout << "[ERROR]: parsing error" << std::endl;
                                            }
                                        }
                                    }
                                    else{
                                        currMsgBody2.extInl = 1;
                                        currMsgBody2.keyInl = 1;
                                        currMsgBody2.valInl = 1;

                                        sessionIdFifo2.write(currSessionID);
                                        msgHeaderFifo2.write(currMsgHeader2);
                                        msgBodyFifo2.write(currMsgBody2);

                                        currSessionState2.reset();
                                        
                                        currMsgBody3.msgID = currentMsgID;
                                        currentMsgID += 1;
                                        // currMsgBody3, currMsgHeader3 will not be updated
                                        if(currWordValidLen5 < MEMCACHED_HDRLEN){
                                            currSessionState3.msgHeaderBuff(MEMCACHED_HDRLEN*8-1, MEMCACHED_HDRLEN*8-currWordValidLen5*8) = 
                                                currWord.data(currWordParsingPos5-1, currWordParsingPos5-currWordValidLen5*8);
                                            
                                            currSessionState3.parsingHeaderState = 0;
                                            currSessionState3.currHdrLen = currWordValidLen5;
                                            currSessionState3.currBodyLen = 0;

                                            msgParsingState = 15;
                                        }
                                        else{
                                            std::cout << "[ERROR]: parsing error" << std::endl;
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }// partial body
            else{
                currMsgHeader.consume_word(currSessionState.msgHeaderBuff);

                // requiring more data to parse body
                if(currWordValidLen < currMsgHeader.bodyLen - currSessionState.currBodyLen){
                    currMsgBody.body(MAX_BODY_LEN-currSessionState.currBodyLen*8-1, MAX_BODY_LEN-currSessionState.currBodyLen*8-currWordValidLen*8) = 
                        currWord.data(currWordParsingPos-1, currWordParsingPos-currWordValidLen*8);
                    
                    currSessionState.currBodyLen += currWordValidLen;

                    msgParsingState = 16;
                }
                // exactly parsed body done
                else if(currWordValidLen == currMsgHeader.bodyLen - currSessionState.currBodyLen){
                    currMsgBody.body(MAX_BODY_LEN-currSessionState.currBodyLen*8-1, MAX_BODY_LEN-currSessionState.currBodyLen*8-currWordValidLen*8) = 
                        currWord.data(currWordParsingPos-1, currWordParsingPos-currWordValidLen*8);
                    
                    currMsgBody.extInl = 1;
                    currMsgBody.keyInl = 1;
                    currMsgBody.valInl = 1;
                    
                    sessionIdFifo.write(currSessionID);
                    msgHeaderFifo.write(currMsgHeader);
                    msgBodyFifo.write(currMsgBody);

                    currSessionState.reset();
                 
                    msgParsingState = 17;
                }
                // there are word data remaining
                else{
                    currMsgBody.body(MAX_BODY_LEN-currSessionState.currBodyLen*8-1, MAX_BODY_LEN-currSessionState.currBodyLen*8-(currMsgHeader.bodyLen - currSessionState.currBodyLen)*8) = 
                        currWord.data(currWordParsingPos-1, currWordParsingPos-(currMsgHeader.bodyLen - currSessionState.currBodyLen)*8);

                    currMsgBody.extInl = 1;
                    currMsgBody.keyInl = 1;
                    currMsgBody.valInl = 1;
                    
                    sessionIdFifo.write(currSessionID);
                    msgHeaderFifo.write(currMsgHeader);
                    msgBodyFifo.write(currMsgBody);

                    currWordValidLen2 = currWordValidLen - (currMsgHeader.bodyLen - currSessionState.currBodyLen);
                    currWordParsingPos2 = currWordParsingPos - (currMsgHeader.bodyLen - currSessionState.currBodyLen)*8;

                    currSessionState.reset();

                    // not enough to cover a header 
                    if(currWordValidLen2 < MEMCACHED_HDRLEN){
                        currSessionState1.msgHeaderBuff(MEMCACHED_HDRLEN*8-1, MEMCACHED_HDRLEN*8-currWordValidLen2*8) = 
                            currWord.data(currWordParsingPos2-1, currWordParsingPos2-currWordValidLen2*8);
                        
                        currSessionState1.parsingHeaderState = 0;
                        currSessionState1.currHdrLen = currWordValidLen2;
                        currSessionState1.currBodyLen = 0;

                        currWordValidLen3 = currWordValidLen2;
                        currWordParsingPos3 = currWordParsingPos2;

                        msgParsingState = 18;
                    }
                    // exactly covering a header
                    else if(currWordValidLen2 == MEMCACHED_HDRLEN){
                        currSessionState1.msgHeaderBuff(MEMCACHED_HDRLEN*8-1, MEMCACHED_HDRLEN*8-currWordValidLen2*8) = 
                            currWord.data(currWordParsingPos2-1, currWordParsingPos2-currWordValidLen2*8);
                        currMsgHeader1.consume_word(currSessionState1.msgHeaderBuff);

                        currSessionState1.parsingHeaderState = 1;
                        currSessionState1.requiredLen = MEMCACHED_HDRLEN + currMsgHeader1.bodyLen;
                        currSessionState1.currHdrLen = MEMCACHED_HDRLEN;
                        currSessionState1.currBodyLen = 0;
                        
                        msgParsingState = 19;
                        
                        currWordValidLen3 = currWordValidLen2;
                        currWordParsingPos3 = currWordParsingPos2;    
                    }
                    // exactly covering a header with body or more than a header. 
                    else{
                        currSessionState1.msgHeaderBuff(MEMCACHED_HDRLEN*8-1, 0) = 
                            currWord.data(currWordParsingPos2-1, currWordParsingPos2-MEMCACHED_HDRLEN*8);
                        currMsgHeader1.consume_word(currSessionState1.msgHeaderBuff);

                        if(currWordValidLen2 > MEMCACHED_HDRLEN){
                            currSessionState1.parsingHeaderState = 1;
                            currSessionState1.requiredLen = MEMCACHED_HDRLEN + currMsgHeader1.bodyLen;
                            currSessionState1.currHdrLen = MEMCACHED_HDRLEN;
                            currSessionState1.currBodyLen = 0;
                            
                            currWordValidLen3 = currWordValidLen2 - MEMCACHED_HDRLEN;
                            currWordParsingPos3 = currWordParsingPos2 - MEMCACHED_HDRLEN*8;
                        }
                        else{
                            currWordValidLen3 = currWordValidLen2;
                            currWordParsingPos3 = currWordParsingPos2;
                        }

                        if(currMsgHeader1.bodyLen > 0){
                            currMsgBody1.msgID = currentMsgID;
                            currentMsgID += 1;
                            // require more word to parse body
                            if(currWordValidLen3 < currMsgHeader1.bodyLen){
                                currMsgBody1.body(MAX_BODY_LEN-1, MAX_BODY_LEN-currWordValidLen3*8) = 
                                    currWord.data(currWordParsingPos3-1, currWordParsingPos3-currWordValidLen3*8);
                                
                                currSessionState1.parsingHeaderState = 1;
                                currSessionState1.requiredLen = MEMCACHED_HDRLEN + currMsgHeader1.bodyLen;
                                currSessionState1.currHdrLen = MEMCACHED_HDRLEN;
                                currSessionState1.currBodyLen = currWordValidLen3;

                                msgParsingState = 21;
                            }
                            // exactly parsing body done
                            else if(currWordValidLen3 == currMsgHeader1.bodyLen){
                                currMsgBody1.body(MAX_BODY_LEN-1, MAX_BODY_LEN-currWordValidLen3*8) = 
                                    currWord.data(currWordParsingPos3-1, currWordParsingPos3-currWordValidLen3*8);
                                
                                currMsgBody1.extInl = 1;
                                currMsgBody1.keyInl = 1;
                                currMsgBody1.valInl = 1;
                                
                                sessionIdFifo1.write(currSessionID);
                                msgHeaderFifo1.write(currMsgHeader1);
                                msgBodyFifo1.write(currMsgBody1);

                                currSessionState1.reset();

                                msgParsingState = 22;
                            }
                            // more than a body or no body
                            else{
                                currMsgBody1.body(MAX_BODY_LEN-1, MAX_BODY_LEN-currMsgHeader1.bodyLen*8) = 
                                    currWord.data(currWordParsingPos3-1, currWordParsingPos3-currMsgHeader1.bodyLen*8);
                                currWordValidLen4 = currWordValidLen3 - currMsgHeader1.bodyLen;
                                currWordParsingPos4 = currWordParsingPos3 - currMsgHeader1.bodyLen*8;
                                
                                currMsgBody1.extInl = 1;
                                currMsgBody1.keyInl = 1;
                                currMsgBody1.valInl = 1;

                                sessionIdFifo1.write(currSessionID);
                                msgHeaderFifo1.write(currMsgHeader1);
                                msgBodyFifo1.write(currMsgBody1);

                                currSessionState1.reset();
                                
                                // not enough to cover a header 
                                if(currWordValidLen4 < MEMCACHED_HDRLEN){
                                    currSessionState2.msgHeaderBuff(MEMCACHED_HDRLEN*8-1, MEMCACHED_HDRLEN*8-currWordValidLen4*8) = 
                                        currWord.data(currWordParsingPos4-1, currWordParsingPos4-currWordValidLen4*8);
                                    
                                    currSessionState2.parsingHeaderState = 0;
                                    currSessionState2.currHdrLen = currWordValidLen4;
                                    currSessionState2.currBodyLen = 0;

                                    currWordValidLen5 = currWordValidLen4;
                                    currWordParsingPos5 = currWordParsingPos4;

                                    msgParsingState = 23;
                                }
                                // exactly covering a header
                                else if(currWordValidLen4 == MEMCACHED_HDRLEN && currMsgHeader2.bodyLen > 0){
                                    currSessionState2.msgHeaderBuff(MEMCACHED_HDRLEN*8-1, MEMCACHED_HDRLEN*8-currWordValidLen4*8) = 
                                        currWord.data(currWordParsingPos4-1, currWordParsingPos4-currWordValidLen4*8);
                                    currMsgHeader2.consume_word(currSessionState2.msgHeaderBuff);

                                    currSessionState2.parsingHeaderState = 1;
                                    currSessionState2.requiredLen = MEMCACHED_HDRLEN + currMsgHeader2.bodyLen;
                                    currSessionState2.currHdrLen = MEMCACHED_HDRLEN;
                                    currSessionState2.currBodyLen = 0;
                                    
                                    msgParsingState = 24;
                                }
                                // exactly covering a header with body or more than a header. 
                                // else if((currWordValidLen4 == MEMCACHED_HDRLEN && currMsgHeader2.bodyLen == 0) || currWordValidLen4 > MEMCACHED_HDRLEN){
                                else{
                                    currSessionState2.msgHeaderBuff(MEMCACHED_HDRLEN*8-1, 0) = 
                                        currWord.data(currWordParsingPos4-1, currWordParsingPos4-MEMCACHED_HDRLEN*8);
                                    currMsgHeader2.consume_word(currSessionState2.msgHeaderBuff);

                                    currWordValidLen5 = currWordValidLen4 - MEMCACHED_HDRLEN;
                                    currWordParsingPos5 = currWordParsingPos4 - MEMCACHED_HDRLEN*8;

                                    if(currMsgHeader2.bodyLen > 0){
                                        currMsgBody2.msgID = currentMsgID;
                                        currentMsgID += 1;
                                        // require more word to parse body
                                        if(currMsgHeader2.bodyLen > 0 && currWordValidLen5 < currMsgHeader2.bodyLen){
                                            currMsgBody2.body(MAX_BODY_LEN-1, MAX_BODY_LEN-currWordValidLen5*8) = 
                                                currWord.data(currWordParsingPos5-1, currWordParsingPos5-currWordValidLen5*8);
                                            
                                            currSessionState2.parsingHeaderState = 1;
                                            currSessionState2.requiredLen = MEMCACHED_HDRLEN + currMsgHeader2.bodyLen;
                                            currSessionState2.currHdrLen = MEMCACHED_HDRLEN;
                                            currSessionState2.currBodyLen = currWordValidLen5;

                                            msgParsingState = 26;
                                        }
                                        // exactly parsing body done
                                        else if(currMsgHeader2.bodyLen > 0 && currWordValidLen5 == currMsgHeader2.bodyLen){
                                            currMsgBody2.body(MAX_BODY_LEN-1, MAX_BODY_LEN-currWordValidLen5*8) = 
                                                currWord.data(currWordParsingPos5-1, currWordParsingPos5-currWordValidLen5*8);
                                            
                                            currMsgBody2.extInl = 1;
                                            currMsgBody2.keyInl = 1;
                                            currMsgBody2.valInl = 1;
                                            
                                            sessionIdFifo2.write(currSessionID);
                                            msgHeaderFifo2.write(currMsgHeader2);
                                            msgBodyFifo2.write(currMsgBody2);

                                            currSessionState2.reset();

                                            msgParsingState = 27;
                                        }
                                        // more than a body or no body
                                        else{
                                            currMsgBody2.body(MAX_BODY_LEN-1, MAX_BODY_LEN-currMsgHeader2.bodyLen*8) = 
                                                currWord.data(currWordParsingPos5-1, currWordParsingPos5-currMsgHeader2.bodyLen*8);
                                            currWordValidLen6 = currWordValidLen5 - currMsgHeader2.bodyLen;
                                            currWordParsingPos6 = currWordParsingPos5 - currMsgHeader2.bodyLen*8;
                                            
                                            currMsgBody2.extInl = 1;
                                            currMsgBody2.keyInl = 1;
                                            currMsgBody2.valInl = 1;

                                            sessionIdFifo2.write(currSessionID);
                                            msgHeaderFifo2.write(currMsgHeader2);
                                            msgBodyFifo2.write(currMsgBody2);

                                            currSessionState2.reset();
                                            
                                            currMsgBody3.msgID = currentMsgID;
                                            currentMsgID += 1;
                                            // currMsgBody3, currMsgHeader3 will not be updated
                                            if(currWordValidLen6 < MEMCACHED_HDRLEN){
                                                currSessionState3.msgHeaderBuff(MEMCACHED_HDRLEN*8-1, MEMCACHED_HDRLEN*8-currWordValidLen6*8) = 
                                                    currWord.data(currWordParsingPos6-1, currWordParsingPos6-currWordValidLen6*8);
                                                
                                                currSessionState3.parsingHeaderState = 0;
                                                currSessionState3.currHdrLen = currWordValidLen6;
                                                currSessionState3.currBodyLen = 0;

                                                msgParsingState = 28;
                                            }
                                            else{
                                                std::cout << "[ERROR]: parsing error" << std::endl;
                                            }
                                        }
                                    }
                                    else{
                                        currMsgBody2.extInl = 1;
                                        currMsgBody2.keyInl = 1;
                                        currMsgBody2.valInl = 1;

                                        sessionIdFifo2.write(currSessionID);
                                        msgHeaderFifo2.write(currMsgHeader2);
                                        msgBodyFifo2.write(currMsgBody2);

                                        currSessionState2.reset();
                                        
                                        currMsgBody3.msgID = currentMsgID;
                                        currentMsgID += 1;
                                        // currMsgBody3, currMsgHeader3 will not be updated
                                        if(currWordValidLen5 < MEMCACHED_HDRLEN){
                                            currSessionState3.msgHeaderBuff(MEMCACHED_HDRLEN*8-1, MEMCACHED_HDRLEN*8-currWordValidLen5*8) = 
                                                currWord.data(currWordParsingPos5-1, currWordParsingPos5-currWordValidLen5*8);
                                            
                                            currSessionState3.parsingHeaderState = 0;
                                            currSessionState3.currHdrLen = currWordValidLen5;
                                            currSessionState3.currBodyLen = 0;

                                            msgParsingState = 28;
                                        }
                                        else{
                                            std::cout << "[ERROR]: parsing error" << std::endl;
                                        }
                                    }                                
                                }
                            }
                        }
                        else{
                            currWordValidLen4 = currWordValidLen3;
                            currWordParsingPos4 = currWordParsingPos3;

                            currMsgBody1.extInl = 1;
                            currMsgBody1.keyInl = 1;
                            currMsgBody1.valInl = 1;

                            sessionIdFifo1.write(currSessionID);
                            msgHeaderFifo1.write(currMsgHeader1);
                            msgBodyFifo1.write(currMsgBody1);

                            currSessionState1.reset();
                            
                            // not enough to cover a header 
                            if(currWordValidLen4 < MEMCACHED_HDRLEN){
                                currSessionState2.msgHeaderBuff(MEMCACHED_HDRLEN*8-1, MEMCACHED_HDRLEN*8-currWordValidLen4*8) = 
                                    currWord.data(currWordParsingPos4-1, currWordParsingPos4-currWordValidLen4*8);
                                
                                currSessionState2.parsingHeaderState = 0;
                                currSessionState2.currHdrLen = currWordValidLen4;
                                currSessionState2.currBodyLen = 0;

                                currWordValidLen5 = currWordValidLen4;
                                currWordParsingPos5 = currWordParsingPos4;

                                msgParsingState = 23;
                            }
                            // exactly covering a header
                            else if(currWordValidLen4 == MEMCACHED_HDRLEN){
                                currSessionState2.msgHeaderBuff(MEMCACHED_HDRLEN*8-1, MEMCACHED_HDRLEN*8-currWordValidLen4*8) = 
                                    currWord.data(currWordParsingPos4-1, currWordParsingPos4-currWordValidLen4*8);
                                currMsgHeader2.consume_word(currSessionState2.msgHeaderBuff);

                                currSessionState2.parsingHeaderState = 1;
                                currSessionState2.requiredLen = MEMCACHED_HDRLEN + currMsgHeader2.bodyLen;
                                currSessionState2.currHdrLen = MEMCACHED_HDRLEN;
                                currSessionState2.currBodyLen = 0;
                                
                                msgParsingState = 24;
                                
                                currWordValidLen5 = currWordValidLen4;
                                currWordParsingPos5 = currWordParsingPos4;    
                            }
                            // exactly covering a header with body or more than a header. 
                            else{
                                currSessionState2.msgHeaderBuff(MEMCACHED_HDRLEN*8-1, 0) = 
                                    currWord.data(currWordParsingPos4-1, currWordParsingPos4-MEMCACHED_HDRLEN*8);
                                currMsgHeader2.consume_word(currSessionState2.msgHeaderBuff);
                                currWordValidLen5 = currWordValidLen4 - MEMCACHED_HDRLEN;
                                currWordParsingPos5 = currWordParsingPos4 - MEMCACHED_HDRLEN*8;

                                if(currMsgHeader2.bodyLen > 0){
                                    currMsgBody2.msgID = currentMsgID;
                                    currentMsgID += 1;
                                    // require more word to parse body
                                    if(currWordValidLen5 < currMsgHeader2.bodyLen){
                                        currMsgBody2.body(MAX_BODY_LEN-1, MAX_BODY_LEN-currWordValidLen5*8) = 
                                            currWord.data(currWordParsingPos5-1, currWordParsingPos5-currWordValidLen5*8);
                                        
                                        currSessionState2.parsingHeaderState = 1;
                                        currSessionState2.requiredLen = MEMCACHED_HDRLEN + currMsgHeader2.bodyLen;
                                        currSessionState2.currHdrLen = MEMCACHED_HDRLEN;
                                        currSessionState2.currBodyLen = currWordValidLen5;

                                        msgParsingState = 26;
                                    }
                                    // exactly parsing body done
                                    else if(currWordValidLen5 == currMsgHeader2.bodyLen){
                                        currMsgBody2.body(MAX_BODY_LEN-1, MAX_BODY_LEN-currWordValidLen5*8) = 
                                            currWord.data(currWordParsingPos5-1, currWordParsingPos5-currWordValidLen5*8);
                                        
                                        currMsgBody2.extInl = 1;
                                        currMsgBody2.keyInl = 1;
                                        currMsgBody2.valInl = 1;
                                        
                                        sessionIdFifo2.write(currSessionID);
                                        msgHeaderFifo2.write(currMsgHeader2);
                                        msgBodyFifo2.write(currMsgBody2);

                                        currSessionState2.reset();

                                        msgParsingState = 27;
                                    }
                                    // more than a body or no body
                                    // else if((currMsgHeader2.bodyLen > 0 && currWordValidLen5 > currMsgHeader2.bodyLen) || currMsgHeader2.bodyLen == 0){
                                    else{
                                        currMsgBody2.body(MAX_BODY_LEN-1, MAX_BODY_LEN-currMsgHeader2.bodyLen*8) = 
                                            currWord.data(currWordParsingPos5-1, currWordParsingPos5-currMsgHeader2.bodyLen*8);
                                        currWordValidLen6 = currWordValidLen5 - currMsgHeader2.bodyLen;
                                        currWordParsingPos6 = currWordParsingPos5 - currMsgHeader2.bodyLen*8;
                                        
                                        currMsgBody2.extInl = 1;
                                        currMsgBody2.keyInl = 1;
                                        currMsgBody2.valInl = 1;

                                        sessionIdFifo2.write(currSessionID);
                                        msgHeaderFifo2.write(currMsgHeader2);
                                        msgBodyFifo2.write(currMsgBody2);

                                        currSessionState2.reset();
                                        
                                        currMsgBody3.msgID = currentMsgID;
                                        currentMsgID += 1;
                                        // currMsgBody3, currMsgHeader3 will not be updated
                                        if(currWordValidLen6 < MEMCACHED_HDRLEN){
                                            currSessionState3.msgHeaderBuff(MEMCACHED_HDRLEN*8-1, MEMCACHED_HDRLEN*8-currWordValidLen6*8) = 
                                                currWord.data(currWordParsingPos6-1, currWordParsingPos6-currWordValidLen6*8);
                                            
                                            currSessionState3.parsingHeaderState = 0;
                                            currSessionState3.currHdrLen = currWordValidLen6;
                                            currSessionState3.currBodyLen = 0;

                                            currWordValidLen7 = currWordValidLen6;
                                            currWordParsingPos7 = currWordParsingPos6;

                                            msgParsingState = 28;
                                        }
                                        else{
                                            std::cout << "[ERROR]: parsing error" << std::endl;
                                        }
                                    }
                                }
                                else{
                                    currMsgBody2.extInl = 1;
                                    currMsgBody2.keyInl = 1;
                                    currMsgBody2.valInl = 1;

                                    sessionIdFifo2.write(currSessionID);
                                    msgHeaderFifo2.write(currMsgHeader2);
                                    msgBodyFifo2.write(currMsgBody2);

                                    currSessionState2.reset();
                                    
                                    currMsgBody3.msgID = currentMsgID;
                                    currentMsgID += 1;
                                    // currMsgBody3, currMsgHeader3 will not be updated
                                    if(currWordValidLen5 < MEMCACHED_HDRLEN){
                                        currSessionState3.msgHeaderBuff(MEMCACHED_HDRLEN*8-1, MEMCACHED_HDRLEN*8-currWordValidLen5*8) = 
                                            currWord.data(currWordParsingPos5-1, currWordParsingPos5-currWordValidLen5*8);
                                        
                                        currSessionState3.parsingHeaderState = 0;
                                        currSessionState3.currHdrLen = currWordValidLen5;
                                        currSessionState3.currBodyLen = 0;

                                        msgParsingState = 28;
                                    }
                                    else{
                                        std::cout << "[ERROR]: parsing error" << std::endl;
                                    }
                                }
                            }
                        }
                    }
                }
            }

            std::cout << "msgParsingState = " << msgParsingState << std::endl;
            // decide which currSessionState to store back based on msgParsingState; 
            // decide which currMsgBody to store back based on msgParsingState; 
            // last and msg not parsed done -> KV_UPDATE_INSERT;
            // msg parsed done -> KV_DELETE;
            switch(msgParsingState){
                case 0:
                case 1:
                case 3:
                case 16:{
                    // end of the AXIS transaction, need to store sessionState and currMsgBody back; 
                    if(currWord.last){
                        s_axis_upd_req.write(hash_table_16_1024::htUpdateReq<16, 1024>(hash_table_16_1024::KV_UPDATE_INSERT, currSessionID, currMsgBody.output_word(), 0));
                        sessionStateTable[currSessionID] = currSessionState;
                    }
                    if(currMsgBody.msgID != 0)std::cout << "KV_UPDATE_INSERT currMsgBody.msgID = " << currMsgBody.msgID << ": ";
                    std::cout << "currSessionState: " << std::endl; currSessionState.display();
                    break;
                }
                case 2:
                case 4:
                case 17:{
                    if(currWord.last){
                        s_axis_upd_req.write(hash_table_16_1024::htUpdateReq<16, 1024>(hash_table_16_1024::KV_DELETE, currSessionID, currMsgBody.output_word(), 0));
                        sessionStateTable[currSessionID] = currSessionState;
                    }
                    if(currMsgBody.msgID != 0)std::cout << "KV_DELETE currMsgBody.msgID = " << currMsgBody.msgID << ": ";
                    std::cout << "currSessionState: " << std::endl; currSessionState.display();
                    break;
                }
                case 5:
                case 6:
                case 8:
                case 18:
                case 19:
                case 21:{
                    if(currWord.last){
                        s_axis_upd_req.write(hash_table_16_1024::htUpdateReq<16, 1024>(hash_table_16_1024::KV_UPDATE_INSERT, currSessionID, currMsgBody1.output_word(), 0));
                        sessionStateTable[currSessionID] = currSessionState1;
                    }
                    else{
                        currMsgBody = currMsgBody1;
                        currSessionState = currSessionState1;
                    }
                    if(currMsgBody1.msgID != 0)std::cout << "KV_UPDATE_INSERT currMsgBody1.msgID = " << currMsgBody1.msgID << ": ";
                    std::cout << "currSessionState1: " << std::endl; currSessionState1.display();
                    break;
                }
                case 7:
                case 9:
                case 20:
                case 22:{
                    if(currWord.last){
                        s_axis_upd_req.write(hash_table_16_1024::htUpdateReq<16, 1024>(hash_table_16_1024::KV_DELETE, currSessionID, currMsgBody1.output_word(), 0));
                        sessionStateTable[currSessionID] = currSessionState1;
                    }
                    else{
                        currMsgBody = currMsgBody1;
                        currSessionState = currSessionState1;
                    }
                    if(currMsgBody1.msgID != 0)std::cout << "KV_DELETE currMsgBody1.msgID = " << currMsgBody1.msgID << ": ";
                    std::cout << "currSessionState1: " << std::endl; currSessionState1.display();
                    break;
                }
                case 10:
                case 11:
                case 13:
                case 23:
                case 24:
                case 26:{
                    if(currWord.last){
                        s_axis_upd_req.write(hash_table_16_1024::htUpdateReq<16, 1024>(hash_table_16_1024::KV_UPDATE_INSERT, currSessionID, currMsgBody2.output_word(), 0));
                        sessionStateTable[currSessionID] = currSessionState2;
                    }
                    else{
                        currMsgBody = currMsgBody2;
                        currSessionState = currSessionState2;
                    }
                    if(currMsgBody2.msgID != 0)std::cout << "KV_UPDATE_INSERT currMsgBody2.msgID = " << currMsgBody2.msgID << ": ";
                    std::cout << "currSessionState2: " << std::endl; currSessionState2.display();
                    break;
                }
                case 12:
                case 14:
                case 25:
                case 27:{
                    if(currWord.last){
                        s_axis_upd_req.write(hash_table_16_1024::htUpdateReq<16, 1024>(hash_table_16_1024::KV_DELETE, currSessionID, currMsgBody2.output_word(), 0));
                        sessionStateTable[currSessionID] = currSessionState2;
                    }
                    else{
                        currMsgBody = currMsgBody2;
                        currSessionState = currSessionState2;
                    }
                    if(currMsgBody2.msgID != 0)std::cout << "KV_DELETE currMsgBody2.msgID = " << currMsgBody2.msgID << ": ";
                    std::cout << "currSessionState2: " << std::endl; currSessionState2.display();
                    break;
                }
                case 15:
                case 28:{
                    if(currWord.last){
                        s_axis_upd_req.write(hash_table_16_1024::htUpdateReq<16, 1024>(hash_table_16_1024::KV_UPDATE_INSERT, currSessionID, currMsgBody3.output_word(), 0));
                        sessionStateTable[currSessionID] = currSessionState3;
                    }
                    else{
                        currMsgBody = currMsgBody3;
                        currSessionState = currSessionState3;
                    }
                    if(currMsgBody3.msgID != 0)std::cout << "KV_UPDATE_INSERT currMsgBody3.msgID = " << currMsgBody3.msgID << ": ";
                    std::cout << "currSessionState3: " << std::endl; currSessionState3.display();
                    break;
                }
            }

            if(currWord.last){
                currAxisState = IDLE;
            }
            else{
                currAxisState = READ_WORD;
            }
            break;
        }
    }
}

void priorityMux3to1(
    hls::stream<ap_uint<16> >&              sessionIdFifo, // input
    hls::stream<msgHeader>&                 msgHeaderFifo, // input
    hls::stream<msgBody>&                   msgBodyFifo, // input
    hls::stream<ap_uint<16> >&              sessionIdFifo1, // input
    hls::stream<msgHeader>&                 msgHeaderFifo1, // input
    hls::stream<msgBody>&                   msgBodyFifo1, // input
    hls::stream<ap_uint<16> >&              sessionIdFifo2, // input
    hls::stream<msgHeader>&                 msgHeaderFifo2, // input
    hls::stream<msgBody>&                   msgBodyFifo2, // input
    hls::stream<ap_uint<16> >&              sessionIdFifo_out, // output
    hls::stream<msgHeader>&                 msgHeaderFifo_out, // output
    hls::stream<msgBody>&                   msgBodyFifo_out // output
){
#pragma HLS PIPELINE II=1
#pragma HLS INLINE off

	enum muxStateType{FWD0=0, FWD1, FWD2};
    static muxStateType fsmState = FWD0;

    ap_uint<1> fifo0_vld = (!sessionIdFifo.empty() && !msgHeaderFifo.empty() && !msgBodyFifo.empty());
    ap_uint<1> fifo1_vld = (!sessionIdFifo1.empty() && !msgHeaderFifo1.empty() && !msgBodyFifo1.empty());
    ap_uint<1> fifo2_vld = (!sessionIdFifo2.empty() && !msgHeaderFifo2.empty() && !msgBodyFifo2.empty());

    std::cout << "priorityMux3to1 state: " << fsmState << std::endl;
    switch(fsmState) {
        case FWD0: {
            if(fifo0_vld){
                sessionIdFifo_out.write(sessionIdFifo.read());
                msgHeaderFifo_out.write(msgHeaderFifo.read());
                msgBodyFifo_out.write(msgBodyFifo.read());
                if(fifo1_vld){
                    fsmState = FWD1;
                }
            }
            break;
        }
        case FWD1: {
            if(fifo1_vld){
                sessionIdFifo_out.write(sessionIdFifo1.read());
                msgHeaderFifo_out.write(msgHeaderFifo1.read());
                msgBodyFifo_out.write(msgBodyFifo1.read());
            }
            if(fifo2_vld){
                fsmState = FWD2;
            }
            else{
                fsmState = FWD0;
            }
            break;
        }
        case FWD2: {
            if(fifo2_vld){
                sessionIdFifo_out.write(sessionIdFifo2.read());
                msgHeaderFifo_out.write(msgHeaderFifo2.read());
                msgBodyFifo_out.write(msgBodyFifo2.read());
            }
            fsmState = FWD0;
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

void deparser(
    hls::stream<ap_uint<16> >& sessionIdFifo, // input
    hls::stream<msgHeader>& msgHeaderFifo, // input
    hls::stream<msgBody>& msgBodyFifo, // input
	hls::stream<appTxMeta>& txMetaData, 
    hls::stream<net_axis<DATA_WIDTH> >& txData
){
#pragma HLS PIPELINE II=1
#pragma HLS INLINE off enable_flush

    static ap_uint<32> requiredSendLen;
    static ap_uint<32> currSendLen;
    static ap_uint<MEMCACHED_HDRLEN*8> hdr;
    static ap_uint<MAX_BODY_LEN> body;
    
    static ap_uint<16> currSessionID;
    static msgHeader currMsgHeader;
    static msgBody currMsgBody;

    // !!! if you specify it in a switch case, that forces that switch case to be scheduled in one cycle??
    static net_axis<DATA_WIDTH> currWord;
    #pragma HLS DEPENDENCE variable=currWord inter false

    enum deparser_fsmType{IDLE=0, SEND_WORD};
#ifndef __SYNTHESIS__
    const char* deparser_fsmName[2] = {"IDLE", "SEND_WORD"};
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
                currMsgHeader = msgHeaderFifo.read();;
                currMsgBody = msgBodyFifo.read();;

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
    			esac_fsmState = SEND_WORD;
    		}
    		break;
        }
        // TODO: need to check txStatus. 
        case SEND_WORD: {
    		if (!txData.full())
    		{
                if(currSendLen < MEMCACHED_HDRLEN*8){
                    if(DATA_WIDTH >= requiredSendLen){
                        ap_uint<32> secondPartLen = requiredSendLen - MEMCACHED_HDRLEN*8;
                        if(currMsgBody.msgID != 0)std::cout << "currMsgBody.msgID = " << currMsgBody.msgID << ": ";
                        std::cout << "secondPartLen " << secondPartLen << std::endl;
                        std::cout << "requiredSendLen " << requiredSendLen << std::endl;
                        
                        currWord.data(DATA_WIDTH-1, DATA_WIDTH-MEMCACHED_HDRLEN*8) = hdr;
                        if(secondPartLen > 0){
                            // this is not supported in HLS -- causing currWord.data to be all zero, not sure why
                            // currWord.data(DATA_WIDTH-1, DATA_WIDTH-requiredSendLen) = (hdr(MEMCACHED_HDRLEN*8-1, 0), body(MAX_BODY_LEN-1, MAX_BODY_LEN-secondPartLen));
                            // currWord.data(DATA_WIDTH-1, DATA_WIDTH-MEMCACHED_HDRLEN*8) = hdr;
                            currWord.data(DATA_WIDTH-MEMCACHED_HDRLEN*8-1, DATA_WIDTH-requiredSendLen) = body(MAX_BODY_LEN-1, MAX_BODY_LEN-secondPartLen);
                        }
                        
                        currWord.keep = lenToKeep(requiredSendLen/8);
                        currWord.last = 1;

                        currSendLen += requiredSendLen;
                        
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

                        currSendLen += DATA_WIDTH;
        				esac_fsmState = SEND_WORD;
                    }
                }
                else{
                    ap_uint<16> remainLen = requiredSendLen - currSendLen;
                    ap_uint<16> body_sendingPos = (MAX_BODY_LEN + MEMCACHED_HDRLEN*8 - currSendLen);
                    if(currMsgBody.msgID != 0)std::cout << "currMsgBody.msgID = " << currMsgBody.msgID << ": ";
                    std::cout << "remainLen = " << remainLen << std::endl;

                    if(DATA_WIDTH >= remainLen){
                        currWord.data(DATA_WIDTH-1, DATA_WIDTH-remainLen) = body(body_sendingPos-1, body_sendingPos-remainLen);
                        currWord.keep = lenToKeep(remainLen/8);
                        currWord.last = 1;
        
                        currSendLen += remainLen;
                        esac_fsmState = IDLE;
                    }
                    else{
                        currWord.data = body(body_sendingPos-1, body_sendingPos-DATA_WIDTH);
                        currWord.keep = lenToKeep(DATA_WIDTH/8);
                        currWord.last = 0;
        
                        currSendLen += DATA_WIDTH;
                        esac_fsmState = SEND_WORD;
                    }
                }
                txData.write(currWord);
    		}
    		break;
    	}
    }
}

void deparser_send(
    
){

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
    
    // read data from network and parse them into msgFifo
    parser(rxMetaData, rxData, mc_sessionIdFifo0, mc_msgHeaderFifo0, mc_msgBodyFifo0, \
            mc_sessionIdFifo4, mc_msgHeaderFifo4, mc_msgBodyFifo4, mc_sessionIdFifo5, mc_msgHeaderFifo5, mc_msgBodyFifo5, \
            s_axis_lup_req, s_axis_upd_req, m_axis_lup_rsp);

    priorityMux3to1(mc_sessionIdFifo0, mc_msgHeaderFifo0, mc_msgBodyFifo0, mc_sessionIdFifo4, mc_msgHeaderFifo4, mc_msgBodyFifo4, \
            mc_sessionIdFifo5, mc_msgHeaderFifo5, mc_msgBodyFifo5, mc_sessionIdFifo6, mc_msgHeaderFifo6, mc_msgBodyFifo6);

    // simple hash function 
    simple_hash(mc_keyFifo, mc_hashFifo);

	// 1) read request from fifo, determine the destination mc, query mc
    // 2) read response from fifo, forward response to client. 
    proxy(mc_sessionIdFifo6, mc_msgHeaderFifo6, mc_msgBodyFifo6, mc_sessionIdFifo1, mc_msgHeaderFifo1, mc_msgBodyFifo1, \
            mc_keyFifo, mc_hashFifo, multiQueue_push, multiQueue_pop_req, multiQueue_rsp, \
            s_axis_lup_req1, s_axis_upd_req1, m_axis_lup_rsp1, m_axis_upd_rsp1,\
            mc_cmdFifo, mc_sessionCountFifo, mc_sessionIdStreamFifo, mc_hashValFifo, mc_sessionIdFifo2, mc_idxFifo, mc_sessionIdFifo3);

    // deparsing the res and resp, sending to destination (client or memcached)
    deparser(mc_sessionIdFifo1, mc_msgHeaderFifo1, mc_msgBodyFifo1, txMetaData, txData);

	dummy(closeConnection, txStatus, m_axis_upd_rsp, regInsertFailureCount, regInsertFailureCount1);

}
