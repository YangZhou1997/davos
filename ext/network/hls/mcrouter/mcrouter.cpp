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
	hls::stream<ap_uint<16> >& closedSessionIdFifo, //input
    ap_uint<14>		useConn,
	ap_uint<32>		regIpAddress0,
	ap_uint<32>		regIpAddress1,
	ap_uint<32>		regIpAddress2,
	ap_uint<32>		regIpAddress3,
	ap_uint<32>		regIpAddress4,
	ap_uint<32>		regIpAddress5,
	ap_uint<32>		regIpAddress6,
	ap_uint<32>		regIpAddress7,
	ap_uint<32>		regIpAddress8,
	ap_uint<32>		regIpAddress9,
    ap_uint<16>		regIpPort0,
	ap_uint<16>		regIpPort1,
	ap_uint<16>		regIpPort2,
	ap_uint<16>		regIpPort3,
	ap_uint<16>		regIpPort4,
	ap_uint<16>		regIpPort5,
	ap_uint<16>		regIpPort6,
	ap_uint<16>		regIpPort7,
	ap_uint<16>		regIpPort8,
	ap_uint<16>		regIpPort9,
    ap_uint<16>     ipAddressIdx_debug,
    ap_uint<16>     useConn_debug,
    ap_uint<16>     sessionCount_debug
    // softTkoFIFO () 
){
#pragma HLS PIPELINE II=1
#pragma HLS INLINE off

static ap_uint<16> connectedSessions[MAX_CONNECTED_SESSIONS];
static ap_uint<1> connectedSessionsSts[MAX_CONNECTED_SESSIONS];
// static SoftTkoCount sessionState[];

// !!! do not specify RAM_T2P_BRAM -- let HLS automatically use register with mux. 
#pragma HLS RESOURCE variable=connectedSessions core=RAM_T2P_URAM
#pragma HLS ARRAY_PARTITION variable=connectedSessions complete
#pragma HLS DEPENDENCE variable=connectedSessions inter false

#pragma HLS RESOURCE variable=connectedSessionsSts core=RAM_T2P_URAM
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
	static ap_uint<16> ipAddressIdx = 0;
    ipAddressIdx_debug = ipAddressIdx;
    useConn_debug = (uint16_t)useConn;
    sessionCount_debug = sessionCount;
	
    if (ipAddressIdx < useConn)
	{
		ipTuple openTuple;
		switch (ipAddressIdx)
		{
		case 0:
			openTuple.ip_address = regIpAddress0;
            openTuple.ip_port = regIpPort0;
			break;
		case 1:
			openTuple.ip_address = regIpAddress1;
            openTuple.ip_port = regIpPort1;
			break;
		case 2:
			openTuple.ip_address = regIpAddress2;
            openTuple.ip_port = regIpPort2;
			break;
		case 3:
			openTuple.ip_address = regIpAddress3;
            openTuple.ip_port = regIpPort3;
			break;
		case 4:
			openTuple.ip_address = regIpAddress4;
            openTuple.ip_port = regIpPort4;
			break;
		case 5:
			openTuple.ip_address = regIpAddress5;
            openTuple.ip_port = regIpPort5;
			break;
		case 6:
			openTuple.ip_address = regIpAddress6;
            openTuple.ip_port = regIpPort6;
			break;
		case 7:
			openTuple.ip_address = regIpAddress7;
            openTuple.ip_port = regIpPort7;
			break;
		case 8:
			openTuple.ip_address = regIpAddress8;
            openTuple.ip_port = regIpPort8;
			break;
		case 9:
			openTuple.ip_address = regIpAddress9;
            openTuple.ip_port = regIpPort9;
			break;
		}
		openConnection.write(openTuple);
		ipAddressIdx++;
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
        // temporary hack to test connection. 
        else{
            ipAddressIdx = 0;
        }
	}
    else if(!hashValFifo.empty()){
        ap_uint<32> hashVal = hashValFifo.read();
        // TODO: handling closed connections, or in policy engine
        ap_uint<16> idx = (hashVal&(MAX_CONNECTED_SESSIONS-1)) % sessionCount;
        // ap_uint<16> idx = (hashVal&(MAX_CONNECTED_SESSIONS-1));
        sessionIdFifo2.write(connectedSessions[idx]);
    }
    // this functionality is not being used right now. 
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
int ac_stash_insert(ap_uint<16> sessionID){
#pragma HLS INLINE
#ifndef __SYNTHESIS__
    // this means no stash slot is available -- this should nevery happen. 
    if(ac_valid_stashTable == 0){
        std::cout << "[ERROR] ac_stash_insert: currSessionID=" << sessionID << std::endl;
        return -1;
    }
    else{
#endif
        int slot = __builtin_ctz(ac_valid_stashTable);
        std::cout << "stash_insert ac_valid_stashTable = " << std::hex << ac_valid_stashTable << " " << std::dec << slot << std::endl;
        std::cout << "stash_insert slot = " << slot << std::endl;
        ac_sessionID_stashTable[slot] = sessionID;
        ac_valid_stashTable[slot] = 0;
        return slot;
#ifndef __SYNTHESIS__
    }
#endif
}

int ac_stash_lookup(ap_uint<16> sessionID){
#pragma HLS INLINE
    ap_uint<AC_STASH_SIZE> slot = 0;
  
    for (int i = 0; i < AC_STASH_SIZE; i++)
    {
        #pragma HLS UNROLL
        slot[i] = (!ac_valid_stashTable[i] && ac_sessionID_stashTable[i] == sessionID);
    }
#ifndef __SYNTHESIS__
    if(slot == 0){
        // this should nevery happen. 
        std::cout << "[ERROR] ac_stash_lookup: currSessionID=" << sessionID << std::endl;
        return -1;
    }
    else{
#endif
        slot = __builtin_ctz(slot);
        std::cout << "stash_lookup ac_valid_stashTable = " << std::hex << ac_valid_stashTable << " " << std::dec << slot << std::endl;
        return slot;
#ifndef __SYNTHESIS__
    }
#endif
}

bool ac_stash_remove(ap_uint<16> sessionID){
#pragma HLS INLINE
    ap_uint<AC_STASH_SIZE> slot = 0;

    for (int i = 0; i < AC_STASH_SIZE; i++)
    {
        #pragma HLS UNROLL
        slot[i] = (!ac_valid_stashTable[i] && ac_sessionID_stashTable[i] == sessionID);
    }
#ifndef __SYNTHESIS__
    if(slot == 0){
        // this should nevery happen. 
        std::cout << "[ERROR] ac_stash_remove: currSessionID=" << sessionID << std::endl;
        return false;
    }
    else{
#endif
        slot = __builtin_ctz(slot);
        ac_valid_stashTable[slot] = 1;
        return true;
#ifndef __SYNTHESIS__
    }
#endif
}

static hls::stream<net_axis<DATA_WIDTH> > ac_wordProcessingQueues[AC_STASH_SIZE];

void state_recovery(
    // with admission_control2
    hls::stream<ap_uint<16> >&              rxMetaData_in, // input from ac2
	hls::stream<net_axis<DATA_WIDTH> >&     rxData_in, // intput from ac2
    hls::stream<ap_uint<16> >&              sessionStashLookupFifo_in, // input from ac2
    hls::stream<bool>&                      sessionStashLookupRspFifo_out, // output to ac2
    // with stateman
    hls::stream<parser_htLookupResp>&       m_axis_lup_rsp, // input from parser_stateman
    hls::stream<parser_htUpdateResp>&       m_axis_upd_rsp, // input from parser_stateman
    hls::stream<parser_htUpdateReq>&        s_axis_upd_req, // output to parser_stateman
    // the word waiting for parsing
    hls::stream<net_axis<DATA_WIDTH> >&     currWordFifo_out, // output to parser
    // the external states before parsing this word
    hls::stream<sessionState>&              currSessionStateFifo_out, // output to parser
    hls::stream<msgBody>&                   currMsgBodyFifo_out, // output to parser
    // the updated states after parsing this word -- note these two might be out of order
    hls::stream<sessionState>&              currSessionStateFifo_in, // states from parser
    hls::stream<msgBody>&                   currMsgBodyFifo_in // states from parser
){
#pragma HLS PIPELINE II=1
#pragma HLS INLINE off

    if(!m_axis_upd_rsp.empty()){
        parser_htUpdateResp response = m_axis_upd_rsp.read();
        if (!response.success){
            std::cout << "[ERROR] update failed" << std::endl;
        }
    }   
    
    // maintaining the stash for all in-pipeline session and their words. 
    // expose the sessionID lookup FIFO to admission_control2;
    if(!sessionStashLookupFifo_in.empty()){
        ap_uint<16> sessionID = sessionStashLookupFifo_in.read();
        int slot = ac_stash_lookup(sessionID);
        sessionStashLookupRspFifo_out.write(slot != -1);
        std::cout << "remove sessionID = " << sessionID << std::endl;
    }

    #pragma HLS stream variable=ac_wordProcessingQueues[0] depth=4
    #pragma HLS stream variable=ac_wordProcessingQueues[1] depth=4
    #pragma HLS stream variable=ac_wordProcessingQueues[2] depth=4
    #pragma HLS stream variable=ac_wordProcessingQueues[3] depth=4
    #pragma HLS stream variable=ac_wordProcessingQueues[4] depth=4
    #pragma HLS stream variable=ac_wordProcessingQueues[5] depth=4
    #pragma HLS stream variable=ac_wordProcessingQueues[6] depth=4
    #pragma HLS stream variable=ac_wordProcessingQueues[7] depth=4
    
    static uint32_t slot_g, slot_a, slot_b;
    static sessionState ss;
    static msgBody mb;
    static ap_uint<16> sessionID;
    static ap_uint<4> fsmState = 0;

    switch(fsmState){
        case 0:{
            if(!rxMetaData_in.empty()){
                ap_uint<16> currSessionID = rxMetaData_in.read();
                slot_g = ac_stash_insert(currSessionID);
                fsmState = 1;
            }
            else if(!m_axis_lup_rsp.empty()){
                parser_htLookupResp rsp = m_axis_lup_rsp.read();
                ap_uint<16> currSessionID = rsp.key;
                slot_a = ac_stash_lookup(currSessionID);

                if(rsp.hit){
                    currSessionStateFifo_out.write(rsp.value1);
                    currMsgBodyFifo_out.write(rsp.value2);
                }
                else{
                    rsp.value1 = sessionState(currSessionID);
                    rsp.value2 = msgBody(currSessionID);
                    currSessionStateFifo_out.write(rsp.value1);
                    currMsgBodyFifo_out.write(rsp.value2);
                }
                fsmState = 2;
            }
            // these two must be in order with each other, as they will both appear in the end of a word. 
            else if(!currSessionStateFifo_in.empty() && !currMsgBodyFifo_in.empty()){
                ss = currSessionStateFifo_in.read();
                mb = currMsgBodyFifo_in.read();
                sessionID = ss.currSessionID;
                
#ifndef __SYNTHESIS__
                if(ss.currSessionID != mb.currSessionID){
                    std::cout << "[ERROR] state_recovery word output state" << std::endl;
                }
#endif
                slot_b = ac_stash_lookup(sessionID);
                fsmState = 3;
            }
            // else if(!currSessionStateFifo_in.empty()){
            //     sessionState ss = currSessionStateFifo_in.read();
            //     ap_uint<16> sessionID = ss.currSessionID;
            //     uint32_t slot = ac_stash_lookup(sessionID);
            //     msgBody mb = ac_parsingState2[slot];

            //     if(ac_parsingState_rsp[slot] == 1){
            //         // this session still has more words -- keep parsing
            //         if(!ac_wordProcessingQueues[slot].empty()){
            //             net_axis<DATA_WIDTH> currWord = ac_wordProcessingQueues[slot].read();
            //             currWordFifo_out.write(currWord);
            //             currSessionStateFifo_out.write(ss);
            //             currMsgBodyFifo_out.write(mb);
            //         }
            //         // no more words for this session -- stop parsing and store states
            //         else{
            //             ac_stash_remove(sessionID);
            //             s_axis_upd_req.write(parser_htUpdateReq(sessionID, ss, mb, 0));
            //         }
            //         ac_parsingState_rsp[slot] = 0;
            //     }
            //     else{
            //         ac_parsingState1[slot] = ss;
            //         ac_parsingState_rsp[slot] = 1;
            //     }
            // }
            // else if(!currMsgBodyFifo_in.empty()){
            //     msgBody mb = currMsgBodyFifo_in.read();
            //     ap_uint<16> sessionID = mb.currSessionID;
            //     uint32_t slot = ac_stash_lookup(sessionID);
            //     sessionState ss = ac_parsingState1[slot];

            //     if(ac_parsingState_rsp[slot] == 1){
            //         if(!ac_wordProcessingQueues[slot].empty()){
            //             net_axis<DATA_WIDTH> currWord = ac_wordProcessingQueues[slot].read();
            //             currWordFifo_out.write(currWord);
            //             currSessionStateFifo_out.write(ss);
            //             currMsgBodyFifo_out.write(mb);
            //         }
            //         else{
            //             ac_stash_remove(sessionID);
            //             s_axis_upd_req.write(parser_htUpdateReq(sessionID, ss, mb, 0));
            //         }
            //         ac_parsingState_rsp[slot] = 0;
            //     }
            //     else{
            //         ac_parsingState2[slot] = mb;
            //         ac_parsingState_rsp[slot] = 1;
            //     }
            // }
            break;
        }
        case 1:{
            if(!rxData_in.empty()){
                net_axis<DATA_WIDTH> currWord = rxData_in.read();
                ac_wordProcessingQueues[slot_g].write(currWord);
                if(currWord.last){
                    fsmState = 0;
                }
            }
            break;
        }
        case 2:{
            // starting processing this session. 
            hls::stream<net_axis<DATA_WIDTH> >& tmpQ = ac_wordProcessingQueues[slot_a];
            if(!tmpQ.empty()){
                net_axis<DATA_WIDTH> currWord = tmpQ.read();
                currWordFifo_out.write(currWord);
                fsmState = 0;
            }
            break;
        }
        case 3:{
            hls::stream<net_axis<DATA_WIDTH> >& tmpQ = ac_wordProcessingQueues[slot_b];
            // this session still has more words -- keep parsing
            if(!tmpQ.empty()){
                net_axis<DATA_WIDTH> currWord = tmpQ.read();
                currWordFifo_out.write(currWord);
                currSessionStateFifo_out.write(ss);
                currMsgBodyFifo_out.write(mb);
            }
            // no more words for this session -- stop parsing and store states
            else{
                ac_stash_remove(sessionID);
                s_axis_upd_req.write(parser_htUpdateReq(sessionID, ss, mb, 0));
            }
            fsmState = 0;
            break;
        }
    }
}

#define NUM_WAITING_Q 4
#define BITS_WAITING_Q 2

// multile queues 
static hls::stream<ap_uint<16> > sessionWaitingQueues[NUM_WAITING_Q];
static hls::stream<net_axis<DATA_WIDTH> > wordWaitingQueues[NUM_WAITING_Q];
static ap_uint<16> buffered_sessionWaiting[NUM_WAITING_Q];

bool sessionWaiting_empty(uint32_t slot){
#pragma HLS INLINE
    return ((buffered_sessionWaiting[slot] == 0) && sessionWaitingQueues[slot].empty());
    // return (buffered_sessionWaiting[slot] == 0);
}

bool sessionWaiting_full(uint32_t slot){
#pragma HLS INLINE
    return ((buffered_sessionWaiting[slot] != 0) && sessionWaitingQueues[slot].full());
}

// You must have checked empty() before; 
ap_uint<16> sessionWaiting_pop(uint32_t slot){
#pragma HLS INLINE
    // ap_uint<16> sessionID = buffered_sessionWaiting[slot];
    if(!sessionWaitingQueues[slot].empty()){
        buffered_sessionWaiting[slot] = sessionWaitingQueues[slot].read();
    }
    else{
        buffered_sessionWaiting[slot] = 0;
    }
    // return sessionID;
}

// You must have checked empty() before; 
ap_uint<16> sessionWaiting_peek(uint32_t slot){
#pragma HLS INLINE
    ap_uint<16> sessionID = buffered_sessionWaiting[slot];
    if(sessionID == 0){
        ap_uint<16> sessionID2 = sessionWaitingQueues[slot].read();
        buffered_sessionWaiting[slot] = sessionID2;
        return sessionID2;
    }
    else{
        return sessionID;
    }
    // return buffered_sessionWaiting[slot];
}

void sessionWaiting_write(uint32_t slot, ap_uint<16> val){
#pragma HLS INLINE
    // if(buffered_sessionWaiting[slot] == 0){
    //     buffered_sessionWaiting[slot] = val;
    // }
    // else{
        sessionWaitingQueues[slot].write(val);
    // }
}

// hashing each session with words into multiple waiting queue
void admission_control1(
    hls::stream<ap_uint<16> >&          rxMetaData, // input
	hls::stream<net_axis<DATA_WIDTH> >& rxData // intput 
){
#pragma HLS PIPELINE II=1
#pragma HLS INLINE off
    #pragma HLS ARRAY_PARTITION variable=ac_sessionID_stashTable complete
    // #pragma HLS ARRAY_PARTITION variable=ac_parsingState_rsp complete
    // #pragma HLS ARRAY_PARTITION variable=ac_parsingState1 complete
    // #pragma HLS ARRAY_PARTITION variable=ac_parsingState2 complete
    

    // static value gets inited to zero by default. 
    #pragma HLS stream variable=sessionWaitingQueues[0] depth=4
    #pragma HLS stream variable=sessionWaitingQueues[1] depth=4
    #pragma HLS stream variable=sessionWaitingQueues[2] depth=4
    #pragma HLS stream variable=sessionWaitingQueues[3] depth=4
    
    #pragma HLS stream variable=wordWaitingQueues[0] depth=8
    #pragma HLS stream variable=wordWaitingQueues[1] depth=8
    #pragma HLS stream variable=wordWaitingQueues[2] depth=8
    #pragma HLS stream variable=wordWaitingQueues[3] depth=8
    
    static ap_uint<4> fsmState = 0;
    static ap_uint<16> hashIdx = 0;

    switch(fsmState){
        case 0:{
            if(!rxMetaData.empty()){
                ap_uint<16> sessionID = rxMetaData.read();
                hashIdx = sessionID & (NUM_WAITING_Q-1);
                sessionWaiting_write(hashIdx, sessionID);
                // sessionWaitingQueues[hashIdx].write(sessionID);
                fsmState = 1;
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
    }
}

// round-robin picking a queue to pop a session with words into the pipeline 
void admission_control2(
    hls::stream<ap_uint<16> >&          sessionStashLookupFifo_out, // output to state recovery
    hls::stream<bool >&                 sessionStashLookupRspFifo_in, // input from state recovery
    hls::stream<parser_htLookupReq>&    s_axis_lup_req, // output
    hls::stream<ap_uint<16> >&          rxMetaData_out, // output
	hls::stream<net_axis<DATA_WIDTH> >& rxData_out // output
){
#pragma HLS PIPELINE II=1
#pragma HLS INLINE off

    #pragma HLS ARRAY_PARTITION variable=buffered_sessionWaiting complete
    
    static ap_uint<BITS_WAITING_Q> startPos = 0;
    static ap_uint<4> fsmState = 0;
    // you should not just use uint32_t slot, as this will incur icmp with longer latency. 
    static ap_uint<BITS_WAITING_Q> slot = 0;
    static ap_uint<16> currSessionID = 0;

    switch(fsmState){
        case 0:{
            ap_uint<NUM_WAITING_Q> vlds = 0;
            for(uint32_t i = 0; i < NUM_WAITING_Q; i++){
                #pragma HLS UNROLL
                // vlds[i] = (!sessionWaitingQueues[i].empty() && !wordWaitingQueues[i].empty());
                vlds[i] = (!sessionWaiting_empty(i) && !wordWaitingQueues[i].empty());
            }

            if(vlds == 0){
                std::cout << "admission_control2: no valid fifo" << std::endl;
            }
            else{
                ap_uint<NUM_WAITING_Q*2> vldsDual = (vlds, vlds);
                vldsDual >>= startPos;
                ap_uint<NUM_WAITING_Q> vldsSingle = vldsDual;
                slot = __builtin_ctz(vldsSingle);
                ap_uint<BITS_WAITING_Q> rem = NUM_WAITING_Q - startPos;
                if(slot < rem){
                    slot = slot + startPos;
                }
                else{
                    slot = slot - rem;
                }
                fsmState = 1;
            }
            startPos += 1;
            break;
        }
        case 1:{
            currSessionID = sessionWaiting_peek(slot);
            // lookup the session stash maintained by the state recovery. 
            sessionStashLookupFifo_out.write(currSessionID);
            fsmState = 2;
            break;
        }
        case 2:{
            if(!sessionStashLookupRspFifo_in.empty()){
                bool ret = sessionStashLookupRspFifo_in.read();
                if(ret){ // in pipeline -- conflict, cannot enter pipeline
                    fsmState = 0;
                }
                else{ // not in pipeline
                    // first pop the sessionID from buffered_waitingQ;
                    sessionWaiting_pop(slot);
                    rxMetaData_out.write(currSessionID);
                    fsmState = 3;
                }
            }
            break;
        }
        case 3:{
            if(!wordWaitingQueues[slot].empty()){
                net_axis<DATA_WIDTH> currWord = wordWaitingQueues[slot].read();
                rxData_out.write(currWord);
                if(currWord.last){
                    // after transfering admitted words to state_recovery
                    s_axis_lup_req.write(parser_htLookupReq(currSessionID, 0));
                    fsmState = 0;
                }
            }
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
    hls::stream<hash_table_16_1024::mqPushReq<16,32> >&         mq_push_req, 
    hls::stream<hash_table_16_1024::mqPopReq<16> >&             mq_pop_req, 
    hls::stream<hash_table_16_1024::mqPopResp<16,32> >&         mq_pop_rsp, 
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
        				mq_pop_req.write(hash_table_16_1024::mqPopReq<16>(currSessionID)); // get request msg ID;
                        std::cout << "mcrouter::proxy IDLE POP currSessionID " << currSessionID << std::endl;
                        proxyFsmState = RSP_MQ;
                        break;
                    }
                    case PROTOCOL_BINARY_CMD_RSET: { // RSet
        				mq_pop_req.write(hash_table_16_1024::mqPopReq<16>(currSessionID));
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
                hash_table_16_1024::mqPushReq<16, 32> insertReq(currSessionID_dst, currMsgBody.msgID);
        		mq_push_req.write(insertReq);
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
                hash_table_16_1024::mqPushReq<16, 32> insertReq(sessionID_dst.sessionID, currMsgBody.msgID);
        		mq_push_req.write(insertReq);
                
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
            if(!mq_pop_rsp.empty())
        	{
        		hash_table_16_1024::mqPopResp<16, 32> ret = mq_pop_rsp.read();
                s_axis_lup_req.write(hash_table_32_32::htLookupReq<32>(ret.value, 0));

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
                else{
                    fsmState = WAIT_TX_META;
                }
            }
            break;
        }
        case SEND_WORD:{
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
    		break;
        }
    }
}

void dummy(	
    hls::stream<ap_uint<16> >& closeConnection,
    hls::stream<hash_table_16_1024::mqPushResp<16,32> >& mq_push_rsp, 
    hls::stream<ap_uint<16> >& regInsertFailureCount,
    hls::stream<ap_uint<16> >& regInsertFailureCount1,
    hls::stream<ap_uint<16> >& regInsertFailureCount2
){
#pragma HLS PIPELINE II=1
#pragma HLS INLINE off
    // you must use each FIFO in order to make it get synthesized into IP
    static bool closed = false;
    if(!closed){
    	closeConnection.write(0);
        closed = true;
    }

    if(!mq_push_rsp.empty()){
        hash_table_16_1024::mqPushResp<16,32> ret = mq_push_rsp.read();
        if(!ret.hit){
            std::cout << "[ERROR] mq_push_rsp failed sessionID" << ret.key << std::endl;
        }
    }
    if(!regInsertFailureCount.empty()){
        ap_uint<16> cnt = regInsertFailureCount.read();
        std::cout << "[ERROR] insert failed cnt " << cnt << std::endl;
    }
    if(!regInsertFailureCount1.empty()){
        ap_uint<16> cnt = regInsertFailureCount1.read();
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
	hls::stream<ipTuple>&			openConnection,
	hls::stream<openStatus>&		openConStatus,
    // for mcrouter closing a connection
	hls::stream<ap_uint<16> >&		closeConnection,
    // for mcrouter sending data out
	hls::stream<appTxMeta>&			txMetaData,
	hls::stream<net_axis<DATA_WIDTH> >&	txData,
	hls::stream<appTxRsp>&			txStatus, 
	ap_uint<14>		useConn,
	ap_uint<32>		regIpAddress0,
	ap_uint<32>		regIpAddress1,
	ap_uint<32>		regIpAddress2,
	ap_uint<32>		regIpAddress3,
	ap_uint<32>		regIpAddress4,
	ap_uint<32>		regIpAddress5,
	ap_uint<32>		regIpAddress6,
	ap_uint<32>		regIpAddress7,
	ap_uint<32>		regIpAddress8,
	ap_uint<32>		regIpAddress9,
    ap_uint<16>		regIpPort0,
	ap_uint<16>		regIpPort1,
	ap_uint<16>		regIpPort2,
	ap_uint<16>		regIpPort3,
	ap_uint<16>		regIpPort4,
	ap_uint<16>		regIpPort5,
	ap_uint<16>		regIpPort6,
	ap_uint<16>		regIpPort7,
	ap_uint<16>		regIpPort8,
	ap_uint<16>		regIpPort9,
    ap_uint<16>     ipAddressIdx_debug,
    ap_uint<16>     useConn_debug,
    ap_uint<16>     sessionCount_debug
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
    #pragma HLS DATA_PACK variable=openConnection
    #pragma HLS DATA_PACK variable=openConStatus

    #pragma HLS INTERFACE axis register port=closeConnection name=m_axis_close_connection

    #pragma HLS INTERFACE axis register port=txMetaData name=m_axis_tx_metadata
    #pragma HLS INTERFACE axis register port=txData name=m_axis_tx_data
    #pragma HLS INTERFACE axis register port=txStatus name=s_axis_tx_status
    #pragma HLS DATA_PACK variable=txMetaData
    #pragma HLS DATA_PACK variable=txStatus

	#pragma HLS INTERFACE ap_none register port=useConn
	#pragma HLS INTERFACE ap_none register port=regIpAddress0
	#pragma HLS INTERFACE ap_none register port=regIpAddress1
	#pragma HLS INTERFACE ap_none register port=regIpAddress2
	#pragma HLS INTERFACE ap_none register port=regIpAddress3
	#pragma HLS INTERFACE ap_none register port=regIpAddress4
	#pragma HLS INTERFACE ap_none register port=regIpAddress5
	#pragma HLS INTERFACE ap_none register port=regIpAddress6
	#pragma HLS INTERFACE ap_none register port=regIpAddress7
	#pragma HLS INTERFACE ap_none register port=regIpAddress8
	#pragma HLS INTERFACE ap_none register port=regIpAddress9
    #pragma HLS INTERFACE ap_none register port=regIpPort0
	#pragma HLS INTERFACE ap_none register port=regIpPort1
	#pragma HLS INTERFACE ap_none register port=regIpPort2
	#pragma HLS INTERFACE ap_none register port=regIpPort3
	#pragma HLS INTERFACE ap_none register port=regIpPort4
	#pragma HLS INTERFACE ap_none register port=regIpPort5
	#pragma HLS INTERFACE ap_none register port=regIpPort6
	#pragma HLS INTERFACE ap_none register port=regIpPort7
	#pragma HLS INTERFACE ap_none register port=regIpPort8
	#pragma HLS INTERFACE ap_none register port=regIpPort9
	#pragma HLS INTERFACE ap_none register port=ipAddressIdx_debug
	#pragma HLS INTERFACE ap_none register port=useConn_debug
	#pragma HLS INTERFACE ap_none register port=sessionCount_debug


	static hls::stream<ap_uint<16> >		mc_sessionIdFifo0("mc_sessionIdFifo0");
	static hls::stream<ap_uint<16> >		mc_sessionIdFifo1("mc_sessionIdFifo1");
    static hls::stream<msgHeader>		    mc_msgHeaderFifo0("mc_msgHeaderFifo0");
    static hls::stream<msgHeader>		    mc_msgHeaderFifo1("mc_msgHeaderFifo1");
	static hls::stream<msgBody>		        mc_msgBodyFifo0("mc_msgBodyFifo0");
	static hls::stream<msgBody>		        mc_msgBodyFifo1("mc_msgBodyFifo1");
    #pragma HLS stream variable=mc_sessionIdFifo0 depth=4
    #pragma HLS stream variable=mc_sessionIdFifo1 depth=4
    #pragma HLS stream variable=mc_msgHeaderFifo0 depth=4
    #pragma HLS stream variable=mc_msgHeaderFifo1 depth=4
    #pragma HLS stream variable=mc_msgBodyFifo0 depth=4
    #pragma HLS stream variable=mc_msgBodyFifo1 depth=4

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
    #pragma HLS stream variable=mc_sessionIdFifo4 depth=4
    #pragma HLS stream variable=mc_sessionIdFifo5 depth=4
    #pragma HLS stream variable=mc_sessionIdFifo6 depth=4
    #pragma HLS stream variable=mc_sessionIdFifo7 depth=4
    #pragma HLS stream variable=mc_msgHeaderFifo4 depth=4
    #pragma HLS stream variable=mc_msgHeaderFifo5 depth=4
    #pragma HLS stream variable=mc_msgHeaderFifo6 depth=4
    #pragma HLS stream variable=mc_msgHeaderFifo7 depth=4
    #pragma HLS stream variable=mc_msgBodyFifo4 depth=4
    #pragma HLS stream variable=mc_msgBodyFifo5 depth=4
    #pragma HLS stream variable=mc_msgBodyFifo6 depth=4
    #pragma HLS stream variable=mc_msgBodyFifo7 depth=4


	static hls::stream<ap_uint<MAX_KEY_LEN> >		mc_keyFifo("mc_keyFifo");
	static hls::stream<ap_uint<32> >		mc_hashFifo("mc_hashFifo");
    #pragma HLS stream variable=mc_keyFifo depth=4
    #pragma HLS stream variable=mc_hashFifo depth=4


    static hls::stream<parser_htLookupReq>    s_axis_lup_req;
    static hls::stream<parser_htUpdateReq>    s_axis_upd_req;
    static hls::stream<parser_htLookupResp>   m_axis_lup_rsp;
    static hls::stream<parser_htUpdateResp>   m_axis_upd_rsp;
    static hls::stream<ap_uint<16> > regInsertFailureCount;
    #pragma HLS stream variable=s_axis_lup_req depth=4
    #pragma HLS stream variable=s_axis_upd_req depth=4
    #pragma HLS stream variable=m_axis_lup_rsp depth=4
    #pragma HLS stream variable=m_axis_upd_rsp depth=4
    #pragma HLS stream variable=regInsertFailureCount depth=4
    #pragma HLS DATA_PACK variable=s_axis_lup_req
    #pragma HLS DATA_PACK variable=s_axis_upd_req
    #pragma HLS DATA_PACK variable=m_axis_lup_rsp
    #pragma HLS DATA_PACK variable=m_axis_upd_rsp


    static hls::stream<hash_table_32_32::htLookupReq<32> >       s_axis_lup_req1;
    static hls::stream<hash_table_32_32::htUpdateReq<32,32> >    s_axis_upd_req1;
    static hls::stream<hash_table_32_32::htLookupResp<32,32> >   m_axis_lup_rsp1;
    static hls::stream<hash_table_32_32::htUpdateResp<32,32> >   m_axis_upd_rsp1;
    static hls::stream<ap_uint<16> > regInsertFailureCount1;
    #pragma HLS stream variable=s_axis_lup_req1 depth=4
    #pragma HLS stream variable=s_axis_upd_req1 depth=4
    #pragma HLS stream variable=m_axis_lup_rsp1 depth=4
    #pragma HLS stream variable=m_axis_upd_rsp1 depth=4
    #pragma HLS stream variable=regInsertFailureCount1 depth=4
    #pragma HLS DATA_PACK variable=s_axis_lup_req1
    #pragma HLS DATA_PACK variable=s_axis_upd_req1
    #pragma HLS DATA_PACK variable=m_axis_lup_rsp1
    #pragma HLS DATA_PACK variable=m_axis_upd_rsp1


	// static hls::stream<mqInsertReq<ap_uint<32>> >	multiQueue_push("multiQueue_push");
	// static hls::stream<hash_table_16_1024::mqPopReq>					multiQueue_pop_req("multiQueue_pop_req");
	// static hls::stream<ap_uint<32>>					multiQueue_rsp("multiQueue_rsp");
    // #pragma HLS stream variable=multiQueue_push depth=4
    // #pragma HLS stream variable=multiQueue_pop_req depth=4
    // #pragma HLS stream variable=multiQueue_rsp depth=4
    // #pragma HLS DATA_PACK variable=multiQueue_push
    // #pragma HLS DATA_PACK variable=multiQueue_pop_req
    // #pragma HLS DATA_PACK variable=multiQueue_rsp
    static hls::stream<hash_table_16_1024::mqPushReq<16,32> >    mq_push_req("mq_push_req");
    static hls::stream<hash_table_16_1024::mqPopReq<16> >        mq_pop_req("mq_pop_req");
    static hls::stream<hash_table_16_1024::mqPushResp<16,32> >   mq_push_rsp("mq_push_rsp");
    static hls::stream<hash_table_16_1024::mqPopResp<16,32> >    mq_pop_rsp("mq_pop_rsp");
    static hls::stream<ap_uint<16> >         regInsertFailureCount2;
    #pragma HLS stream variable=mq_push_req depth=4
    #pragma HLS stream variable=mq_pop_req depth=4
    #pragma HLS stream variable=mq_push_rsp depth=4
    #pragma HLS stream variable=mq_pop_rsp depth=4
    #pragma HLS stream variable=regInsertFailureCount2 depth=4
    #pragma HLS DATA_PACK variable=mq_push_req
    #pragma HLS DATA_PACK variable=mq_pop_req
    #pragma HLS DATA_PACK variable=mq_push_rsp
    #pragma HLS DATA_PACK variable=mq_pop_rsp


	static hls::stream<ap_uint<2> >	    mc_cmdFifo("mc_cmdFifo");
	static hls::stream<ap_uint<16> >	mc_sessionCountFifo("mc_sessionCountFifo");
	static hls::stream<sessionID_stream>	mc_sessionIdStreamFifo("mc_sessionIdStreamFifo");
	static hls::stream<ap_uint<32> >	mc_hashValFifo("mc_hashValFifo");
	static hls::stream<ap_uint<16> >	mc_sessionIdFifo2("mc_sessionIdFifo2");
	static hls::stream<ap_uint<16> >	mc_idxFifo("mc_idxFifo");
	static hls::stream<ap_uint<16> >	mc_sessionIdFifo3("mc_sessionIdFifo3");
	static hls::stream<ap_uint<16> >    mc_closedSessionIdFifo("mc_closedSessionIdFifo");
    #pragma HLS stream variable=mc_cmdFifo depth=4
    #pragma HLS stream variable=mc_sessionCountFifo depth=4
    #pragma HLS stream variable=mc_sessionIdStreamFifo depth=4
    #pragma HLS stream variable=mc_hashValFifo depth=4
    #pragma HLS stream variable=mc_sessionIdFifo2 depth=4
    #pragma HLS stream variable=mc_idxFifo depth=4
    #pragma HLS stream variable=mc_sessionIdFifo3 depth=4
    #pragma HLS stream variable=mc_closedSessionIdFifo depth=4


    static hls::stream<ap_uint<MEMCACHED_HDRLEN*8+MAX_BODY_LEN> > mc_sendBufFifo("mc_sendBufFifo");
    #pragma HLS stream variable=mc_sendBufFifo depth=4
    static hls::stream<ap_uint<32> > mc_lengthFifo("mc_lengthFifo");
    #pragma HLS stream variable=mc_lengthFifo depth=4
    static hls::stream<ap_uint<32> > mc_msgIDFifo("mc_msgIDFifo");
    #pragma HLS stream variable=mc_msgIDFifo depth=4
	static hls::stream<ap_uint<16> >	mc_sessionIdFifo8("mc_sessionIdFifo8");
    #pragma HLS stream variable=mc_sessionIdFifo8 depth=4

    static hls::stream<ap_uint<16> > sessionStashLookupFifo("sessionStashLookupFifo");
    #pragma HLS stream variable=sessionStashLookupFifo depth=4
    static hls::stream<bool > sessionStashLookupRspFifo("sessionStashLookupRspFifo");
    #pragma HLS stream variable=sessionStashLookupRspFifo depth=4
    static hls::stream<ap_uint<16> > rxMetaData_sr("rxMetaData_sr");
    #pragma HLS stream variable=rxMetaData_sr depth=4
    static hls::stream<net_axis<DATA_WIDTH> > rxData_sr("rxData_sr");
    #pragma HLS stream variable=rxData_sr depth=4
    #pragma HLS DATA_PACK variable=rxData_sr
    
    static hls::stream<net_axis<DATA_WIDTH> > currWordFiFo_parser("currWordFiFo_parser");
    #pragma HLS stream variable=currWordFiFo_parser depth=4
    #pragma HLS DATA_PACK variable=currWordFiFo_parser
    static hls::stream<sessionState> currSessionStateFifo_parser("currSessionStateFifo_parser");
    #pragma HLS stream variable=currSessionStateFifo_parser depth=4
    #pragma HLS DATA_PACK variable=currSessionStateFifo_parser
    static hls::stream<msgBody> currMsgBodyFifo_parser("currMsgBodyFifo_parser");
    #pragma HLS stream variable=currMsgBodyFifo_parser depth=4
    #pragma HLS DATA_PACK variable=currMsgBodyFifo_parser
    static hls::stream<sessionState> currSessionStateFifo_sr("currSessionStateFifo_sr");
    #pragma HLS stream variable=currSessionStateFifo_sr depth=4
    #pragma HLS DATA_PACK variable=currSessionStateFifo_sr
    static hls::stream<msgBody> currMsgBodyFifo_sr("currMsgBodyFifo_sr");
    #pragma HLS stream variable=currMsgBodyFifo_sr depth=4
    #pragma HLS DATA_PACK variable=currMsgBodyFifo_sr


    // initing a hash table block
    parser_stateman_top(s_axis_lup_req, s_axis_upd_req, m_axis_lup_rsp, m_axis_upd_rsp, regInsertFailureCount);
    hash_table_32_32::hash_table_top(s_axis_lup_req1, s_axis_upd_req1, m_axis_lup_rsp1, m_axis_upd_rsp1, regInsertFailureCount1);

    // initing a multi queue block
    // multi_queue<ap_uint<32>, MAX_CONNECTED_SESSIONS, MAX_CONNECTED_SESSIONS*16>(multiQueue_push, multiQueue_pop_req, multiQueue_rsp);
    // multi_queue<ap_uint<32>, 65535, 65535*8>(multiQueue_push, multiQueue_pop_req, multiQueue_rsp);
    hash_table_16_1024::hash_table_top(mq_push_req, mq_pop_req, mq_push_rsp, mq_pop_rsp, regInsertFailureCount2);


    // opening the mcrouter listening port
	open_port(listenPort, listenPortStatus);

    // connecting to remote memcached;
    conn_manager(openConStatus, openConnection, mc_cmdFifo, mc_sessionCountFifo, mc_sessionIdStreamFifo, 
            mc_hashValFifo, mc_sessionIdFifo2, mc_idxFifo, mc_sessionIdFifo3, mc_closedSessionIdFifo, 
            useConn, regIpAddress0, regIpAddress1, regIpAddress2, regIpAddress3, regIpAddress4, regIpAddress5, regIpAddress6, regIpAddress7, regIpAddress8, regIpAddress9, 
            regIpPort0, regIpPort1, regIpPort2, regIpPort3, regIpPort4, regIpPort5, regIpPort6, regIpPort7, regIpPort8, regIpPort9, 
            ipAddressIdx_debug, useConn_debug, sessionCount_debug);
    
    // handling new data arriving (it might come from a new session), and connection closed
	notification_handler(notifications, readRequest, mc_closedSessionIdFifo);
    
    // read data from network and parse them into msgFifo
    // parser(rxMetaData, rxData, mc_sessionIdFifo0, mc_msgHeaderFifo0, mc_msgBodyFifo0, 
    //         s_axis_lup_req, s_axis_upd_req, m_axis_lup_rsp);
    admission_control1(rxMetaData, rxData);
    admission_control2(sessionStashLookupFifo, sessionStashLookupRspFifo, s_axis_lup_req, rxMetaData_sr, rxData_sr);
    state_recovery(rxMetaData_sr, rxData_sr, sessionStashLookupFifo, sessionStashLookupRspFifo, m_axis_lup_rsp, m_axis_upd_rsp, s_axis_upd_req, 
            currWordFiFo_parser, currSessionStateFifo_parser, currMsgBodyFifo_parser, currSessionStateFifo_sr, currMsgBodyFifo_sr);
    parser(currWordFiFo_parser, currSessionStateFifo_parser, currMsgBodyFifo_parser, currSessionStateFifo_sr, currMsgBodyFifo_sr, 
            mc_sessionIdFifo0, mc_msgHeaderFifo0, mc_msgBodyFifo0);
    
    // simple hash function 
    simple_hash(mc_keyFifo, mc_hashFifo);

	// 1) read request from fifo, determine the destination mc, query mc
    // 2) read response from fifo, forward response to client. 
    proxy(mc_sessionIdFifo0, mc_msgHeaderFifo0, mc_msgBodyFifo0, mc_sessionIdFifo1, mc_msgHeaderFifo1, mc_msgBodyFifo1, \
            mc_keyFifo, mc_hashFifo, mq_push_req, mq_pop_req, mq_pop_rsp, \
            s_axis_lup_req1, s_axis_upd_req1, m_axis_lup_rsp1, m_axis_upd_rsp1,\
            mc_cmdFifo, mc_sessionCountFifo, mc_sessionIdStreamFifo, mc_hashValFifo, mc_sessionIdFifo2, mc_idxFifo, mc_sessionIdFifo3);

    // deparsing the res and resp, sending to destination (client or memcached)
    deparser(mc_sessionIdFifo1, mc_msgHeaderFifo1, mc_msgBodyFifo1, txMetaData, txStatus, txData);

	dummy(closeConnection, mq_push_rsp, regInsertFailureCount, regInsertFailureCount1, regInsertFailureCount2);

}
