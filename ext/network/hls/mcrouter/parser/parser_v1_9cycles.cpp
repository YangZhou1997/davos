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

template <int IDX, int W>
void msg_strip(
    // the initial states before parsing this word
    hls::stream<net_axis<DATA_WIDTH> >& currWordFifo,
    hls::stream<ap_uint<32> >& currWordValidLenFifo,
    hls::stream<sessionState>& currSessionStateFifo,
    hls::stream<msgBody>& currMsgBodyFifo,
    // the updated states to the next msg_strip after parsing this msg
    hls::stream<net_axis<DATA_WIDTH> >& currWordOutFifo,
    hls::stream<ap_uint<32> >& currWordValidLenOutFifo,
    hls::stream<sessionState>& currSessionStateOutFifo,
    hls::stream<msgBody>& currMsgBodyOutFifo,
    // the output of the parsed msg
    hls::stream<ap_uint<16> >& sessionIDOutFifo, // output
    hls::stream<msgHeader>& msgHeaderOutFifo,
    hls::stream<msgBody>& msgBodyOutFifo, 
    // the output of the parser states
    hls::stream<sessionState>& sessionStateOutFifo,
    hls::stream<msgBody>& msgBodyStateOutFifo
){
#pragma HLS PIPELINE II=1
#pragma HLS INLINE off

    if(!currWordFifo.empty() && !currWordValidLenFifo.empty() && !currSessionStateFifo.empty() && !currMsgBodyFifo.empty()){
        std::cout << "=======================================" << std::endl;
        std::cout << "msg_strip IDX = " << IDX << std::endl;
        ap_uint<32> currWordValidLen = currWordValidLenFifo.read();
        net_axis<DATA_WIDTH> currWord = currWordFifo.read();
        sessionState currSessionState = currSessionStateFifo.read();
        msgBody currMsgBody = currMsgBodyFifo.read();

        std::cout << "currSessionState: " << std::endl;
        currSessionState.display();

        msgHeader currMsgHeader;

        ap_uint<32> currWordValidLen_init = keepToLen(currWord.keep);
        // in the begining of a msg; 
        if(currWordValidLen == 0){
            currWordValidLen = currWordValidLen_init;
        }
        ap_uint<32> currWord_parsingPos = DATA_WIDTH - (currWordValidLen_init - currWordValidLen)*8;

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

                std::cout << "case 1: parser receives data" << std::endl;
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

                std::cout << "case 2: parser receives data" << std::endl;
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

            std::cout << "case 3: parser receives data" << std::endl;
            currMsgHeader.display();

            std::cout << "currSessionState.requiredLen=" << currSessionState.requiredLen 
                << " MEMCACHED_HDRLEN + currSessionState.currBodyLen=" << MEMCACHED_HDRLEN + currSessionState.currBodyLen << std::endl;

            // parsingMsgBody(currSessionState, currMsgBody, currMsgHeader, currWord, currWordValidLen, currWordValidLen_init);
            ap_uint<16> currWord_parsingPos = DATA_WIDTH - (currWordValidLen_init - currWordValidLen)*8;
            ap_uint<32> currBodyLen = currSessionState.currBodyLen;
            ap_uint<32> requiredBodyLen = currSessionState.requiredLen - MEMCACHED_HDRLEN - currBodyLen;

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

        ap_uint<1> parsingMsgState = (currSessionState.parsingHeaderState == 1 && currSessionState.parsingBodyState == 1);
   
        if(parsingMsgState){
            std::cout << "writing currMsgBody (path1) for currMsgBody.msgID " << currMsgBody.msgID << std::endl;

            // !!! only write out when current msg is parsed
            sessionIDOutFifo.write(currMsgBody.currSessionID);
            msgHeaderOutFifo.write(currMsgHeader);
            msgBodyOutFifo.write(currMsgBody);

            currMsgHeader.display();
            currMsgBody.display(currMsgHeader.extLen, currMsgHeader.keyLen, currMsgHeader.val_len());
        }

        // this word contains more than one msg, let next msg_strip process it. 
        if(currWordValidLen > 0){
            // the last msg_strip does not need to output currWord. 
            if(IDX != 4){
                currWordOutFifo.write(currWord);
            }
            currWordValidLenOutFifo.write(currWordValidLen);
            // write out brand new sessionState and msgBody. 
            currSessionStateOutFifo.write(sessionState(currSessionState.currSessionID));
            currMsgBodyOutFifo.write(msgBody(currMsgBody.currSessionID));
        }
        std::cout << "End of msgStrip: currWordValidLen=" << currWordValidLen << std::endl;
        
        if(parsingMsgState && currWordValidLen == 0){
            currSessionState.reset();
            currMsgBody.reset();
        }
        
        // one word outputs a sessionState. 
        if(currWordValidLen == 0){
            currMsgBody.reserved = 1;
        }
        sessionStateOutFifo.write(currSessionState);
        msgBodyStateOutFifo.write(currMsgBody);
    }
}

void msgMux3to1(
    hls::stream<ap_uint<16> >&              sessionIDFifo1, // input
    hls::stream<msgHeader>&                 msgHeaderFifo1, // input
    hls::stream<msgBody>&                   msgBodyFifo1, // input
    hls::stream<ap_uint<16> >&              sessionIDFifo2, // input
    hls::stream<msgHeader>&                 msgHeaderFifo2, // input
    hls::stream<msgBody>&                   msgBodyFifo2, // input
    hls::stream<ap_uint<16> >&              sessionIDFifo3, // input
    hls::stream<msgHeader>&                 msgHeaderFifo3, // input
    hls::stream<msgBody>&                   msgBodyFifo3, // input
    hls::stream<ap_uint<16> >&              sessionIDFifo_out, // output
    hls::stream<msgHeader>&                 msgHeaderFifo_out, // output
    hls::stream<msgBody>&                   msgBodyFifo_out // output
){
#pragma HLS PIPELINE II=1
#pragma HLS INLINE off

    ap_uint<1> fifo1_vld = (!msgHeaderFifo1.empty() && !msgBodyFifo1.empty());
    ap_uint<1> fifo2_vld = (!msgHeaderFifo2.empty() && !msgBodyFifo2.empty());
    ap_uint<1> fifo3_vld = (!msgHeaderFifo3.empty() && !msgBodyFifo3.empty());

    if(fifo3_vld){
        ap_uint<16> sessionID3 = sessionIDFifo3.read();
        msgHeader msghdr3 = msgHeaderFifo3.read();
        msgBody msgbody3 = msgBodyFifo3.read();
        sessionIDFifo_out.write(sessionID3);
        msgHeaderFifo_out.write(msghdr3);
        msgBodyFifo_out.write(msgbody3);
        std::cout << "msgMux3to1 state3" << std::endl;
    }
    else if(fifo2_vld){
        ap_uint<16> sessionID2 = sessionIDFifo2.read();
        msgHeader msghdr2 = msgHeaderFifo2.read();
        msgBody msgbody2 = msgBodyFifo2.read();
        sessionIDFifo_out.write(sessionID2);
        msgHeaderFifo_out.write(msghdr2);
        msgBodyFifo_out.write(msgbody2);
        std::cout << "msgMux3to1 state2" << std::endl;
    }
    else if(fifo1_vld){
        ap_uint<16> sessionID1 = sessionIDFifo1.read();
        msgHeader msghdr1 = msgHeaderFifo1.read();
        msgBody msgbody1 = msgBodyFifo1.read();
        sessionIDFifo_out.write(sessionID1);
        msgHeaderFifo_out.write(msghdr1);
        msgBodyFifo_out.write(msgbody1);
        std::cout << "msgMux3to1 state1" << std::endl;
    }
}

void statesMux4to1(
    hls::stream<sessionState>&        sessionStateFifo1,
    hls::stream<msgBody>&             msgBodyFifo1,
    hls::stream<sessionState>&        sessionStateFifo2,
    hls::stream<msgBody>&             msgBodyFifo2,
    hls::stream<sessionState>&        sessionStateFifo3,
    hls::stream<msgBody>&             msgBodyFifo3,
    hls::stream<sessionState>&        sessionStateFifo4,
    hls::stream<msgBody>&             msgBodyFifo4,
    hls::stream<sessionState>&        sessionStateFifo_out,
    hls::stream<msgBody>&             msgBodyFifo_out
){
#pragma HLS PIPELINE II=1
#pragma HLS INLINE off

	enum muxStateType{FWD1=0, FWD2, FWD3, FWD4};
    static muxStateType fsmState = FWD1;

    ap_uint<1> fifo1_vld = (!sessionStateFifo1.empty() && !msgBodyFifo1.empty());
    ap_uint<1> fifo2_vld = (!sessionStateFifo2.empty() && !msgBodyFifo2.empty());
    ap_uint<1> fifo3_vld = (!sessionStateFifo3.empty() && !msgBodyFifo3.empty());
    ap_uint<1> fifo4_vld = (!sessionStateFifo4.empty() && !msgBodyFifo4.empty());
    
    std::cout << "statesMux4to1 state: " << fsmState << std::endl;
    switch(fsmState) {
        case FWD1: {
            if(fifo1_vld){
                sessionState sessionstate1 = sessionStateFifo1.read();
                msgBody msgbody1 = msgBodyFifo1.read();
                std::cout << "statesMux4to1 sessionstate1" << std::endl;
                sessionstate1.display();
                if(!msgbody1.reserved){
                    fsmState = FWD2;
                }
                else{
                    fsmState = FWD1;
                    msgbody1.reserved = 0;
                    sessionStateFifo_out.write(sessionstate1);
                    msgBodyFifo_out.write(msgbody1);
                }
            }
            break;
        }
        case FWD2: {
            if(fifo2_vld){
                sessionState sessionstate2 = sessionStateFifo2.read();
                msgBody msgbody2 = msgBodyFifo2.read();
                std::cout << "statesMux4to1 sessionstate2" << std::endl;
                sessionstate2.display();
                if(!msgbody2.reserved){
                    fsmState = FWD3;
                }
                else{
                    fsmState = FWD1;
                    msgbody2.reserved = 0;
                    sessionStateFifo_out.write(sessionstate2);
                    msgBodyFifo_out.write(msgbody2);
                }
            }
            break;
        }
        case FWD3: {
            if(fifo3_vld){
                sessionState sessionstate3 = sessionStateFifo3.read();
                msgBody msgbody3 = msgBodyFifo3.read();
                std::cout << "statesMux4to1 sessionstate3" << std::endl;
                sessionstate3.display();
                if(!msgbody3.reserved){
                    fsmState = FWD4;
                }
                else{
                    fsmState = FWD1;
                    msgbody3.reserved = 0;
                    sessionStateFifo_out.write(sessionstate3);
                    msgBodyFifo_out.write(msgbody3);
                }
            }
            break;
        }
        case FWD4: {
            if(fifo4_vld){
                sessionState sessionstate4 = sessionStateFifo4.read();
                msgBody msgbody4 = msgBodyFifo4.read();
                std::cout << "statesMux4to1 sessionstate4" << std::endl;
                sessionstate4.display();
                if(!msgbody4.reserved){
                    std::cout << "statesMux4to1 error" << std::endl;
                }
                fsmState = FWD1;
                msgbody4.reserved = 0;
                sessionStateFifo_out.write(sessionstate4);
                msgBodyFifo_out.write(msgbody4);
            }
            break;
        }
    }
}

void word1to3(
    hls::stream<net_axis<DATA_WIDTH> >& currWordFifo,
    hls::stream<net_axis<DATA_WIDTH> >& currWordFifo1,
    hls::stream<ap_uint<32> >&          currWordValidLenFifo
){
#pragma HLS PIPELINE II=1
#pragma HLS INLINE off

    if(!currWordFifo.empty()){
        net_axis<DATA_WIDTH> currWord = currWordFifo.read();
        currWordFifo1.write(currWord);
        currWordValidLenFifo.write(0);
    }
}

void parser(
    // the word waiting for parsing
    hls::stream<net_axis<DATA_WIDTH> >& currWordFifo,
    // the initial states before parsing this word
    hls::stream<sessionState>& currSessionStateFifo,
    hls::stream<msgBody>& currMsgBodyFifo,
    // the updated states after parsing this word
    hls::stream<sessionState>& currSessionStateOutFifo,
    hls::stream<msgBody>& currMsgBodyStateOutFifo,
    // the output of the parsed msg
    hls::stream<ap_uint<16> >& sessionIDOutFifo,
    hls::stream<msgHeader>& msgHeaderOutFifo,
    hls::stream<msgBody>& msgBodyOutFifo
){
#pragma HLS DATAFLOW disable_start_propagation
#pragma HLS INTERFACE ap_ctrl_none port=return

    static hls::stream<ap_uint<32> >    currWordValidLenFifo("currWordValidLenFifo");
    #pragma HLS stream variable=currWordValidLenFifo depth=2

    static hls::stream<ap_uint<32> >    currWordValidLenFifo1("currWordValidLenFifo1");
    static hls::stream<sessionState>    currSessionStateFifo1("currSessionStateFifo1");
    static hls::stream<msgBody>         currMsgBodyFifo1("currMsgBodyFifo1");
    static hls::stream<ap_uint<32> >    currWordValidLenFifo2("currWordValidLenFifo2");
    static hls::stream<sessionState>    currSessionStateFifo2("currSessionStateFifo2");
    static hls::stream<msgBody>         currMsgBodyFifo2("currMsgBodyFifo2");
    static hls::stream<ap_uint<32> >    currWordValidLenFifo3("currWordValidLenFifo3");
    static hls::stream<sessionState>    currSessionStateFifo3("currSessionStateFifo3");
    static hls::stream<msgBody>         currMsgBodyFifo3("currMsgBodyFifo3");
    static hls::stream<ap_uint<32> >    currWordValidLenFifo4("currWordValidLenFifo4");
    static hls::stream<sessionState>    currSessionStateFifo4("currSessionStateFifo4");
    static hls::stream<msgBody>         currMsgBodyFifo4("currMsgBodyFifo4");
    #pragma HLS stream variable=currWordValidLenFifo1 depth=2
    #pragma HLS stream variable=currSessionStateFifo1 depth=2
    #pragma HLS stream variable=currMsgBodyFifo1 depth=2
    #pragma HLS stream variable=currWordValidLenFifo2 depth=2
    #pragma HLS stream variable=currSessionStateFifo2 depth=2
    #pragma HLS stream variable=currMsgBodyFifo2 depth=2
    #pragma HLS stream variable=currWordValidLenFifo3 depth=2
    #pragma HLS stream variable=currSessionStateFifo3 depth=2
    #pragma HLS stream variable=currMsgBodyFifo3 depth=2
    #pragma HLS stream variable=currWordValidLenFifo4 depth=2
    #pragma HLS stream variable=currSessionStateFifo4 depth=2
    #pragma HLS stream variable=currMsgBodyFifo4 depth=2
    #pragma HLS DATA_PACK variable=currSessionStateFifo1
    #pragma HLS DATA_PACK variable=currMsgBodyFifo1
    #pragma HLS DATA_PACK variable=currSessionStateFifo2
    #pragma HLS DATA_PACK variable=currMsgBodyFifo2
    #pragma HLS DATA_PACK variable=currSessionStateFifo3
    #pragma HLS DATA_PACK variable=currMsgBodyFifo3
    #pragma HLS DATA_PACK variable=currSessionStateFifo4
    #pragma HLS DATA_PACK variable=currMsgBodyFifo4

    static hls::stream<sessionState>        sessionStateFifo1("sessionStateFifo1");
    static hls::stream<sessionState>        sessionStateFifo2("sessionStateFifo2");
    static hls::stream<sessionState>        sessionStateFifo3("sessionStateFifo3");
    static hls::stream<sessionState>        sessionStateFifo4("sessionStateFifo4");
    static hls::stream<msgBody>             msgBodyStateFifo1("msgBodyStateFifo1");
    static hls::stream<msgBody>             msgBodyStateFifo2("msgBodyStateFifo2");
    static hls::stream<msgBody>             msgBodyStateFifo3("msgBodyStateFifo3");
    static hls::stream<msgBody>             msgBodyStateFifo4("msgBodyStateFifo4");
    #pragma HLS stream variable=sessionStateFifo1 depth=2
    #pragma HLS stream variable=sessionStateFifo2 depth=2
    #pragma HLS stream variable=sessionStateFifo3 depth=2
    #pragma HLS stream variable=sessionStateFifo4 depth=2
    #pragma HLS stream variable=msgBodyStateFifo1 depth=2
    #pragma HLS stream variable=msgBodyStateFifo2 depth=2
    #pragma HLS stream variable=msgBodyStateFifo3 depth=2
    #pragma HLS stream variable=msgBodyStateFifo4 depth=2
    #pragma HLS DATA_PACK variable=sessionStateFifo1
    #pragma HLS DATA_PACK variable=sessionStateFifo2
    #pragma HLS DATA_PACK variable=sessionStateFifo3
    #pragma HLS DATA_PACK variable=sessionStateFifo4
    #pragma HLS DATA_PACK variable=msgBodyStateFifo1
    #pragma HLS DATA_PACK variable=msgBodyStateFifo2
    #pragma HLS DATA_PACK variable=msgBodyStateFifo3
    #pragma HLS DATA_PACK variable=msgBodyStateFifo4

    static hls::stream<msgHeader>		    msgHeaderFifo1("msgHeaderFifo1");
    static hls::stream<msgHeader>		    msgHeaderFifo2("msgHeaderFifo2");
	static hls::stream<msgHeader>		    msgHeaderFifo3("msgHeaderFifo3");
    static hls::stream<msgHeader>		    msgHeaderFifo4("msgHeaderFifo4");
	static hls::stream<msgBody>		        msgBodyFifo1("msgBodyFifo1");
	static hls::stream<msgBody>		        msgBodyFifo2("msgBodyFifo2");
    static hls::stream<msgBody>		        msgBodyFifo3("msgBodyFifo3");
	static hls::stream<msgBody>		        msgBodyFifo4("msgBodyFifo4");
    #pragma HLS stream variable=msgHeaderFifo1 depth=2
    #pragma HLS stream variable=msgHeaderFifo2 depth=2
    #pragma HLS stream variable=msgHeaderFifo3 depth=2
    #pragma HLS stream variable=msgHeaderFifo4 depth=2
    #pragma HLS stream variable=msgBodyFifo1 depth=2
    #pragma HLS stream variable=msgBodyFifo2 depth=2
    #pragma HLS stream variable=msgBodyFifo3 depth=2
    #pragma HLS stream variable=msgBodyFifo4 depth=2
    #pragma HLS DATA_PACK variable=msgHeaderFifo1
    #pragma HLS DATA_PACK variable=msgHeaderFifo2
    #pragma HLS DATA_PACK variable=msgHeaderFifo3
    #pragma HLS DATA_PACK variable=msgHeaderFifo4
    #pragma HLS DATA_PACK variable=msgBodyFifo1
    #pragma HLS DATA_PACK variable=msgBodyFifo2
    #pragma HLS DATA_PACK variable=msgBodyFifo3
    #pragma HLS DATA_PACK variable=msgBodyFifo4

    static hls::stream<net_axis<DATA_WIDTH> >   currWordFifo1;
    static hls::stream<net_axis<DATA_WIDTH> >   currWordFifo2;
    static hls::stream<net_axis<DATA_WIDTH> >   currWordFifo3;
    static hls::stream<net_axis<DATA_WIDTH> >   currWordFifo4;
    static hls::stream<net_axis<DATA_WIDTH> >   currWordFifo5;
    #pragma HLS stream variable=currWordFifo1 depth=2
    #pragma HLS stream variable=currWordFifo2 depth=2
    #pragma HLS stream variable=currWordFifo3 depth=2
    #pragma HLS stream variable=currWordFifo4 depth=2
    #pragma HLS stream variable=currWordFifo5 depth=2
    #pragma HLS DATA_PACK variable=currWordFifo1
    #pragma HLS DATA_PACK variable=currWordFifo2
    #pragma HLS DATA_PACK variable=currWordFifo3
    #pragma HLS DATA_PACK variable=currWordFifo4
    #pragma HLS DATA_PACK variable=currWordFifo5

    static hls::stream<ap_uint<16> >   sessionIDFifo1;
    static hls::stream<ap_uint<16> >   sessionIDFifo2;
    static hls::stream<ap_uint<16> >   sessionIDFifo3;
    static hls::stream<ap_uint<16> >   sessionIDFifo4;
    #pragma HLS stream variable=sessionIDFifo1 depth=2
    #pragma HLS stream variable=sessionIDFifo2 depth=2
    #pragma HLS stream variable=sessionIDFifo3 depth=2
    #pragma HLS stream variable=sessionIDFifo4 depth=2

    word1to3(currWordFifo, currWordFifo1, currWordValidLenFifo);

    msg_strip<1, 0xa>(currWordFifo1, currWordValidLenFifo, currSessionStateFifo, currMsgBodyFifo, 
                      currWordFifo2, currWordValidLenFifo1, currSessionStateFifo1, currMsgBodyFifo1, 
                      sessionIDFifo1, msgHeaderFifo1, msgBodyFifo1, sessionStateFifo1, msgBodyStateFifo1);
    msg_strip<2, 0xb>(currWordFifo2, currWordValidLenFifo1, currSessionStateFifo1, currMsgBodyFifo1, 
                      currWordFifo3, currWordValidLenFifo2, currSessionStateFifo2, currMsgBodyFifo2, 
                      sessionIDFifo2, msgHeaderFifo2, msgBodyFifo2, sessionStateFifo2, msgBodyStateFifo2);
    msg_strip<3, 0xc>(currWordFifo3, currWordValidLenFifo2, currSessionStateFifo2, currMsgBodyFifo2, 
                      currWordFifo4, currWordValidLenFifo3, currSessionStateFifo3, currMsgBodyFifo3, 
                      sessionIDFifo3, msgHeaderFifo3, msgBodyFifo3, sessionStateFifo3, msgBodyStateFifo3);
    msg_strip<4, 0xd>(currWordFifo4, currWordValidLenFifo3, currSessionStateFifo3, currMsgBodyFifo3, 
                      currWordFifo5, currWordValidLenFifo4, currSessionStateFifo4, currMsgBodyFifo4, 
                      sessionIDFifo4, msgHeaderFifo4, msgBodyFifo4, sessionStateFifo4, msgBodyStateFifo4);

    msgMux3to1(sessionIDFifo1, msgHeaderFifo1, msgBodyFifo1, 
                sessionIDFifo2, msgHeaderFifo2, msgBodyFifo2, 
                sessionIDFifo3, msgHeaderFifo3, msgBodyFifo3, 
                sessionIDOutFifo, msgHeaderOutFifo, msgBodyOutFifo);
    
    statesMux4to1(sessionStateFifo1, msgBodyStateFifo1, sessionStateFifo2, msgBodyStateFifo2, 
                sessionStateFifo3, msgBodyStateFifo3, sessionStateFifo4, msgBodyStateFifo4, 
                currSessionStateOutFifo, currMsgBodyStateOutFifo);
}
