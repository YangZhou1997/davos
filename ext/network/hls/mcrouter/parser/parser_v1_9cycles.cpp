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
    hls::stream<ap_uint<32> >& currWord_parsingPosFifo,
    hls::stream<sessionState>& currSessionStateFifo,
    hls::stream<msgBody>& currMsgBodyFifo,
    // the updated states to the next msg_strip after parsing this msg
    hls::stream<net_axis<DATA_WIDTH> >& currWordOutFifo,
    hls::stream<ap_uint<32> >& currWordValidLenOutFifo,
    hls::stream<ap_uint<32> >& currWord_parsingPosOutFifo,
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
        ap_uint<32> currWord_parsingPos = currWord_parsingPosFifo.read();

        std::cout << "currSessionState: " << std::endl;
        currSessionState.display();
        msgHeader currMsgHeader;

        // parse partial header + body; 
        if(currSessionState.parsingHeaderState == 0){
            ap_uint<5> requiredHeaderLen = MEMCACHED_HDRLEN - currSessionState.currHdrLen;
            if(currWordValidLen < requiredHeaderLen){
                currSessionState.msgHeaderBuff(requiredHeaderLen*8-1, requiredHeaderLen*8-currWordValidLen*8) = currWord.data(currWord_parsingPos-1, currWord_parsingPos-currWordValidLen*8);
                currSessionState.currHdrLen += currWordValidLen;
                currWord_parsingPos = 0;
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
                currWord_parsingPos = 0;
                currWordValidLen = 0;
            }
            else{
                currSessionState.msgHeaderBuff(requiredHeaderLen*8-1, 0) = currWord.data(currWord_parsingPos-1, currWord_parsingPos-requiredHeaderLen*8);
                currMsgHeader.consume_word(currSessionState.msgHeaderBuff);

                std::cout << "case 2: parser receives data" << std::endl;
                currMsgHeader.display();

                currWord_parsingPos -= requiredHeaderLen*8;
                currWordValidLen -= requiredHeaderLen;
                currSessionState.currHdrLen += requiredHeaderLen;
                currSessionState.parsingHeaderState = 1; // header is parsed. 
                
                if(currMsgHeader.bodyLen > 0){
                    currSessionState.requiredLen = currMsgHeader.bodyLen + MEMCACHED_HDRLEN;
                    std::cout << "currSessionState.requiredLen=" << currSessionState.requiredLen 
                        << " MEMCACHED_HDRLEN + currSessionState.currBodyLen=" << MEMCACHED_HDRLEN + currSessionState.currBodyLen << std::endl;
                    
                    ap_uint<32> requiredBodyLen = currMsgHeader.bodyLen;

                    std::cout << "currWordValidLen=" << currWordValidLen << std::endl;
                    std::cout << "currBodyLen=" << 0 << " requiredBodyLen=" << requiredBodyLen << std::endl;
                    
                    if(currWordValidLen >= requiredBodyLen){
                        currMsgBody.body(MAX_BODY_LEN-1, MAX_BODY_LEN-requiredBodyLen*8) = currWord.data(currWord_parsingPos-1, currWord_parsingPos-requiredBodyLen*8);
                        currMsgBody.extInl = 1;
                        currMsgBody.keyInl = 1;
                        currMsgBody.valInl = 1;
                        
                        currSessionState.parsingBodyState = 1; // body is parsed
                        currWord_parsingPos -= requiredBodyLen*8;
                        currWordValidLen -= requiredBodyLen;
                    }
                    else{
                        currMsgBody.body(MAX_BODY_LEN-1, MAX_BODY_LEN-currWordValidLen*8) = currWord.data(currWord_parsingPos-1, currWord_parsingPos-currWordValidLen*8);
                        currSessionState.currBodyLen += currWordValidLen;
                        currWord_parsingPos = 0;
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
                currWord_parsingPos -= requiredBodyLen*8;
                currWordValidLen -= requiredBodyLen;
            }
            else{
                currMsgBody.body(MAX_BODY_LEN-1-currBodyLen*8, MAX_BODY_LEN-currBodyLen*8-currWordValidLen*8) = currWord.data(currWord_parsingPos-1, currWord_parsingPos-currWordValidLen*8);
                currSessionState.currBodyLen += currWordValidLen;
                currWord_parsingPos = 0;
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
            currWordOutFifo.write(currWord);
            currWordValidLenOutFifo.write(currWordValidLen);
            currWord_parsingPosOutFifo.write(currWord_parsingPos);
            // write out brand new sessionState and msgBody, as you are gonna parsing new msg;
            currSessionStateOutFifo.write(sessionState(currSessionState.currSessionID));
            currMsgBodyOutFifo.write(msgBody(currMsgBody.currSessionID));
        }
        std::cout << "End of msgStrip: currWordValidLen=" << currWordValidLen << std::endl;
        
        if(parsingMsgState && currWordValidLen == 0){
            currSessionState.reset2();
            currMsgBody.reset();
        }
        
        // one word outputs a sessionState. 
        if(currWordValidLen == 0){
            sessionStateOutFifo.write(currSessionState);
            msgBodyStateOutFifo.write(currMsgBody);
        }
    }
}

template <class T>
void mux2to1(hls::stream<T>& in1, hls::stream<T>& in2, hls::stream<T>& out){
#pragma HLS PIPELINE II=1
#pragma HLS INLINE off
    if(!in1.empty()){
        T tmp = in1.read();
        out.write(tmp);
    }
    else if(!in2.empty()){
        T tmp = in2.read();
        out.write(tmp);
    }
}

void wordLen_fwd(
    hls::stream<net_axis<DATA_WIDTH> >& currWordFifo,
    hls::stream<net_axis<DATA_WIDTH> >& currWordFifo_out,
    hls::stream<ap_uint<32> >&          currWordValidLenFifo,
    hls::stream<ap_uint<32> >&          currWord_parsingPosFifo
){
#pragma HLS PIPELINE II=1
#pragma HLS INLINE off

    if(!currWordFifo.empty()){
        net_axis<DATA_WIDTH> currWord = currWordFifo.read();
        currWordFifo_out.write(currWord);
        currWordValidLenFifo.write(keepToLen(currWord.keep));
        currWord_parsingPosFifo.write(DATA_WIDTH);
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

    static hls::stream<net_axis<DATA_WIDTH> >   currWordFifo_init("currWordFifo_init");
    static hls::stream<ap_uint<32> >    currWordValidLenFifo_init("currWordValidLenFifo_init");
    static hls::stream<ap_uint<32> >    currWord_parsingPosFifo_init("currWord_parsingPosFifo_init");
    #pragma HLS stream variable=currWordFifo_init depth=4
    #pragma HLS stream variable=currWordValidLenFifo_init depth=4
    #pragma HLS stream variable=currWord_parsingPosFifo_init depth=4
    #pragma HLS DATA_PACK variable=currWordFifo_init

    static hls::stream<net_axis<DATA_WIDTH> >   currWordFifo_inner("currWordFifo_inner");
    static hls::stream<ap_uint<32> >    currWordValidLenFifo_inner("currWordValidLenFifo_inner");
    static hls::stream<ap_uint<32> >    currWord_parsingPosFifo_inner("currWord_parsingPosFifo_inner");
    static hls::stream<sessionState>    currSessionStateFifo_inner("currSessionStateFifo_inner");
    static hls::stream<msgBody>         currMsgBodyFifo_inner("currMsgBodyFifo_inner");
    #pragma HLS stream variable=currWordFifo_inner depth=4
    #pragma HLS stream variable=currWordValidLenFifo_inner depth=4
    #pragma HLS stream variable=currWord_parsingPosFifo_inner depth=4
    #pragma HLS stream variable=currSessionStateFifo_inner depth=4
    #pragma HLS stream variable=currMsgBodyFifo_inner depth=4
    #pragma HLS DATA_PACK variable=currWordFifo_inner
    #pragma HLS DATA_PACK variable=currSessionStateFifo_inner
    #pragma HLS DATA_PACK variable=currMsgBodyFifo_inner
   
    static hls::stream<net_axis<DATA_WIDTH> >   currWordFifo_merged("currWordFifo_merged");
    static hls::stream<ap_uint<32> >    currWordValidLenFifo_merged("currWordValidLenFifo_merged");
    static hls::stream<ap_uint<32> >    currWord_parsingPosFifo_merged("currWord_parsingPosFifo_merged");
    static hls::stream<sessionState>    currSessionStateFifo_merged("currSessionStateFifo_merged");
    static hls::stream<msgBody>         currMsgBodyFifo_merged("currMsgBodyFifo_merged");
    #pragma HLS stream variable=currWordFifo_merged depth=4
    #pragma HLS stream variable=currWordValidLenFifo_merged depth=4
    #pragma HLS stream variable=currWord_parsingPosFifo_merged depth=4
    #pragma HLS stream variable=currSessionStateFifo_merged depth=4
    #pragma HLS stream variable=currMsgBodyFifo_merged depth=4
    #pragma HLS DATA_PACK variable=currWordFifo_merged
    #pragma HLS DATA_PACK variable=currSessionStateFifo_merged
    #pragma HLS DATA_PACK variable=currMsgBodyFifo_merged
   
    
    wordLen_fwd(currWordFifo, currWordFifo_init, currWordValidLenFifo_init, currWord_parsingPosFifo_init);

    mux2to1(currWordFifo_init, currWordFifo_inner, currWordFifo_merged);
    mux2to1(currWordValidLenFifo_init, currWordValidLenFifo_inner, currWordValidLenFifo_merged);
    mux2to1(currWord_parsingPosFifo_init, currWord_parsingPosFifo_inner, currWord_parsingPosFifo_merged);
    mux2to1(currSessionStateFifo, currSessionStateFifo_inner, currSessionStateFifo_merged);
    mux2to1(currMsgBodyFifo, currMsgBodyFifo_inner, currMsgBodyFifo_merged);

    msg_strip<1, 0xa>(currWordFifo_merged, currWordValidLenFifo_merged, currWord_parsingPosFifo_merged, currSessionStateFifo_merged, currMsgBodyFifo_merged, 
                      currWordFifo_inner, currWordValidLenFifo_inner, currWord_parsingPosFifo_inner, currSessionStateFifo_inner, currMsgBodyFifo_inner, 
                      sessionIDOutFifo, msgHeaderOutFifo, msgBodyOutFifo, currSessionStateOutFifo, currMsgBodyStateOutFifo);
    
}
