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
void bodyID(
    // the initial states before parsing this word
    hls::stream<net_axis<DATA_WIDTH> >& currWordFifo_in,
    hls::stream<ap_uint<32> >& currWordValidLenFifo_in,
    hls::stream<sessionState>& currSessionStateFifo_in,
    // the updated states to the next msgStripper iteration (via mux2to1)
    hls::stream<net_axis<DATA_WIDTH> >& currWordFifo_inner,
    hls::stream<ap_uint<32> >& currWordValidLenFifo_inner,
    hls::stream<sessionState>& currSessionStateFifo_inner,
    // the states to bodyExtractor
    hls::stream<net_axis<DATA_WIDTH> >& currWordFifo_bodyExtract,
    hls::stream<ap_uint<32> >& currWord_parsingPosFifo_bodyExtract,
    hls::stream<ap_uint<32> >& currWordValidLenFifo_bodyExtract,
    hls::stream<ap_uint<32> >& currBodyLenFifo_bodyExtract,
    hls::stream<ap_uint<32> >& requiredBodyLenFifo_bodyExtract,
    // the states to bodyMerger
    hls::stream<msgHeader>& msgHeaderFifo_bodyMerger,
    hls::stream<ap_uint<4> >& lastMsgIndicator_bodyMerger,
    // the output of the parser states
    hls::stream<sessionState>& currSessionStateFifo_out
){
#pragma HLS PIPELINE II=1
#pragma HLS INLINE off

    if(!currWordFifo_in.empty() && !currWordValidLenFifo_in.empty() && !currSessionStateFifo_in.empty()){
        cout << "==============================bodyID IDX =" << IDX << "===================================" << endl;
        ap_uint<32> currWordValidLen = currWordValidLenFifo_in.read();
        net_axis<DATA_WIDTH> currWord = currWordFifo_in.read();
        sessionState currSessionState = currSessionStateFifo_in.read();

        std::cout << "currSessionState: " << std::endl;
        currSessionState.display();

        msgHeader currMsgHeader;

        ap_uint<32> currWordValidLen_init = keepToLen(currWord.keep);
        // in the begining of a word: currWordValidLen_init equals to currWordValidLen;
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
                // output to bodyExtractor, but bodyExtractor will just forward a zero body to bodyMerger. 
                currWordFifo_bodyExtract.write(currWord);
                currWord_parsingPosFifo_bodyExtract.write(currWord_parsingPos);
                currWordValidLenFifo_bodyExtract.write(currWordValidLen);
                currBodyLenFifo_bodyExtract.write(0);
                requiredBodyLenFifo_bodyExtract.write(0);

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
                    ap_uint<32> currWord_parsingPos = DATA_WIDTH - (currWordValidLen_init - currWordValidLen)*8;
                    ap_uint<32> currBodyLen = 0;
                    ap_uint<32> requiredBodyLen = currMsgHeader.bodyLen;
    
                    std::cout << "currSessionState.requiredLen=" << currSessionState.requiredLen 
                        << " MEMCACHED_HDRLEN + currSessionState.currBodyLen=" << MEMCACHED_HDRLEN + currSessionState.currBodyLen << std::endl;
                    std::cout << "currWordValidLen=" << currWordValidLen << std::endl;
                    std::cout << "currBodyLen=" << currBodyLen << " requiredBodyLen=" << requiredBodyLen << std::endl;

                    // output to bodyExtractor
                    currWordFifo_bodyExtract.write(currWord);
                    currWord_parsingPosFifo_bodyExtract.write(currWord_parsingPos);
                    currWordValidLenFifo_bodyExtract.write(currWordValidLen);
                    currBodyLenFifo_bodyExtract.write(currBodyLen);
                    requiredBodyLenFifo_bodyExtract.write(requiredBodyLen);

                    if(currWordValidLen >= requiredBodyLen){
                        // currMsgBody.body(MAX_BODY_LEN-1, MAX_BODY_LEN-requiredBodyLen*8) = currWord.data(currWord_parsingPos-1, currWord_parsingPos-requiredBodyLen*8);
                        currSessionState.parsingBodyState = 1; // body is parsed
                        currWordValidLen -= requiredBodyLen;
                    }
                    else{
                        // currMsgBody.body(MAX_BODY_LEN-1, MAX_BODY_LEN-currWordValidLen*8) = currWord.data(currWord_parsingPos-1, currWord_parsingPos-currWordValidLen*8);
                        currSessionState.currBodyLen += currWordValidLen;
                        currWordValidLen = 0;
                    }
                }
                else{
                    currSessionState.parsingBodyState = 1;

                    // output to bodyExtractor, but bodyExtractor will just forward a zero body to bodyMerger. 
                    currWordFifo_bodyExtract.write(currWord);
                    currWord_parsingPosFifo_bodyExtract.write(currWord_parsingPos);
                    currWordValidLenFifo_bodyExtract.write(currWordValidLen);
                    currBodyLenFifo_bodyExtract.write(0);
                    requiredBodyLenFifo_bodyExtract.write(0);
                }
            }
        }
        // parse partial body; 
        else{
            currMsgHeader.consume_word(currSessionState.msgHeaderBuff);
            ap_uint<16> currWord_parsingPos = DATA_WIDTH - (currWordValidLen_init - currWordValidLen)*8;
            ap_uint<32> currBodyLen = currSessionState.currBodyLen;
            ap_uint<32> requiredBodyLen = currSessionState.requiredLen - MEMCACHED_HDRLEN - currBodyLen;

            std::cout << "case 3: parser receives data" << std::endl;
            currMsgHeader.display();
            std::cout << "currSessionState.requiredLen=" << currSessionState.requiredLen 
                << " MEMCACHED_HDRLEN + currSessionState.currBodyLen=" << MEMCACHED_HDRLEN + currSessionState.currBodyLen << std::endl;
            std::cout << "currWordValidLen=" << currWordValidLen << std::endl;
            std::cout << "currBodyLen=" << currBodyLen << " requiredBodyLen=" << requiredBodyLen << std::endl;
            
            // output to bodyExtractor
            currWordFifo_bodyExtract.write(currWord);
            currWord_parsingPosFifo_bodyExtract.write(currWord_parsingPos);
            currWordValidLenFifo_bodyExtract.write(currWordValidLen);
            currBodyLenFifo_bodyExtract.write(currBodyLen);
            requiredBodyLenFifo_bodyExtract.write(requiredBodyLen);

            if(currWordValidLen >= requiredBodyLen){
                // currMsgBody.body(MAX_BODY_LEN-1-currBodyLen*8, MAX_BODY_LEN-currBodyLen*8-requiredBodyLen*8) = currWord.data(currWord_parsingPos-1, currWord_parsingPos-requiredBodyLen*8);
                currSessionState.parsingBodyState = 1; // body is parsed
                currWordValidLen -= requiredBodyLen;
            }
            else{
                // currMsgBody.body(MAX_BODY_LEN-1-currBodyLen*8, MAX_BODY_LEN-currBodyLen*8-currWordValidLen*8) = currWord.data(currWord_parsingPos-1, currWord_parsingPos-currWordValidLen*8);
                currSessionState.currBodyLen += currWordValidLen;
                currWordValidLen = 0;
            }
        }

        ap_uint<1> parsingMsgState = (currSessionState.parsingHeaderState == 1 && currSessionState.parsingBodyState == 1);
   
        // !!! write out when current msgHeader is parsed, or msgHeader not parsed but currWordValidLen==0
        if(currSessionState.parsingHeaderState || currWordValidLen == 0){
            std::cout << "writing currMsgHeader (path1): currWordValidLen=" << currWordValidLen << std::endl;
            msgHeaderFifo_bodyMerger.write(currMsgHeader);

            if(!currSessionState.parsingHeaderState){
                // currWordValidLen == 0, partial header is in the end of the word, bodyMerger should immediately stop; 
                lastMsgIndicator_bodyMerger.write(2); // header is not parsed
                std::cout << "bodyID lastMsgIndicator_bodyMerger.write(2)" << std::endl;
            }
            else{
                // indicating this is the last msgbody or header for this word
                lastMsgIndicator_bodyMerger.write(currWordValidLen == 0); 
                std::cout << "bodyID lastMsgIndicator_bodyMerger.write(" << (currWordValidLen == 0) << ")" << std::endl;
            }

            currMsgHeader.display();
            // currMsgBody.display(currMsgHeader.extLen, currMsgHeader.keyLen, currMsgHeader.val_len());
        }

        std::cout << "End of one bodyID iteration: currWordValidLen=" << currWordValidLen << std::endl;
        // this word contains more than one msg, let next bodyID interation process it. 
        if(currWordValidLen > 0){
            currWordFifo_inner.write(currWord);
            currWordValidLenFifo_inner.write(currWordValidLen);
            // write out brand new sessionState to next bodyID interation. 
            currSessionStateFifo_inner.write(sessionState());
        }
        else if(currWordValidLen == 0){
            if(parsingMsgState){
                currSessionStateFifo_out.write(sessionState());
            }
            else{
                currSessionStateFifo_out.write(currSessionState);
            }
        }
    }
}

template <int IDX, int W>
void bodyExtracter(
    // initial state
    hls::stream<net_axis<DATA_WIDTH> >& currWordFifo_bodyExtract,
    hls::stream<ap_uint<32> >& currWord_parsingPosFifo_bodyExtract,
    hls::stream<ap_uint<32> >& currWordValidLenFifo_bodyExtract,
    hls::stream<ap_uint<32> >& currBodyLenFifo_bodyExtract,
    hls::stream<ap_uint<32> >& requiredBodyLenFifo_bodyExtract,
    // the output of the partial body
    hls::stream<msgBody>& msgBodyFifo_bodyMerger,
    hls::stream<ap_uint<32> >& startPosFifo_bodyMerger,
    hls::stream<ap_uint<32> >& lengthFifo_bodyMerger
){
#pragma HLS PIPELINE II=1
#pragma HLS INLINE off

    if(!currWordFifo_bodyExtract.empty() && !currWord_parsingPosFifo_bodyExtract.empty() && !currWordValidLenFifo_bodyExtract.empty() \
                && !currBodyLenFifo_bodyExtract.empty() && !requiredBodyLenFifo_bodyExtract.empty()){
        net_axis<DATA_WIDTH> currWord = currWordFifo_bodyExtract.read();
        ap_uint<32> currWord_parsingPos = currWord_parsingPosFifo_bodyExtract.read();
        ap_uint<32> currWordValidLen = currWordValidLenFifo_bodyExtract.read();
        ap_uint<32> currBodyLen = currBodyLenFifo_bodyExtract.read();
        ap_uint<32> requiredBodyLen = requiredBodyLenFifo_bodyExtract.read();

        msgBody currMsgBody = msgBody();
        ap_uint<32> startPos;
        ap_uint<32> length;
        if(requiredBodyLen == 0){
            startPos = MAX_BODY_LEN;
            length = 0;
        }
        else{
            if(currWordValidLen >= requiredBodyLen){
                currMsgBody.body(MAX_BODY_LEN-currBodyLen*8-1, MAX_BODY_LEN-currBodyLen*8-requiredBodyLen*8) = currWord.data(currWord_parsingPos-1, currWord_parsingPos-requiredBodyLen*8);
                startPos = MAX_BODY_LEN-currBodyLen*8;
                length = requiredBodyLen*8;
            }
            else{
                currMsgBody.body(MAX_BODY_LEN-currBodyLen*8-1, MAX_BODY_LEN-currBodyLen*8-currWordValidLen*8) = currWord.data(currWord_parsingPos-1, currWord_parsingPos-currWordValidLen*8);
                startPos = MAX_BODY_LEN-currBodyLen*8;
                length = currWordValidLen*8;
            }
        }
        startPosFifo_bodyMerger.write(startPos);
        lengthFifo_bodyMerger.write(length);
        msgBodyFifo_bodyMerger.write(currMsgBody);
    }
}

template <int IDX, int W>
void bodyMerger(
    // external input msgbody state and output msgbody state
    hls::stream<msgBody>& currMsgBodyFifo_in,
    hls::stream<msgBody>& currMsgBodyFifo_out, 
    // msgheader from body extractor and output to external
    hls::stream<ap_uint<4> >& lastMsgIndicator_in,
    hls::stream<msgHeader>& msgHeaderFifo_in,
    hls::stream<msgHeader>& msgHeaderFifo_out,
    // partial body and merged final body
    hls::stream<ap_uint<32> >& startPosFifo_in,
    hls::stream<ap_uint<32> >& lengthFifo_in, 
    hls::stream<msgBody>& msgBodyFifo_in,
    hls::stream<msgBody>& msgBodyFifo_out
){
#pragma HLS PIPELINE II=1
#pragma HLS INLINE off

    static ap_uint<4> fsmState = 0;
    static msgBody lastMsgBody;

    std::cout << "bodyMerger state=" << fsmState << std::endl;
    switch(fsmState){
        case 0: {
            if(!currMsgBodyFifo_in.empty()){
                lastMsgBody = currMsgBodyFifo_in.read();
                fsmState = 1;
            }
            break;
        }
        case 1: {
            if(!lastMsgIndicator_in.empty() && !msgHeaderFifo_in.empty() && !startPosFifo_in.empty() && !lengthFifo_in.empty() && !msgBodyFifo_in.empty()){
                ap_uint<4> lastMsgIndicator = lastMsgIndicator_in.read();
                msgHeader msgHeader = msgHeaderFifo_in.read();
                ap_uint<32> startPos = startPosFifo_in.read();
                ap_uint<32> length = lengthFifo_in.read();
                msgBody msgBody = msgBodyFifo_in.read();

                std::cout << "bodyMerger: msgHeader.bodyLen = " << msgHeader.bodyLen << std::endl;
                if(msgHeader.bodyLen == 0){ // message has no body
                    lastMsgBody.reset();
                    msgBodyFifo_out.write(lastMsgBody);
                    msgHeaderFifo_out.write(msgHeader);
                }
                else{
                    if(length == 0){// full header exactly in the end of the word
                        std::cout << "bodyMerger: length=0" << std::endl;
                        lastMsgBody.reset();
                    }
                    else{
                        std::cout << "bodyMerger: startPos=" << startPos << "; length=" << length << std::endl;
                        lastMsgBody.body(startPos-1, startPos-length) = msgBody.body(startPos-1, startPos-length);
                        if(MAX_BODY_LEN-startPos+length == msgHeader.bodyLen*8){
                            msgBodyFifo_out.write(lastMsgBody);
                            msgHeaderFifo_out.write(msgHeader);
                        }   
                    }
                }
                // parsed this word done. 
                if(lastMsgIndicator){
                    currMsgBodyFifo_out.write(lastMsgBody);
                    fsmState = 0;
                }
            }
            else if(!lastMsgIndicator_in.empty() && !msgHeaderFifo_in.empty()){
                ap_uint<4> lastMsgIndicator = lastMsgIndicator_in.read();
                msgHeader msgHeader = msgHeaderFifo_in.read();
                if(lastMsgIndicator == 2){// partial header in the end of the word
                    lastMsgBody.reset();
                    currMsgBodyFifo_out.write(lastMsgBody);
                    fsmState = 0;
                }
            }
            break;
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

template <int IDX, int W>
void msgStripper(
    // the initial states before parsing this word
    hls::stream<net_axis<DATA_WIDTH> >& currWordFifo_in,
    hls::stream<ap_uint<32> >& currWordValidLenFifo_in,
    hls::stream<sessionState>& currSessionStateFifo_in,
    hls::stream<msgBody>& currMsgBodyFifo_in,
    // the output of the parser states
    hls::stream<sessionState>& currSessionStateFifo_out,
    hls::stream<msgBody>& currMsgBodyFifo_out, 
    // the output of the parsed msg
    hls::stream<msgHeader>& msgHeaderFifo_out,
    hls::stream<msgBody>& msgBodyFifo_out
){
#pragma HLS DATAFLOW disable_start_propagation
    
    static hls::stream<net_axis<DATA_WIDTH> > currWordFifo_inner;
    #pragma HLS stream variable=currWordFifo_inner depth=8
    static hls::stream<ap_uint<32> > currWordValidLenFifo_inner;
    #pragma HLS stream variable=currWordValidLenFifo_inner depth=8
    static hls::stream<sessionState> currSessionStateFifo_inner;
    #pragma HLS stream variable=currSessionStateFifo_inner depth=8
    #pragma HLS DATA_PACK variable=currSessionStateFifo_inner
    
    static hls::stream<net_axis<DATA_WIDTH> > currWordFifo_in2;
    #pragma HLS stream variable=currWordFifo_in2 depth=8
    static hls::stream<ap_uint<32> > currWordValidLenFifo_in2;
    #pragma HLS stream variable=currWordValidLenFifo_in2 depth=8
    static hls::stream<sessionState> currSessionStateFifo_in2;
    #pragma HLS stream variable=currSessionStateFifo_in2 depth=8
    #pragma HLS DATA_PACK variable=currSessionStateFifo_in2

    static hls::stream<net_axis<DATA_WIDTH> > currWordFifo_bodyExtract;
    #pragma HLS stream variable=currWordFifo_bodyExtract depth=8
    static hls::stream<ap_uint<32> > currWord_parsingPosFifo_bodyExtract;
    #pragma HLS stream variable=currWord_parsingPosFifo_bodyExtract depth=8
    static hls::stream<ap_uint<32> > currWordValidLenFifo_bodyExtract;
    #pragma HLS stream variable=currWordValidLenFifo_bodyExtract depth=8
    static hls::stream<ap_uint<32> > currBodyLenFifo_bodyExtract;
    #pragma HLS stream variable=currBodyLenFifo_bodyExtract depth=8
    static hls::stream<ap_uint<32> > requiredBodyLenFifo_bodyExtract;
    #pragma HLS stream variable=requiredBodyLenFifo_bodyExtract depth=8
    
    static hls::stream<ap_uint<4> > lastMsgIndicator_bodyMerger;
    #pragma HLS stream variable=lastMsgIndicator_bodyMerger depth=8
    static hls::stream<msgHeader> msgHeaderFifo_bodyMerger;
    #pragma HLS stream variable=msgHeaderFifo_bodyMerger depth=8
    #pragma HLS DATA_PACK variable=msgHeaderFifo_bodyMerger
    static hls::stream<msgBody> msgBodyFifo_bodyMerger;
    #pragma HLS stream variable=msgBodyFifo_bodyMerger depth=8
    #pragma HLS DATA_PACK variable=msgBodyFifo_bodyMerger
    static hls::stream<ap_uint<32> > startPosFifo_bodyMerger;
    #pragma HLS stream variable=startPosFifo_bodyMerger depth=8
    static hls::stream<ap_uint<32> > lengthFifo_bodyMerger;
    #pragma HLS stream variable=lengthFifo_bodyMerger depth=8
    
    mux2to1(currWordFifo_in, currWordFifo_inner, currWordFifo_in2);
    mux2to1(currWordValidLenFifo_in, currWordValidLenFifo_inner, currWordValidLenFifo_in2);
    mux2to1(currSessionStateFifo_in, currSessionStateFifo_inner, currSessionStateFifo_in2);

    bodyID<IDX, W>(currWordFifo_in2, currWordValidLenFifo_in2, currSessionStateFifo_in2,
        currWordFifo_inner, currWordValidLenFifo_inner, currSessionStateFifo_inner, 
        currWordFifo_bodyExtract, currWord_parsingPosFifo_bodyExtract, currWordValidLenFifo_bodyExtract, currBodyLenFifo_bodyExtract, requiredBodyLenFifo_bodyExtract,
        msgHeaderFifo_bodyMerger, lastMsgIndicator_bodyMerger, currSessionStateFifo_out);
    
    bodyExtracter<IDX, W>(currWordFifo_bodyExtract, currWord_parsingPosFifo_bodyExtract, currWordValidLenFifo_bodyExtract, currBodyLenFifo_bodyExtract, requiredBodyLenFifo_bodyExtract, 
        msgBodyFifo_bodyMerger, startPosFifo_bodyMerger, lengthFifo_bodyMerger);

    bodyMerger<IDX, W>(currMsgBodyFifo_in, currMsgBodyFifo_out, lastMsgIndicator_bodyMerger, msgHeaderFifo_bodyMerger, msgHeaderFifo_out, 
        startPosFifo_bodyMerger, lengthFifo_bodyMerger, msgBodyFifo_bodyMerger, msgBodyFifo_out);

}

void wordLen_fwd(
    hls::stream<net_axis<DATA_WIDTH> >& currWordFifo,
    hls::stream<net_axis<DATA_WIDTH> >& currWordFifo_msgStripper,
    hls::stream<ap_uint<32> >&          currWordValidLenFifo_msgStripper
){
#pragma HLS PIPELINE II=1
#pragma HLS INLINE off

    if(!currWordFifo.empty()){
        net_axis<DATA_WIDTH> currWord = currWordFifo.read();
        currWordFifo_msgStripper.write(currWord);
        ap_uint<32> wordLen = keepToLen(currWord.keep);
        currWordValidLenFifo_msgStripper.write(wordLen);
    }
}

// one word at a time in maximum
void parser(
    // the word waiting for parsing
    hls::stream<net_axis<DATA_WIDTH> >& currWordFifo,
    // the external states before parsing this word
    hls::stream<sessionState>& currSessionStateFifo,
    hls::stream<msgBody>& currMsgBodyFifo,
    // the updated states after parsing this word
    hls::stream<sessionState>& currSessionStateFifo_out,
    hls::stream<msgBody>& currMsgBodyFifo_out,
    // the output of the parsed msg
    hls::stream<msgHeader>& msgHeaderFifo_out,
    hls::stream<msgBody>& msgBodyFifo_out
){
#pragma HLS DATAFLOW disable_start_propagation
#pragma HLS INTERFACE ap_ctrl_none port=return

    hls::stream<net_axis<DATA_WIDTH> > currWordFifo_msgStripper;
    #pragma HLS stream variable=currWordFifo_msgStripper depth=8

    hls::stream<ap_uint<32> >          currWordValidLenFifo_msgStripper;
    #pragma HLS stream variable=currWordValidLenFifo_msgStripper depth=8

    wordLen_fwd(currWordFifo, currWordFifo_msgStripper, currWordValidLenFifo_msgStripper);

    msgStripper<1, 0xa>(currWordFifo_msgStripper, currWordValidLenFifo_msgStripper, currSessionStateFifo, currMsgBodyFifo, 
                        currSessionStateFifo_out, currMsgBodyFifo_out, msgHeaderFifo_out, msgBodyFifo_out);
}
