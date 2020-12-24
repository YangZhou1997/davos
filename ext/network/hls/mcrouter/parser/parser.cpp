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
    hls::stream<ap_uint<32> >& currWordValidLen_initFifo_in,
    hls::stream<sessionState>& currSessionStateFifo_in,
    // the updated states to the next msgStripper iteration (via mux2to1)
    hls::stream<net_axis<DATA_WIDTH> >& currWordFifo_inner,
    hls::stream<ap_uint<32> >& currWordValidLenFifo_inner,
    hls::stream<ap_uint<32> >& currWordValidLen_initFifo_inner,
    hls::stream<sessionState>& currSessionStateFifo_inner,
    // the states to bodyExtractor
    hls::stream<bodyExtState>& bodyExtractorStateFifo_out,
    // the output of the parser states
    hls::stream<sessionState>& currSessionStateFifo_out
){
#pragma HLS PIPELINE II=1
#pragma HLS INLINE off

    if(!currWordFifo_in.empty() && !currWordValidLenFifo_in.empty() && !currSessionStateFifo_in.empty() && !currWordValidLen_initFifo_in.empty()){
        cout << "==============================bodyID IDX =" << IDX << "===================================" << endl;
        ap_uint<32> currWordValidLen = currWordValidLenFifo_in.read();
        ap_uint<32> currWordValidLen_init = currWordValidLen_initFifo_in.read();
        net_axis<DATA_WIDTH> currWord = currWordFifo_in.read();
        sessionState currSessionState = currSessionStateFifo_in.read();
        ap_uint<16> currSessionID = currSessionState.currSessionID;

        std::cout << "currSessionID" << currSessionID << " currSessionState: " << std::endl;
        currSessionState.display();

        msgHeader currMsgHeader;

        // in the begining of a word: currWordValidLen_init equals to currWordValidLen;
        ap_uint<32> currWord_parsingPos = DATA_WIDTH - (currWordValidLen_init - currWordValidLen)*8;

        // parse partial header + body; 
        if(currSessionState.parsingHeaderState == 0){
            ap_uint<5> requiredHeaderLen = MEMCACHED_HDRLEN - currSessionState.currHdrLen;
            if(currWordValidLen < requiredHeaderLen){
                currSessionState.msgHeaderBuff(requiredHeaderLen*8-1, requiredHeaderLen*8-currWordValidLen*8) = currWord.data(currWord_parsingPos-1, currWord_parsingPos-currWordValidLen*8);
                currSessionState.currHdrLen += currWordValidLen;
                currWordValidLen = 0;

                std::cout << "case 0: parser receives partial hdr" << std::endl;

                std::cout << "bodyID lastMsgIndicator_bodyMerger.write(2)" << std::endl;
                // lastMsgIndicator=2 indicating this is a partial msg header. 
                bodyExtractorStateFifo_out.write(bodyExtState(currSessionID, currWord.data, currWord_parsingPos, currWordValidLen, 0, 0, currMsgHeader, 2));
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
                std::cout << "bodyID lastMsgIndicator_bodyMerger.write(" << 1 << ")" << std::endl;
                // output to bodyExtractor, but bodyExtractor will just forward a zero body to bodyMerger, as we specify requiredBodyLen=0
                bodyExtractorStateFifo_out.write(bodyExtState(currSessionID, currWord.data, currWord_parsingPos, currWordValidLen, 0, 0, currMsgHeader, 1));
                
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

                    std::cout << "bodyID lastMsgIndicator_bodyMerger.write(" << (currWordValidLen <= requiredBodyLen) << ")" << std::endl;
                    // output to bodyExtractor, currWordValidLen <= requiredBodyLen means currWordValidLen==0;
                    bodyExtractorStateFifo_out.write(bodyExtState(currSessionID, currWord.data, currWord_parsingPos, currWordValidLen, currBodyLen, requiredBodyLen, currMsgHeader, currWordValidLen <= requiredBodyLen));

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

                    std::cout << "bodyID lastMsgIndicator_bodyMerger.write(" << (currWordValidLen == 0) << ")" << std::endl;
                    // output to bodyExtractor, but bodyExtractor will just forward a zero body to bodyMerger. 
                    bodyExtractorStateFifo_out.write(bodyExtState(currSessionID, currWord.data, currWord_parsingPos, currWordValidLen, 0, 0, currMsgHeader, currWordValidLen == 0));
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
            
            std::cout << "bodyID lastMsgIndicator_bodyMerger.write(" << (currWordValidLen <= requiredBodyLen) << ")" << std::endl;
            // output to bodyExtractor, currWordValidLen <= requiredBodyLen means currWordValidLen==0;
            bodyExtractorStateFifo_out.write(bodyExtState(currSessionID, currWord.data, currWord_parsingPos, currWordValidLen, currBodyLen, requiredBodyLen, currMsgHeader, currWordValidLen <= requiredBodyLen));

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
   
        std::cout << "End of one bodyID iteration: currWordValidLen=" << currWordValidLen << std::endl;
        // this word contains more than one msg, let next bodyID interation process it. 
        if(currWordValidLen > 0){
            currWordFifo_inner.write(currWord);
            currWordValidLenFifo_inner.write(currWordValidLen);
            currWordValidLen_initFifo_inner.write(currWordValidLen_init);
            // write out brand new sessionState to next bodyID iteration. 
            currSessionStateFifo_inner.write(sessionState(currSessionID));
        }
        else if(currWordValidLen == 0){
            if(parsingMsgState){
                currSessionStateFifo_out.write(sessionState(currSessionID));
            }
            else{
                currSessionStateFifo_out.write(currSessionState);
            }
        }
    }
}

template <int IDX, int W>
void bodyExtractor(
    // initial state
    hls::stream<bodyExtState>&   bodyExtractorStateFifo_in,
    // output state to body merger
    hls::stream<bodyMergeState>&   bodyMergerStateFifo_out, 
    // the output of the partial body
    hls::stream<msgBody>& msgBodyFifo_bodyMerger
    
){
#pragma HLS PIPELINE II=1
#pragma HLS INLINE off

    if(!bodyExtractorStateFifo_in.empty()){
        bodyExtState state = bodyExtractorStateFifo_in.read();
        
        ap_uint<16> currSessionID = state.currSessionID;
        ap_uint<DATA_WIDTH> currWordData = state.data; 
        ap_uint<32> currWord_parsingPos = state.currWord_parsingPos;
        ap_uint<32> currWordValidLen = state.currWordValidLen;
        ap_uint<32> currBodyLen = state.currBodyLen;
        ap_uint<32> requiredBodyLen = state.requiredBodyLen;
        
        msgHeader   currMsgHeader = state.currMsgHeader;
        ap_uint<4>  lastMsgIndicator = state.lastMsgIndicator;
    
        msgBody currMsgBody = msgBody();
        ap_uint<32> startPos;
        ap_uint<32> length;
        if(requiredBodyLen == 0){ // either the msg does not have body, the full msg hdr exactly in the end of the word, or the partial msg hdr in the end
            startPos = MAX_BODY_LEN;
            length = 0;
        }
        else{
            if(currWordValidLen >= requiredBodyLen){
                currMsgBody.body(MAX_BODY_LEN-currBodyLen*8-1, MAX_BODY_LEN-currBodyLen*8-requiredBodyLen*8) = currWordData(currWord_parsingPos-1, currWord_parsingPos-requiredBodyLen*8);
                startPos = MAX_BODY_LEN-currBodyLen*8;
                length = requiredBodyLen*8;
            }
            else{
                currMsgBody.body(MAX_BODY_LEN-currBodyLen*8-1, MAX_BODY_LEN-currBodyLen*8-currWordValidLen*8) = currWordData(currWord_parsingPos-1, currWord_parsingPos-currWordValidLen*8);
                startPos = MAX_BODY_LEN-currBodyLen*8;
                length = currWordValidLen*8;
            }
        }
        bodyMergerStateFifo_out.write(bodyMergeState(currSessionID, currMsgHeader, lastMsgIndicator, startPos, length));
        msgBodyFifo_bodyMerger.write(currMsgBody);
    }
}

template <int IDX, int W>
void bodyMerger(
    hls::stream<ap_uint<16> >& currSessionIDFifo_in,
    // external input msgbody state 
    hls::stream<msgBody>& currMsgBodyFifo_in,
    // states from body extractor
    hls::stream<bodyMergeState>& bodyMergerStateFifo_in, 
    // partial body input from bodyExtractor
    hls::stream<msgBody>& msgBodyFifo_in,
    // external output msgbody state
    hls::stream<msgBody>& currMsgBodyFifo_out, 
    // final msg output
    hls::stream<msgHeader>& msgHeaderFifo_out,
    hls::stream<msgBody>& msgBodyFifo_out
){
#pragma HLS PIPELINE II=1
#pragma HLS INLINE off

    // these global state needs to be session-specific, if we want multiple words from different sessions mixed. 
    static ap_uint<4> fsmState = 0;
    // static msgBody lastMsgBody;
    #pragma HLS ARRAY_PARTITION variable=parser_stashTable complete
    #pragma HLS DEPENDENCE variable=parser_stashTable inter false

    std::cout << "bodyMerger state=" << fsmState << std::endl;
    switch(fsmState){
        case 0: {
            // read currSessionID from currSessionIDFifo_in;
            if(!currMsgBodyFifo_in.empty() && !currSessionIDFifo_in.empty()){
                msgBody lastMsgBody = currMsgBodyFifo_in.read();
                ap_uint<16> currSessionID = currSessionIDFifo_in.read();
                bool ret = parser_stash_insert(currSessionID, lastMsgBody);
                if(!ret){
                    std::cout << "[ERROR] parser_stash_insert: currSessionID=" << currSessionID << std::endl;
                }
                fsmState = 1;
            }
            break;
        }
        case 1: {
            if(!bodyMergerStateFifo_in.empty() && !msgBodyFifo_in.empty()){
                bodyMergeState state = bodyMergerStateFifo_in.read();
                ap_uint<16> currSessionID = state.currSessionID; // use currSessionID to select lastMsgBody. 
                msgHeader msgHeader = state.currMsgHeader;
                ap_uint<4> lastMsgIndicator = state.lastMsgIndicator;
                ap_uint<32> startPos = state.startPos;
                ap_uint<32> length = state.length;
                msgBody partialMsgBody = msgBodyFifo_in.read();

                msgBody lastMsgBody;
                int slot = parser_stash_lookup(currSessionID);
                if(slot != -1){
                    lastMsgBody = parser_stashTable[slot].msgbody;
                }
                else{
                    std::cout << "[ERROR] parser_stash_lookup: currSessionID=" << currSessionID << std::endl;
                }

                std::cout << "bodyMerger: msgHeader.bodyLen = " << msgHeader.bodyLen << std::endl;
                if(lastMsgIndicator == 2){// partial header in the end of the word
                    lastMsgBody.reset();
                    currMsgBodyFifo_out.write(lastMsgBody);
                    parser_stashTable[slot].valid = false;
                    fsmState = 0;
                }
                else{
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
                            lastMsgBody.body(startPos-1, startPos-length) = partialMsgBody.body(startPos-1, startPos-length);
                            if(MAX_BODY_LEN-startPos+length == msgHeader.bodyLen*8){
                                msgBodyFifo_out.write(lastMsgBody);
                                msgHeaderFifo_out.write(msgHeader);
                            }   
                        }
                    }
                    // parsed this word done. 
                    if(lastMsgIndicator == 1){
                        currMsgBodyFifo_out.write(lastMsgBody);
                        parser_stashTable[slot].valid = false;
                        fsmState = 0;
                    }
                    else{
                        parser_stashTable[slot].msgbody = lastMsgBody;
                    }
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
    hls::stream<ap_uint<32> >& currWordValidLen_initFifo_in,
    hls::stream<sessionState>& currSessionStateFifo_in,
    hls::stream<ap_uint<16> >& currSessionIDFifo_in,
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
    static hls::stream<ap_uint<32> > currWordValidLen_initFifo_inner;
    #pragma HLS stream variable=currWordValidLen_initFifo_inner depth=8
    static hls::stream<sessionState> currSessionStateFifo_inner;
    #pragma HLS stream variable=currSessionStateFifo_inner depth=8
    #pragma HLS DATA_PACK variable=currSessionStateFifo_inner
    
    static hls::stream<net_axis<DATA_WIDTH> > currWordFifo_in2;
    #pragma HLS stream variable=currWordFifo_in2 depth=8
    static hls::stream<ap_uint<32> > currWordValidLenFifo_in2;
    #pragma HLS stream variable=currWordValidLenFifo_in2 depth=8
    static hls::stream<ap_uint<32> > currWordValidLen_initFifo_in2;
    #pragma HLS stream variable=currWordValidLen_initFifo_in2 depth=8
    static hls::stream<sessionState> currSessionStateFifo_in2;
    #pragma HLS stream variable=currSessionStateFifo_in2 depth=8
    #pragma HLS DATA_PACK variable=currSessionStateFifo_in2

    static hls::stream<bodyExtState> bodyExtractorStateFifo;
    #pragma HLS stream variable=bodyExtractorStateFifo depth=8
    #pragma HLS DATA_PACK variable=bodyExtractorStateFifo

    static hls::stream<bodyMergeState> bodyMergerStateFifo;
    #pragma HLS stream variable=bodyMergerStateFifo depth=8
    #pragma HLS DATA_PACK variable=bodyMergerStateFifo
    static hls::stream<msgBody> msgBodyFifo_bodyMerger;
    #pragma HLS stream variable=msgBodyFifo_bodyMerger depth=8
    #pragma HLS DATA_PACK variable=msgBodyFifo_bodyMerger
    

    mux2to1(currWordFifo_in, currWordFifo_inner, currWordFifo_in2);
    mux2to1(currWordValidLenFifo_in, currWordValidLenFifo_inner, currWordValidLenFifo_in2);
    mux2to1(currWordValidLen_initFifo_in, currWordValidLen_initFifo_inner, currWordValidLen_initFifo_in2);
    mux2to1(currSessionStateFifo_in, currSessionStateFifo_inner, currSessionStateFifo_in2);

    bodyID<IDX, W>(currWordFifo_in2, currWordValidLenFifo_in2, currWordValidLen_initFifo_in2, currSessionStateFifo_in2,
        currWordFifo_inner, currWordValidLenFifo_inner, currWordValidLen_initFifo_inner, currSessionStateFifo_inner, 
        bodyExtractorStateFifo, currSessionStateFifo_out);
    
    bodyExtractor<IDX, W>(bodyExtractorStateFifo, bodyMergerStateFifo, msgBodyFifo_bodyMerger);

    bodyMerger<IDX, W>(currSessionIDFifo_in, currMsgBodyFifo_in, bodyMergerStateFifo, msgBodyFifo_bodyMerger, 
        currMsgBodyFifo_out, msgHeaderFifo_out, msgBodyFifo_out);

}

void wordLen_fwd(
    hls::stream<net_axis<DATA_WIDTH> >& currWordFifo, // in
    hls::stream<sessionState>&          currSessionStateFifo, // in
    hls::stream<net_axis<DATA_WIDTH> >& currWordFifo_msgStripper,
    hls::stream<ap_uint<32> >&          currWordValidLenFifo_msgStripper,
    hls::stream<ap_uint<32> >&          currWordValidLen_initFifo_msgStripper, 
    hls::stream<sessionState>&          currSessionStateFifo_msgStripper,
    hls::stream<ap_uint<16> >&          currSessionIDFifo_msgStripper
    
){
#pragma HLS PIPELINE II=1
#pragma HLS INLINE off

    if(!currWordFifo.empty() && !currSessionStateFifo.empty()){
        net_axis<DATA_WIDTH> currWord = currWordFifo.read();
        sessionState currSessionState = currSessionStateFifo.read();
        currWordFifo_msgStripper.write(currWord);
        ap_uint<32> wordLen = keepToLen(currWord.keep);
        currWordValidLenFifo_msgStripper.write(wordLen);
        currWordValidLen_initFifo_msgStripper.write(wordLen);
        currSessionStateFifo_msgStripper.write(currSessionState);
        currSessionIDFifo_msgStripper.write(currSessionState.currSessionID);
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

    static hls::stream<net_axis<DATA_WIDTH> > currWordFifo_msgStripper;
    #pragma HLS stream variable=currWordFifo_msgStripper depth=8
    #pragma HLS DATA_PACK variable=currWordFifo_msgStripper

    static hls::stream<sessionState>          currSessionStateFifo_msgStripper;
    #pragma HLS stream variable=currSessionStateFifo_msgStripper depth=8
    #pragma HLS DATA_PACK variable=currSessionStateFifo_msgStripper

    static hls::stream<ap_uint<16> >          currSessionIDFifo_msgStripper;
    #pragma HLS stream variable=currSessionIDFifo_msgStripper depth=8

    static hls::stream<ap_uint<32> >          currWordValidLenFifo_msgStripper;
    #pragma HLS stream variable=currWordValidLenFifo_msgStripper depth=8
        
    static hls::stream<ap_uint<32> >          currWordValidLen_initFifo_msgStripper;
    #pragma HLS stream variable=currWordValidLen_initFifo_msgStripper depth=8

    wordLen_fwd(currWordFifo, currSessionStateFifo, currWordFifo_msgStripper, currWordValidLenFifo_msgStripper, currWordValidLen_initFifo_msgStripper, 
        currSessionStateFifo_msgStripper, currSessionIDFifo_msgStripper);

    msgStripper<1, 0xa>(currWordFifo_msgStripper, currWordValidLenFifo_msgStripper, currWordValidLen_initFifo_msgStripper, 
        currSessionStateFifo_msgStripper, currSessionIDFifo_msgStripper, 
        currMsgBodyFifo, currSessionStateFifo_out, currMsgBodyFifo_out, msgHeaderFifo_out, msgBodyFifo_out);
}


bool parser_stash_insert(ap_uint<16> sessionID, msgBody msgbody){
#pragma HLS INLINE
    bool ret = false;

    int slot = -1;
    for (int i = 0; i < PARSER_STASH_SIZE; i++)
    {
        #pragma HLS UNROLL
        if(!parser_stashTable[i].valid)
        {
            slot = i;
        }
    }
    std::cout << "stash_insert slot = " << slot << std::endl;
    if(slot != -1){
        ret = true;
        parser_stashTable[slot].sessionID = sessionID;
        parser_stashTable[slot].msgbody = msgbody;
        parser_stashTable[slot].valid = true;
    }
    return ret;
}

int parser_stash_lookup(ap_uint<16> sessionID){
#pragma HLS INLINE
    // parser_stashRet stashRet;

    int slot = -1;
    for (int i = 0; i < PARSER_STASH_SIZE; i++)
    {
        #pragma HLS UNROLL
        if(parser_stashTable[i].valid && parser_stashTable[i].sessionID == sessionID)
        {
            std::cout << "stash_lookup i = " << i << std::endl;
            slot = i;
        }
    }
    // if(slot != -1){
    //     stashRet.ret = true;
    //     stashRet.msgbody = parser_stashTable[slot].msgbody;
    // }
    return slot;
}

bool parser_stash_remove(ap_uint<16> sessionID){
#pragma HLS INLINE
    bool ret = false;

    int slot = -1;
    for (int i = 0; i < PARSER_STASH_SIZE; i++)
    {
        #pragma HLS UNROLL
        if(parser_stashTable[i].valid && parser_stashTable[i].sessionID == sessionID)
        {
            std::cout << "stash_remove i = " << i << std::endl;
            slot = i;
        }
    }
    if(slot != -1){
        parser_stashTable[slot].valid = false;
        ret = true;
    }
    return ret;
}