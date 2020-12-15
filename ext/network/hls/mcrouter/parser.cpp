// require more word to parse body
if(currMsgHeader.bodyLen > 0 && currWordValidLen1 < currMsgHeader.bodyLen){
    currMsgBody.body(MAX_BODY_LEN-1, MAX_BODY_LEN-currWordValidLen1*8) = 
        currWord.data(currWordParsingPos1-1, currWordParsingPos1-currWordValidLen1*8);
    
    currSessionState.parsingHeaderState = 1;
    currSessionState.requiredLen = MEMCACHED_HDRLEN + currMsgHeader.bodyLen;
    currSessionState.currHdrLen = MEMCACHED_HDRLEN;
    currSessionState.currBodyLen = currWordValidLen1;

    currWordValidLen2 = currWordValidLen1;
    currWordParsingPos2 = currWordParsingPos1;

    msgParsingState = 3;
}
// exactly parsing body done
if(currMsgHeader.bodyLen > 0 && currWordValidLen1 == currMsgHeader.bodyLen){
    currMsgBody.body(MAX_BODY_LEN-1, MAX_BODY_LEN-currWordValidLen1*8) = 
        currWord.data(currWordParsingPos1-1, currWordParsingPos1-currWordValidLen1*8);
    
    currMsgBody.extInl = 1;
    currMsgBody.keyInl = 1;
    currMsgBody.valInl = 1;
    
    sessionIdFifo.write(currSessionID);
    msgHeaderFifo.write(currMsgHeader);
    msgBodyFifo.write(currMsgBody);

    currSessionState.reset();

    currWordValidLen2 = currWordValidLen1;
    currWordParsingPos2 = currWordParsingPos1;

    msgParsingState = 4;
}
// more than a body or no body
if((currMsgHeader.bodyLen > 0 && currWordValidLen1 > currMsgHeader.bodyLen) || currMsgHeader.bodyLen == 0){
    if(currMsgHeader.bodyLen > 0){
        currMsgBody.body(MAX_BODY_LEN-1, MAX_BODY_LEN-currMsgHeader.bodyLen*8) = 
            currWord.data(currWordParsingPos1-1, currWordParsingPos1-currMsgHeader.bodyLen*8);
        currWordValidLen2 = currWordValidLen1 - currMsgHeader.bodyLen;
        currWordParsingPos2 = currWordParsingPos1 - currMsgHeader.bodyLen*8;
    }
    else{
        currWordValidLen2 = currWordValidLen1;
        currWordParsingPos2 = currWordParsingPos1;
    }
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
    if(currWordValidLen2 == MEMCACHED_HDRLEN && currMsgHeader1.bodyLen > 0){
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
    if((currWordValidLen2 == MEMCACHED_HDRLEN && currMsgHeader1.bodyLen == 0) || currWordValidLen2 > MEMCACHED_HDRLEN){
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

        currMsgBody1.msgID = currentMsgID;
        currentMsgID += 1;
        // require more word to parse body
        if(currMsgHeader1.bodyLen > 0 && currWordValidLen3 < currMsgHeader1.bodyLen){
            currMsgBody1.body(MAX_BODY_LEN-1, MAX_BODY_LEN-currWordValidLen3*8) = 
                currWord.data(currWordParsingPos3-1, currWordParsingPos3-currWordValidLen3*8);
            
            currSessionState1.parsingHeaderState = 1;
            currSessionState1.requiredLen = MEMCACHED_HDRLEN + currMsgHeader1.bodyLen;
            currSessionState1.currHdrLen = MEMCACHED_HDRLEN;
            currSessionState1.currBodyLen = currWordValidLen3;

            currWordValidLen4 = currWordValidLen3;
            currWordParsingPos4 = currWordParsingPos3;

            msgParsingState = 8;
        }
        // exactly parsing body done
        if(currMsgHeader1.bodyLen > 0 && currWordValidLen3 == currMsgHeader1.bodyLen){
            currMsgBody1.body(MAX_BODY_LEN-1, MAX_BODY_LEN-currWordValidLen3*8) = 
                currWord.data(currWordParsingPos3-1, currWordParsingPos3-currWordValidLen3*8);
            
            currMsgBody1.extInl = 1;
            currMsgBody1.keyInl = 1;
            currMsgBody1.valInl = 1;
            
            sessionIdFifo1.write(currSessionID);
            msgHeaderFifo1.write(currMsgHeader1);
            msgBodyFifo1.write(currMsgBody1);

            currSessionState1.reset();

            currWordValidLen4 = currWordValidLen3;
            currWordParsingPos4 = currWordParsingPos3;

            msgParsingState = 9;
        }
        // more than a body or no body
        if((currMsgHeader1.bodyLen > 0 && currWordValidLen3 > currMsgHeader1.bodyLen) || currMsgHeader1.bodyLen == 0){
            if(currMsgHeader1.bodyLen > 0){
                currMsgBody1.body(MAX_BODY_LEN-1, MAX_BODY_LEN-currMsgHeader1.bodyLen*8) = 
                    currWord.data(currWordParsingPos3-1, currWordParsingPos3-currMsgHeader1.bodyLen*8);
                currWordValidLen4 = currWordValidLen3 - currMsgHeader1.bodyLen;
                currWordParsingPos4 = currWordParsingPos3 - currMsgHeader1.bodyLen*8;
            }
            else{
                currWordValidLen4 = currWordValidLen3;
                currWordParsingPos4 = currWordParsingPos3;
            }
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
            if(currWordValidLen4 == MEMCACHED_HDRLEN && currMsgHeader2.bodyLen > 0){
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
            if((currWordValidLen4 == MEMCACHED_HDRLEN && currMsgHeader2.bodyLen == 0) || currWordValidLen4 > MEMCACHED_HDRLEN){
                currSessionState2.msgHeaderBuff(MEMCACHED_HDRLEN*8-1, 0) = 
                    currWord.data(currWordParsingPos4-1, currWordParsingPos4-MEMCACHED_HDRLEN*8);
                currMsgHeader2.consume_word(currSessionState2.msgHeaderBuff);

                if(currWordValidLen4 > MEMCACHED_HDRLEN){
                    currSessionState2.parsingHeaderState = 1;
                    currSessionState2.requiredLen = MEMCACHED_HDRLEN + currMsgHeader2.bodyLen;
                    currSessionState2.currHdrLen = MEMCACHED_HDRLEN;
                    currSessionState2.currBodyLen = 0;
                    
                    currWordValidLen5 = currWordValidLen4 - MEMCACHED_HDRLEN;
                    currWordParsingPos5 = currWordParsingPos4 - MEMCACHED_HDRLEN*8;
                }
                else{
                    currWordValidLen5 = currWordValidLen4;
                    currWordParsingPos5 = currWordParsingPos4;
                }

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

                    currWordValidLen6 = currWordValidLen5;
                    currWordParsingPos6 = currWordParsingPos5;

                    msgParsingState = 13;
                }
                // exactly parsing body done
                if(currMsgHeader2.bodyLen > 0 && currWordValidLen5 == currMsgHeader2.bodyLen){
                    currMsgBody2.body(MAX_BODY_LEN-1, MAX_BODY_LEN-currWordValidLen5*8) = 
                        currWord.data(currWordParsingPos5-1, currWordParsingPos5-currWordValidLen5*8);
                    
                    currMsgBody2.extInl = 1;
                    currMsgBody2.keyInl = 1;
                    currMsgBody2.valInl = 1;
                    
                    sessionIdFifo2.write(currSessionID);
                    msgHeaderFifo2.write(currMsgHeader2);
                    msgBodyFifo2.write(currMsgBody2);

                    currSessionState2.reset();

                    currWordValidLen6 = currWordValidLen5;
                    currWordParsingPos6 = currWordParsingPos5;

                    msgParsingState = 14;
                }
                // more than a body or no body
                if((currMsgHeader2.bodyLen > 0 && currWordValidLen5 > currMsgHeader2.bodyLen) || currMsgHeader2.bodyLen == 0){
                    if(currMsgHeader2.bodyLen > 0){
                        currMsgBody2.body(MAX_BODY_LEN-1, MAX_BODY_LEN-currMsgHeader2.bodyLen*8) = 
                            currWord.data(currWordParsingPos5-1, currWordParsingPos5-currMsgHeader2.bodyLen*8);
                        currWordValidLen6 = currWordValidLen5 - currMsgHeader2.bodyLen;
                        currWordParsingPos6 = currWordParsingPos5 - currMsgHeader2.bodyLen*8;
                    }
                    else{
                        currWordValidLen6 = currWordValidLen5;
                        currWordParsingPos6 = currWordParsingPos5;
                    }
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
        }
    }
}