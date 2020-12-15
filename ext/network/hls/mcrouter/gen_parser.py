parseMsgheader = '''
// not enough to cover a header 
if({currWordValidLen} < MEMCACHED_HDRLEN){{
    {currSessionState}.msgHeaderBuff(MEMCACHED_HDRLEN*8-1, MEMCACHED_HDRLEN*8-{currWordValidLen}*8) = 
        currWord.data({currWordParsingPos}-1, {currWordParsingPos}-{currWordValidLen}*8);
    
    {currSessionState}.parsingHeaderState = 0;
    {currSessionState}.currHdrLen = {currWordValidLen};
    {currSessionState}.currBodyLen = 0;

    {currWordValidLen1} = {currWordValidLen};
    {currWordParsingPos1} = {currWordParsingPos};

    msgParsingState = {state1};
}}
// exactly covering a header
if({currWordValidLen} == MEMCACHED_HDRLEN && {currMsgHeader}.bodyLen > 0){{
    {currSessionState}.msgHeaderBuff(MEMCACHED_HDRLEN*8-1, MEMCACHED_HDRLEN*8-{currWordValidLen}*8) = 
        currWord.data({currWordParsingPos}-1, {currWordParsingPos}-{currWordValidLen}*8);
    {currMsgHeader}.consume_word({currSessionState}.msgHeaderBuff);

    {currSessionState}.parsingHeaderState = 1;
    {currSessionState}.requiredLen = MEMCACHED_HDRLEN + {currMsgHeader}.bodyLen;
    {currSessionState}.currHdrLen = MEMCACHED_HDRLEN;
    {currSessionState}.currBodyLen = 0;
    
    msgParsingState = {state2};
    
    {currWordValidLen1} = {currWordValidLen};
    {currWordParsingPos1} = {currWordParsingPos};    
}}
// exactly covering a header with body or more than a header. 
if(({currWordValidLen} == MEMCACHED_HDRLEN && {currMsgHeader}.bodyLen == 0) || {currWordValidLen} > MEMCACHED_HDRLEN){{
    {currSessionState}.msgHeaderBuff(MEMCACHED_HDRLEN*8-1, 0) = 
        currWord.data({currWordParsingPos}-1, {currWordParsingPos}-MEMCACHED_HDRLEN*8);
    {currMsgHeader}.consume_word({currSessionState}.msgHeaderBuff);

    if({currWordValidLen} > MEMCACHED_HDRLEN){{
        {currSessionState}.parsingHeaderState = 1;
        {currSessionState}.requiredLen = MEMCACHED_HDRLEN + {currMsgHeader}.bodyLen;
        {currSessionState}.currHdrLen = MEMCACHED_HDRLEN;
        {currSessionState}.currBodyLen = 0;
        
        {currWordValidLen1} = {currWordValidLen} - MEMCACHED_HDRLEN;
        {currWordParsingPos1} = {currWordParsingPos} - MEMCACHED_HDRLEN*8;
    }}
    else{{
        {currWordValidLen1} = {currWordValidLen};
        {currWordParsingPos1} = {currWordParsingPos};
    }}

    msgParsingState = {state3};
}}
'''


parseMsgBody = '''
// require more word to parse body
if({currMsgHeader}.bodyLen > 0 && {currWordValidLen} < {currMsgHeader}.bodyLen){{
    {currMsgBody}.body(MAX_BODY_LEN-1, MAX_BODY_LEN-{currWordValidLen}*8) = 
        currWord.data({currWordParsingPos}-1, {currWordParsingPos}-{currWordValidLen}*8);
    
    {currSessionState}.parsingHeaderState = 1;
    {currSessionState}.requiredLen = MEMCACHED_HDRLEN + {currMsgHeader}.bodyLen;
    {currSessionState}.currHdrLen = MEMCACHED_HDRLEN;
    {currSessionState}.currBodyLen = {currWordValidLen};

    {currWordValidLen1} = {currWordValidLen};
    {currWordParsingPos1} = {currWordParsingPos};

    msgParsingState = {state1};
}}
// exactly parsing body done
if({currMsgHeader}.bodyLen > 0 && {currWordValidLen} == {currMsgHeader}.bodyLen){{
    {currMsgBody}.body(MAX_BODY_LEN-1, MAX_BODY_LEN-{currWordValidLen}*8) = 
        currWord.data({currWordParsingPos}-1, {currWordParsingPos}-{currWordValidLen}*8);
    
    {currMsgBody}.extInl = 1;
    {currMsgBody}.keyInl = 1;
    {currMsgBody}.valInl = 1;
    
    {sessionIdFifo}.write(currSessionID);
    {msgHeaderFifo}.write({currMsgHeader});
    {msgBodyFifo}.write({currMsgBody});

    {currSessionState}.reset();

    {currWordValidLen1} = {currWordValidLen};
    {currWordParsingPos1} = {currWordParsingPos};

    msgParsingState = {state2};
}}
// more than a body or no body
if(({currMsgHeader}.bodyLen > 0 && {currWordValidLen} > {currMsgHeader}.bodyLen) || {currMsgHeader}.bodyLen == 0){{
    if({currMsgHeader}.bodyLen > 0){{
        {currMsgBody}.body(MAX_BODY_LEN-1, MAX_BODY_LEN-{currMsgHeader}.bodyLen*8) = 
            currWord.data({currWordParsingPos}-1, {currWordParsingPos}-{currMsgHeader}.bodyLen*8);
        {currWordValidLen1} = {currWordValidLen} - {currMsgHeader}.bodyLen;
        {currWordParsingPos1} = {currWordParsingPos} - {currMsgHeader}.bodyLen*8;
    }}
    else{{
        {currWordValidLen1} = {currWordValidLen};
        {currWordParsingPos1} = {currWordParsingPos};
    }}
    {currMsgBody}.extInl = 1;
    {currMsgBody}.keyInl = 1;
    {currMsgBody}.valInl = 1;

    {sessionIdFifo}.write(currSessionID);
    {msgHeaderFifo}.write({currMsgHeader});
    {msgBodyFifo}.write({currMsgBody});

    {currSessionState}.reset();
    
    msgParsingState = {state3};
}}
'''

print(parseMsgBody.format(currSessionState='currSessionState', currMsgBody='currMsgBody', currMsgHeader='currMsgHeader', 
    currWordValidLen='currWordValidLen1', currWordParsingPos='currWordParsingPos1', currWordValidLen1='currWordValidLen2', currWordParsingPos1='currWordParsingPos2', 
    sessionIdFifo='sessionIdFifo', msgHeaderFifo='msgHeaderFifo', msgBodyFifo='msgBodyFifo', state1='3', state2='4', state3='5'))

print(parseMsgheader.format(currSessionState='currSessionState1', currMsgBody='currMsgBody1', currMsgHeader='currMsgHeader1', 
    currWordValidLen='currWordValidLen2', currWordParsingPos='currWordParsingPos2', currWordValidLen1='currWordValidLen3', currWordParsingPos1='currWordParsingPos3', 
    sessionIdFifo='sessionIdFifo1', msgHeaderFifo='msgHeaderFifo1', msgBodyFifo='msgBodyFifo1', state1='5', state2='6', state3='7'))

print(parseMsgBody.format(currSessionState='currSessionState1', currMsgBody='currMsgBody1', currMsgHeader='currMsgHeader1', 
    currWordValidLen='currWordValidLen3', currWordParsingPos='currWordParsingPos3', currWordValidLen1='currWordValidLen4', currWordParsingPos1='currWordParsingPos4', 
    sessionIdFifo='sessionIdFifo1', msgHeaderFifo='msgHeaderFifo1', msgBodyFifo='msgBodyFifo1', state1='8', state2='9', state3='10'))

print(parseMsgheader.format(currSessionState='currSessionState2', currMsgBody='currMsgBody2', currMsgHeader='currMsgHeader2', 
    currWordValidLen='currWordValidLen4', currWordParsingPos='currWordParsingPos4', currWordValidLen1='currWordValidLen5', currWordParsingPos1='currWordParsingPos5', 
    sessionIdFifo='sessionIdFifo2', msgHeaderFifo='msgHeaderFifo2', msgBodyFifo='msgBodyFifo2', state1='10', state2='11', state3='12'))

print(parseMsgBody.format(currSessionState='currSessionState2', currMsgBody='currMsgBody2', currMsgHeader='currMsgHeader2', 
    currWordValidLen='currWordValidLen5', currWordParsingPos='currWordParsingPos5', currWordValidLen1='currWordValidLen6', currWordParsingPos1='currWordParsingPos6', 
    sessionIdFifo='sessionIdFifo2', msgHeaderFifo='msgHeaderFifo2', msgBodyFifo='msgBodyFifo2', state1='13', state2='14', state3='15'))

print(parseMsgheader.format(currSessionState='currSessionState3', currMsgBody='currMsgBody3', currMsgHeader='currMsgHeader3', 
    currWordValidLen='currWordValidLen6', currWordParsingPos='currWordParsingPos6', currWordValidLen1='currWordValidLen7', currWordParsingPos1='currWordParsingPos7', 
    sessionIdFifo='sessionIdFifo2', msgHeaderFifo='msgHeaderFifo2', msgBodyFifo='msgBodyFifo2', state1='15', state2='16', state3='17'))
