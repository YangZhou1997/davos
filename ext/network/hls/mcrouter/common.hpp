#pragma once
#include <iostream>
#include <arpa/inet.h>
using namespace std;

#define DATA_WIDTH 512

struct msgHeader {
    ap_uint<8> magic;
    ap_uint<8> opcode;
    ap_uint<16> keyLen;
    ap_uint<8> extLen;
    ap_uint<8> dataType;
    ap_uint<16> status;
    ap_uint<32> bodyLen;
    ap_uint<32> opaque;
    ap_uint<64> cas;
    msgHeader() {
        magic = 0;
        opcode = 0;
        keyLen = 0;
        extLen = 0;
        dataType = 0;
        status = 0;
        bodyLen = 0;
        opaque = 0;
        cas = 0;
    }
    void reset(){
        magic = 0;
        opcode = 0;
        keyLen = 0;
        extLen = 0;
        dataType = 0;
        status = 0;
        bodyLen = 0;
        opaque = 0;
        cas = 0;
    }
    void consume_word(ap_uint<24*8>& w){
        magic = w(191, 184);
        opcode = w(183, 176);
        keyLen = w(175, 160);
        extLen = w(159, 152);
        dataType = w(151, 144);
        status = w(143, 128);
        bodyLen = w(127, 96);
        opaque = w(95, 64);
        cas = w(63, 0);
    }
    ap_uint<24*8> output_word(){
        return (magic, opcode, keyLen, extLen, dataType, status, bodyLen, opaque, cas);
    }
    ap_uint<32> val_len(){
        return bodyLen-extLen-keyLen;
    }
    void display(){
        std::cout << "magic " << std::hex << magic << std::endl;
        std::cout << "opcode " << std::hex << opcode << std::endl;
        std::cout << "keyLen " << std::dec << keyLen << std::endl;
        std::cout << "extlen " << std::dec << extLen << std::endl;
        std::cout << "dataType " << std::dec << dataType << std::endl;
        std::cout << "status " << std::dec << status << std::endl;
        std::cout << "bodyLen " << std::dec << bodyLen << std::endl;
        std::cout << "opaque " << std::hex << opaque << std::endl;
        std::cout << "cas " << std::dec << cas << std::endl;
    }
};

#define MAX_BODY_LEN (1024-35-48*3-16-5) // 824 bits -> 103 bytes
#define MAX_KEY_LEN (40*8)
struct msgBody {
	ap_uint<32>                 msgID;  // globally unique msgID
    ap_uint<1>                  extInl; // indicating if extra data is stored inline.
    ap_uint<1>                  keyInl; // indicating if key is stored inline. 
    ap_uint<1>                  valInl; // indicating if val is stored inline.
    ap_uint<48>                 extPtr; // ptr to the memory region managed by slab memory allocator
    ap_uint<48>			        keyPtr; // ptr to the memory region managed by slab memory allocator
    ap_uint<48>                 valPtr; // ptr to the memory region managed by slab memory allocator
    ap_uint<16>                 currSessionID;
    ap_uint<5>                  reserved;
    ap_uint<MAX_BODY_LEN>       body;
    msgBody() {
        msgID = 0;
        extInl = 0;
        keyInl = 0;
        valInl = 0;
        extPtr = 0;
        keyPtr = 0;
        valPtr = 0;
        currSessionID = 0;
        reserved = 0;
        body = 0;
    }
    msgBody(ap_uint<16> _currSessionID) {
        msgID = 0;
        extInl = 0;
        keyInl = 0;
        valInl = 0;
        extPtr = 0;
        keyPtr = 0;
        valPtr = 0;
        currSessionID = _currSessionID;
        reserved = 0;
        body = 0;
    }
    void consume_word(ap_uint<1024>& w){
        msgID = w(1023, 992);
        extInl = w(991, 991);
        keyInl = w(990, 990);
        valInl = w(989, 989);
        extPtr = w(988, 941);
        keyPtr = w(940, 893);
        valPtr = w(892, 845);
        currSessionID = w(844, 829);
        reserved = w(828, 824);
        body = w(823, 0);
    }
    ap_uint<1024> output_word(){
        return (msgID, extInl, keyInl, valInl, extPtr, keyPtr, valPtr, currSessionID, reserved, body);
    }
    void reset(){
        msgID = 0;
        extInl = 0;
        keyInl = 0;
        valInl = 0;
        extPtr = 0;
        keyPtr = 0;
        valPtr = 0;
        // currSessionID = 0;
        reserved = 0;
        body = 0;
    }

    void display(uint32_t extlen, uint32_t keylen, uint32_t vallen){
        uint32_t pos = 0;
        std::cout << "currSessionID: " << currSessionID << std::endl;

        std::cout << "ext: ";
        for(int i = 0; i < extlen; i++){
#ifndef __SYNTHESIS__
            std::cout << std::hex << std::setfill ('0') << std::setw(2) << +uint8_t(body(MAX_BODY_LEN-pos-1, MAX_BODY_LEN-pos-8));
#endif
            pos += 8;
        }
#ifndef __SYNTHESIS__
        std::cout << std::setw(0);
#endif

        std::cout << std::endl << "key: ";
        for(int i = 0; i < keylen; i++){
            std::cout << char(body(MAX_BODY_LEN-pos-1, MAX_BODY_LEN-pos-8));
            pos += 8;
        }

        std::cout << std::endl << "val: ";
        for(int i = 0; i < vallen; i++){
            std::cout << char(body(MAX_BODY_LEN-pos-1, MAX_BODY_LEN-pos-8));
            pos += 8;
        }
        std::cout << std::endl;
    }
};

#define MEMCACHED_HDRLEN 24 // bytes
struct msgSessionState {
    ap_uint<MEMCACHED_HDRLEN*8>       msgHeaderBuff; // assembling buffer for memcached message header 
    
    ap_uint<1>          parsingHeaderState; // indicate whether parsing header is done
    ap_uint<1>          parsingBodyState; // indicating whether parsing body is done. 

    ap_uint<32>         requiredLen; // required length of header+body 
    ap_uint<8>			currHdrLen;
    ap_uint<32>			currBodyLen;

    ap_uint<16>         currSessionID;
    
    msgSessionState() {
        msgHeaderBuff = 0;
        parsingHeaderState = 0;
        parsingBodyState = 0;
        requiredLen = 0;
        currHdrLen = 0;
        currBodyLen = 0;
        currSessionID = 0;
    }
    msgSessionState(ap_uint<16> _currSessionID) {
        msgHeaderBuff = 0;
        parsingHeaderState = 0;
        parsingBodyState = 0;
        requiredLen = 0;
        currHdrLen = 0;
        currBodyLen = 0;
        currSessionID = _currSessionID;
    }
    void reset() {
        // msgHeaderBuff = 0;
        
        parsingHeaderState = 0;
        parsingBodyState = 0;

        requiredLen = 0;
        currHdrLen = 0;
        currBodyLen = 0;
        currSessionID = 0;
    }

    void reset2() {
        // msgHeaderBuff = 0;
        
        parsingHeaderState = 0;
        parsingBodyState = 0;

        requiredLen = 0;
        currHdrLen = 0;
        currBodyLen = 0;
        // currSessionID = 0;
    }

    void display(){
        std::cout << "parsingHeaderState " << std::dec << parsingHeaderState << std::endl;
        std::cout << "parsingBodyState " << std::dec << parsingBodyState << std::endl;
        std::cout << "requiredLen " << std::dec << requiredLen << std::endl;
        std::cout << "currHdrLen " << std::dec << currHdrLen << std::endl;
        std::cout << "currBodyLen " << std::dec << currBodyLen << std::endl;
        std::cout << "currSessionID " << std::dec << currSessionID << std::endl;
    }
};

// https://github.com/memcached/memcached/blob/master/protocol_binary.h

typedef enum {
    PROTOCOL_BINARY_REQ = 0x80,
    PROTOCOL_BINARY_RES = 0x81
} protocol_binary_magic;

typedef enum {
    PROTOCOL_BINARY_RESPONSE_SUCCESS = 0x00,
    PROTOCOL_BINARY_RESPONSE_KEY_ENOENT = 0x01,
    PROTOCOL_BINARY_RESPONSE_KEY_EEXISTS = 0x02,
    PROTOCOL_BINARY_RESPONSE_E2BIG = 0x03,
    PROTOCOL_BINARY_RESPONSE_EINVAL = 0x04,
    PROTOCOL_BINARY_RESPONSE_NOT_STORED = 0x05,
    PROTOCOL_BINARY_RESPONSE_DELTA_BADVAL = 0x06,
    PROTOCOL_BINARY_RESPONSE_AUTH_ERROR = 0x20,
    PROTOCOL_BINARY_RESPONSE_AUTH_CONTINUE = 0x21,
    PROTOCOL_BINARY_RESPONSE_UNKNOWN_COMMAND = 0x81,
    PROTOCOL_BINARY_RESPONSE_ENOMEM = 0x82
} protocol_binary_response_status;

/**
 * Definition of the different command opcodes.
 * See section 3.3 Command Opcodes
 */
typedef enum {
    PROTOCOL_BINARY_CMD_GET = 0x00,
    PROTOCOL_BINARY_CMD_SET = 0x01,
    PROTOCOL_BINARY_CMD_ADD = 0x02,
    PROTOCOL_BINARY_CMD_REPLACE = 0x03,
    PROTOCOL_BINARY_CMD_DELETE = 0x04,
    PROTOCOL_BINARY_CMD_INCREMENT = 0x05,
    PROTOCOL_BINARY_CMD_DECREMENT = 0x06,
    PROTOCOL_BINARY_CMD_QUIT = 0x07,
    PROTOCOL_BINARY_CMD_FLUSH = 0x08,
    PROTOCOL_BINARY_CMD_GETQ = 0x09,
    PROTOCOL_BINARY_CMD_NOOP = 0x0a,
    PROTOCOL_BINARY_CMD_VERSION = 0x0b,
    PROTOCOL_BINARY_CMD_GETK = 0x0c,
    PROTOCOL_BINARY_CMD_GETKQ = 0x0d,
    PROTOCOL_BINARY_CMD_APPEND = 0x0e,
    PROTOCOL_BINARY_CMD_PREPEND = 0x0f,
    PROTOCOL_BINARY_CMD_STAT = 0x10,
    PROTOCOL_BINARY_CMD_SETQ = 0x11,
    PROTOCOL_BINARY_CMD_ADDQ = 0x12,
    PROTOCOL_BINARY_CMD_REPLACEQ = 0x13,
    PROTOCOL_BINARY_CMD_DELETEQ = 0x14,
    PROTOCOL_BINARY_CMD_INCREMENTQ = 0x15,
    PROTOCOL_BINARY_CMD_DECREMENTQ = 0x16,
    PROTOCOL_BINARY_CMD_QUITQ = 0x17,
    PROTOCOL_BINARY_CMD_FLUSHQ = 0x18,
    PROTOCOL_BINARY_CMD_APPENDQ = 0x19,
    PROTOCOL_BINARY_CMD_PREPENDQ = 0x1a,
    PROTOCOL_BINARY_CMD_TOUCH = 0x1c,
    PROTOCOL_BINARY_CMD_GAT = 0x1d,
    PROTOCOL_BINARY_CMD_GATQ = 0x1e,
    PROTOCOL_BINARY_CMD_GATK = 0x23,
    PROTOCOL_BINARY_CMD_GATKQ = 0x24,

    PROTOCOL_BINARY_CMD_SASL_LIST_MECHS = 0x20,
    PROTOCOL_BINARY_CMD_SASL_AUTH = 0x21,
    PROTOCOL_BINARY_CMD_SASL_STEP = 0x22,

    /* These commands are used for range operations and exist within
     * this header for use in other projects.  Range operations are
     * not expected to be implemented in the memcached server itself.
     */
    PROTOCOL_BINARY_CMD_RGET      = 0x30,
    PROTOCOL_BINARY_CMD_RSET      = 0x31,
    PROTOCOL_BINARY_CMD_RSETQ     = 0x32,
    PROTOCOL_BINARY_CMD_RAPPEND   = 0x33,
    PROTOCOL_BINARY_CMD_RAPPENDQ  = 0x34,
    PROTOCOL_BINARY_CMD_RPREPEND  = 0x35,
    PROTOCOL_BINARY_CMD_RPREPENDQ = 0x36,
    PROTOCOL_BINARY_CMD_RDELETE   = 0x37,
    PROTOCOL_BINARY_CMD_RDELETEQ  = 0x38,
    PROTOCOL_BINARY_CMD_RINCR     = 0x39,
    PROTOCOL_BINARY_CMD_RINCRQ    = 0x3a,
    PROTOCOL_BINARY_CMD_RDECR     = 0x3b,
    PROTOCOL_BINARY_CMD_RDECRQ    = 0x3c
    /* End Range operations */

} protocol_binary_command;

union protocol_binary_request_header{
    struct request_t{
        uint8_t magic;
        uint8_t opcode;
        uint16_t keylen;
        uint8_t extlen;
        uint8_t datatype;
        uint16_t reserved;
        uint32_t bodylen;
        uint32_t opaque;
        uint64_t cas;
    } request;
    uint8_t bytes[24];
    protocol_binary_request_header() {}
    ~protocol_binary_request_header() {}
    void display(){
        // cout << uint8_t will be treated as char. 
        cout << "magic " << hex << (uint16_t)request.magic << endl;
        cout << "opcode " << hex << (uint16_t)request.opcode << endl;
        cout << "keyLen " << dec << htons(request.keylen) << endl;
        cout << "extlen " << dec << (uint16_t)request.extlen << endl;
        cout << "dataType " << dec << (uint16_t)request.datatype << endl;
        cout << "reserved " << dec << htons(request.reserved) << endl;
        cout << "bodyLen " << dec << htonl(request.bodylen) << endl;
        cout << "opaque " << hex << htonl(request.opaque) << endl;
        cout << "cas " << dec << request.cas << endl;
    }
};

union protocol_binary_response_header{
    struct response_t{
        uint8_t magic;
        uint8_t opcode;
        uint16_t keylen;
        uint8_t extlen;
        uint8_t datatype;
        uint16_t status;
        uint32_t bodylen;
        uint32_t opaque;
        uint64_t cas;
    } response;
    uint8_t bytes[24];
    protocol_binary_response_header() {}
    ~protocol_binary_response_header() {}
};
