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

#include "../axi_utils.hpp"
#include "../toe/toe.hpp"
#include "hash_table_16_1024/hash_table_16_1024.hpp"
#include "hash_table_32_32/hash_table_32_32.hpp"
#include "multi_queue/multi_queue.hpp"

#define DATA_WIDTH 512
#define MAX_CONNECTED_SESSIONS 512

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
    msgHeader() {}
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
        return (cas, opaque, bodyLen, status, dataType, extLen, keyLen, opcode, magic);
    }
    ap_uint<32> val_len(){
        return bodyLen-extLen-keyLen;
    }
};

#define KV_MAX_EXT_SIZE (5*8)
#define KV_MAX_KEY_SIZE (40*8)
#define KV_MAX_VAL_SIZE (1024-192-KV_MAX_EXT_SIZE-KV_MAX_KEY_SIZE)

struct msgBody {
	ap_uint<32>                 msgID;  // globally unique msgID
    ap_uint<1>                  extInl; // indicating if extra data is stored inline.
    ap_uint<1>                  keyInl; // indicating if key is stored inline. 
    ap_uint<1>                  valInl; // indicating if val is stored inline.
    ap_uint<48>                 extPtr; // ptr to the memory region managed by slab memory allocator
    ap_uint<48>			        keyPtr; // ptr to the memory region managed by slab memory allocator
    ap_uint<48>                 valPtr; // ptr to the memory region managed by slab memory allocator
    ap_uint<13>                 unused; // make sure SENDBUF_LEN is within 1024 bits
    ap_uint<KV_MAX_EXT_SIZE>    ext;
    ap_uint<KV_MAX_KEY_SIZE>    key;
    ap_uint<KV_MAX_VAL_SIZE>    val;
    msgBody() {}
    void consume_word(ap_uint<1024>& w){
        msgID = w(1023, 992);
        extInl = w(991, 991);
        keyInl = w(990, 990);
        valInl = w(989, 989);
        extPtr = w(988, 941);
        keyPtr = w(940, 893);
        valPtr = w(892, 845);
        unused = w(844, 832);
        ext = w(831, 792);
        key = w(791, 472);
        val = w(471, 0);
    }
    ap_uint<1024> output_word(){
        return (val, key, ext, valPtr, keyPtr, extPtr, valInl, keyInl, extInl, msgID);
    }
};

struct sessionState {
    ap_uint<24*8>   msgHeaderBuff; // assembling buffer for memcached message header 
    ap_uint<5>      currLen; // current len of value msg header. 
    ap_uint<1>      parsingHeader; // indicate whether parsing header is done
    // 
    ap_uint<8>			currExtLen;
    ap_uint<16>			currKeyLen;
    ap_uint<32>			currValLen;
    ap_uint<2>          parsingBody; // indicating whether parsing ext/key/val is done. 
    sessionState() {}
};

struct msgContext {
    ap_uint<16> numRsp;
    ap_uint<16> srcSessionID;
    msgContext() {}
    msgContext(ap_uint<16> _numRsp, ap_uint<16> _srcSessionID) :
        numRsp(_numRsp), 
        srcSessionID(_srcSessionID)
    {}
    void consume_word(ap_uint<32>& w){
        numRsp = w(31, 16);
        srcSessionID = w(15, 0);
    }
    ap_uint<32> output_word(){
        return (srcSessionID, numRsp);
    }
};

// enum healthCmdType {GET_SESSION_COUNT, GET_SESSIONID_HASH, GET_SESSIONID_IDX};

// struct healthReq {
//     healthCmdType cmd;
//     ap_uint<16> val;
//     healthReq() {}
//     healthReq(healthCmdType _cmd, ap_uint<16> _val): 
//         cmd(_cmd), val(_val) {}
// };

/** @defgroup mcrouter Echo Server Application
 *
 */
void mcrouter(hls::stream<ap_uint<16> >& listenPort, hls::stream<bool>& listenPortStatus,
			hls::stream<appNotification>& notifications, hls::stream<appReadRequest>& readRequest,
			hls::stream<ap_uint<16> >& rxMetaData, hls::stream<net_axis<DATA_WIDTH> >& rxData,
			hls::stream<ipTuple>& openTuples, hls::stream<ipTuple>& openConnection, hls::stream<openStatus>& openConStatus,
			hls::stream<ap_uint<16> >& closeConnection,
			hls::stream<appTxMeta>& txMetaData, hls::stream<net_axis<DATA_WIDTH> >& txData,
			hls::stream<appTxRsp>& txStatus);


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
        ap_uint<8> magic;
        ap_uint<8> opcode;
        ap_uint<16> keylen;
        ap_uint<8> extlen;
        ap_uint<8> datatype;
        ap_uint<16> reserved;
        ap_uint<32> bodylen;
        ap_uint<32> opaque;
        ap_uint<64> cas;
    } request;
    uint8_t bytes[24];
    protocol_binary_request_header() {}
    ~protocol_binary_request_header() {}
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
