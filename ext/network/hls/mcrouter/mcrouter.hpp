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

#define KV_MAX_KEY_SIZE (40*8)
#define KV_MAX_VAL_SIZE (1024-32-8-16*2-48-1-32*2-48-1-40*8)

// https://github.com/memcached/memcached/wiki/BinaryProtocolRevamped#get-get-quietly-get-key-get-key-quietly
#define GET_OPCODE 0x00
#define SET_OPCODE 0x01
#define GET_RESP_OPCODE 0x02
#define SET_RESP_OPCODE 0x03

struct kvReq {
	ap_uint<32>         reqID;  // globally unique reqID
    ap_uint<8>          opcode; // indicating the operation type
    ap_uint<16>			keyLen; // key length in byte; memcached support up to 250 by default
	ap_uint<16>			currKeyLen;
    ap_uint<48>			keyPtr; // ptr to the memory region managed by slab memory allocator
    ap_uint<1>          keyInl; // indicating if key is stored inline. 
    ap_uint<32>         valLen; // val length in byte
	ap_uint<32>			currValLen;
    ap_uint<48>         valPtr; // ptr to the memory region managed by slab memory allocator
    ap_uint<1>          valInl; // indicating if val is stored inline.
    ap_uint<KV_MAX_KEY_SIZE>   key;
    ap_uint<KV_MAX_VAL_SIZE>   val;
    kvReq() :
        reqID  (0),
        opcode (0),
        keyLen (0),
        currKeyLen (0),
        keyPtr (0),
        keyInl (0),
        valLen (0),
        currValLen (0),
        valPtr (0),
        valInl (0),
        key (0),
        val (0)
    {}
    kvReq(const ap_uint<1024>& d) :
        reqID  (d( 31,  0)),
        opcode(d( 39, 32)),
        keyLen (d( 55, 40)),
        currKeyLen (d( 71, 56)),
        keyPtr (d( 119, 72)),
        keyInl (d( 120, 120)),
        valLen (d( 152, 121)),
        currValLen (d( 184, 153)),
        valPtr (d( 232, 185)),
        valInl (d( 233, 233)),
        key (d( 553, 234)),
        val (d( 1023, 554))
    {} 

    ap_uint<1024> toBits()
    {
        return (val, key, valInl, valPtr, currValLen, valLen, keyInl, keyPtr, currKeyLen, keyLen, opcode, reqID);
    }
};

struct kvResp
{
	ap_uint<32>         respID;  // globally unique respID
    ap_uint<8>          opcode; // indicating the operation type
    ap_uint<16>			keyLen; // key length in byte; memcached support up to 250 by default
	ap_uint<16>			currKeyLen;
	ap_uint<48>			keyPtr; // ptr to the memory region managed by slab memory allocator
    ap_uint<1>          keyInl; // indicating if key is stored inline. 
    ap_uint<32>         valLen; // val length in byte
	ap_uint<32>			currValLen;
    ap_uint<48>         valPtr; // ptr to the memory region managed by slab memory allocator
    ap_uint<1>          valInl; // indicating if val is stored inline.
    ap_uint<KV_MAX_KEY_SIZE>   key;
    ap_uint<KV_MAX_VAL_SIZE>   val;
    
    kvResp() :
        respID  (0),
        opcode (0),
        keyLen (0),
        currKeyLen (0),
        keyPtr (0),
        keyInl (0),
        valLen (0),
        currValLen (0),
        valPtr (0),
        valInl (0),
        key (0),
        val (0)
    {}
    kvResp(const ap_uint<1024>& d) :
        respID  (d( 31,  0)),
        opcode(d( 39, 32)),
        keyLen (d( 55, 40)),
        currKeyLen (d( 71, 56)),
        keyPtr (d( 119, 72)),
        keyInl (d( 120, 120)),
        valLen (d( 152, 121)),
        currValLen (d( 184, 153)),
        valPtr (d( 232, 185)),
        valInl (d( 233, 233)),
        key (d( 553, 234)),
        val (d( 1023, 554))
    {} 

    ap_uint<1024> toBits()
    {
        return (val, key, valInl, valPtr, currValLen, valLen, keyInl, keyPtr, currKeyLen, keyLen, opcode, respID);
    }
};


/** @defgroup mcrouter Echo Server Application
 *
 */
void mcrouter(hls::stream<ap_uint<16> >& listenPort, hls::stream<bool>& listenPortStatus,
								hls::stream<kvReq>& notifications, hls::stream<appReadRequest>& readRequest,
								hls::stream<ap_uint<16> >& rxMetaData, hls::stream<net_axis<64> >& rxData,
								hls::stream<ipTuple>& openConnection, hls::stream<openStatus>& openConStatus,
								hls::stream<ap_uint<16> >& closeConnection,
								hls::stream<appTxMeta>& txMetaData, hls::stream<net_axis<64> >& txData,
								hls::stream<appTxRsp>& txStatus);
