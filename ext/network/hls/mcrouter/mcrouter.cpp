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
#include "hash_table/hash_table.hpp"

void open_port(	hls::stream<ap_uint<16> >&		listenPort,
				hls::stream<bool>&				listenSts)
{
#pragma HLS PIPELINE II=1
#pragma HLS INLINE off

	static ap_uint<2> state = 0;
	#pragma HLS reset variable=state

	bool listenDone = false;

	switch (state)
	{
	case 0:
        // mcrouter uses 5000 port
		listenPort.write(5000);
		state = 1;
		break;
	case 1:
		if (!listenSts.empty())
		{
			listenSts.read(listenDone);
			if (listenDone)
			{
				state = 2;
			}
			else
			{
				state = 0;
			}
		}
		break;
	case 2:
		//IDLE
		break;
	}//switch

}

void notification_handler(	hls::stream<appNotification>&	notific, // input from toe
							hls::stream<appReadRequest>&	readReq) // output to toe
{
#pragma HLS PIPELINE II=1
#pragma HLS INLINE off

	appNotification notification;

	// Receive notifications, about new data which is available
	if (!notific.empty())
	{
		notific.read(notification);
		std::cout << notification.ipAddress << "\t" << notification.dstPort << std::endl;
		if (notification.length != 0)
		{
            // requesting the data from this session
			readReq.write(appReadRequest(notification.sessionID, notification.length));
		}
        else if(notification.closed) {
            // TODO: handling closed connections
        }
	}
}
// generating globally unique reqID. 
static ap_uint<32> currentReqID = 0;
#define MAX_SESSION_NUM ((1 << 16) - 1)

static hls::stream<htLookupReq<16> >       s_axis_lup_req;
static hls::stream<htUpdateReq<16,1024> >    s_axis_upd_req;
static hls::stream<htLookupResp<16,1024> >   m_axis_lup_rsp;
static hls::stream<htUpdateResp<16,1024> >   m_axis_upd_rsp;
static ap_uint<16> regInsertFailureCount;

void parser(	hls::stream<ap_uint<16> >& rxMetaData, // input from toe
				hls::stream<net_axis<512> >& rxData, // intput from toe 
				hls::stream<ap_uint<16> >& reqSessioIdFifo, // output to req_handler
				hls::stream<ap_uint<16> >& respSessioIdFifo, // output to resp_handler
				hls::stream<kvReq>& reqFifo, // output the parsed requests to req_handler
                hls::stream<kvResp>& respFifo) // output the parsed responce to resp_hanlder
{
#pragma HLS PIPELINE II=1
#pragma HLS INLINE off

    // static value gets inited to zero by default. 
    static ap_uint<1> msgParsing_fsmState[MAX_SESSION_NUM]; // 0-> IDLE, 1-> MSG-ONGOING
    #pragma HLS ARRAY_PARTITION variable=msgParsing_fsmState block dim=1 factor=1024
    
    enum dataRecv_fsmType {IDLE, STATE_RECOVER, AXIS_ONGOING};
    static dataRecv_fsmType dataRecv_fsmState = IDLE;

	static ap_uint<16> sessionID; // valid when dataRecv_fsmState becomes on-going
    static kvReq currKvReq;
    static kvResp currKvResp;
    static ap_uint<16> currKeyLen;
    static ap_uint<32> currValLen;

    switch (dataRecv_fsmState){
        case IDLE: 
            // processing the first chunk of this AXIS transmission
            if(!rxMetaData.empty()){
                rxMetaData.read(sessionID);
                
                // recovering currKvReq from hash table. 
                if (msgParsing_fsmState[sessionID] == 0){
                    dataRecv_fsmState = AXIS_ONGOING;
                }
                else{
                    s_axis_lup_req.write(htLookupReq<16>(sessionID, 0));
                    dataRecv_fsmState = STATE_RECOVER;
                }
            }
            break;
        case STATE_RECOVER: 
            // reciving resp from hash, expecting in one cycle. 
            if (!m_axis_lup_rsp.empty()){
                htLookupResp<16,1024> response = m_axis_lup_rsp.read();
                // state recovered from hash table
                currKvReq = kvReq(response.value);
                currKeyLen = currKvReq.currKeyLen;
                dataRecv_fsmState = AXIS_ONGOING;
            }
            break;
        case AXIS_ONGOING:
            // TODO: two messages might mix into one ASIX transmission. 
            // processing the non-first chunks of this AXIS transmission. 
            if(!rxData.empty()){
            	net_axis<512> currWord;
                rxData.read(currWord);

                switch (msgParsing_fsmState[sessionID]){
                    case 0:{
                        // processing the first chunk of this message
                        ap_uint<32> reqID = currentReqID;
                        currentReqID ++;
                        currKvReq.reqID = reqID;
                        currKvReq.opcode = currWord.data(511, 496);

                        if (currWord.data(511, 496) == GET_OPCODE) {
                            ap_uint<16> keyLen = currWord.data(495, 480);
                            currKvReq.keyLen = keyLen;
                            ap_uint<16> remainLen = 480 - __builtin_ctz(currWord.keep)*8;
                            
                            if(keyLen <= remainLen/8){
                                currKvReq.key(KV_MAX_KEY_SIZE-1, KV_MAX_KEY_SIZE-keyLen*8) = currWord.data(480-1, 480-keyLen*8);
                                // pricessing the last chunk of this message
                                reqFifo.write(currKvReq);
                                reqSessioIdFifo.write(sessionID);
                                dataRecv_fsmState = IDLE;
                                msgParsing_fsmState[sessionID] = 0;
                            }
                            else{
                                currKvReq.key(KV_MAX_KEY_SIZE-1, KV_MAX_KEY_SIZE-remainLen) = currWord.data(480-1, 480-remainLen);
                                currKeyLen = remainLen/8;
                                // still need more parsing
                                msgParsing_fsmState[sessionID] = 1;
         
                                // processing the last chunk of this AXIS transmission. 
                                // reaching here means we still need more processing. we need to save currKvReq into hash table. 
                                if(currWord.last){
                                    dataRecv_fsmState = IDLE;
                                    currKvReq.currKeyLen = currKeyLen;
                                    s_axis_upd_req.write(htUpdateReq<16, 1024>(KV_INSERT, sessionID, currKvReq.toBits(), 0));
                                }
                            }
                        }
                        // else if(currWord.data(511, 496) == SET_OPCODE) {

                        // }
                        else if(currWord.data(511, 496) == GET_RESP_OPCODE) {
                            respSessioIdFifo.write(sessionID);
                            respFifo.write(currKvResp);
                        }
                        break;
                    }
                    case 1:{
                        ap_uint<16> remainLen = 512 - __builtin_ctz(currWord.keep)*8;
                        // processing the non-first chunks of this message
                        if(currKvReq.keyLen - currKeyLen <= remainLen/8){
                            currKvReq.key(KV_MAX_KEY_SIZE - currKeyLen*8 - 1, KV_MAX_KEY_SIZE - currKvReq.keyLen*8) = currWord.data(511, (currKvReq.keyLen - currKeyLen)*8);
                            // processing the last chunk of this message
                            reqFifo.write(currKvReq);
                            reqSessioIdFifo.write(sessionID);
                            dataRecv_fsmState = IDLE;
                            msgParsing_fsmState[sessionID] = 0;
                            break;
                        }
                        else{
                            currKvReq.key(KV_MAX_KEY_SIZE-1, KV_MAX_KEY_SIZE-remainLen) = currWord.data(512-1, 512-remainLen);
                            currKeyLen += remainLen/8;
                            // still need more parsing
                            msgParsing_fsmState[sessionID] = 1;
                            // processing the last chunk of this AXIS transmission. 
                            // reaching here means we still need more processing. we need to save currKvReq into hash table. 
                            if(currWord.last){
                                dataRecv_fsmState = IDLE;
                                currKvReq.currKeyLen = currKeyLen;
                                s_axis_upd_req.write(htUpdateReq<16, 1024>(KV_INSERT, sessionID, currKvReq.toBits(), 0));
                            }
                        }
                        break;
                    }
                }
            }
            break;
    }
}

void req_handler(hls::stream<ap_uint<16> >& sessioIdFifo,
				hls::stream<kvReq>& reqFifo,
                hls::stream<ap_uint<16> >& destSessioIdFifo,
                hls::stream<kvReq>& finalReqFifo)
{
#pragma HLS PIPELINE II=1
#pragma HLS INLINE off

	// Reads new data from memory and writes it into fifo
	// Read & write metadata only once per package
	static ap_uint<1> esac_fsmState = 0;

	ap_uint<16> sessionID;
    kvReq currKvReq;

	switch (esac_fsmState)
	{
	case 0:
		if (!sessioIdFifo.empty())
		{
			sessioIdFifo.read(sessionID);
			esac_fsmState = 1;
		}
		break;
	case 1:
		if (!reqFifo.empty())
		{
			reqFifo.read(currKvReq);
            destSessioIdFifo.write(sessionID);
            finalReqFifo.write(currKvReq);
            // continuously read and writing until the current AXIS transmission done. 
			if (true)
			{
				esac_fsmState = 0;
			}
		}
		break;
	}

}

void resp_handler(hls::stream<ap_uint<16> >& sessioIdFifo,
				hls::stream<kvResp>& respFifo,
                hls::stream<ap_uint<16> >& destSessioIdFifo,
                hls::stream<kvResp>& finalRespFifo)
{
#pragma HLS PIPELINE II=1
#pragma HLS INLINE off

	// Reads new data from memory and writes it into fifo
	// Read & write metadata only once per package
	static ap_uint<1> esac_fsmState = 0;

	ap_uint<16> sessionID;
    kvResp currKvResp;

	switch (esac_fsmState)
	{
	case 0:
		if (!sessioIdFifo.empty())
		{
			sessioIdFifo.read(sessionID);
			esac_fsmState = 1;
		}
		break;
	case 1:
		if (!respFifo.empty())
		{
			respFifo.read(currKvResp);
            destSessioIdFifo.write(sessionID);
            finalRespFifo.write(currKvResp);
            // continuously read and writing until the current AXIS transmission done. 
			if (true)
			{
				esac_fsmState = 0;
			}
		}
		break;
	}

}

void deparser(hls::stream<ap_uint<16> >& reqDestSessioIdFifo,
            hls::stream<ap_uint<16> >& respDestSessioIdFifo,
            hls::stream<kvReq>& finalReqFifo,
            hls::stream<kvResp>& finalRespFifo,
			hls::stream<appTxMeta>& txMetaData, 
            hls::stream<net_axis<512> >& txData)
{
	ap_uint<16> reqSessionID;
	ap_uint<16> respSessionID;
    kvReq currKvReq;
    kvResp currKvResp;
    
	if (!reqDestSessioIdFifo.empty())
	{   
        reqDestSessioIdFifo.read(reqSessionID);
    }
	if (!respDestSessioIdFifo.empty())
	{   
        respDestSessioIdFifo.read(respSessionID);
    }


	if (!finalReqFifo.empty())
	{   
        finalReqFifo.read(currKvReq);
    }
	if (!finalRespFifo.empty())
	{   
        finalRespFifo.read(currKvResp);
    }

    static ap_uint<1> esac_fsmState = 0;
	ap_uint<16> sessionID;
	ap_uint<16> length;
	net_axis<512> currWord;

	switch (esac_fsmState)
	{
	case 0:
		if (!txMetaData.full())
		{
			txMetaData.write(appTxMeta(reqSessionID, length));
			esac_fsmState = 1;
		}
		break;
	case 1:
		if (!txData.full())
		{
			txData.write(currWord);
			if (true)
			{
				esac_fsmState = 0;
			}
		}
		break;
	}
}

void dummy(	hls::stream<ipTuple>& openConnection, 
            hls::stream<openStatus>& openConStatus,
			hls::stream<ap_uint<16> >& closeConnection,
			hls::stream<appTxRsp>& txStatus)
{
#pragma HLS PIPELINE II=1
#pragma HLS INLINE off

	openStatus newConn;
	ipTuple tuple;

	// Dummy code should never be executed, this is necessary because every hls::streams has to be written/read
	if (!openConStatus.empty())
	{
		openConStatus.read(newConn);
		tuple.ip_address = 0x0a010101;
		tuple.ip_port = 0x3412;
		openConnection.write(tuple);
		if (newConn.success)
		{
			closeConnection.write(newConn.sessionID);
			//closePort.write(tuple.ip_port);
		}
	}

	if (!txStatus.empty()) //Make Checks
	{
		txStatus.read();
	}

    if (!m_axis_upd_rsp.empty()){
        m_axis_upd_rsp.read();
    }

}


// @yang, this implements a TCP interface 
void mcrouter(	// for mcrouter listening on a TCP port
                hls::stream<ap_uint<16> >&		listenPort,
				hls::stream<bool>&				listenPortStatus,
                // for mcrouter receiving notifications: either new data available or connection closed
				hls::stream<appNotification>&	notifications,
                // for mcrouter requesting and receiving data
				hls::stream<appReadRequest>&	readRequest,
				hls::stream<ap_uint<16> >&		rxMetaData,
				hls::stream<net_axis<512> >&		rxData,
                // for mcrouter openning a connection
				hls::stream<ipTuple>&			openConnection,
				hls::stream<openStatus>&		openConStatus,
                // for mcrouter closing a connection
				hls::stream<ap_uint<16> >&		closeConnection,
                // for mcrouter sending data
				hls::stream<appTxMeta>&			txMetaData,
				hls::stream<net_axis<512> > &	txData,
				hls::stream<appTxRsp>&			txStatus)
{
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


	static hls::stream<ap_uint<16> >		mc_reqSessioIdFifo("mc_reqSessioIdFifo");
	static hls::stream<ap_uint<16> >		mc_respSessioIdFifo("mc_respSessioIdFifo");
	static hls::stream<ap_uint<16> >		mc_reqDestSessioIdFifo("mc_reqDestSessioIdFifo");
	static hls::stream<ap_uint<16> >		mc_respDestSessioIdFifo("mc_respDestSessioIdFifo");
	static hls::stream<kvReq>		        mc_reqFifo("mc_reqFifo");
	static hls::stream<kvResp>		        mc_respFifo("mc_respFifo");
    static hls::stream<kvReq>		        mc_finalReqFifo("mc_finalReqFifo");
	static hls::stream<kvResp>		        mc_finalRespFifo("mc_finalRespFifo");

#pragma HLS stream variable=mc_reqSessioIdFifo depth=64
#pragma HLS stream variable=mc_respSessioIdFifo depth=64
#pragma HLS stream variable=mc_reqDestSessioIdFifo depth=64
#pragma HLS stream variable=mc_respDestSessioIdFifo depth=64
#pragma HLS stream variable=mc_reqFifo depth=2048
#pragma HLS stream variable=mc_respFifo depth=2048
#pragma HLS stream variable=mc_finalReqFifo depth=2048
#pragma HLS stream variable=mc_finalRespFifo depth=2048

#pragma HLS stream variable=s_axis_lup_req depth=64
#pragma HLS stream variable=s_axis_upd_req depth=64
#pragma HLS stream variable=m_axis_lup_rsp depth=64
#pragma HLS stream variable=m_axis_upd_rsp depth=64
#pragma HLS DATA_PACK variable=s_axis_lup_req
#pragma HLS DATA_PACK variable=s_axis_upd_req
#pragma HLS DATA_PACK variable=m_axis_lup_rsp
#pragma HLS DATA_PACK variable=m_axis_upd_rsp
#pragma HLS INTERFACE ap_stable port=regInsertFailureCount

    // initing a hash table block
    hash_table<16, 1024>(s_axis_lup_req, s_axis_upd_req, m_axis_lup_rsp, m_axis_upd_rsp, regInsertFailureCount);

    // opening the mcrouter listening port
	open_port(listenPort, listenPortStatus);
    
    // handling new data arriving (it might come from a new session), and connection closed
	notification_handler(notifications, readRequest);
    
    // read data from network and parse them into reqFifo
    parser(rxMetaData, rxData, mc_reqSessioIdFifo, mc_respSessioIdFifo, mc_reqFifo, mc_respFifo);

	// 1) read request from reqFifo, determine the destination mc, query mc
    req_handler(mc_reqSessioIdFifo, mc_reqFifo, mc_reqDestSessioIdFifo, mc_finalReqFifo);

    // 2) read response from reqFifo, forward to client. 
    resp_handler(mc_respSessioIdFifo, mc_respFifo, mc_respDestSessioIdFifo, mc_finalRespFifo);

    // deparsing the res and resp, sending to destination (client or memcached)
    deparser(mc_reqDestSessioIdFifo, mc_respDestSessioIdFifo, mc_finalReqFifo, mc_finalRespFifo, txMetaData, txData);

	dummy(openConnection, openConStatus, closeConnection, txStatus);

}
