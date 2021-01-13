#include <netdb.h> 
#include <stdio.h> 
#include <stdlib.h> 
#include <string.h> 
#include <sys/socket.h> 
#include <arpa/inet.h>
#include <unistd.h>

#include <iostream>
#include <vector>
#include <queue>
#include <string>
#include <map>
#include "utils.hpp"

using namespace std;

#define NUM_KV 1024
static vector<string> keys;
static vector<string> vals;
static vector<bool> ops;
static map<string, string> kvMap;

// 62 chars
static const char* baseStr = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz";

string getStr(uint32_t startPos, uint32_t length){
    static char buf[128];
    if(startPos + length <= strlen(baseStr)){
        memcpy(buf, baseStr+startPos, length);
    }
    else{
        uint32_t remainLen = startPos + length - strlen(baseStr);
        memcpy(buf, baseStr+startPos, length-remainLen);
        memcpy(buf+length-remainLen, baseStr, remainLen);
    }
    buf[length] = '\0';
    return string(buf);
}

void workload_gen(int sockfd){    
    srand(0xdeadbeef);
    // key: 1-40bytes
    // value: 1-50bytes
    for(int i = 0; i < NUM_KV; i++){
        uint32_t startPos = rand() % strlen(baseStr);
        uint32_t length = rand() % (40-1) + 1;
        string key = getStr(startPos, length);

        startPos = rand() % strlen(baseStr);
        length = rand() % (50-1) + 1;
        string val = getStr(startPos, length);
        if(kvMap.find(key) != kvMap.end()){
            i--;
            continue;
        }
        keys.push_back(key);
        vals.push_back(val);
        ops.push_back(rand() % 2);
        // ops.push_back(0);

        kvMap[key] = val;
    }

    uint8_t* buf = new uint8_t[24+40+50+8];
    uint8_t* rbuf = new uint8_t[24+40+50+8];
    for(int i = 0; i < NUM_KV; i++){
        int currPos = 0;
        
        bool op = ops[i];
        string key = keys[i];
        string val = vals[i];
        
        static protocol_binary_request_header req; 
        req.request = {
            .magic = PROTOCOL_BINARY_REQ, 
            .opcode = PROTOCOL_BINARY_CMD_GET, 
            .keylen = 0,
            .extlen = 0,
            .datatype = 0,
            .reserved = 0,
            .bodylen = 0,
            .opaque = htonl(i), // used as request id
            .cas = 0
        };
        if(!op){
            req.request.keylen = htons((uint16_t)key.length());
            req.request.bodylen = htonl(key.length());
            memcpy(buf + currPos, req.bytes, 24);
            currPos += 24;
            memcpy(buf + currPos, key.c_str(), key.length());
            currPos += key.length();
        }
        else{
            req.request.opcode = PROTOCOL_BINARY_CMD_SET;
            req.request.keylen = htons((uint16_t)key.length());
            req.request.extlen = 8;
            req.request.bodylen = htonl(key.length() + val.length() + 8);
            memcpy(buf + currPos, req.bytes, 24);
            currPos += 24;
            uint32_t Flags = htonl(0xdeadbeef);
            uint32_t Expiry = htonl(0x00001c20);
            memcpy(buf + currPos, (char *)&Flags, 4);
            currPos += 4;
            memcpy(buf + currPos, (char *)&Expiry, 4);
            currPos += 4;
            memcpy(buf + currPos, key.c_str(), key.length());
            currPos += key.length();
            memcpy(buf + currPos, val.c_str(), val.length());
            currPos += val.length();
        }
        req.display();

        write(sockfd, buf, currPos); 
        read(sockfd, rbuf, 24);
        protocol_binary_response_header rsp;
        memcpy(rsp.bytes, rbuf, 24);
        rsp.display();
        uint32_t bodylen = htonl(rsp.response.bodylen);
        if(bodylen != 0){
            read(sockfd, rbuf, bodylen);
            cout << "body: " << string((char*)rbuf, bodylen) << endl;
        }
    }
    delete buf;
}

#define SA struct sockaddr 

int main(){
    int sockfd, connfd; 
    struct sockaddr_in servaddr, cli; 
  
    // socket create and varification 
    sockfd = socket(AF_INET, SOCK_STREAM, 0); 
    if (sockfd == -1) { 
        printf("socket creation failed...\n"); 
        exit(0); 
    } 
    else
        printf("Socket successfully created..\n"); 
    bzero(&servaddr, sizeof(servaddr)); 
  
    // assign IP, PORT 
    servaddr.sin_family = AF_INET; 
    servaddr.sin_addr.s_addr = inet_addr("192.168.0.5"); 
    // servaddr.sin_addr.s_addr = inet_addr("127.0.0.1"); 
    servaddr.sin_port = htons(5001); 
  
    // connect the client socket to server socket 
    if (connect(sockfd, (SA*)&servaddr, sizeof(servaddr)) != 0) { 
        printf("connection with the server failed...\n"); 
        exit(0); 
    } 
    else
        printf("connected to the server..\n"); 
  
    // function for chat 
    workload_gen(sockfd); 
  
    // close the socket 
    close(sockfd); 
}