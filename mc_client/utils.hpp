#ifndef __UTILS_HPP
#define __UTILS_HPP

using namespace std;

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

#define ntohll(x) ( ( (uint64_t)(ntohl( (uint32_t)((x << 32) >> 32) )) << 32) | ntohl( ((uint32_t)(x >> 32)) ) )
#define htonll(x) ntohll(x)

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
        cout << "cas " << dec << htonll(request.cas) << endl;
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
    void display(){
        // cout << uint8_t will be treated as char. 
        cout << "magic " << hex << (uint16_t)response.magic << endl;
        cout << "opcode " << hex << (uint16_t)response.opcode << endl;
        cout << "keyLen " << dec << htons(response.keylen) << endl;
        cout << "extlen " << dec << (uint16_t)response.extlen << endl;
        cout << "dataType " << dec << (uint16_t)response.datatype << endl;
        cout << "status " << dec << htons(response.status) << endl;
        cout << "bodyLen " << dec << htonl(response.bodylen) << endl;
        cout << "opaque " << hex << htonl(response.opaque) << endl;
        cout << "cas " << dec << htonll(response.cas) << endl;
    }
};

#endif //__UTILS_HPP