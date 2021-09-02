#include "rosbridge_protocol.h"

rosbridge_protocol::Encoding
rosbridge_protocol::compressionToEncoding(const std::string& compression)
{
    rosbridge_protocol::Encoding encoding = rosbridge_protocol::Encoding::JSON;
    if(compression == "cbor")
    {
        encoding = rosbridge_protocol::Encoding::CBOR;
    }
    else if(compression == "cbor-raw")
    {
        encoding = rosbridge_protocol::Encoding::CBOR_RAW;
    }
    else
    {
        // Should be 'none' for JSON
        encoding = rosbridge_protocol::Encoding::JSON;
    }

    return encoding;
}
