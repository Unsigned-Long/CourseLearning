//
// Created by csl on 9/22/22.
//

#include "novatel_oem.h"
#include "utils/buffer.hpp"
#include "artwork/logger/logger.h"

ns_spp::NovAtelOEMFileHandler::~NovAtelOEMFileHandler() {
    delete[] this->buffer;
}

const ns_spp::Byte *ns_spp::NovAtelOEMFileHandler::getBuffer() const {
    return buffer;
}

ns_spp::NovAtelOEMFileHandler::NovAtelOEMFileHandler(const std::string &binFilePath) : buffer(nullptr) {
    using namespace ns_spp;
    this->bufferSize = BufferHelper::readBuffer(&this->buffer, binFilePath);

    for (int i = 0; i != bufferSize - 3;) {
        LOG_VAR(i);
        if (buffer[i + 0] == MessageHeader::firSync &&
            buffer[i + 1] == MessageHeader::sedSync &&
            buffer[i + 2] == MessageHeader::trdSync) {
            std::size_t byteUsed;
            auto messageItem = MessageItem::tryParseMessage(buffer + i, &byteUsed);
            if (messageItem != nullptr) {
                this->msgVector.push_back(messageItem);
            }
            i += byteUsed;
        } else {
            ++i;
        }
        std::cin.get();
    }
}
