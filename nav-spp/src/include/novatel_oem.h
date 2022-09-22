//
// Created by csl on 9/22/22.
//

#ifndef SPP_NOVATEL_OEM_H
#define SPP_NOVATEL_OEM_H

#include "data_parser.h"

namespace ns_spp {


    class NovAtelOEMFileHandler {
    private:
        Byte *buffer;
        std::size_t bufferSize;

        std::vector<std::shared_ptr<MessageItem>> msgVector;

    public:
        explicit NovAtelOEMFileHandler(const std::string &binFilePath);

        virtual ~NovAtelOEMFileHandler();

        [[nodiscard]] const Byte *getBuffer() const;


    };
}


#endif //SPP_NOVATEL_OEM_H
