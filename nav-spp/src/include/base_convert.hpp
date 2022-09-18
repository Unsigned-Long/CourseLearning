//
// Created by csl on 9/18/22.
//

#ifndef SPP_BASE_CONVERT_HPP
#define SPP_BASE_CONVERT_HPP

#include "stack"
#include "string"
#include "exception"

namespace ns_spp {
    struct BaseConvert {
    public:
        enum Base : unsigned short {
            BIT = 2, OCT = 8, DEC = 10, HEX = 16
        };

        template<unsigned short Base, class DecType>
        static std::string decTo(DecType num) {
            std::stack<unsigned short> stack;
            while (num != 0) {
                stack.push(num % Base);
                num = num / Base;
            }
            std::string result(stack.size(), ' ');
            for (char &i: result) {
                i = i2cStr[stack.top()];
                stack.pop();
            }
            return result;
        }

        template<unsigned short Base, class DecType>
        static DecType toDec(const std::string &numStr) {
            DecType factor = 1;
            DecType result = 0;
            for (int i = numStr.size() - 1; i >= 0; --i) {
                DecType val;
                bool validChar = false;
                for (int j = 0; j < Base; ++j) {
                    if (toupper(numStr[i]) == i2cStr[j]) {
                        val = j, validChar = true;
                        break;
                    }
                }
                if (!validChar) {
                    throw std::runtime_error(
                            "wrong number string for base 'Base = " + std::to_string(Base)
                            + "' at 'pos = " + std::to_string(i) + "'"
                    );
                }
                result += val * factor;
                factor *= Base;
            }
            return result;
        }

    protected:
        static const std::string i2cStr;
    };

    const std::string BaseConvert::i2cStr = "0123456789ABCDEF";
}

#endif //SPP_BASE_CONVERT_HPP
