//
// Created by csl on 9/18/22.
//

#ifndef SPP_BASE_CAST_HPP
#define SPP_BASE_CAST_HPP

#include "stack"
#include "string"
#include "utils/enum_cast.hpp"

namespace ns_spp {

    struct BaseCast {
    public:
        template<unsigned short Base, class DecType>
        static std::string decTo(DecType num) {
            std::stack<unsigned short> stack;
            while (num != 0) {
                stack.push(num % Base);
                num = num / Base;
            }
            std::string result(stack.size(), ' ');
            for (char &i: result) {
                i = EnumCast::integerToString<BaseChar>(stack.top())[1];
                stack.pop();
            }
            return result;
        }

        template<unsigned short Base, class DecType>
        static DecType toDec(const std::string &numStr) {
            DecType factor = 1, result = 0;
            for (int i = static_cast<int>(numStr.size()) - 1; i >= 0; --i) {
                auto baseCharValue = EnumCast::stringToInteger<BaseChar>(
                        std::string("_") + static_cast<char>(std::toupper(numStr[i]))
                );
                result += baseCharValue * factor;
                factor *= Base;
            }
            return result;
        }

    protected:
        enum class BaseChar {
            _0 = 0, _1, _2, _3, _4, _5, _6, _7, _8, _9, _A, _B, _C, _D, _E, _F
        };
    };
}

#endif //SPP_BASE_CAST_HPP
