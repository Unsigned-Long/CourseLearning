#pragma once
#include <array>
#include <algorithm>
#include <initializer_list>

namespace ns_ml
{
    enum class FeatuerProp
    {
        CONTINUOUS,
        DISCRETE
    };

    template <std::size_t DIME>
    struct SamplePrope
    {
        std::array<FeatuerProp, DIME> _props;

        SamplePrope() = delete;
        SamplePrope(const std::array<FeatuerProp, DIME> &props)
            : _props(props) {}

        SamplePrope(const std::initializer_list<FeatuerProp> &ls) { std::copy(ls.begin(), ls.end(), _props.begin()); }

        static SamplePrope<DIME> continuous()
        {
            std::array<FeatuerProp, DIME> props;
            std::fill(props.begin(), props.end(), FeatuerProp::CONTINUOUS);
            return SamplePrope<DIME>(props);
        }

        static SamplePrope<DIME> discrete()
        {
            std::array<FeatuerProp, DIME> props;
            std::fill(props.begin(), props.end(), FeatuerProp::DISCRETE);
            return SamplePrope<DIME>(props);
        }
    };

} // namespace ns_ml
