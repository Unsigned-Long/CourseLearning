#pragma once

#include "UWBContext.h"

namespace ns_uwb
{
    ns_point::Point3d linear(std::vector<UWBContext::DataItem>::const_iterator start,
                             std::vector<UWBContext::DataItem>::const_iterator end);
} // namespace ns_uwb
