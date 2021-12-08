#pragma once

#include "ceres/ceres.h"
#include "UWBContext.h"
#include "point.h"

namespace ns_uwb
{
    /**
     * \brief a ceres cost function for solving the 3d position problem
     */ 
    struct CeresPosition
    {
        const UWBContext::DataItem *_dataItem;

        CeresPosition(const UWBContext::DataItem *dataItem)
            : _dataItem(dataItem) {}

        // the main solving function
        template <typename _Ty>
        bool operator()(const _Ty *const params, _Ty *out) const
        {
            auto &x = params[0];
            auto &y = params[1];
            auto &z = params[2];
            auto &baseStation = UWBContext::baseStation().at(this->_dataItem->_refStationID);

            out[0] = ceres::pow(x - baseStation.x(), 2) +
                     ceres::pow(y - baseStation.y(), 2) +
                     ceres::pow(z - baseStation.z(), 2) -
                     ceres::pow(this->_dataItem->_range, 2);
            return true;
        }
    };
    
    // calculate the 3d position refering to the data from 'start' to 'end'
    ns_point::Point3d ceresPosition(std::vector<UWBContext::DataItem>::const_iterator start,
                                    std::vector<UWBContext::DataItem>::const_iterator end);
} // namespace ns_uwb
