#pragma once

#include "configInit.h"
#include "ceres/ceres.h"

namespace ns_res
{
    /**
     * \brief a ceres struct to calculate the undistortion image point
     */ 
    struct Ceres_RemoveDistortion
    {
        const ns_point::Point2d *_point;
        Ceres_RemoveDistortion(const ns_point::Point2d *point) : _point(point) {}

        bool operator()(const double *const undisPoint, double *out) const;
    };

    /**
     * \brief a ceres struct to calculate the H matrix
     */
    struct Ceres_CalHMatrix
    {
        const ns_point::Point2d *_imgPoint;
        const ns_point::Point2d *_objPoint;

        Ceres_CalHMatrix(const ns_point::Point2d *imgPoint, const ns_point::Point2d *objPoint)
            : _imgPoint(imgPoint), _objPoint(objPoint) {}

        bool operator()(const double *const h, double *out) const;
    };

    class Handler
    {
    public:
        static void process();

    private:
        Handler() = delete;

        // step one : remove distortion
        static void removeDistortion();

        // step two : from pixel to image
        static void cvtImgCoor();

        // step three : calculate H matrix
        static void calculateH();

        // step four : other mats[Rh, R, t, C]
        static void constructOtherMats();
    };
} // namespace ns_res
