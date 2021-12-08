#pragma once

#include "legendre.h"
#include "colorPrj.h"

namespace ns_pygeo
{
#pragma region class SurPharmonic
    /**
     * \brief this is a display-class for visualation based on pcl library
     */
    class SurPharmonic
    {
    public:
        /**
         * \brief the function pointer
         * \param val the float value
         * \return float value
         */
        using tri_fun = float (*)(float val);

    private:
        SurPharmonic() = default;
        ~SurPharmonic() {}
        /**
         * \brief project the value range to rgb color space
         * \param val the value to project
         * \param min the low boundary of the value range
         * \param max the high boundary of the value range
         * \param tri_fun the trigonometric function
         * \param colorModel the display color type
         * \param classify the color grades
         */
        static void harmonic(int index_n, int order_m, float min, float max, SurPharmonic::tri_fun tri_fun,
                             const ns_clp::Color::ColorType &colorModel = ns_clp::Color::gray,
                             int classify = 0);

    public:
        // the R_nm function to display the SurPharmonic with pcl window [color]
        static void harmonic_Rnm(int index_n, int order_m,
                                 const ns_clp::Color::ColorType &colorModel = ns_clp::Color::gray,
                                 int classify = 0, float min = -3.5, float max = 3.5);
        static void harmonic_Rnm(const IndexOrder &i,
                                 const ns_clp::Color::ColorType &colorModel = ns_clp::Color::gray,
                                 int classify = 0, float min = -3.5, float max = 3.5);

        // the S_nm function to display the SurPharmonic with pcl window [color]
        static void harmonic_Snm(int index_n, int order_m,
                                 const ns_clp::Color::ColorType &colorModel = ns_clp::Color::gray,
                                 int classify = 0, float min = -3.5, float max = 3.5);
        static void harmonic_Snm(const IndexOrder &i,
                                 const ns_clp::Color::ColorType &colorModel = ns_clp::Color::gray,
                                 int classify = 0, float min = -3.5, float max = 3.5);
    };
#pragma endregion

} // namespace ns_ns_pygeo
