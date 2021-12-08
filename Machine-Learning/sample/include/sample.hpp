#pragma once

#include <iostream>
#include <eigen3/Eigen/Dense>
#include <string>
#include <initializer_list>

namespace ns_ml
{

    template <std::size_t Dime, typename LabelType>
    class Sample
    {
    public:
        using label_type = LabelType;

        using value_type = Eigen::Vector<double, Dime>;

    private:
        value_type _values;

        label_type _label;
    public:
        /**
         * \brief constructors
         * [1] default construct;
         * [2] constructed by double[] and label_type();
         * [3] constructed by initializer_list<double> and label_type();
         */
        Sample() = default;

        Sample(const double values[Dime], const label_type &label = label_type())
            : _values(values), _label(label) {}

        Sample(const std::initializer_list<double> &values, const label_type &label = label_type())
            : _values(values.begin()), _label(label) {}

        /**
         * \brief get dime
         */
        std::size_t dime() const { return this->_values.rows(); }

        /**
         * \brief get value at index
         */
        double &valueAt(std::size_t index) { return this->_values(index, 0); }

        const double &valueAt(std::size_t index) const { return this->_values(index, 0); }

        value_type &values() { return this->_values; }

        const value_type &values() const { return this->_values; }

        /**
         * \brief get label
         */
        label_type &label() { return this->_label; }

        const label_type &label() const { return this->_label; }
    };

    /**
     * \brief overload operator '<<' for Sample
     */
    template <std::size_t Dime, typename LabelType>
    std::ostream &operator<<(std::ostream &os, const Sample<Dime, LabelType> &s)
    {
        std::string str("{'values': [");
        for (std::size_t i = 0; i != s.dime(); ++i)
            str += std::to_string(s.valueAt(i)) + ", ";
        str.pop_back();
        str.pop_back();
        str += "], 'label': " + std::to_string(s.label()) + '}';
        os << str;
        return os;
    }

    /**
     * \brief examples for different problems
     * [1] classification
     * [2] regression
     */
    template <std::size_t Dime>
    using ClassifySample = Sample<Dime, int>;

    template <std::size_t Dime>
    using RegressSample = Sample<Dime, double>;

} // namespace ns_ml
