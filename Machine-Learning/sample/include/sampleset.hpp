#pragma once

#include "sample.hpp"
#include <vector>
#include "sampleProperty.hpp"

namespace ns_ml
{
    template <std::size_t Dime, typename LabelType>
    class SampleSet
    {
    public:
        using sample_type = ns_ml::Sample<Dime, LabelType>;

        using sampleProp_type = ns_ml::SamplePrope<Dime>;

    private:
        std::vector<sample_type> _samples;
        sampleProp_type _props;

    public:
        SampleSet(const sampleProp_type &props = sampleProp_type::continuous())
            : _samples(std::vector<sample_type>()), _props(props){};

        SampleSet(std::size_t size, const sampleProp_type &props = sampleProp_type::continuous(), const sample_type &sample = sample_type())
            : _samples(size, sample), _props(props) {}

        SampleSet(const std::initializer_list<sample_type> &samples, const sampleProp_type &props = sampleProp_type::continuous())
            : _samples(samples.begin(), samples.end()), _props(props) {}

        /**
         * \brief get samples vector
         */
        std::vector<sample_type> &samples() { return this->_samples; }

        const std::vector<sample_type> &samples() const { return this->_samples; }

        /**
         * \brief get property array
         */
        const auto &properties() const { return this->_props._props; }

        Eigen::VectorXd featuerVectorAt(std::size_t index) const
        {
            auto dime = this->_samples.size();
            Eigen::VectorXd vec(dime);
            for (int i = 0; i != this->_samples.size(); ++i)
                vec(i) = this->_samples.at(i).valueAt(index);
            return vec;
        }

        /**
         * \brief output the result to the ostream
         */
        void output(std::ostream &os = std::cout)
        {
            os << *this;
            return;
        }
    };
    /**
     * \brief output the result to the ostream
     */
    template <std::size_t Dime, typename LabelType>
    std::ostream &operator<<(std::ostream &os, const SampleSet<Dime, LabelType> &set)
    {
        for (const auto &elem : set.samples())
            os << elem << std::endl;
        return os;
    }
} // namespace ns_ml
