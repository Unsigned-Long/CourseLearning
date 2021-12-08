#include "helper/include/prinComAnaly.hpp"
#include <random>
#include <fstream>

using testPCA = ns_ml::PCA<2, 2, int>;
using sample_type = testPCA::origin_sample_type;

int main(int argc, char const *argv[])
{
    std::default_random_engine e;
    std::normal_distribution<> n1(15.0, 2.0);
    std::normal_distribution<> n2(20.0, 5.0);
    Eigen::Matrix2d mat;
    mat(0, 0) = 1.0 / sqrt(2.0);
    mat(0, 1) = 1.0 / sqrt(2.0);
    mat(1, 0) = -1.0 / sqrt(2.0);
    mat(1, 1) = 1.0 / sqrt(2.0);
    auto set = testPCA::origin_sampleset_type();
    for (int i = 0; i != 500; ++i)
    {
        Eigen::Vector2d vec(n1(e), n2(e));
        Eigen::Vector2d vec2 = mat * vec;
        set.samples().push_back(sample_type({vec2(0), vec2(1)}));
    }
    std::ofstream file("../pyDrawer/pca/pca_origin.csv", std::ios::out);
    for (const auto &elem : set.samples())
        file << elem.valueAt(0) << ',' << elem.valueAt(1) << std::endl;
    auto res = testPCA::process(set);
    file.close();
    file.open("../pyDrawer/pca/pca_res.csv", std::ios::out);
    for (const auto &elem : res.samples())
        file << elem.valueAt(0) << ',' << elem.valueAt(1) << std::endl;
    file.close();
    return 0;
}
