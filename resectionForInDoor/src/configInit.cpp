#include "configInit.h"
#include "rapidjson/document.h"

namespace ns_res
{
    Eigen::Matrix4d ConfigInit::Rt_pl = Eigen::Matrix4d::Identity();

    Eigen::Matrix3d ConfigInit::H = Eigen::Matrix3d::Identity();

    Eigen::Matrix3d ConfigInit::R = Eigen::Matrix3d::Identity();

    Eigen::Vector3d ConfigInit::t = Eigen::Vector3d();

    Eigen::Vector3d ConfigInit::C = Eigen::Vector3d();

    Eigen::Matrix3d ConfigInit::Rh = Eigen::Matrix3d::Identity();

    std::vector<ns_point::Point2d> ConfigInit::objPoints = std::vector<ns_point::Point2d>(4);

    std::vector<ns_point::Point2d> ConfigInit::imgPoints = std::vector<ns_point::Point2d>(4);

    double ConfigInit::f = 0.0;
    double ConfigInit::cx = 0.0;
    double ConfigInit::cy = 0.0;

    double ConfigInit::k1 = 0.0;
    double ConfigInit::k2 = 0.0;
    double ConfigInit::k3 = 0.0;

    double ConfigInit::p1 = 0.0;
    double ConfigInit::p2 = 0.0;

    int ConfigInit::itemNum = 0;

    void ConfigInit::init(const std::string &config_directory)
    {
        std::string dir = config_directory + '/';
        ConfigInit::init_Rt_pl(dir + "plane2world.json");
        ConfigInit::init_points(dir + "objPoints.json", ConfigInit::objPoints);
        ConfigInit::init_points(dir + "imgPoints.json", ConfigInit::imgPoints);
        if (ConfigInit::imgPoints.size() != ConfigInit::objPoints.size())
            throw std::runtime_error("the size of object and image points is wrong");
        ConfigInit::itemNum = imgPoints.size();
        ConfigInit::init_inner_params(dir + "innerParams.json");
        return;
    }

    std::string ConfigInit::readString(std::fstream &file)
    {
        file.seekg(0, std::ios::end);
        auto size = file.tellg();
        file.seekg(0, std::ios::beg);
        std::string str(size, ' ');
        file.read(const_cast<char *>(str.c_str()), size);
        return str;
    }

    void ConfigInit::init_Rt_pl(const std::string &filename)
    {
        std::fstream file(filename, std::ios::in);
        if (!file.is_open())
            throw std::runtime_error("config file [plane2world.json] not found");
        auto str = ConfigInit::readString(file);
        file.close();
        // for json analysis
        using namespace rapidjson;
        Document doc;
        doc.Parse(str.c_str());
        const auto &rot_colm1 = doc["rot_colm1"].GetArray();
        const auto &rot_colm2 = doc["rot_colm2"].GetArray();
        const auto &rot_colm3 = doc["rot_colm3"].GetArray();
        const auto &trans_colm = doc["trans_colm"].GetArray();
        for (int i = 0; i != 3; ++i)
        {
            Rt_pl(i, 0) = rot_colm1[i].GetDouble();
            Rt_pl(i, 1) = rot_colm2[i].GetDouble();
            Rt_pl(i, 2) = rot_colm3[i].GetDouble();
            Rt_pl(i, 3) = trans_colm[i].GetDouble();
        }
        return;
    }

    void ConfigInit::init_points(const std::string &filename, std::vector<ns_point::Point2d> &vec)
    {
        std::fstream file(filename, std::ios::in);
        if (!file.is_open())
            throw std::runtime_error("config file [corners.json | pixels.json] not found");
        auto str = ConfigInit::readString(file);
        file.close();
        // for json analysis
        using namespace rapidjson;
        Document doc;
        doc.Parse(str.c_str());
        const auto &lowerLeft = doc["lowerLeft"].GetArray();
        const auto &upperLeft = doc["upperLeft"].GetArray();
        const auto &upperRight = doc["upperRight"].GetArray();
        const auto &lowerRight = doc["lowerRight"].GetArray();

        vec.at(0).x() = lowerLeft[0].GetDouble();
        vec.at(0).y() = lowerLeft[1].GetDouble();

        vec.at(1).x() = upperLeft[0].GetDouble();
        vec.at(1).y() = upperLeft[1].GetDouble();

        vec.at(2).x() = upperRight[0].GetDouble();
        vec.at(2).y() = upperRight[1].GetDouble();

        vec.at(3).x() = lowerRight[0].GetDouble();
        vec.at(3).y() = lowerRight[1].GetDouble();

        return;
    }

    void ConfigInit::init_inner_params(const std::string &filename)
    {
        std::fstream file(filename, std::ios::in);
        if (!file.is_open())
            throw std::runtime_error("config file [corners.json | pixels.json] not found");
        auto str = ConfigInit::readString(file);
        file.close();
        // for json analysis
        using namespace rapidjson;
        Document doc;
        doc.Parse(str.c_str());
        ConfigInit::f = doc["f"].GetDouble();
        ConfigInit::cx = doc["cx"].GetDouble();
        ConfigInit::cy = doc["cy"].GetDouble();

        ConfigInit::k1 = doc["k1"].GetDouble();
        ConfigInit::k2 = doc["k2"].GetDouble();
        ConfigInit::k3 = doc["k3"].GetDouble();
        ConfigInit::p1 = doc["p1"].GetDouble();
        ConfigInit::p2 = doc["p2"].GetDouble();

        return;
    }

    void ConfigInit::info()
    {
        console("Rt[plane2world] matrix");
        std::cout << ConfigInit::Rt_pl << std::endl;
        console("image points");
        for (const auto &imgPoint : imgPoints)
            std::cout << imgPoint << ',';
        std::cout << std::endl;
        console("object points");
        for (const auto &objPoint : objPoints)
            std::cout << objPoint << ',';
        std::cout << std::endl;
        console("inner params");
        std::cout << " f : " << f << std::endl;
        std::cout << "cx : " << cx << std::endl;
        std::cout << "cy : " << cy << std::endl;
        std::cout << "k1 : " << k1 << std::endl;
        std::cout << "k2 : " << k2 << std::endl;
        std::cout << "k3 : " << k3 << std::endl;
        std::cout << "p1 : " << p1 << std::endl;
        std::cout << "p2 : " << p2 << std::endl;
        return;
    }

    void console(const std::string &str, bool upLine, bool downLine)
    {
        if (upLine)
            std::cout << std::string(str.size(), '-') << std::endl;
        std::cout << str << std::endl;
        if (downLine)
            std::cout << std::string(str.size(), '-') << std::endl;
    }
} // namespace ns_res