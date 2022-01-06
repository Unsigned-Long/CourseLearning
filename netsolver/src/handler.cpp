#include "handler.h"
#include <fstream>
#include <algorithm>
#include "eigen3/Eigen/Dense"

namespace ns_net
{
#pragma region base structures
    std::ostream &operator<<(std::ostream &os, const Arc &arc)
    {
        os << "Arc{fromID': " << arc._from << ", 'toID': " << arc._to << ", 'len': "
           << arc._length << ", 'obv': " << arc._obvVal << ", 'res': " << arc._resVal << '}';
        return os;
    }

    std::ostream &operator<<(std::ostream &os, const Node &node)
    {
        os << "Node{'id': " << node._id << ", 'height': " << node._height << '}';
        return os;
    }
#pragma endregion

#pragma region Handler
    void Handler::init(const std::vector<Node> &baseNodes)
    {
        for (const auto &elem : this->_arcs)
        {
            this->_nodes.insert(std::make_pair(elem._from, Node(elem._from, 0.0)));
            this->_nodes.insert(std::make_pair(elem._to, Node(elem._to, 0.0)));
        }

        for (const auto &elem : baseNodes)
            this->_baseNodes.insert(std::make_pair(elem._id, Node(elem._id, elem._height)));

        for (const auto &elem : this->_baseNodes)
            this->_nodes.at(elem.first)._height = elem.second._height;

        return;
    }

    void Handler::solve(bool outputLogs)
    {

        std::unordered_map<uint, int> nodeOrder;
        int count = 0;

        for (const auto &[id, Node] : this->_nodes)
            if (this->_baseNodes.find(id) == this->_baseNodes.end())
                nodeOrder.insert(std::make_pair(id, count++));

        {
            /**
             * @brief just for output
             */
            if (outputLogs)
            {
                output("Arcs[Initialize arcs' information]");
                for (const auto &elem : _arcs)
                    std::cout << elem << std::endl;

                output("Base Nodes[Initialize base nodes' information]");

                for (const auto &elem : _baseNodes)
                    std::cout << elem.second << std::endl;

                output("Nodes[Initialize nodes' information]");

                for (const auto &elem : _nodes)
                    std::cout << elem.second << std::endl;

                output("Nodes Order[Used when constructing the matrix]");
                for (const auto &[id, order] : nodeOrder)
                    std::cout << "{'id': " << id << ", 'order': " << order << '}' << std::endl;
            }
        }

        Eigen::MatrixXf mat_B = Eigen::MatrixXf::Zero(this->_arcs.size(), this->_nodes.size() - this->_baseNodes.size());

        Eigen::MatrixXf mat_P = Eigen::MatrixXf::Zero(this->_arcs.size(), this->_arcs.size());

        Eigen::MatrixXf mat_L = Eigen::MatrixXf::Zero(this->_arcs.size(), 1);

        count = 0;
        for (const auto &elem : this->_arcs)
        {
            mat_P(count, count) = 1.0f / elem._length;

            auto fromIter = this->_baseNodes.find(elem._from);
            auto toIter = this->_baseNodes.find(elem._to);

            if (fromIter == this->_baseNodes.end())
            {
                if (toIter == this->_baseNodes.end())
                {
                    // from[not base], to[not base]
                    mat_B(count, nodeOrder.at(elem._from)) = -1.0f;
                    mat_B(count, nodeOrder.at(elem._to)) = 1.0f;
                    mat_L(count, 0) = elem._obvVal;
                }
                else
                {
                    // from[not base], to[base]
                    mat_B(count, nodeOrder.at(elem._from)) = -1.0f;
                    mat_L(count, 0) = elem._obvVal - _baseNodes.at(elem._to)._height;
                }
            }
            else
            {
                if (toIter == this->_baseNodes.end())
                {
                    // from[base], to[not base]
                    mat_B(count, nodeOrder.at(elem._to)) = 1.0f;
                    mat_L(count, 0) = elem._obvVal + _baseNodes.at(elem._from)._height;
                }
                else
                {
                    // from[base], to[base]
                    mat_L(count, 0) = elem._obvVal - _baseNodes.at(elem._to)._height + -_baseNodes.at(elem._from)._height;
                }
            }

            ++count;
        }

        {
            /**
             * @brief just for output
             */
            if (outputLogs)
            {
                output("Matrix P");
                std::cout << mat_P << std::endl;

                output("Matrix B");
                std::cout << mat_B << std::endl;

                output("Matrix L");
                std::cout << mat_L << std::endl;
            }
        }

        auto mat_X = (mat_B.transpose() * mat_P * mat_B).inverse() * (mat_B.transpose() * mat_P * mat_L);

        auto mat_V = mat_B * mat_X - mat_L;

        {
            /**
             * @brief just for output
             */
            if (outputLogs)
            {
                output("Matrix X");
                std::cout << mat_X << std::endl;

                output("Matrix V");
                std::cout << mat_V << std::endl;
            }
        }

        for (auto &[id, Node] : this->_nodes)
            if (this->_baseNodes.find(id) == this->_baseNodes.end())
                Node._height = mat_X(nodeOrder.at(id), 0);
        
        {
            /**
             * @brief just for output
             */
            if (outputLogs)
            {
                output("Nodes[Final]");
                for (const auto &elem : _nodes)
                    std::cout << elem.second << std::endl;
            }
        }

        return;
    }

    std::vector<Arc> Handler::readData(const std::string &filename)
    {
        std::fstream file(filename, std::ios::in);
        if (!file.is_open())
            throw std::runtime_error("open arcs file failed");

        std::vector<Arc> arcs;
        std::string strLine;
        while (std::getline(file, strLine))
        {
            auto vec = split(strLine, ',');
            arcs.push_back(Arc(
                static_cast<uint>(std::stoi(vec[0])),
                static_cast<uint>(std::stoi(vec[1])),
                std::stof(vec[2]),
                std::stof(vec[3]),
                0.0f));
        }
        file.close();

        return arcs;
    }

    std::vector<std::string> Handler::split(const std::string &str, char splitor, bool ignoreEmpty)
    {
        std::vector<std::string> vec;
        auto iter = str.cbegin();
        while (true)
        {
            auto pos = std::find(iter, str.cend(), splitor);
            auto elem = std::string(iter, pos);
            if ((!ignoreEmpty) || (ignoreEmpty && !elem.empty()))
                vec.push_back(elem);
            if (pos == str.cend())
                break;
            iter = ++pos;
        }
        return vec;
    }

    void Handler::output(const std::string &str, bool ceil, bool floor,
                         char symbol, std::ostream &os)
    {
        if (ceil)
            os << std::string(str.length(), symbol) << std::endl;
        os << str << std::endl;
        if (floor)
            os << std::string(str.length(), symbol) << std::endl;
        return;
    }
#pragma endregion
} // namespace ns_net