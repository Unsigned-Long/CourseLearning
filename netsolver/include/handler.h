#pragma once

#include <iostream>
#include <vector>
#include <string>
#include <unordered_map>

namespace ns_net
{
#pragma region base structures
    struct Arc
    {
        uint _from;
        uint _to;
        float _length;
        float _obvVal;
        float _resVal;

        Arc(uint from, uint to, float length, float obvVal, float resVal)
            : _from(from), _to(to), _length(length), _obvVal(obvVal), _resVal(resVal) {}
    };

    std::ostream &operator<<(std::ostream &os, const Arc &arc);

    struct Node
    {
        uint _id;
        float _height;

        Node(uint id, float height)
            : _id(id), _height(height) {}
    };

    std::ostream &operator<<(std::ostream &os, const Node &node);
#pragma endregion

#pragma region Handler

    class Handler
    {
    private:
        std::vector<Arc> _arcs;
        std::unordered_map<uint, Node> _baseNodes;
        std::unordered_map<uint, Node> _nodes;

    public:
        Handler(const std::vector<Arc> &arcs, const std::vector<Node> &baseNodes)
            : _arcs(arcs) { this->init(baseNodes); }

        Handler() = delete;

    public:
        static std::vector<Arc> readData(const std::string &filename);

        /**
         * @brief solve the net
         * @param outputLogs output the info matrix to the console if it's true
         */
        void solve(bool outputLogs = true);

        const auto &arcs() const { return this->_arcs; }

        const auto &baseNodes() const { return this->_baseNodes; }

        const auto &nodes() const { return this->_nodes; }

    protected:
        void init(const std::vector<Node> &baseNodes);

        /**
         * \brief a function to split a string to some string elements according the splitor
         * \param str the string to be splited
         * \param splitor the splitor char
         * \param ignoreEmpty whether ignoring the empty string element or not
         * \return the splited string vector
         */
        static std::vector<std::string> split(const std::string &str, char splitor, bool ignoreEmpty = true);

        /**
         * \brief a function to output info to the std::ostream
         * \param str the target string
         * \param ceil whether output the ceil line or not
         * \param floor whether output the floor line or not
         * \param symbol the char type to construct the lin
         * \param os the output stream type
         * \return void
         */
        void output(const std::string &str, bool ceil = true, bool floor = true,
                    char symbol = '-', std::ostream &os = std::cout);
    };
#pragma endregion
} // namespace ns_net
