#include "handler.h"

namespace ns_dj
{
    std::pair<Dijkstra::path_type, float> Dijkstra::findPath(int startNode, int endNode) const
    {
        if (startNode == endNode)
            return std::make_pair(path_type(), 0.0);
        std::map<int, float> nodeCost;
        std::map<int, path_type> nodePath;
        std::set<int> nodeFound;
        nodeFound.insert(startNode);

        auto pathList = _pls;

        for (auto iter = pathList.cbegin(); iter != pathList.cend(); ++iter)
        {
            if (iter->_snode != startNode)
                nodeCost[iter->_snode] = __FLT_MAX__ / 2.0f;
            if (iter->_enode != startNode)
                nodeCost[iter->_enode] = __FLT_MAX__ / 2.0f;
        }

        int lastNode = startNode;
        float lastCost = 0.0;
        path_type lastPath = {startNode};

        while (true)
        {
            for (auto iter = pathList.cbegin(); iter != pathList.cend(); ++iter)
            {
                if ((iter->_snode == lastNode) &&
                    (nodeFound.find(iter->_enode) == nodeFound.cend()) &&
                    (iter->_cost + lastCost < nodeCost[iter->_enode]))
                {
                    nodeCost[iter->_enode] = iter->_cost + lastCost;
                    nodePath[iter->_enode] = lastPath;
                    nodePath[iter->_enode].push_back(iter->_enode);
                }
            }
            auto minIter = std::min_element(nodeCost.cbegin(), nodeCost.cend(), [](const auto &p1, const auto &p2)
                                            { return p1.second < p2.second; });
            lastNode = minIter->first;
            lastPath = nodePath[lastNode];
            lastCost = minIter->second;
            nodeFound.insert(lastNode);
            nodeCost.erase(minIter);
            if (lastNode == endNode)
                break;
        }

        return std::make_pair(lastPath, lastCost);
    }

} // namespace ns_dj
