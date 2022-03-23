/**
 * @file hfmtree.cpp
 * @author shlchen (3079625093@qq.com)
 * @version 0.1
 * @date 2022-03-23
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "include/hfmtree.h"

namespace ns_db {
  Node::Node(const code_type &code, const weight_type &weight)
      : _parent(-1), _lchild(-1), _rchild(-1),
        _code(code), _weight(weight), _hfmCode() {
  }

  std::ostream &operator<<(std::ostream &os, const Node &node) {
    os << '{';
    os << "'pa': " << node._parent << ", ";
    os << "'lc': " << node._lchild << ", ";
    os << "'rc': " << node._rchild << ", ";
    os << "'w': " << node._weight;
    os << '}';
    return os;
  }

  HFMTree::HFMTree(Node nodes[], std::size_t size)
      : _nodeSize(2 * size - 1) {
    this->_nodes = new Node[this->_nodeSize];
    // 初始化原节点
    for (int i = 0; i != this->_nodeSize; ++i) {
      this->_nodes[i]._parent = -1;
      this->_nodes[i]._lchild = -1;
      this->_nodes[i]._rchild = -1;
      if (i < size) {
        this->_nodes[i]._code = nodes[i]._code;
        this->_nodes[i]._weight = nodes[i]._weight;
      }
    }
    this->constructHFMT();
  }

  HFMTree::~HFMTree() {
    delete[] this->_nodes;
  }

  HFMTree &HFMTree::printHFMCode() {
    for (int i = 0; i != (this->_nodeSize + 1) / 2; ++i) {
      Node node = this->_nodes[i];
      std::cout << "'" << node._code << "': " << node._hfmCode << std::endl;
    }
    return *this;
  }

  void HFMTree::constructHFMT() {
    // 构建哈夫曼树
    for (int i = (this->_nodeSize + 1) / 2; i != this->_nodeSize; ++i) {
      auto [min1, min2] = this->minWeightNodes(0, i);
      this->_nodes[min1]._parent = i;
      this->_nodes[min2]._parent = i;
      this->_nodes[i]._lchild = min1;
      this->_nodes[i]._rchild = min2;
      this->_nodes[i]._weight = this->_nodes[min1]._weight + this->_nodes[min2]._weight;
    }
    // 生成哈夫曼编码
    for (int i = 0; i != (this->_nodeSize + 1) / 2; ++i) {
      int j = i;
      std::string hfmCode;
      // 一直找到最上边的节点
      while (this->_nodes[j]._parent != -1) {
        int p = this->_nodes[j]._parent;
        if (j == this->_nodes[p]._lchild) {
          hfmCode.push_back('0');
        } else {
          hfmCode.push_back('1');
        }
        j = p;
      }
      // 编码逆向
      for (int k = 0; k != hfmCode.size(); ++k) {
        this->_nodes[i]._hfmCode.push_back(hfmCode[hfmCode.size() - k - 1]);
      }
    }
    return;
  }

  std::pair<std::size_t, std::size_t> HFMTree::minWeightNodes(std::size_t start, std::size_t end) {
    int label = 0;
    // weight[min1] < weight[min2]
    std::size_t min1, min2;
    for (int i = start; i != end; ++i) {
      if (this->_nodes[i]._parent != -1) {
        continue;
      }
      if (label == 0) {
        min1 = i;
      } else if (label == 1) {
        min2 = i;
        if (this->_nodes[min1]._weight > this->_nodes[min2]._weight) {
          std::size_t temp = min1;
          min1 = min2;
          min2 = temp;
        }
      } else {
        if (this->_nodes[i]._weight < this->_nodes[min1]._weight) {
          min2 = min1;
          min1 = i;
        } else if (this->_nodes[i]._weight < this->_nodes[min2]._weight) {
          min2 = i;
        }
      }
      ++label;
    }
    return {min1, min2};
  }

  std::ostream &operator<<(std::ostream &os, const HFMTree &hfmt) {
    for (int i = 0; i != hfmt._nodeSize; ++i) {
      std::cout << i << ": " << hfmt._nodes[i] << std::endl;
    }
    return os;
  }
} // namespace ns_db
