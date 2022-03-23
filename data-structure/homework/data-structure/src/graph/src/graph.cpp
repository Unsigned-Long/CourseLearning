/**
 * @file graph.cpp
 * @author shlchen (3079625093@qq.com)
 * @version 0.1
 * @date 2022-03-23
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "include/graph.h"
#include <unordered_map>

namespace ns_db {

  UDGraph::UDGraph(UDGraphInfo info[], std::size_t size) {
    this->construct(info, size);
  }

  void UDGraph::construct(UDGraphInfo info[], std::size_t size) {
    std::unordered_map<vex_symbol_type, std::size_t> vex;
    std::size_t count = 0;
    // 找到一共存在的顶点并为其进行编号
    for (int i = 0; i != size; ++i) {
      if (vex.insert({info[i]._vex1, count}).second) {
        ++count;
      }
      if (vex.insert({info[i]._vex2, count}).second) {
        ++count;
      }
    }
    this->_vexSize = count;
    // 初始化顶点数组
    this->_vexArray = new Vex[this->_vexSize];
    for (const auto &[symbol, pos] : vex) {
      // 头节点, 不存储任何信息
      this->_vexArray[pos]._firArc = new Arc(0, nullptr);
      this->_vexArray[pos]._symbol = symbol;
    }

    // 构建邻接表
    for (int i = 0; i != size; ++i) {
      vex_symbol_type v1 = info[i]._vex1;
      vex_symbol_type v2 = info[i]._vex2;

      std::size_t v1Pos = vex.at(v1);
      std::size_t v2Pos = vex.at(v2);

      // 由于是无向图, 所以两个方向都要进行处理
      Arc *ptr = this->_vexArray[v1Pos]._firArc;
      while (ptr->_nextArc != nullptr) {
        ptr = ptr->_nextArc;
      }
      ptr->_nextArc = new Arc(v2Pos, nullptr);

      ptr = this->_vexArray[v2Pos]._firArc;
      while (ptr->_nextArc != nullptr) {
        ptr = ptr->_nextArc;
      }
      ptr->_nextArc = new Arc(v1Pos, nullptr);
    }
  }

  UDGraph &UDGraph::traverseDFS(traverse_fun fun) {
    // 用于记录节点是否被访问到
    bool *visited = new bool[this->_vexSize];
    for (int i = 0; i != this->_vexSize; ++i) {
      visited[i] = false;
    }
    for (int i = 0; i != this->_vexSize; ++i) {
      if (visited[i] == false) {
        this->traverseDFS(fun, i, visited);
      }
    }
    delete[] visited;
    return *this;
  }

  void UDGraph::traverseDFS(traverse_fun fun, std::size_t vexPos, bool visited[]) {
    Vex &curVex = this->_vexArray[vexPos];
    // 访问该节点
    fun(curVex);
    visited[vexPos] = true;
    Arc *ptr = curVex._firArc->_nextArc;
    // 递归访问
    while (ptr != nullptr) {
      if (visited[ptr->_adjVex] == false) {
        this->traverseDFS(fun, ptr->_adjVex, visited);
      }
      ptr = ptr->_nextArc;
    }
  }

  UDGraph::~UDGraph() {
    // 删除邻接表
    for (int i = 0; i != this->_vexSize; ++i) {
      Arc *ptr = this->_vexArray[i]._firArc;
      while (ptr != nullptr) {
        Arc *temp = ptr;
        ptr = ptr->_nextArc;
        delete temp;
      }
    }
    // 删除节点数组
    delete[] this->_vexArray;
  }

  std::ostream &operator<<(std::ostream &os, const UDGraph &graph) {
    for (int i = 0; i != graph._vexSize; ++i) {
      os << "{'pos': " << i << ", 'symbol': " << graph._vexArray[i]._symbol << ", 'links': [";
      Arc *ptr = graph._vexArray[i]._firArc->_nextArc;
      while (ptr != nullptr) {
        os << '(' << ptr->_adjVex << ")-";
        ptr = ptr->_nextArc;
      }
      os << "]}\n";
    }
    return os;
  }
} // namespace ns_db
