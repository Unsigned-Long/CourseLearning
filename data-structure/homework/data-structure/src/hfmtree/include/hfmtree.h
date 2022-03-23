/**
 * @file hfmtree.h
 * @author shlchen (3079625093@qq.com)
 * @version 0.1
 * @date 2022-03-22
 *
 * @copyright Copyright (c) 2022
 *
 */

#pragma once

#include <iostream>

namespace ns_db {
/**
 * @brief 定义宏,用于在哈夫曼树中方便的抛出异常
 */
#define THROW(where, what) \
  throw std::runtime_error(std::string("[ error from 'HFMTree'-'") + #where + "' ] " + what)

  /**
   * @brief
   * weight_type 权值的类型
   * code_type 编码字图类型
   */
  using weight_type = float;
  using code_type = char;

  /**
   * @brief 组织哈夫曼树中的节点的数据结构
   */
  struct Node {
  public:
    // 节点父亲
    int _parent;
    // 节点左右孩子
    int _lchild;
    int _rchild;
    // 节点码
    code_type _code;
    // 节点权重
    weight_type _weight;
    // 节点哈夫曼编码
    std::string _hfmCode;

    /**
     * @brief 构造一个节点
     *
     * @param code 节点码
     * @param weight 节点权重
     */
    Node(const code_type &code, const weight_type &weight);

    Node() = default;
  };

  /**
   * @brief 输出节点的信息
   */
  std::ostream &operator<<(std::ostream &os, const Node &node);

  /**
   * @brief 哈夫曼树的数据结构
   *
   */
  class HFMTree {
  public:
    friend std::ostream &operator<<(std::ostream &os, const HFMTree &hfmt);

  private:
    // 节点数组
    Node *_nodes;
    // 哈夫曼树中节点总个数
    std::size_t _nodeSize;

  public:
    /**
     * @brief 构造一颗哈夫曼树
     *
     * @param nodes 节点
     * @param size 节点个数
     */
    HFMTree(Node nodes[], std::size_t size);

    /**
     * @brief 析构函数
     */
    ~HFMTree();

    /**
     * @brief 打印最后生成的哈夫曼编码
     */
    HFMTree &printHFMCode();

  protected:
    /**
     * @brief 构建哈夫曼树
     */
    void constructHFMT();

    /**
     * @brief 求从start到end的权值最小的两个节点(没有父亲)
     */
    std::pair<std::size_t, std::size_t> minWeightNodes(std::size_t start, std::size_t end);
  };

  /**
   * @brief 输出哈夫曼树的构造细节
   */
  std::ostream &operator<<(std::ostream &os, const HFMTree &hfmt);

} // namespace ns_db
