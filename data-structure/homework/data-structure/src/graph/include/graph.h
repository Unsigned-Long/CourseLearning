/**
 * @file graph.h
 * @author shlchen (3079625093@qq.com)
 * @brief
 * @version 0.1
 * @date 2022-03-22
 *
 * @copyright Copyright (c) 2022
 *
 */

#include <iostream>

namespace ns_db {
  /**
   * @brief 标识顶点的数据类型
   */
  using vex_symbol_type = char;

  /**
   * @brief 表示弧段的数据类型
   */
  struct Arc {
  public:
    // 指向的顶点
    std::size_t _adjVex;
    // 有共同出发点的下一弧段
    Arc *_nextArc;

    Arc(std::size_t adjVex, Arc *nextArc)
        : _adjVex(adjVex), _nextArc(nextArc) {}
  };
  struct Vex {
  public:
    // 顶点标识
    vex_symbol_type _symbol;
    // 所连弧段的头指针, 不存储任何数据
    Arc *_firArc;
  };

  /**
   * @brief 遍历函数类型
   */
  using traverse_fun = void (*)(Vex &);

  /**
   * @brief 帮助用户构建图的数据结构
   */
  struct UDGraphInfo {
  public:
    // 一个弧段两端的顶点标识
    vex_symbol_type _vex1;
    vex_symbol_type _vex2;
    UDGraphInfo(vex_symbol_type vex1, vex_symbol_type vex2)
        : _vex1(vex1), _vex2(vex2) {}
  };

  /**
   * @brief 无向图数据结构
   */
  class UDGraph {
  public:
    friend std::ostream &operator<<(std::ostream &os, const UDGraph &graph);

  private:
    // 表示顶点的数组
    Vex *_vexArray;
    // 顶点个数
    std::size_t _vexSize;

  public:
    UDGraph(UDGraphInfo info[], std::size_t size);

    ~UDGraph();
    /**
     * @brief 深度优先遍历
     *
     * @param fun 访问函数
     * @return UDGraph& 该无向图
     */
    UDGraph &traverseDFS(traverse_fun fun);

  protected:
    /**
     * @brief 构建无向图邻接表
     */
    void construct(UDGraphInfo info[], std::size_t size);

    /**
     * @brief 深度优先遍历的辅助函数
     */
    void traverseDFS(traverse_fun fun, std::size_t vexPos, bool visited[]);
  };

  /**
   * @brief 输出该无向图的相关信息
   */
  std::ostream &operator<<(std::ostream &os, const UDGraph &graph);

} // namespace ns_db
