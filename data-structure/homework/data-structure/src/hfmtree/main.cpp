/**
 * @file main.cpp
 * @author shlchen (3079625093@qq.com)
 * @version 0.1
 * @date 2022-03-23
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "include/hfmtree.h"

int main(int argc, char const *argv[]) {
  try {
    /**
     * {a, b, c, d, e, f, g, h}
     * {0.07, 0.19, 0.02, 0.06, 0.32, 0.03, 0.21, 0.10}
     */
    ns_db::Node nodes[] = {
        ns_db::Node('a', 0.07f), ns_db::Node('b', 0.19f), ns_db::Node('c', 0.02f), ns_db::Node('d', 0.06f),
        ns_db::Node('e', 0.32f), ns_db::Node('f', 0.03f), ns_db::Node('g', 0.21f), ns_db::Node('h', 0.10f)};
    // 构建哈夫曼数
    ns_db::HFMTree hfmt(nodes, 8);
    // 输出哈夫曼树的内部构造数据
    std::cout << hfmt << std::endl;
    /**
     * @brief
     *
     * 0: {'pa': 10, 'lc': -1, 'rc': -1, 'w': 0.07}
     * 1: {'pa': 12, 'lc': -1, 'rc': -1, 'w': 0.19}
     * 2: {'pa': 8, 'lc': -1, 'rc': -1, 'w': 0.02}
     * 3: {'pa': 9, 'lc': -1, 'rc': -1, 'w': 0.06}
     * 4: {'pa': 13, 'lc': -1, 'rc': -1, 'w': 0.32}
     * 5: {'pa': 8, 'lc': -1, 'rc': -1, 'w': 0.03}
     * 6: {'pa': 12, 'lc': -1, 'rc': -1, 'w': 0.21}
     * 7: {'pa': 10, 'lc': -1, 'rc': -1, 'w': 0.1}
     * 8: {'pa': 9, 'lc': 2, 'rc': 5, 'w': 0.05}
     * 9: {'pa': 11, 'lc': 8, 'rc': 3, 'w': 0.11}
     * 10: {'pa': 11, 'lc': 0, 'rc': 7, 'w': 0.17}
     * 11: {'pa': 13, 'lc': 9, 'rc': 10, 'w': 0.28}
     * 12: {'pa': 14, 'lc': 1, 'rc': 6, 'w': 0.4}
     * 13: {'pa': 14, 'lc': 11, 'rc': 4, 'w': 0.6}
     * 14: {'pa': -1, 'lc': 12, 'rc': 13, 'w': 1}
     */

    // 打印最后的哈夫曼编码
    hfmt.printHFMCode();
    /**
     * @brief output
     * 'a': 0101
     * 'b': 00
     * 'c': 00001
     * 'd': 1001
     * 'e': 11
     * 'f': 10001
     * 'g': 10
     * 'h': 1101
     *
     */
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
  }

  return 0;
}
