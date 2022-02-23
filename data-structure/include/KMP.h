#pragma once

#include "seqlist.h"
#include <vector>

namespace ns_ds {
  void next_map(char_stream &target, int *next) {
    next[0] = -1;
    int i = 1, j = 0;
    char elem1, elem2;
    while (i != target.length()) {
      if (next[j] == -1) {
        next[i] = 0;
        j = i, ++i;
      } else {
        target.getElem(i - 1, elem1);
        j = next[j];
        target.getElem(j, elem2);
        if (elem1 == elem2) {
          next[i] = j + 1;
          j = i, ++i;
        }
      }
    }
    return;
  }
  using char_stream = ns_ds::SqeList<char>;
  int matchKMP(char_stream &source, char_stream &target, int pos = 0) {
    if (target.empty() || source.empty() || source.length() - pos < target.length()) {
      return -1;
    }
    int i = pos, j = 0, result = -1;
    char s, t;
    int *next = new int[target.length()];
    ns_ds::next_map(target, next);
    while (i < source.length() && j < target.length()) {
      source.getElem(i, s);
      target.getElem(j, t);
      if (s == t) {
        ++i, ++j;
        if (j == target.length()) {
          result = i - j;
          break;
        }
      } else {
        if (j == 0) {
          ++i;
        } else {
          j = next[j];
        }
      }
    }
    delete[] next;
    return result;
  }
} // namespace ns_ds
