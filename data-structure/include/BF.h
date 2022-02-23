#pragma once

#include "seqlist.h"

namespace ns_ds {
  using char_stream = ns_ds::SqeList<char>;
  int matchBF(char_stream &source, char_stream &target, int pos = 0) {
    if (target.empty() || source.empty() || source.length() - pos < target.length()) {
      return -1;
    }
    int i = pos, j = 0;
    char s, t;
    while (i < source.length() && j < target.length()) {
      source.getElem(i, s);
      target.getElem(j, t);
      if (s == t) {
        ++i, ++j;
      } else {
        i = i - j + 1;
        j = 0;
      }
      if (j == target.length()) {
        return i - j;
      }
    }
    return -1;
  }
} // namespace ns_ds
