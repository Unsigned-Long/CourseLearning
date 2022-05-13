#include "muti_thread.h"

namespace ns_fusion {
  std::vector<std::vector<cv::Mat>> gridding(cv::Mat img, std::size_t rows, std::size_t cols, bool clone) {
    std::vector<std::vector<cv::Mat>> grids(rows, std::vector<cv::Mat>(cols));
    int imgRows = img.rows, imgCols = img.cols;
    int gridWidth = imgCols / cols, gridHeight = imgRows / rows;
    // for loop
    for (int i = 0; i != rows; ++i) {
      for (int j = 0; j != cols; ++j) {
        int startRow = i * gridHeight, endRow;
        int startCol = j * gridWidth, endCol;
        if (i == rows - 1) {
          // last row
          endRow = imgRows;
        } else {
          endRow = (i + 1) * gridHeight;
        }
        if (j == cols - 1) {
          // last cols
          endCol = imgCols;
        } else {
          endCol = (j + 1) * gridWidth;
        }
        // assign
        if (clone) {
          grids[i][j] = img.rowRange(startRow, endRow).colRange(startCol, endCol).clone();
        } else {
          grids[i][j] = img.rowRange(startRow, endRow).colRange(startCol, endCol);
        }
      }
    }
    return grids;
  }

} // namespace ns_fusion
