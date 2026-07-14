/**
 * @file tool_func.cc
 * @author HKUST Aerial Robotics Group
 * @brief 实现字符串切分和整数候选集合笛卡尔积枚举。
 * @version 0.1
 * @date 2019-03-17
 *
 * @copyright Copyright (c) 2019
 */
#include "common/basics/tool_func.h"

namespace common {
void SplitString(const std::string& s, const std::string& c,
                 std::vector<std::string>* v) {
  // 逐次查找分隔符，追加当前分隔符之前的片段。
  std::string::size_type pos1, pos2;
  pos2 = s.find(c);
  pos1 = 0;
  while (std::string::npos != pos2) {
    v->push_back(s.substr(pos1, pos2 - pos1));

    pos1 = pos2 + c.size();
    pos2 = s.find(c, pos1);
  }
  // 字符串不以分隔符结束时，补充最后一个非空尾段。
  if (pos1 != s.length()) v->push_back(s.substr(pos1));
}

void GetResultInVector(const std::vector<std::vector<int>>& vec, const int& N,
                       std::vector<int>* tmp,
                       std::vector<std::vector<int>>* tmp_result) {
  // 在当前层依次选择一个元素，递归处理下一层并在返回时撤销选择。
  for (int i = 0; i < vec[N].size(); ++i) {
    tmp->push_back(vec[N][i]);
    if (N < vec.size() - 1) {
      GetResultInVector(vec, N + 1, tmp, tmp_result);
    } else {
      // 到达最后一层时复制当前回溯路径，形成一个完整组合。
      std::vector<int> one_result;
      for (int i = 0; i < tmp->size(); ++i) {
        one_result.push_back(tmp->at(i));
      }
      tmp_result->push_back(one_result);
    }
    tmp->pop_back();
  }
}

void GetAllCombinations(const std::vector<std::vector<int>>& vec_in,
                        std::vector<std::vector<int>>* res) {
  // tmp_vec 在递归过程中保存当前路径，res 收集所有叶节点组合。
  std::vector<int> tmp_vec;
  GetResultInVector(vec_in, 0, &tmp_vec, res);
}

}  // namespace common
