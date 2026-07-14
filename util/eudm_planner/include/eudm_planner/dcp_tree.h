/**
 * @file dcp_tree.h
 * @author HKUST Aerial Robotics Group
 * @brief 生成 EUDM 离散纵横向动作候选脚本。
 * @version 0.1
 * @date 2019-07-07
 *
 * @copyright Copyright (c) 2019
 *
 */

#ifndef _CORE_EUDM_PLANNER_INC_EUDM_PLANNER_BEHAVIOR_TREE_H_
#define _CORE_EUDM_PLANNER_INC_EUDM_PLANNER_BEHAVIOR_TREE_H_

#include <map>
#include <memory>
#include <string>

#include "common/basics/basics.h"
#include "common/basics/semantics.h"

namespace planning {

/**
 * @brief 枚举恒定纵向动作和至多一次横向行为切换的有限深度 DCP 脚本。
 *
 * 每条脚本包含 tree_height 个 DcpAction。首层继承 ongoing action 的横向行为/剩余时间，
 * 纵向动作从保持、加速、减速中枚举；后续横向行为可一直保持，或在某一层切到左/右/保持
 * 中另一个动作并维持到时域结束。最后一层时长可独立配置。
 */
class DcpTree {
 public:
  using LateralBehavior = common::LateralBehavior;

  /// DCP 纵向离散动作。
  enum class DcpLonAction {
    kMaintain = 0,
    kAccelerate,
    kDecelerate,
    MAX_COUNT = 3
  };

  /// DCP 横向离散动作。
  enum class DcpLatAction {
    kLaneKeeping = 0,
    kLaneChangeLeft,
    kLaneChangeRight,
    MAX_COUNT = 3
  };

  /// 单层联合动作及其持续时间。
  struct DcpAction {
    DcpLonAction lon = DcpLonAction::kMaintain;
    DcpLatAction lat = DcpLatAction::kLaneKeeping;

    decimal_t t = 0.0;

    /// 以整数枚举值和时长输出动作，供日志诊断。
    friend std::ostream& operator<<(std::ostream& os, const DcpAction& action) {
      os << "(lon: " << static_cast<int>(action.lon)
         << ", lat: " << static_cast<int>(action.lat) << ", t: " << action.t
         << ")";
      return os;
    }

    /// 构造保持纵向、保持车道、零时长动作。
    DcpAction() {}
    /// 使用指定纵向动作、横向动作和持续时间构造。
    DcpAction(const DcpLonAction& lon_, const DcpLatAction& lat_,
              const decimal_t& t_)
        : lon(lon_), lat(lat_), t(t_) {}
  };

  /// 使用统一 layer_time 构造并立即生成候选脚本。
  DcpTree(const int& tree_height, const decimal_t& layer_time);
  /// 允许最后一层使用独立时长，构造后立即生成候选脚本。
  DcpTree(const int& tree_height, const decimal_t& layer_time,
          const decimal_t& last_layer_time);
  ~DcpTree() = default;

  /// 更新当前正在执行的联合动作；需调用 UpdateScript 才会重建候选。
  void set_ongoing_action(const DcpAction& a) { ongoing_action_ = a; }

  /// 按值返回全部候选动作序列。
  std::vector<std::vector<DcpAction>> action_script() const {
    return action_script_;
  }

  /// 返回第一条脚本各层时长之和；无脚本时返回 0。
  decimal_t planning_horizon() const;

  /// 返回配置树高。
  int tree_height() const { return tree_height_; }

  /// 返回普通层持续时间。
  decimal_t sim_time_per_layer() const { return layer_time_; }

  /// 使用当前 ongoing action 和配置重新生成脚本。
  ErrorType UpdateScript();

  /// 把纵向动作转为日志缩写 M/A/D，未知值返回 Null。
  static std::string RetLonActionName(const DcpLonAction a) {
    std::string a_str;
    switch (a) {
      case DcpLonAction::kMaintain: {
        a_str = std::string("M");
        break;
      }
      case DcpLonAction::kAccelerate: {
        a_str = std::string("A");
        break;
      }
      case DcpLonAction::kDecelerate: {
        a_str = std::string("D");
        break;
      }
      default: {
        a_str = std::string("Null");
        break;
      }
    }
    return a_str;
  }

  /// 把横向动作转为日志缩写 K/L/R，未知值返回 Null。
  static std::string RetLatActionName(const DcpLatAction a) {
    std::string a_str;
    switch (a) {
      case DcpLatAction::kLaneKeeping: {
        a_str = std::string("K");
        break;
      }
      case DcpLatAction::kLaneChangeLeft: {
        a_str = std::string("L");
        break;
      }
      case DcpLatAction::kLaneChangeRight: {
        a_str = std::string("R");
        break;
      }
      default: {
        a_str = std::string("Null");
        break;
      }
    }
    return a_str;
  }

 private:
  /// 枚举全部纵向动作和单次横向切换时机，重建 action_script_。
  ErrorType GenerateActionScript();

  /// 复制输入序列，并把动作 a 追加 n 次后返回新序列。
  std::vector<DcpAction> AppendActionSequence(
      const std::vector<DcpAction>& seq_in, const DcpAction& a,
      const int& n) const;

  // 树高、普通层时长、末层时长和当前正在执行的动作。
  int tree_height_ = 5;
  decimal_t layer_time_ = 1.0;
  decimal_t last_layer_time_ = 1.0;
  DcpAction ongoing_action_;
  /// 候选脚本缓存。
  std::vector<std::vector<DcpAction>> action_script_;
};
}  // namespace planning

#endif  //  _CORE_EUDM_PLANNER_INC_EUDM_PLANNER_BEHAVIOR_TREE_H_
