#ifndef _CORE_SEMANTIC_MAP_INC_SEMANTIC_MAP_MANAGER_CONFIG_LOADER_H_
#define _CORE_SEMANTIC_MAP_INC_SEMANTIC_MAP_MANAGER_CONFIG_LOADER_H_

#include <assert.h>
#include <iostream>
#include <vector>

#include <Eigen/Geometry>
#include <Eigen/StdVector>

#include <json/json.hpp>

#include "common/basics/basics.h"
#include "common/basics/semantics.h"
#include "common/state/free_state.h"
#include "common/state/state.h"
#include "semantic_map_manager/basics.h"

namespace semantic_map_manager {

/**
 * @brief 按 ego ID 从 agent JSON 配置中提取 AgentConfigInfo。
 *
 * 类只保存配置文件路径和目标 ego ID，不缓存 JSON。ParseAgentConfig 每次重新打开并遍历
 * `agent_config.info`，匹配项会直接覆盖调用方提供的配置对象。
 */
class ConfigLoader {
 public:
  /// 默认构造；ego_id_ 和路径需由调用方在解析前设置。
  ConfigLoader() {}

  /// 只初始化 agent 配置文件路径，ego_id_ 仍需另行设置。
  ConfigLoader(const std::string &agent_config_path)
      : agent_config_path_(agent_config_path) {}

  /// 类不拥有外部资源，析构函数为空。
  ~ConfigLoader() {}

  /// 返回当前配置文件路径的值拷贝。
  inline std::string agent_config_path() const { return agent_config_path_; }

  /// 覆盖当前 agent JSON 配置文件路径。
  inline void set_agent_config_path(const std::string &path) {
    agent_config_path_ = path;
  };

  /// 返回待匹配 ego ID；默认构造后未赋值时该值未初始化。
  inline int ego_id() const { return ego_id_; }

  /// 设置 ParseAgentConfig 需要匹配的 ego ID。
  inline void set_ego_id(const int &id) { ego_id_ = id; }

  /// 打开 JSON 并把匹配 ego 的栅格、搜索半径和功能开关写入输出配置。
  ErrorType ParseAgentConfig(AgentConfigInfo *p_agent_config);

 private:
  // 目标 ego ID 与 agent JSON 文件路径。
  int ego_id_;
  std::string agent_config_path_;
};

}  // namespace semantic_map_manager

#endif  // _CORE_SEMANTIC_MAP_INC_SEMANTIC_MAP_MANAGER_CONFIG_LOADER_H_
