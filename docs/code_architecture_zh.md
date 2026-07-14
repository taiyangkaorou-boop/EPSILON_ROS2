# EPSILON ROS2 代码架构与职责索引

## 1. 文档目的

本文档记录项目自有代码的职责边界、主要数据流和中文注释覆盖状态。它与源码内的
Doxygen/docstring 注释配套使用：源码解释单个文件、类和函数，本文档解释包之间的
协作关系。后续研究代码必须先确定所属职责域，再通过接口连接，禁止把 belief、风险
评估、语义地图和 QP 求解逻辑堆叠到集成入口中。

注释范围包括 `app/`、`aux_tools/`、`core/` 和 `util/` 下的项目自有源码、启动文件、
构建文件及配置。`thirdparty/`、自动生成的 protobuf/ROS 消息代码、模型/图片等二进制
资源不直接改写，只记录其用途和调用边界。

## 2. 在线规划主数据流

```text
物理仿真器/环境话题
        |
        v
semantic_map_manager::RosAdapter
        |
        v
SemanticMapManager（统一车道、车辆和任务语义）
        |
        +-----------------------------+
        |                             |
        v                             v
BehaviorPlannerServer (MPDM)     EudmPlannerServer (EUDM)
        |                             |
        +-------------+---------------+
                      v
            带行为决策的语义地图
                      |
                      v
              SscPlannerServer
                      |
                      v
              连续轨迹/控制话题
```

`planning_integrated` 只选择 MPDM 或 EUDM 行为服务器并连接上述数据流。行为层之间的
公平比较必须共享语义地图、SSC、车辆模型、碰撞检查和控制接口。

## 3. M0.1：系统入口与辅助工具

| 文件 | 职责 | 非职责 |
|---|---|---|
| `app/planning_integrated/src/test_ssc_with_eudm.cc` | 创建 EUDM、语义地图和 SSC 服务器，连接回调并管理生命周期 | 不实现 EUDM、预测或 QP 算法 |
| `app/planning_integrated/src/test_ssc_with_mpdm.cc` | 创建 MPDM、语义地图和 SSC 服务器；提供同运动层对照入口 | 不改变 MPDM/SSC 内部代价与约束 |
| `app/planning_integrated/launch/*_launch.py` | 声明场景、参数路径、话题重映射并启动 ROS2 节点 | 不保存实验结果，不承担参数校准 |
| `app/planning_integrated/launch/*.launch` | ROS1 XML 风格历史入口，用于迁移核对 | 不是 ROS2 Humble 正式启动入口 |
| `app/planning_integrated/CMakeLists.txt` | 编译两个集成可执行文件并连接规划/数值依赖 | 不构建独立算法库 |
| `app/planning_integrated/package.xml` | 声明集成包依赖和 ament 构建类型 | 不定义消息接口 |
| `aux_tools/src/terminal_server.py` | 键盘 HMI、Joy 消息发布和二维俯视可视化 | 不参与规划、安全判定或定量评测 |
| `aux_tools/CMakeLists.txt` | 注册辅助包及预留 C++ 依赖 | 当前不生成 C++ 可执行文件 |
| `aux_tools/package.xml` | 声明辅助脚本的 ROS2 消息依赖 | 不声明规划器算法依赖 |
| `aux_tools/src/steer_wheel.png` | 方向盘显示资源 | 不包含可执行逻辑 |

## 4. 注释覆盖与研究开发规则

- [x] M0.1 系统入口、launch 和辅助工具。
- [ ] M0.2 `core/common` 公共类型、数学、轨迹与求解器。
- [ ] M0.3 语义地图、前向仿真、预测和行为规划。
- [ ] M0.4 SSC、车辆模型、物理仿真、playground 与配置。
- [ ] M0.5 全仓覆盖审计和遗漏补齐。

后续算法任务使用固定 `dev` 分支；每个小任务必须满足：工作树范围清晰、静态检查
通过、提交信息包含模块名、创建 annotated tag、推送提交和标签，并在本索引中更新
新增模块的数据输入、输出、状态所有权和失败回退边界。
