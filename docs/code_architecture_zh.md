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
- [x] M0.2a1 `core/common` 通用配置、宏、计时、线程池、色图与工具函数。
- [x] M0.2a2 `core/common` 几何类型与碰撞/投影工具。
- [x] M0.2a3a `semantics` 车辆、行为概率、语义车辆与控制信号。
- [x] M0.2a3b1 `semantics` GridMapMetaInfo/GridMapND。
- [x] M0.2a3b2 `semantics` 车道、障碍物与 KD-tree 适配。
- [x] M0.2a3c `semantics` SSC cube/corridor、交通信号与枚举工具。
- [x] M0.2a4a `core/common` State/FreeState/FrenetState/Waypoint/StateTransformer。
- [ ] M0.2a4b `core/common` Lane 与 LaneGenerator。
- [ ] M0.2b `core/common` 数学、样条、轨迹与圆弧。
- [ ] M0.2c `core/common` 求解器、安全模型、车辆行为模型与可视化。
- [ ] M0.3 语义地图、前向仿真、预测和行为规划。
- [ ] M0.4 SSC、车辆模型、物理仿真、playground 与配置。
- [ ] M0.5 全仓覆盖审计和遗漏补齐。

后续算法任务使用固定 `dev` 分支；每个小任务必须满足：工作树范围清晰、静态检查
通过、提交信息包含模块名、创建 annotated tag、推送提交和标签，并在本索引中更新
新增模块的数据输入、输出、状态所有权和失败回退边界。

## 5. M0.2a1：公共通用工具

| 组件 | 职责与边界 |
|---|---|
| `basics.h` | 统一误差码、双精度数值类型、Eigen 对齐容器和数值容差；不承载业务状态 |
| `config.h` | 固定车道/轨迹多项式阶数与空间维度；后续若改为运行时配置需独立迁移 |
| `macros.h` | 注册 backward-cpp 崩溃回溯；每个可执行文件只应展开一次 |
| `tic_toc.h` | 记录墙钟耗时；不等同于 ROS 仿真时间或确定性 deadline 监控 |
| `thread_pool.h` | FIFO 异步任务执行与 future 返回；任务队列状态由互斥锁和条件变量保护 |
| `colormap.*` | 数值/名称到 ARGB 颜色的可视化映射；不参与风险数值计算 |
| `tool_func.*` | 字符串切分、笛卡尔积、数值格式化和区间采样；调用方负责输入前置条件 |

## 6. M0.2a2：几何对象与碰撞工具

| 组件 | 职责与边界 |
|---|---|
| `Point` / `Point2i` | 世界坐标点与整数栅格/像素点；不隐式执行坐标系转换 |
| `OrientedBoundingBox2D` | 使用中心、航向、宽度和长度描述车辆等有向矩形 |
| `AxisAlignedBoundingBoxND` | 使用中心和各维长度描述 N 维 AABB |
| `AxisAlignedCubeNd` | 使用各维上下界描述 N 维超立方体，是 SSC cube 的基础容器 |
| `Circle` / `PolyLine` / `Polygon` | 障碍物和地图几何的轻量数据结构，不自行验证拓扑合法性 |
| `ShapeUtils` | OBB 分离轴碰撞、AABB 包含/碰撞/表面相交和 OpenCV 点转换；不持有状态 |

当前 OBB 碰撞把零长度投影重叠视为不碰撞；AABB 工具也把仅边界接触视为分离。
`CheckIfAxisAlignedCubeNdIntersect` 专门检测表面穿越，因此完整包含时返回 false。这些
判定语义会直接影响后续安全验证，修改时必须通过独立任务和回归场景进行，而不能在
注释任务中顺带改变。

## 7. M0.2a3a：车辆、行为与控制语义

| 组件 | 职责与边界 |
|---|---|
| `VehicleParam` | 保存车辆尺寸、轴距、悬长、转向/加速度限制；不验证参数物理合理性 |
| `Vehicle` | 组合单时刻状态与车辆参数，并提供后轴中心到几何中心、OBB 和车身顶点转换 |
| `LongitudinalBehavior` / `LateralBehavior` | 行为规划、预测与语义地图共享的离散意图枚举 |
| `ProbDistOfLatBehaviors` | 保存 LK/LCL/LCR 概率；有效标记和归一化必须由上游显式维护 |
| `SemanticBehavior` | 保存行为层 winner、参考车道、自车/周车 rollout 和关联状态 |
| `SemanticVehicle` | 在原始车辆上附加最近车道、沿车道位置和横向行为预测 |
| `VehicleControlSignal` | 二选一表达开环期望状态或闭环加速度/转向率，禁止同时消费两种表示 |

这里的 `ProbDistOfLatBehaviors` 是当前 baseline 将不确定性压缩为有限横向意图的主要
接口。BR-EUDM 后续会通过 adapter 扩展跨周期 intent/style belief，但必须保持旧接口
可配置兼容，不能让 MPDM/EUDM 对照因消息结构差异失去公平性。

## 8. M0.2a3b1：N 维规则栅格

`GridMapND<T, N_DIM>` 使用一维 `std::vector` 保存 N 维规则栅格，第 0 维在内存中
连续变化最快，步长依次为 `{1, size[0], size[0]*size[1], ...}`。世界位置转换使用
`round((position-origin)/resolution)`，因此得到的是最近栅格，而不是包含该位置的
`floor` 栅格。

需要保留到 baseline 修复阶段处理的既有接口风险：

- `GetValueUsingGlobalPosition` 和 `SetValueUsingGlobalPosition` 不传播内部越界错误；
- `set_dims_size` 更新理论元素数但不调整 `data_` 实际长度；
- `set_data` 不检查输入长度，`data(i)` 和裸指针接口也不做边界检查；
- `FREE` 与 `UNKNOWN` 当前使用相同数值 0；
- 单维/N 维坐标转换函数不验证维度、分辨率或输出指针。

这些行为会影响 SSC 时空占据栅格的安全性，但本注释任务只冻结真实语义，不修改 API。

## 9. M0.2a3b2：地图车道与静态障碍物语义

- `LaneRaw` 保存配置文件直接给出的纵横向拓扑、换道可用性和离散中心线；
- `SemanticLane` 保留拓扑属性，并把离散点转换为可投影/插值的 `Lane` 几何对象；
- `LaneNet` 与 `SemanticLaneSet` 均以 ID 为键，调试打印不保证稳定顺序；
- `ObstacleSet` 分开存储圆形与多边形静态障碍物，`type` 只是上层解释的整数标签；
- `PointVecForKdTree` 只向 nanoflann 暴露二维 x/y，不使用点携带的 values，且不提供
  预计算包围盒。

本层只表示地图语义，不决定障碍物在 SSC 时间维的占据方式。静态障碍时间坐标必须由
SSC map adapter 明确填充，不能因为障碍物本身没有时间字段就沿用未初始化时间索引。

## 10. M0.2a3c：SSC 约束与交通语义

- `SpatioTemporalSemanticCubeNd` 保存时间以及各维位置/速度/加速度上下界；默认使用
  有限宽松值，避免 QP 中无穷边界导致数值不稳定；
- `DrivingCube` 把 3D 栅格 cube 与生成它的种子体素关联，`DrivingCorridor` 保存一组
  有序 cube 及整体有效标记；
- `TrafficSignal` 用二维作用线段、有效时间、速度范围和横向范围表达统一交通约束，
  起止角目前只用于可视化；
- `SpeedLimit` 与 `StoppingSign` 是全时间有效的特化，`TrafficLight` 只额外保存灯色；
- `SemanticsUtils` 负责行为缩写、车辆 OBB/顶点和尺寸膨胀，不持有运行时状态。

已确认的后续修复/验证点：`TrafficLight::type_` 默认未初始化；交通信号 setter 不检查
区间顺序；`GetVehicleVertices` 会向输出容器追加而不清空；车辆尺寸膨胀允许得到非正
尺寸。这些问题不能在纯注释标签内修改，将进入 baseline 静态修复清单。

## 11. M0.2a4a：状态表示与坐标转换

- `State` 表示车辆后轴中心处的世界位姿、标量速度/加速度、曲率和转向角；
- `FreeState` 把速度和加速度展开为世界 x/y 向量，转回 `State` 时速度取模，不能恢复
  倒车速度符号；
- `FrenetState` 同时保存 `[s,s_dot,s_ddot]`、d 对时间导数和 d 对弧长导数；当
  `s_dot` 接近零时，时间导数无法稳定转换为弧长导数；
- `StateTransformer` 按值持有参考 `Lane`，使用车道投影、切向、曲率和曲率导数完成
  世界/Frenet 状态转换；
- `Waypoint` 表达轨迹位置到 jerk 的可选硬约束及可选时间戳。

批量转换接口采用 fail-fast，但失败前已经写入的输出前缀不会回滚；世界到 Frenet 的
弧长投影使用有限采样，并以 0.5 m 切向偏差作为拒绝阈值。后续连续性和投影鲁棒性
实验必须显式记录这些近似，而不能把转换失败静默当作有效轨迹。
