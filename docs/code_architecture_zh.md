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
- [x] M0.2a4b `core/common` Lane 与 LaneGenerator。
- [x] M0.2b1 `core/common` 数学基础与圆弧运动基元。
- [x] M0.2b2a `core/common` 多项式、基础样条与边界逆矩阵查找表。
- [x] M0.2b2b `core/common` Bezier 曲线与分段 Bezier 样条。
- [x] M0.2b2c1 `core/common` SplineGenerator 插值、拟合与状态连接。
- [x] M0.2b2c2 `core/common` 带参考接近项的 Bezier corridor QP。
- [x] M0.2b2c3 `core/common` 基础 Bezier corridor QP。
- [x] M0.2b3 `core/common` 轨迹抽象、Frenet Bezier 与 FrenetPrimitive 包装。
- [x] M0.2c1 `core/common` 加权最小二乘 QP 与 OOQP/MA27 接口。
- [x] M0.2c2 `core/common` RSS 安全距离、速度区间与车辆检查。
- [x] M0.2c3a `core/common` IDM、IIDM 与 ACC 纵向模型。
- [x] M0.2c3b `core/common` MOBIL 换道收益与横向行为概率。
- [x] M0.2c4 `core/common` FrenetPrimitive 双模式五次运动基元。
- [x] M0.2c5a `core/common` 轨迹线条与通用 Marker 属性工具。
- [x] M0.2c5b `core/common` Pose、PointCloud 与基础几何 Marker。
- [x] M0.2c5c `core/common` Mesh、箭头、线条、文本与车辆 Marker。
- [x] M0.2c5d `core/common` 障碍物、SemanticBehavior 与 GridMap 可视化。
- [x] M0.3a1 OnLaneFsPredictor 与驾驶风格参数查表。
- [x] M0.3a2a 目标车道间隙状态与高级 LK/LC 单步传播。
- [x] M0.3a2b 标准单步传播、控制器辅助函数与车辆模型积分。
- [x] M0.3b1 BehaviorPlannerMapItf 与 SemanticMapManager 适配器。
- [x] M0.3b2a BehaviorPlanner 生命周期、候选枚举与 MPDM winner 输出。
- [x] M0.3b2b 多车前向 rollout 与碰撞筛选。
- [x] M0.3b2c 安全/效率/行为代价评估。
- [x] M0.3b2d 参考 Lane、Lane ID 状态机与参数访问器。
- [x] M0.3b3 BehaviorPlanner ROS 服务与可视化。
- [x] M0.3c SemanticMapManager 支撑组件与主类。
- [x] M0.3c1 SemanticMapManager 基础配置类型与 JSON ConfigLoader。
- [x] M0.3c2 TrafficSignalManager。
- [x] M0.3c3 DataRenderer。
- [x] M0.3c4 ROS adapter。
- [x] M0.3c5 SemanticMapManager visualizer。
- [x] M0.3c6 SemanticMapManager 主类分段审计。
- [x] M0.3c6a 构造、UpdateSemanticMap、日志与基础访问器。
- [x] M0.3c6b 行为/轨迹预测与语义车辆。
- [x] M0.3c6c 语义 Lane、本地 Lane 与快速 LUT。
- [x] M0.3c6d Lane 距离、碰撞、可达性与最近 Lane。
- [x] M0.3c6e 关键车辆筛选。
- [x] M0.3c6f 局部/参考 Lane 生成与采样。
- [x] M0.3c6g 前后车、交通查询与 LaneNet 距离。
- [x] M0.4 SSC、车辆模型、物理仿真、playground 与配置。
- [x] M0.4a1 车辆模型 PID、IDM/CTX-IDM 速度包装与 Pure Pursuit 控制器。
- [x] M0.4a2 IDM 与 Context-IDM 连续模型。
- [x] M0.4a3 VehicleModel 基类与 IdealSteerModel。
- [x] M0.4b SSC 地图、规划器、ROS/可视化与配置。
- [x] M0.4b1 SSC 地图抽象接口与 SemanticMapManager 适配器。
- [x] M0.4b2 SSC 时空占用栅格与 corridor 地图。
- [x] M0.4b3 SSC 轨迹规划与优化主流程。
- [x] M0.4b4 SSC ROS2 服务端与可视化。
- [x] M0.4b5 SSC proto、配置、RViz 与构建元数据。
- [x] M0.4c 物理仿真器与 arena loader。
- [x] M0.4c1 场景基础依赖与 ArenaLoader。
- [x] M0.4c2 PhySimulation 状态推进与临时障碍物。
- [x] M0.4c3 ROS2 adapter、visualizer 与仿真节点。
- [x] M0.4d playground、集成入口、launch 与构建配置。
- [x] M0.4d1 物理仿真 GeoJSON 工具与 ROS1/ROS2 launch。
- [x] M0.4d2 物理仿真 CMake、package.xml 与 RViz 资源。
- [x] M0.4d3 playground 场景资源与包元数据。
- [x] M0.4d4 planning_integrated 集成入口与剩余构建/launch 审计。
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

## 12. M0.2a4b：车道连续几何与离散样本建模

- `Lane` 按值持有 `Spline<5, 2>`，负责位置、各阶导数、单位切/法向、航向、曲率及
  曲率导数查询；它不包含车道拓扑、宽度、限速或交通规则；
- 项目通常用离散中心线的累计弦长作为样条参数，但 `Lane` 不执行严格弧长重参数化，
  因而 `arc_length` 接口名不代表位置一阶导数的模恒为 1；
- 世界点投影先用覆盖全参数域的三个候选点进行最多 4 次粗搜索，任一候选进入 30 m
  半径后即选择当前最近候选，再执行最多 8 次 Newton 局部最小化；
- `LaneGenerator::GetLaneBySamplePoints` 通过相邻二维点的欧氏距离累加参数，再调用
  自然三次样条插值；三次系数封装在统一的五次样条表示中；
- `GetLaneBySampleFitting` 调用分段五次样条拟合，通过等式约束保持段间位置到 jerk
  连续，并对高阶系数施加调用方给定的正则权重。

已确认的后续修复/验证点：曲率接口未防止一阶导数模长接近零，且曲率导数表达式需要
独立数值推导复核；投影 Newton 步未防止二阶项接近零，没有线搜索、局部极小/全局最近
性验证，外层还忽略下层错误码并固定返回成功；生成器不在本层验证空输出指针、参数严格
递增、`breaks` 与样本参数覆盖关系或样本自交。上述行为本标签只记录，不改变 baseline。

## 13. M0.2b1：数学基础与圆弧运动基元

- `calculations.*` 提供符号、阶乘/组合数、一次角度回绕、二维旋转、区间截断/映射和
  零附近二次压缩；这些函数位于全局命名空间，是多个规划与车辆模型模块的基础依赖；
- `poly_roots.h` 对二至四次多项式使用闭式公式，对五、六次使用 Eigen 数值求根，只
  返回被精确判定为实数的根，且不排序、不去重；
- `CircleArc` 用 `[x,y,theta]` 起点、恒曲率和有符号弧长解析表示圆弧/直线运动基元，
  支持位置、导数、切/法向和固定横向偏移查询；
- `CircleArcBranch` 表示从同一起点独立展开的多条候选基元，而不是首尾相接的曲线；
  它按输入顺序返回所有候选末状态或采样状态。

已确认的后续修复/验证点：`normalize_angle` 只加减一次 `2*pi`；阶乘、组合数和区间
映射缺少负数、溢出及零宽区间保护；多项式求根依赖精确浮点零判断，六次入口对
`a=0,b!=0` 的五次退化处理不正确。`CircleArc` 默认构造和直线分支存在未初始化可读
字段，偏移曲线的两个一阶导数接口只有声明没有定义；采样要求步长非零且与弧长同号，
否则可能无限循环，并且默认不包含末点。`CircleArcBranch` 又固定使用正 0.2 m 步长，
因此负弧长候选会触发该风险；其默认构造只有声明没有定义，输出均为追加语义。以上
问题留待 M1 独立修复和回归验证。

## 14. M0.2b2a：多项式、基础样条与边界映射缓存

- `Polynomial<N_DEG>` 以“最高阶在前、系数乘以对应阶乘”的形式保存主系数，并维护
  一份常数项在前的普通幂基缓存；前者便于导数求值，后者用于更快的零阶 Horner 求值；
- 五次 jerk-optimal 连接通过 6x6 逆矩阵把两端位置、速度、加速度映射为系数，常用
  3.0--5.0 s 时长可按浮点键精确命中全局预计算表；
- `PolynomialND` 只把多个独立一维多项式组合成向量输出，不表达维度间耦合；
- `Spline` 以全局断点选择分段，再将全局参数转换为段内局部参数；内部断点精确命中
  时使用左段，域外查询使用首/末段外推且仍返回成功；段间连续性完全由生成器保证。

已确认的后续修复/验证点：多项式导数阶数、输出指针和数组负下标未验证，平方导数积分
公式只适合五次系数布局。`GetJerkOptimalConnection` 在 `S<kEPS` 分支写入主系数后未
调用 `update()`，导致零阶快速求值可能读取旧缓存，而带导数重载读取新主系数；负时长
也进入同一退化分支。`GetAInverse(0)` 会对奇异矩阵求逆，全局缓存可被外部修改且只按
浮点精确值命中。`Spline` 不检查断点严格递增，空域检查不足以保护单断点/零分段异常
对象，域外外推无法从返回码辨别，调试 `print()` 又固定为二维。以上留待 M1 修复。

## 15. M0.2b2b：时间缩放 Bezier 样条

- `BezierSpline<N_DEG,N_DIM>` 以全局断点选择分段，将局部参数归一化到 `[0,1]`，再
  使用 Bernstein 基函数求值；d 阶导数整体乘以 `duration^(1-d)`；
- 因零阶位置还乘以一个分段时长，`ctrl_pts_` 实际保存 SSC 优化使用的时间缩放控制
  变量，不应直接当作常规几何 Bezier 控制点解释；
- 内部断点精确命中左段；左越界返回错误，右越界的归一化参数被基函数截断到 1，
  因而返回末点并报告成功；
- `BezierUtils` 以硬编码解析式提供五次基函数的 0--3 阶导数，以及三阶导数平方积分
  使用的固定 Hessian，供后续 corridor QP 目标和约束复用。

已确认的后续修复/验证点：`bezier.h` 本身不包含项目数值类型、Eigen、算法/数学和输出
所需头文件，依赖包含顺序；断点数量、严格递增、零时长、控制变量段数和下标均缺少
运行时验证。五次基函数收到 0--3 以外导数阶数时直接返回未初始化向量，其他次数和
不支持的 Hessian 在关闭 assert 后同样可能返回未初始化矩阵。左右越界契约不对称，
输出指针也未检查。以上在 M1 中通过自包含编译检查和数值单元测试修复。

## 16. M0.2b2c1：样本插值、正则拟合与状态边界连接

- `SplineGenerator` 的实现位于 `.cc`，只显式实例化 `<5,2>` 和 `<5,1>`；当前通用
  模板外观不代表任意阶数/维数都具备可链接实现；
- 三次插值对各维独立调用 `tk::spline`，相邻参数差超过 7 m 时先在原样本连线上加密，
  再把普通三次系数转换为项目的阶乘缩放五次容器；
- 五次拟合构造单位样本权重的最小二乘项，对每段最高三个系数施加递减正则，并通过
  等式约束保证相邻段的位置、速度、加速度和 jerk 连续；
- Waypoint 转换会覆盖输出，只固定位置并写入参数戳；State/FreeState 转换对每个相邻
  状态段的 x/y 独立构造两端二阶边界五次连接，更高维度统一置零。

已确认的后续修复/验证点：插值只检查样本/参数数量，不检查参数有限、严格递增或输出
指针；线性加密会改变原始中心线形状假设。五次拟合没有检查 `samples.size()==para.size()`、
参数/断点排序和覆盖关系、正分段时长、非负正则或输出指针；共享断点样本归入左段，
最后断点外样本可能形成未填充的观测行。State 两个入口仅用 assert 检查数量一致，release
构建中不安全，且非递增参数会触发 `Polynomial` 的近零/负时长退化与缓存不同步问题。

## 17. M0.2b2c2：带参考接近项的 Bezier corridor QP

该入口把每个维度、每个走廊分段的 `N_DEG+1` 个时间缩放 Bernstein 控制变量按
“维度--分段--控制点”顺序堆叠，并分五阶段求解：

1. 以分段时长的负三次方缩放五次 Bezier jerk 平方积分 Hessian；在离散参考时间戳
   处展开位置平方误差，形成额外二次矩阵与线性项；
2. 施加相邻段端点连续性及首末位置/速度/加速度等式边界；
3. 通过位置控制点、一阶差分控制点和二阶差分控制点的盒约束，利用 Bezier 凸包性质
   保证整段位置、速度和加速度处于 cube 上下界内；
4. 调用 OOQP 求解带双边不等式的二次规划；
5. 以首 cube 下界和各 cube 上界构造参数域，并把解向量回填为 `BezierSpline`。

已确认的后续修复/验证点：代码把 `num_continuity` 设为 3，实际只保证 C2（位置、速度、
加速度）连续，虽然保留了 jerk 分支和“连续到 jerk”的旧注释；这会直接影响跨 cube
舒适性指标。入口未检查空走廊、正且连续的 cube 时域、上下界顺序、输出指针、参考
时间戳与参考点数量一致或非负接近权重。内部断点参考样本选右段，而样条求值选左段。
首末约束数组超过三项时会分配额外全零等式行；全局参数域忽略后续 cube 的 `t_lb`，
所以时间不连续时优化尺度与回填求值尺度不一致。凸包盒约束是充分但可能保守的安全
条件，后续实验需区分“QP 不可行”与“真实连续轨迹不可行”。

## 18. M0.2b2c3：纯平滑 Bezier corridor QP

无参考项重载复用相同的时间缩放控制变量、首末二阶状态边界、跨段 C2 连续性和
位置/速度/加速度凸包盒约束，但线性目标恒为零，也不构造离散参考位置的外积矩阵。
因此它在硬约束可行域内只优化积分 jerk 平方，是当前 SSC 轨迹生成的基础求解入口。

该重载继承第 17 节记录的空走廊、时间连续性、边界顺序、约束阶数和输出指针风险。
此外，两套 corridor 函数复制了约三百行等式/不等式组装逻辑，已经出现维护漂移：带
参考项入口为适配 `0.5*x'Qx+c'x` 将总二次矩阵乘 2，纯平滑入口没有乘 2。纯平滑时
这只是整体目标尺度变化，理论最优解不变，但会改变求解器数值尺度与容差表现；一旦
加入其他软目标便不再等价。后续应抽取共享 QP builder，并对两重载做矩阵级一致性测试。

## 19. M0.2b3：轨迹抽象与 Frenet 实现

- `Trajectory` 统一世界状态查询、参数域、有效性和优化变量读写；`FrenetTrajectory`
  增加 FrenetState 查询与纵横向 jerk 指标接口；
- `FrenetBezierTrajectory` 按值持有二维 `[s(t),d(t)]` Bezier 样条和 StateTransformer，
  用位置、一阶导、二阶导构造时间参数化 FrenetState；纵向速度过小时退化为只保留
  s/d 位置的弧长模式；
- `FrenetPrimitiveTrajectory` 把 FrenetPrimitive 适配为相同接口，并将纵向、横向两个
  五次多项式的阶乘缩放系数序列化为 12 维优化变量；时域和模式不包含在变量向量中；
- 两种实现从 Frenet 转回世界状态后都把标量速度截断为非负，因此不表达倒车轨迹。

已确认的后续修复/验证点：两种实现的查询函数都不先检查 `IsValid()`。Primitive 默认
对象通常会被内部零时长检查拒绝；Bezier 默认对象在 `t=0` 可通过外层范围检查，又忽略
三次 `BezierSpline::evaluate` 的错误码，随后使用未初始化位置/导数并返回成功。允许
`kEPS` 的左边界外查询也有同一问题。Bezier 的 `variables()` 固定为空、setter 无操作、
`Jerk()` 不写输出，属于静默未实现接口。Primitive setter 仅用 assert 检查 12 维长度，
release 下短输入会越界，且改系数不更新有效标记或端状态。以上属于 M1 的高优先级修复。

## 20. M0.2c1：加权最小二乘 QP 与 OOQP 接口

- `QuadraticProblem` 将 `(Ax-b)'S(Ax-b)+x'Wx` 展开后，以正比例缩放的标准形式传给
  OOQP；完整入口支持等式、双边线性不等式和变量界，简化入口构造零行不等式与无穷界；
- `OoQpItf` 复制可能被 OOQP 修改的输入，提取 Q 下三角并压缩 Eigen 稀疏矩阵，再把
  CSR 数组交给 `QpGenSparseMa27` 和 Gondzio 内点求解器；
- `generateLimits` 把 `+/-double_max` 识别为未启用边界，其他有限界均传入 OOQP；
- 输出 x 在求解前清零，只在成功终止或调用方明确允许 UNKNOWN 时复制当前迭代解。

已确认的后续修复/验证点：接口不验证 Q 方阵、对称、半正定和有限性，只取下三角；
除少数 assert 外，A/b、C/d/f、变量界与列数一致性、`lower<=upper` 均未系统检查。
零等式或零不等式场景仍对空 Eigen 向量调用 `coeffRef(0)`，debug 会断言，release 属于
未定义行为；这正是等式-only 重载的正常输入。corridor 调用设置
`ignoreUnknownError=true`，会把未确认收敛的 UNKNOWN 迭代点当成功，且返回前不复核
等式/不等式残差。W 经稠密矩阵再转稀疏，打印也会稠密化大矩阵；裸 `new/delete` 在
异常路径不具备 RAII。M1 应先加维度/残差检查、零约束安全指针和严格状态策略。

## 21. M0.2c2：简化 RSS 安全距离与速度区间

- `RssConfig` 保存响应时间、纵横向响应加速度、主动/被动制动幅值和固定横向裕量；
- 纵向距离公式按对象在前/后及速度方向，比较“响应后被动制停”和“立即主动制停”
  路程；横向公式按左右相对位置和双方横向速度符号组合主动/被动制停路程；
- FrenetState 重载把车辆视为点，只有纵向距离和横向距离同时侵入阈值才判不安全；
- Vehicle 重载先投影到同一 Lane，横向阈值加入两车半宽，纵向后轴间距扣除前后悬，
  再反解自车安全速度区间；前车威胁产生上界，后车威胁可能产生“不能过慢”的下界；
- 该点质量重载被 MOBIL 用于换道前后车安全筛选，默认采用 RssConfig 固定参数。

这并不是完整 RSS 状态机：没有危险开始时间、proper response 持续控制、责任归属、
多对象联合约束或对感知/预测不确定性的处理，只是单时刻 Frenet 运动学阈值检查。
已确认的工程风险包括：配置不验证非负、制动分母和能力顺序，二次反解不检查判别式；
自车倒车直接判安全；批量速度数组不检查长度；Frenet 可用性标志和输出指针不检查。
点质量版本忽略车身尺寸，车辆版也只用轴向长度/宽度而非航向 OBB；合法时速度上下界
统一写零，不能解释为真实可行区间。阈值相等被视为安全，后向重叠没有前向重叠的立即
处理。后续 BR-EUDM/backup-RSS 必须把这些简化与正式安全层清楚区分。

## 22. M0.2c3a：IDM、IIDM 与 ACC 纵向模型

- `State` 用一维 s/v 表达自车和前车，`Param` 提供期望速度、固定车长、最小间距、
  期望时距、最大加速、舒适/硬制动幅值和自由流指数；
- 原始 IDM 由自由流速度项和动态期望间距平方项组成，不截断输出；
- IIDM 针对超速制动过强与期望速度附近时距失真进行分段修正，最终把加速度限制在
  `[-hard_brake, max_acc]`；
- ACC 入口在 IIDM 基础上加入假设前车恒定舒适制动的 CAH，并以固定 0.99 coolness
  通过 tanh 平滑融合；vehicle_model 和 MOBIL 都复用这些静态公式。

已确认的后续修复/验证点：默认期望速度为零，三个入口均不验证输出指针、参数有限性、
正期望速度、正加速/制动或指数范围。净间距在零处截断后继续作为分母；IIDM 在
`v==desired_velocity` 附近还可能出现 `a_free=0` 后的除零指数。原始 IDM 可直接输出
Inf，IIDM 虽做幅值截断但 NaN 不一定被修复。ACC 的 CAH 分母在前车速度和间距同时为
零时为零，且其 ds 未扣除 Param 中的固定车长，与 IDM 净间距定义不一致；coolness
也不可配置。M1 需添加有限性回退、统一净间距和边界工况单元测试。

## 23. M0.2c3b：简化 MOBIL 换道收益与行为概率

- `MobilLaneChangingModel` 计算自车 c、原车道后车 o、目标车道后车 n 在换道前后的
  IDM/ACC 加速度；缺少真实前车时使用同速远端虚拟前车；
- 目标车道先用第 21 节的点质量 RSS 分别检查投影自车与前/后车，两个方向均安全才
  计算换道后加速度；
- `MobilBehaviorPrediction` 固定接收 `[当前,左,右]` 三条 Lane，分别计算左右收益，
  再把安全方向的收益从 `[-1,6]` 截断映射为 LK/LCL/LCR 启发式分布；
- 近似静止车辆直接给 LK=1，无有效目标车道时相应换道概率为零。

当前实现与标准 MOBIL 有明显差异：`politeness_coeff` 硬编码为 0，虽然计算了新/旧后车
加速度变化，最终收益只保留自车加速度改善；没有换道收益阈值、方向偏置或安全制动
阈值参数，负收益只要高于 -1 仍可映射为非零换道概率。IDM 期望速度被设为各车当前
速度，车辆长度使用 IDM 固定 5 m，而不是传入 Vehicle 尺寸。因此输出是未标定启发式
分数，不是可解释行为概率。

工程风险包括：只检查 lanes 数量，不检查四组邻车/状态数组长度；`nearby_vehicles`
完全未使用；Lane 投影和目标车道模型返回码被忽略。RSS 安全门即使邻车 ID 无效也会
检查对应 FrenetState，缺失邻车的默认状态可能误判；近似静止后车被当作不存在。目标
车道不安全时三个加速度输出不写值；每辆车每周期无条件打印收益。上述问题是后续
BR-EUDM 用持续 belief 与交互 rollout 替代单帧 MOBIL 启发式的重要 baseline 依据。

## 24. M0.2c4：双模式 Frenet 五次运动基元

- `FrenetPrimitive` 始终用五次多项式表示 `s(t)`；横向可用高速/时间独立模式 `d(t)`，
  或弧长模式 `d(s-s0)`；
- `Connect` 从两端 Frenet 二阶状态构造 jerk-optimal 连接。弧长模式的纵向位移小于
  2 m（包括负位移）时用 100 m 虚拟跨度拟合横向多项式，降低近零参数奇异性；
- `Propagate` 直接构造恒定纵/横向加速度的二次多项式，并查询终点状态；
- 状态查询允许时域外多项式外推；批量采样覆盖输出并生成 `[begin+offset,end)` 半开
  区间；`GetJ` 返回纵向和横向三阶导数平方积分；
- 多项式 setter 和公开模式字段允许轨迹包装器/优化器直接改内部表示，但不会同步端状态。

已确认的后续修复/验证点：类没有独立有效标记，Connect/Propagate 不验证正时长、有限
状态或输出指针且固定返回成功；Propagate 忽略终点查询错误。`GetFrenetStateSamples`
未验证正 step，零/负步长可能除零、超大 reserve 或死循环。弧长模式构造可能使用
100 m 虚拟跨度，但 jerk 指标和 `lateral_T()` 仍使用实际 delta_s；负 delta_s 会使从
0 到负上限的“平方积分”得到负值。公开 `is_lateral_independent_` 或单独替换多项式后，
缓存端状态和导数解释可能不一致；域外外推也没有幅值保护。这些需纳入 primitive 回归。

## 25. M0.2c5a：轨迹线条与 Marker 通用属性

- `VisualizationUtil` 是无状态静态转换层，不参与规划数值；大多数接口向已有
  Marker/MarkerArray 追加数据，header、frame、namespace 和 id 由调用方后置补全；
- 多项式、Spline、Lane、Trajectory 和 State 序列被采样为半开参数区间的 LINE_STRIP；
  单个轨迹采样失败会跳过该点；
- 固定维向量只映射前三维到 Point/Point32，缺失分量补零、额外分量忽略；
- 通用填充函数可统一颜色、尺度、时间戳和 lifetime；`FillHeaderIdInMarkerArray` 从 0
  重编号当前数组，并为上一帧多余 id 追加 DELETE Marker。

已确认的后续修复/验证点：`GetMarkerByPolynomial` 调用不存在的
`PolynomialND::evaluate(s)` 返回值重载，模板实例化时无法编译。所有采样循环都未验证
正 step，且默认排除终点、保留旧 points/markers。`FillScaleColorInMarker` 会重置已有
pose；渐变函数忽略 `if_ascending`、不清空 colors，最后一点也达不到色图上界。
删除 Marker 没有继承被删对象 namespace，非空 namespace 时可能删不到。输出指针和
NaN/Inf 均不检查；头文件使用 rclcpp::Time/Duration 但依赖间接包含。以上不影响规划
结果，但会妨碍 ROS2 自包含编译、调试复现和长时间运行的消息大小稳定性。

## 26. M0.2c5b：Pose、PointCloud 与基础几何 Marker

- `[x,y,yaw]` 与 ROS Pose 双向转换固定 z/roll/pitch 为零；PoseStamped、PoseArray 和
  PointCloud 固定使用 `map` frame，并在函数内通过临时 ROS clock 取当前时间；
- State、Point、CircleArc 和 CircleArcBranch 可追加到旧式 `sensor_msgs/PointCloud`；
  圆弧固定以 0.2 参数步长采样；
- Point/Circle 可转换为 SPHERE/CYLINDER，OBB 可转换为带 yaw 的 CUBE，三维 AABB 或
  上下界 Cube 可转换为单位姿态的轴对齐 CUBE；
- 这些基础形状接口只构造几何/颜色/id，header、namespace 和生命周期多由上层补充。

已确认的后续修复/验证点：PoseArray/PointCloud 和各 Marker 均保留旧数组内容；多层
圆弧转换重复覆盖时间戳并复制 CircleArc 数组。CircleArc 固定正 0.2 步长与负弧长不
兼容，可能死循环。Sphere 和两个 Cylinder 接口未设置单位 orientation，ROS 默认全零
四元数无效；半径、尺度、AABB 长度和上下界顺序均不验证，默认构造的几何数组还可能
未初始化。Pose 反解不验证四元数归一化；frame 与时间源硬编码，输出指针不检查。

## 27. M0.2c5c：Mesh、箭头、线条、文本与车辆 Marker

- OBB 车辆网格、交通锥和六边形标志使用 `package://common/materials/...` 固定资源，
  并以固定尺度/模型轴校正姿态显示；
- 箭头支持“两个几何点”与“Pose+长度”两种 ROS 表达；二维/三维路径、Point 列表可
  追加为单色或 Jet 渐变 LINE_STRIP，文本使用 TEXT_VIEW_FACING；
- 单车可视化一次构造 OBB/网格、速度箭头、ID+速度文本以及三种转向辅助图元；当前
  只发布前三类，且约定 vehicle.id()==0 显示网格、其他车辆显示 OBB。

已确认的后续修复/验证点：资源、frame、尺寸和模型类型均硬编码，BMW 网格不按实际
VehicleParam 缩放，启用嵌入材质时调用方颜色的效果依赖 RViz。几何点箭头不设置
scale/color/id 并保留旧 points；Pose 箭头把负长度显示为前向 0.15 m。线条只使用
scale.x，文本/部分线条仍可能带默认全零 orientation，所有容器继续采用追加语义。
车辆函数每周期计算两条 10 m 转向圆弧和一条横轴后却不发布，形成纯开销；ego=id0
约定、固定 id 偏移和 `cmap.at("black")` 都依赖上层隐式条件。M1 可删除死计算、统一
Marker 初始化和资源/样式配置，但不应混入规划算法对照提交。

## 28. M0.2c5d：障碍物、SemanticBehavior 与 GridMap 可视化

- 圆障碍显示为圆柱；普通多边形复制首点闭合为轮廓线，type=1 多边形则把每个顶点
  显示为交通锥；
- SemanticBehavior 依次追加参考车道方向短线、曲率渐变、纵向行为竖直箭头，以及
  周车 rollout 的逐状态圆柱和连接折线；
- 二维 GridMap 原始数据被复制到 OccupancyGrid；三维 GridMap 的非零单元被转换为
  CUBE_LIST，单元尺度使用各维分辨率，颜色固定为半透明红色。

已确认的静态问题：空 Polygon 会解引用 `begin()`；圆/多边形 id 在空 namespace 中可
冲突。语义方向短线使用 `arrow_width/acos(angle)` 而不是常见的 cos 投影，Lane 查询
错误码均被忽略；停止行为没有显式分支，竖直 ARROW 未设置 scale.z。周车 rollout 被
完整复制，并为每个状态创建独立 Marker/时间戳，消息数量和开销随候选树快速增长。

二维 OccupancyGrid 把内存最快变化的第 0 维写成 height、第 1 维写成 width，可能与
ROS row-major 宽度语义转置；只使用第 0 维分辨率，T 到 int8 数据不做 [-1,100] 限制。
三维接口在 `i>dims_step(2)` 时退出，只画完整 z=0 层和 z=1 的首个体素；点坐标仅 z
减原点而 x/y 保持全局，又叠加 Marker pose，坐标约定不一致。M0.2c 注释阶段至此完成，
这些问题进入 M1 可视化/静态修复，不与规划创新实现混合。

## 29. M0.3a1：开环单车预测与驾驶风格参数表

- `OnLaneFsPredictor` 清空输出后先写入车辆当前状态，再循环调用
  `OnLaneForwardSimulation::PropagateOnce`；有效 Lane 使用 Frenet/车道重载，无效
  Lane 使用自由空间重载；
- 预测不输入前车，IDM 期望速度固定为当前速度，属于确定性开环保持趋势外推；
- `MultiModalForward::ParamLookUp` 只把 1--5 五档等级映射为时距、最小间距、最大
  加速、舒适制动和固定转向增益，本身没有生成多条 rollout 或管理模态概率。

已确认的后续修复/验证点：预测步数为 `round(t_pred/t_step)`，未验证正步长、有限时长
或 int 范围，实际终点不一定等于请求时长；失败时保留部分输出。预测器析构函数只有
声明没有定义，实例化对象会有链接风险。参数查表只覆盖 Param 子集，非法等级仅 assert，
release 中仍返回成功且保持旧参数；当前仓库没有实际调用 `MultiModalForward` 的位置。
这些限制说明 baseline 的周车预测还不是带持续驾驶风格 belief 的真正多模态模型。

## 30. M0.3a2a：目标间隙状态与高级 LK/LC 传播

- `Param` 汇总 IDM、Pure Pursuit 前视距离、横纵向加速度/jerk、曲率、转角和转角率
  限制，并可在横向失败时把期望速度降为零；
- `GetTargetStateOnTargetLane` 把自车和目标间隙前后车投影到目标 Lane，以保险杠位置、
  最小间距和时距构造前后阈值，再用硬编码位置误差增益生成目标 s/v；
- AdvancedLK 在同一 Lane 上追踪横向 offset，并按真实/虚拟前车执行 IDM；
- AdvancedLC 横向追踪目标 Lane，纵向用 Context-IDM 融合当前 Lane 前车与目标间隙
  期望状态；两者最终都交给 IdealSteerModel 施加动力学限制并积分。

已确认的后续修复/验证点：间隙调节增益、速度修正范围和 Context-IDM 权重均硬编码，
目标状态不继承时间戳；前后速度参考区间可能倒置但未验证。高级 LK 中自车投影失败后
仍可能在前车投影成功时使用无效 `current_fs` 做 IDM。高级 LC 对目标/当前 Lane 多个
投影错误使用空分支，随后继续读取输出；其 Context-IDM `current_pos` 取目标 Lane s，
而 leading/target_pos 取当前 Lane s，除非两 Lane 参数化严格一致，否则混用了坐标。
所有辅助函数返回码基本被忽略，入口也不验证正 dt、轴距和参数有限性却固定返回成功。

## 31. M0.3a2b：标准前向传播与控制器/车辆模型组合

- 标准 `PropagateOnce` 把自车投影到 Lane，按速度截断前视距离后追踪 d=0，并按真实
  前车或远端同速虚拟前车计算 IDM 期望速度；
- 等效车辆长度通过投影前车两个保险杠和后轴，选择最小 s 近似后端，再加自车前悬，
  使后轴位置差可用于 IDM 净间距；
- `CalcualateSteer` 把前视 Frenet 点转换回世界坐标，用实际直线距离和航向误差执行
  Pure Pursuit；普通/Context-IDM 控制器都使用车身标量速度而非高曲率下放大的 s_dot；
- `CalculateDesiredState` 每步新建 IdealSteerModel，写入期望转角/速度，施加 Param 中
  的动态限制并积分 dt，输出时间戳增加 dt。

已确认的后续修复/验证点：无 Lane 重载声明接收 `desired_vel`，实现却完全忽略它，
直接保持当前速度和转角；这是 OnLaneFsPredictor 无效 Lane 分支的实际语义。标准 Lane
入口与 AdvancedLK 一样，在自车投影失败但前车投影成功时可能读取无效 current_fs。
Pure Pursuit 不检查零世界前视距离；角度归一化只回绕一次。虚拟前车距离
`100+100*v` 对足够负速度可落到后方。所有控制器/车辆模型错误码、输出指针、dt、轴距
和有限性均未检查，入口固定成功；辅助函数名称还长期保留 `Calcualate` 拼写错误。

## 32. M0.3b1：行为规划地图抽象与语义地图适配器

- `BehaviorPlannerMapItf` 把规划器所需能力限制为自车、最近 Lane/拓扑、参考 Lane、
  关键周车、碰撞、前车、限速和预测行为查询，使规划核心不直接依赖地图内部容器；
- `BehaviorPlannerMapAdapter` 通过 shared_ptr 共享 SemanticMapManager，并在大多数入口
  先检查自身有效标记，再转发或复制结果；
- 左右 Lane 查询额外检查换道可用性，Lane/参考 Lane 查询验证连续几何有效；父子 Lane
  输出使用 assign 覆盖旧内容；关键原始车辆当前直接等于全部 surrounding_vehicles。

已确认的后续修复/验证点：接口含大量虚函数却没有虚析构，经基类指针销毁派生对象
不安全；所有接口均非常量且使用裸输出指针。`set_map` 不检查 nullptr 就把 is_valid_ 置
真，后续会解引用空指针；有效标记也不随地图状态更新。多次车道查询会复制完整
SemanticLaneSet；两个最近 Lane 函数各保留未使用的 `dist_set`。`CheckIfCollision` 忽略
底层错误码，`GetPredictedBehavior` 用 `.at(vehicle_id)`，缺失 ID 会抛异常而不是返回
ErrorType。M1 应统一空指针/异常边界并增加 const、虚析构和轻量只读视图。

## 33. M0.3b2a：BehaviorPlanner 生命周期与候选决策主流程

- `Init` 创建 RoutePlanner；`RunOnce` 查询自车 Lane/车辆，按需要刷新导航路径，更新
  Lane ID 与当前行为，L3 及以上调用 MPDM，最后为输出行为构造参考 Lane；
- MPDM 候选始终包含 LK，左右换道仅在相应潜在 Lane ID 缓存非空时加入；
- 每辆语义周车根据预测横向行为构造至少 50 m 的参考 Lane，随后每个自车候选独立
  rollout，失败候选被丢弃；有效候选统一评估 winner 并缓存供 SemanticBehavior 输出；
- winner 速度相对当前速度限制为最多 5 m/s 跳变，HMI 锁定可在有效候选中覆盖横向行为。

已确认的后续修复/验证点：Init 忽略 config、使用裸 new 且类无析构，重复初始化泄漏；
RunRoutePlanner 忽略全部地图查询和 RoutePlanner 失败并固定成功。RunOnce 未验证地图/
RoutePlanner 指针，多处更新返回码被忽略，初始 Lane 判断还使用 Agent 无效常量。
`previous_desired_vel` 参数完全未使用，没有跨周期速度平滑/迟滞；HMI 覆盖横向 winner
时仍输出原 MPDM winner 的期望速度，横纵向策略可能不一致。5 m/s 限幅后也不保证
非负速度。候选 Lane 依赖上周期缓存，日志在每周期大量输出。

## 34. M0.3b2b：候选行为的同步多车 rollout 与开环降级

- `SimulateEgoBehavior` 按候选 LK/LCL/LCR 构造自车参考 Lane：前向长度为
  `max(10*v, 50 m)`、后向长度固定 10 m；随后把语义自车插入车辆集合并首先执行
  `MultiAgentSimForward`，失败时自动回退到 `OpenloopSimForward`；
- `MultiAgentSimForward` 的每个离散步都基于同一时间切片计算全体车辆下一状态，再通过
  `state_cache` 统一提交，避免容器遍历顺序污染预测。自车使用规划参考速度，周车以各自
  初始速度为期望速度；若可查询限速，则进一步限制为限速的 90%；
- 交互 rollout 会为当前车辆构造排除自身的环境集合，在固定参考 Lane 上查询前车，检查
  当前车与该前车的当前状态是否碰撞，再调用 OnLaneForwardSimulation 完成一步跟驰传播；
  输出的自车和周车轨迹都包含初始状态，预测时间戳按初始时间加离散步长重写；
- `OpenloopSimForward` 是更弱的降级路径：自车与周车均不查询前车、互不响应，沿各自
  固定参考 Lane 独立传播；自车使用参考期望速度，周车使用各自初始速度，并只通过地图
  接口检查自车下一状态是否碰撞。

已确认的后续修复/验证点：三个入口均未检查输出指针、正仿真步长/时域或 Lane 有效性，
步数还会对 `sim_horizon_/sim_resolution_` 向下截断。多车入口用 `.at(ego_id)`，缺失
自车会抛异常；插入语义自车使用 `insert`，同 ID 已存在时不会覆盖。碰撞接口错误码被
忽略；交互 rollout 只在传播前检查当前车与查询到的前车，缺少传播后的全体车辆两两
碰撞检查。开环路径的地图碰撞检查不一定包含同一步预测后的周车，而任意交互仿真失败
都会回退到这种更弱的模型，可能掩盖碰撞或前车查询失败。所有车辆参考 Lane 在整个
rollout 内固定，周车期望速度不随场景更新；每车每步重建其余车辆集合使复杂度约为
O(N²)，`TicToc` 计时结果未使用。M1 应先补齐输入契约、错误传播和统一的时序碰撞筛查，
再决定哪些失败允许降级以及降级候选应如何施加安全惩罚。

## 35. M0.3b2c：MPDM 候选轨迹代价评估与建议速度

- `EvaluateMultiPolicyTrajs` 假定行为、自车轨迹和周车轨迹三个数组按下标对齐，逐候选
  调用单轨迹评估，并用严格小于比较保留总代价最低者；同分时自然偏向枚举顺序更靠前
  的候选，当前枚举顺序通常使 LK 先于 LCL/LCR；
- 单候选效率代价由两部分等权平均：自车终端速度与参考速度之差除以 10；以及终端
  120 m 搜索范围内前车造成的低速阻塞项。后者只在自车和前车都低于参考速度、欧氏
  距离小于 100 m 时启用，并同时使用前车搜索返回的剩余距离比例；
- 安全代价将自车与每辆周车的等长 rollout 按离散下标对齐，把两车宽长各增加 1 m 后
  做 OBB 相交检查；每个相交采样累加 `0.005*|v_ego-v_agent|`，再对所有周车求和；
- 动作代价对任意非 LK 候选固定加 0.5。动作、安全、效率三项没有配置权重，直接相加；
  建议速度则通过 `GetDesiredVelocityOfTrajectory` 独立从自车 rollout 中提取。

已确认的后续修复/验证点：候选评估只检查行为数组非空，不验证三个数组等长、输出指针
或自车轨迹非空；它忽略单候选评估错误并可能使用未初始化的 score/vel。若代价为 NaN
或全部不小于初始 `kInf`，函数仍会以未定义行为和空轨迹返回成功。安全函数把空的等长
轨迹视为零代价，调用方又忽略长度不一致和碰撞接口的错误码；碰撞软代价在两车同速时
为零，并随时域、离散分辨率和周车数量线性变化，未归一化且通常与 0.5 的换道惩罚量级
不匹配。它也不包含 TTC、RSS 责任、静态障碍、碰撞时刻或碰撞后果硬约束。效率项只看
终端状态，参考 Lane 构造失败后仍继续使用默认 Lane；前车距离使用欧氏距离而非纵向
Frenet 间距，常数和三类代价权重均硬编码。最严重的是建议速度函数从不更新
`max_acc_normal`：每个正横向加速度采样都会覆盖结果，实际返回最后一个曲率非零状态的
速度；全直线或空轨迹直接输出 `kInf`。该函数还按值复制完整轨迹。M1 应优先修正此速度
提取错误、候选维度/有限性校验和错误传播，再用可配置、归一化且可解释的安全效率指标
替代当前经验代价。

## 36. M0.3b2d：最终参考 Lane、Lane 拓扑状态机与参数接口

- `ConstructReferenceLane` 将 LK/Undefined 映射到当前 Lane，将 LCL/LCR 映射到直接相邻
  Lane；相邻 Lane 不存在时回退当前 Lane，并直接把最终行为改写为 LK。随后沿导航路径
  截取前 150 m、后 20 m 样本，以累计弦长为参数、固定 20 个 break 和 `1e6` 正则系数
  拟合连续 Lane；
- 参考速度从自车在最终 Lane 上的投影位置开始，以 0.2 m 分辨率扫描曲率。前视长度至少
  20 m，高速时采用 `v²/1.67`，曲率速度约束为 `sqrt(1.5/|kappa|)`；取区间最低值后
  再减 2 m/s、限制到用户速度上限并向下取整；
- Lane 归属状态机先基于旧 `ego_lane_id_` 构造三组缓存：LK 为当前 Lane 的子 Lane；
  LCL/LCR 为相邻 Lane 加其子 Lane。新观测 Lane 先匹配旧 Lane，再依次匹配 LK、LCL、
  LCR 集合，由此解释为保持、换道完成或 Undefined；
- 当前行为为 LCL/LCR 时，仍匹配旧 Lane 表示换道进行中，匹配对应相邻集合表示完成并
  回到 LK，Undefined 表示取消并回到 LK，匹配反方向集合则进入 Undefined；后三种情况
  都解除 HMI 锁定。L2 HMI 命令立即改写行为，L3 HMI 命令只锁定 MPDM 横向 winner；
- 参数 setter 直接保存自动驾驶等级、激进程度、仿真步长/时域和状态源开关；用户期望
  速度只在 L2 以上生效并截断负值。行为和候选轨迹 getter 均返回完整值拷贝。

已确认的后续修复/验证点：最终参考 Lane 在 MPDM 完成后才更新
`reference_desired_velocity_`，因此本周期候选 rollout 使用上一周期速度约束，而且所有
候选共用一个速度，没有候选 Lane 独立的曲率限速。相邻 Lane 回退函数还隐式修改全局
行为。自车状态查询、Frenet 投影和曲率查询错误被忽略；前视上界把“长度”与 Lane 绝对
终点弧长混用，循环终点可能越过 Lane。零曲率依赖除零产生无穷大，NaN/负速/有限性未
处理，固定减 2 和向下取整会造成不连续速度阶跃。Lane 拟合不检查输出指针、最少样本、
重复点或有限坐标，固定 break 数和极大正则项也不随道路长度/形状自适应。状态机的集合
可重复或重叠，固定优先级会把重叠 ID 优先解释为 LK；所有拓扑查询错误均被忽略，函数
仍返回成功。当前行为为 Undefined 时没有显式恢复分支。HMI 只处理恰好 L2/L3，L4
命令被静默忽略；等级、激进度、仿真参数和裸地图指针均无校验，等级切换也不清理锁定。
大对象 getter 产生不必要复制。M1 应将候选专属参考 Lane/速度约束前移到 rollout，明确
Lane 状态机事件和恢复策略，并统一 setter、拓扑查询和数值有效性契约。

## 37. M0.3b3a：BehaviorPlanner ROS2 Server、地图队列与 HMI

- `BehaviorPlannerServer` 持有规划核心、地图适配器和可视化器。构造时创建容量 100 的
  ReaderWriterQueue；默认工作频率为 20 Hz，也可由调用方传入自定义频率；
- 上层通过 `PushSemanticMap` 值拷贝输入地图。独立规划线程每周期清空全部积压，只用最后
  一帧更新 map adapter 和运行 `BehaviorPlanner::RunOnce`，从而优先保证新鲜度而不是逐帧
  处理；规划成功才把行为写回本地 SMM，但无论成功与否都会执行绑定回调和发布可视化；
- 结果回调在 detached 规划线程内同步执行，返回值被忽略。可视化使用发布瞬间的 ROS
  clock；输入地图时间戳、规划耗时对象和全局起始时间戳当前均未参与调度或消息时间；
- `Init` 在当时等级不低于 L2 时订阅 `/joy`，读取 `use_sim_state` 并初始化可视化器。
  HMI 还需显式 enable，并用 Joy `frame_id` 过滤 ego；按键 2/1 控制左/右换道，3/0 以
  1 m/s 调整期望速度，多键优先级依次为左、右、加、减。

已确认的后续修复/验证点：`try_enqueue` 结果被忽略，队列满时静默丢图；整个 SMM 在入队、
出队和 shared_ptr 快照阶段多次深拷贝。ReaderWriterQueue 的生产者模型、实际调用线程数
和容量需要在集成层明确。工作频率不校验，零/负值会导致除零或非法周期，高于 1000 Hz
会截断为 0 ms 忙循环；使用 `system_clock` 还会受系统时间跳变影响。Start 可重复调用并
创建多个 detached thread，没有 stop/join 或析构同步，server 销毁后线程仍可能访问
`this`。规划线程、ROS Joy 回调及外部 setter 无锁共享 `bp_`、回调标记和函数对象，存在
数据竞争。Joy 不检查空消息或 buttons 至少四项，`frame_id` 非数字时 `stoi` 抛异常，
按键也没有边沿检测/去抖。参数声明被注释，默认 ROS2 参数策略下读取未声明参数可能
失败；Init/规划/回调返回码大多被忽略，规划失败仍回调旧行为并发布旧轨迹。空回调也可
被标记为已绑定。M1 应改为有生命周期的 timer/jthread 或 executor callback group，增加
队列丢帧指标、线程安全配置快照、输入校验和明确的失败输出语义。

## 38. M0.3b3b：BehaviorPlanner 候选轨迹可视化

- `BehaviorPlannerVisualizer` 共享 ROS2 Node，并非拥有地引用同一 server 内的规划器；
  `Init` 为每个 ego 创建 `/vis/agent_<id>/forward_trajs` MarkerArray publisher，QoS 深度 1；
- 发布时值拷贝全部有效自车候选 rollout。每个状态生成一个 z=0.3 m、尺寸
  0.5×0.5×0.1 m 的金色圆柱，每条候选再生成一条宽 0.1 m 的金色 LineStrip；
- `FillHeaderIdInMarkerArray` 将所有 Marker 统一写入 `map` frame、赋予给定 ROS 时间戳和
  从零连续 ID；若新帧 Marker 更少，则为上一帧多出的 ID 追加 DELETE Marker。可视化器
  只记录填充前的 ADD 数量，因此删除消息不会污染下一帧计数。

已确认的后续修复/验证点：构造和 Init 不检查 node/规划器指针，未 Init 就发布会解引用
空 publisher；`cmap.at("gold")` 依赖固定键存在。规划器 getter 先深拷贝全部轨迹，可视化
又为每个状态分配独立 Cylinder Marker，消息大小和 CPU/内存开销随候选数×时域采样点
线性增长。所有候选同色、同 namespace，未标注行为类型、代价或 winner，也不显示周车
rollout、安全裕量和碰撞点；帧名、topic、颜色、尺寸和 z 高度全部硬编码，轨迹自身时间戳
未使用。Marker 构造和 header 填充错误码均被忽略，空轨迹仍追加空 LineStrip。原文件头
保护宏命名为 ROS_ADAPTER 而非 VISUALIZER，且包含多项未使用的 tf2/vehicle_msgs 头；
package.xml/CMake 对直接依赖的声明也不完整，当前可能依赖传递 include/link。M1/构建
配置审计应补齐依赖和空值契约，并采用抽样折线、行为分色、winner 强调及可选风险图层。

## 39. M0.3c1：AgentConfigInfo 与 JSON ConfigLoader

- `AgentConfigInfo` 集中保存局部障碍物 GridMap 元信息、周车搜索半径、开环预测/跟踪噪声/
  日志/快速 Lane LUT 开关和日志路径；布尔项除快速 LUT 默认为 false，快速 LUT 默认为 true；
- `ConfigLoader` 只保存目标 ego ID 和 JSON 路径。解析时重新打开文件，读取固定的
  `agent_config.info` 数组并线性遍历 ID；匹配条目把宽、高、分辨率、搜索半径和开环
  预测作为必填字段，跟踪噪声与日志为可选字段；
- 如果 `enable_log` 键存在，函数同时读取 `log_file`。遍历结束后统一打印输出配置，显式
  关闭文件并固定返回成功；重复 ego ID 会按遍历顺序由后项覆盖前项。

已确认的后续修复/验证点：默认构造的 `ConfigLoader::ego_id_` 和
`AgentConfigInfo::surrounding_search_radius` 未初始化；输出指针、空路径、文件打开和 JSON
解析状态均不检查，文件/键/类型异常直接向外抛出。没有匹配 ego 时仍打印可能未初始化的
配置并返回成功，也没有检测重复 ID。`enable_log` 存在但 `log_file` 缺失会抛异常，所有
尺寸/分辨率/半径缺少正值和有限性验证。`enable_fast_lane_lut` 从未解析，始终保持 true。
头文件还未直接包含其实际使用的 string/fstream 依赖，依赖其他头的传递包含。M1 应为
配置类型提供完整确定性默认值，采用临时对象事务式解析和 schema 校验，区分文件、语法、
缺键、无匹配 ego 等错误，并让所有功能开关可显式配置和可测试。

## 40. M0.3c2：TrafficSignalManager 限速作用区间

- 构造函数立即调用 `LoadSignals`；当前 google_urban/highway 的全部限速示例均被注释，
  因此默认限速和交通灯列表都为空，`Init` 也只是固定成功的占位入口；
- `CheckIntersectionTypeWithSignal` 把信号首尾点投影到参考 Lane。只有首点纵向弧长不晚于
  尾点，且首尾点的 lateral_range 平移后都覆盖 Lane 中心 d=0，才认为信号与 Lane 相交；
  自车在起点前为 Ahead，在 `[start_s,end_s)` 内为 Controlled，其他位置不相交；
- `GetSpeedLimit` 先投影自车状态，再逐个检查限速。当前速度高于信号最大速度时，以固定
  1 m/s² 计算 `|v_limit²-v²|/(2a)` 制动距离；信号起点进入该距离或自车已在控制区时，
  限速生效。多个信号取最小最大速度；无信号时成功输出 `kInf`；
- `UpdateSignals` 遍历并永久删除当前时刻不在 valid_time 闭区间内的限速。交通灯列表不
  更新，`GetTrafficStoppingState` 也不写输出，仅固定返回成功。

已确认的后续修复/验证点：信号完全硬编码且当前为空，重复 LoadSignals 也不先清容器。
UpdateSignals 会把“尚未开始”的未来信号同过期信号一样永久删除，时间回拨或仿真循环后
无法恢复；它也不处理交通灯。所有输出指针、Lane 有效性、时间和数值有限性均未校验。
相交判断只是两个投影端点和 Lane 中心的启发式，不使用自车横向位置、信号首尾朝向、
真实几何交集或 Lane 拓扑；略微反向的投影还会因 kEPS 被接受。限速只使用 vel_range 的
上界，固定减速度、严格小于阈值和零/负速度边界均未解释。无信号返回 `kInf` 虽可作为
“无约束”哨兵，但调用方必须正确处理。停车状态接口“成功但未赋值”会传播未初始化数据，
属于 M1 必须优先消除的静态可确认错误。后续应改为不可变信号全集加按时刻查询，接入
地图/仿真信号源，并统一限速、红灯、停车线的 Lane-aware 时空约束输出。

## 41. M0.3c3：DataRenderer 局部感知与语义地图输入渲染

- 构造函数从绑定 SMM 读取 ego ID、GridMap 元信息和周车搜索半径，以
  `{height,width}` 和统一 resolution 动态创建工作栅格。每帧 Render 必须先从 VehicleSet
  取自车，再依次生成障碍图、完整/周边 LaneNet 和周车集合；
- 局部障碍图以自车为中心计算左下角并把世界坐标取整，先重置 UNKNOWN，再借助 OpenCV
  把圆形和多边形静态障碍实心填成 OCCUPIED。随后从自车几何中心向八个象限做 FOV
  ray casting，将可见障碍世界坐标跨帧保存在 `obs_grids_`；
- FakeMapper 以地图半高度的 80%（即全高度 40%）作为 x/y 共同保留阈值，删除方形
  范围外历史点，并把
  剩余点写回新栅格为 SCANNED_OCCUPIED；`free_grids_` 当前未参与建图；
- 周边 Lane 每帧展平全部 LaneRaw 采样点并重建 KD-tree，半径查询实际覆盖
  `2*surrounding_search_radius`，命中点按 Lane ID 去重后复制完整 LaneRaw。周车则线性
  筛选严格小于搜索半径的非 ego 车辆；
- 跟踪噪声只对 ego 0 生效，每 10 次 Render 打乱周车 ID 并选最多三辆，注入标准差
  0.2 m 横向、0.7 m 纵向和 0.22 rad 航向高斯噪声；只有航向噪声超过 1.5σ 且非
  brokencar 才加入 uncertain ID。最终全部派生数据写入 `UpdateSemanticMap`。

已确认的后续修复/验证点：构造函数无空 SMM/配置校验，裸 new 的 GridMap 因空析构而
泄漏；构造后 setter 只改元信息、不重建栅格。Render 忽略所有子步骤和最终更新错误并
固定成功，自车 ID 缺失会由 `.at` 抛异常。GridMap 原点只取整到 1 m 而非 resolution，
矩形地图的 height/width、x/y 半幅和射线半径混用第一维，圆半径截断、退化多边形和越界
坐标均未校验。历史障碍从不整体清空，按方形而非量测距离衰减，可能产生幽灵障碍和无界
细粒度坐标集合。Lane KD-tree 每帧全量重建，多个 updated/车辆/障碍 KD-tree 成员完全
未使用，空 LaneNet 和无效/负 Lane ID 也没有边界处理。周车排除比较 Vehicle 内部 ID 而
非容器 key。噪声随机引擎使用默认确定性种子；噪声只在第 10 帧实际注入，但 uncertain
ID 会在其后九帧继续回写，和恢复为无噪声的车辆状态不一致。位置受扰但航向未越阈值的
车辆又不会标记不确定。M1 应采用 RAII、事务式错误传播、可复用空间索引、带时间戳的
占据证据衰减，以及显式可复现实验 seed/噪声持续模型。

## 42. M0.3c4：RosAdapter 仿真消息解码与渲染触发

- `RosAdapter` 共享 ROS2 Node、非拥有地引用 SMM，并用裸 new 创建其拥有的 DataRenderer；
  构造函数立即 Init，订阅相对 topic `arena_info`、`arena_info_static` 和
  `arena_info_dynamic`，三个 QoS 深度均为 2；
- 完整消息路径一次解码 LaneNet、VehicleSet、ObstacleSet 和时间戳，立即调用 Render，
  随后在当前 executor 回调线程同步执行可选地图更新回调；
- 拆分路径用静态消息缓存 LaneNet/ObstacleSet 并永久置 ready 标记；动态消息更新
  VehicleSet 和时间戳，只有 ready 后才把最新动态数据与最近静态缓存组合渲染。静态消息
  自身时间戳不参与组合匹配；
- 更新回调保存为 `std::function<int(...)>`，但返回值被忽略。析构函数只删除 DataRenderer，
  SMM 和 Node 仍归外部所有。

已确认的后续修复/验证点：node/SMM/消息指针均不检查；SMM 为空会在 DataRenderer 构造
阶段立即解引用。类拥有裸指针但未禁止默认复制，复制 RosAdapter 会产生双重 delete 风险。
Init 既在构造中调用又保持 public，重复调用会重复创建订阅。完整与拆分两套 topic 始终
同时启用，若仿真器都发布会重复更新；三类回调在 MultiThreadedExecutor 下还会无锁并发
读写 LaneNet、VehicleSet、ObstacleSet、SMM 和回调函数。所有 Decoder/Render/回调错误码
被忽略；静态解码失败也会置 ready，动态消息可能使用空或旧缓存。拆分数据没有时间戳、
序号或 frame 一致性检查，静态更新与动态帧可能跨场景组合；完整路径与拆分路径也共享
同一缓存。空回调可被标记为已绑定，回调执行时间直接阻塞订阅处理。M1 应采用 unique_ptr、
不可复制语义、显式输入模式、callback group/锁或消息快照，以及带时间同步和错误状态的
单一渲染触发入口。

## 43. M0.3c5：SemanticMapManager 多图层 ROS2 可视化

- 每个 Visualizer 构造九个 `/vis/agent_<id>/...` 深度 1 publisher，覆盖自车、OccupancyGrid、
  原始/本地 Lane、SemanticBehavior、开环轨迹、意图概率、周车和限速；另准备 ego TF；
- 实时入口要求 SMM 时间戳大于 kEPS，用它构造 ROS Time，依次发布九个图层后发送
  `map -> ego_vehicle_vis_<id>` TF。显式时间戳入口不发送 TF；播放入口额外把删除 Lane ID
  传给原始/本地 Lane 过滤；
- 周车和自车复用通用车辆 Marker，brokencar 单独着色，key 车辆提高 alpha。原始 Lane
  每条发布中心线、首尾球和 ID 文本四个 Marker；本地 Lane 发布半透明洋红面片；
- 行为层直接可视化 SemanticBehavior。意图层在车辆上方用长度 `2*prob` 的黄色箭头表示
  LK/LCL/LCR；开环轨迹用逐状态圆柱加折线。限速每段用起终点六边形和文本四个 Marker，
  最大速度近零时显示 Red light/Forbidden；
- 除自车和 OccupancyGrid 外，变长 MarkerArray 大都记录上一帧 ADD 数量，并通过统一工具
  追加 DELETE Marker 清理残留 ID。

已确认的后续修复/验证点：构造函数不检查 node，topic/frame/颜色/尺度全部硬编码且创建
大量 publisher。实时入口 `rclcpp::Time(smm.time_stamp())` 把 double 秒传给以纳秒为核心的
构造接口，存在严重时间单位错误；各 SMM getter 又可能深拷贝大对象。TF 函数不使用已构造
的成员 broadcaster，而使用绑定首个实例 node 的函数静态 broadcaster，多 ego/生命周期
语义错误。局部 Lane 删除计数也是函数 static，所有 Visualizer 实例共享。原始 Lane 直接
解引用首尾采样，空 lane_points 会崩溃；删除列表和 key ID 均为嵌套线性查找。意图概率和
开环轨迹来自无序容器，Marker ID 会随遍历顺序漂移；Undefined 行为会沿 LK 方向显示，
概率不校验 [0,1]，ARROW 的 scale.z 未设置。开环与 Behavior 图层按每个状态生成 Marker，
带宽/CPU 随车辆×模态×时域增长，空轨迹仍生成空折线。限速把零速度编码为红灯，忽略
最小速度/有效时间，offset 恒零。Marker 工具返回码、publisher 状态和颜色键异常均未处理，
marker_lifetime_ 未使用。M1 应先修复时间/静态状态问题，再做稳定 namespace+语义 ID、
可配置图层、轨迹抽样和行为/风险分色。

## 44. M0.3c6a：SemanticMapManager 生命周期与更新流水线

- JSON 构造路径保存 ego ID/路径，裸 new ConfigLoader 后解析 AgentConfigInfo，并启动全局
  计时器；直接参数构造路径设置搜索半径、开环预测、左右轴约定，关闭噪声/日志、开启
  fast LUT 和 simple-lane 模式，但不创建 loader、设置栅格元信息或启动计时器；
- `UpdateSemanticMap` 先深拷贝时间、自车、完整/周边 LaneNet、GridMap、障碍坐标和周车，
  再固定按“语义 Lane → 本地 Lane/LUT → 语义车辆 → 关键车辆 → 可选开环预测 → 可选日志”
  顺序更新，最后无条件返回成功；
- SaveMapToLog 每次以 append 打开配置路径，复制周车并 insert ego，然后为每辆车写状态
  时间戳、ID、x/y、速度、加速度、航向、曲率和转角 CSV 行；无表头且容器顺序不稳定；
- 绝大多数 getter 返回完整值拷贝；另外暴露可变 obstacle_map 指针和内部 SemanticLaneSet
  只读指针。setter 都直接覆盖单个字段，不联动刷新派生缓存或验证一致性。

已确认的后续修复/验证点：默认构造不初始化 ego_id_/p_config_loader_，直接参数构造也不
初始化 loader；JSON 构造 new 的 loader 因空析构永久泄漏。默认复制/赋值会浅拷贝该裸
指针，使“是否有效/是否拥有”语义更混乱。配置解析返回码被忽略。Update 的所有阶段和
日志返回码都被忽略，部分失败后仍保留半更新状态并返回成功；时间戳不检查单调/有限性，
关闭 fast LUT 或开环预测也不清理旧缓存。局部 timer 未使用。日志不检查路径/打开/写入，
每行 endl 强制 flush，ego ID 已存在时 insert 不覆盖，也不记录车辆类型、尺寸、行为、地图
时间或实验配置。大量大对象 getter 深拷贝造成规划/可视化额外开销；可变内部指针绕过
不变量，内部只读指针又缺少并发/生命周期契约。M1 应采用 RAII 和显式 copy/move 语义、
不可变快照或读视图、事务式阶段错误传播，以及有 schema/版本/帧标识的结构化实验日志。

## 45. M0.3c6b：周车横向行为、语义关联与开环轨迹

- Naive 预测先把车辆投影到最近语义 Lane。右手轴下 `d>0.4 m` 且 `d_dot>0.35 m/s`、
  左 Lane 存在且可换时输出 one-hot LCL；对称负阈值输出 LCR；非右手轴时交换左右语义，
  其余情况 one-hot LK；
- MOBIL 路径为 LK/LCL/LCR 分别构造前后长度等于搜索半径的参考 Lane，查询 2.2 m 横向
  范围内前后车及 FrenetState，再把三组 Lane/车辆上下文交给 MobilBehaviorPrediction；
  但 UpdateSemanticVehicles 当前没有调用 MOBIL，只调用 Naive；
- 每辆周车无导航路径地匹配最近 Lane，执行 Naive 预测并取最大概率行为，再构造前向
  `max(10*v,50 m)`、后向 10 m 的固定参考 Lane。临时集合完成后 swap 提交；
- 开环预测启用时每帧清空旧结果，沿每辆语义周车的单一参考 Lane 调用 OnLaneFsPredictor，
  固定预测 5 s、步长 0.2 s，并按 Vehicle 内部 ID 保存状态序列。

已确认的后续修复/验证点：所有输出指针和数值均无校验；nearest Lane ID 存在但不在
SemanticLaneSet 时 `.at` 抛异常。Naive 是单帧硬阈值 one-hot，没有历史、转向灯、速度/
曲率自适应或不确定度；阈值边界直接落 LK，日志会在每个换道预测周期刷屏。MOBIL 准备
阶段忽略参考 Lane 和前后车查询错误，计算的 has_leading/following 标志没有传给下游，
可能把默认 Vehicle/FrenetState 当成有效上下文。语义车辆更新忽略最近 Lane、Naive、
最大概率和参考 Lane 的全部错误，仍固定成功；失败项会以 Invalid/Undefined/空 Lane
进入集合。插入和预测都使用 Vehicle 内部 ID 而非输入 map key，重复/错误 ID 会静默丢车。
轨迹包装忽略 predictor 错误并固定成功，预测时域/步长/Lane 有效性不检查；失败仍插入
空轨迹。预测只跟随单一最大概率模态，无法表达换道多模态占用。M1 应统一预测状态机和
错误契约，并引入带概率的多模态轨迹、历史平滑/意图特征及可校准不确定度。

## 46. M0.3c6c：SemanticLane、长本地 Lane 与 segment/local LUT

- 每帧先清空 SemanticLaneSet，只把 surrounding LaneNet 中成功由采样点生成连续 Lane 的
  LaneRaw 转为 SemanticLane，同时复制方向、父子、左右相邻、换道可用性、行为和原始长度；
- 随后裁剪局部一致性：相邻 Lane 不在当前 SemanticLaneSet 时关闭对应换道并写 Invalid，
  父子列表也删除集合外 ID。因此语义拓扑是周边窗口内的截断图，而非完整 LaneNet；
- fast LUT 先匹配自车当前 Lane，再选当前、左/左左、右/右右最多五条根 Lane；每条根
  Lane 从自车投影位置沿 child 递归到前向累计 250 m，沿 father 递归到后向 150 m；
- 对所有前后路径做笛卡尔积并去掉重复 root，调用高质量局部 Lane 拟合。成功路径按本帧
  递增 local ID 写入 local_lanes_、local_to_segment_lut_ 和 segment_to_local_lut_；
- 前/后递归在累计长度达阈值或无 child/father 时终止，后向路径在输出前反转为道路前进
  顺序。最后无论成功 Lane 数量多少都置 `has_fast_lut_=true`。

已确认的后续修复/验证点：单条 Lane 拟合失败被静默丢弃，集合更新仍成功；重复 ID insert
不覆盖，SemanticLane.length 沿用原始字段而非拟合 Lane 实长，也不检查左右/父子关系互反。
局部窗口裁剪会把地图中存在但不在 surrounding 集合的相邻/父子关系当成不存在，影响
换道和路由判断。fast LUT 最近 Lane 失败发生在清缓存前，上一帧 local_lanes_/LUT 和
has_fast_lut_ 会残留。根 Lane 使用 ID>0 判断，合法 ID 0 被排除；`.at` 假设 whole 与
semantic 集合包含全部当前/相邻 ID，缺失即抛异常。根投影错误被忽略，arc_len 可能无效。
递归把 Lane ID 声明为 decimal_t，且无环检测、深度/分支上限或重复路径去重；零长度环可
无限递归，分叉图的路径数和前后笛卡尔积可能指数增长。拼接失败仍最终把 LUT 标为有效，
local ID 又不跨帧稳定。M1 应以有向无环/visited 约束的长度受限图搜索替代递归，明确窗口
边界拓扑语义，并用稳定路径哈希、缓存增量更新和“至少一个有效 Lane”成功条件。

## 47. M0.3c6d：Lane 匹配、碰撞查询与有限 BFS 可达性

- 状态到 Lane 查询遍历全部局部 SemanticLane，把 x/y 投影到中心线，计算欧氏距离、弧长
  和归一化有符号航向差，只保留 10 m 内结果，并按距离优先存入有序 tuple set；
- 最近 Lane 在 1.5 m 内候选中优先选绝对航向差最小者。若无近候选或最佳差超过 90°，
  则按距离扫描有符号角差 `<π/2` 的首条 Lane；仍无结果时直接取距离最近 Lane。当前
  navi_path/拓扑代价选择代码全部被注释；
- 两车碰撞把参数/状态转为 OBB 后做 SAT 相交。车辆对环境的静态碰撞只查询 OBB 四顶点
  是否落在 OCCUPIED；动态碰撞仅在开环预测开启时，按时间差/0.2 s 四舍五入选择每辆
  周车预测状态并做 OBB 相交，超出预测下标范围直接跳过；
- 拓扑可达性从起始 Lane BFS，邻边包含 child 及可换左右 Lane，最多展开 20 节点；命中
  path 任一 ID 时返回首次发现路径的换道边数，搜索耗尽或截断都成功返回 false。

已确认的后续修复/验证点：Lane 投影、位置和航向查询错误码全部忽略，输出集合不清空，
NaN/无效弧长可进入排序。最近 Lane 的兜底条件漏掉 fabs，接近反向但为负的角差会被误判
为方向合适；1.5 m 内又只按角度不综合距离，navi_path 完全无效。静态碰撞只采四角，无法
发现障碍落在车身边/内部但不在顶点的情况；GridMap 越界错误被忽略，且 SCANNED_OCCUPIED
历史障碍不算碰撞。关闭 openloop_prediction 时完全不检查动态周车；开启时 `.at` 假设
每辆语义车都有预测，时间只做最近下标无插值，5 s 外直接视为无碰撞。所有 OBB/栅格
错误被吞掉。`CheckCollisionUsingStateVec` 只有声明无定义。BFS 的 20 节点上限把“截断”与
“不可达”混为一谈，未命中时不写 num_lane_changes；visited 只保留首次路径，换道数不
保证最小，邻接重复还可能把 child 误计作横向边。M1 应建立连续扫掠体/时序碰撞与明确
Unknown 策略，修复航向绝对值和导航约束，并让可达性返回截断状态及最小换道代价。

## 48. M0.3c6e：基于 Lane 偏移近似的关键车辆筛选

- 函数先把全部周车复制为 key 集合；成功匹配自车当前 Lane 后才清空并执行精筛。因此
  最近 Lane 失败会返回错误但保留“全部周车都是 key”的初始回退状态；
- key Lane 图包含当前 Lane、可换左右相邻 Lane，以及搜索半径内的 child/father。当前
  Lane 起点 offset 为 `-ego_arc_len`，相邻 Lane 直接复用该值；前向初始剩余长度也统一
  复用当前 Lane `length-ego_arc_len`；
- 前方关键距离由 `v_ego*max(5,v_ego/1.6)+100` 计算，再限制到 30--170 m。车辆近似纵距
  为其 Lane 起点 offset 加 Lane 上弧长；落在前方窗口即纳入，但当前 Lane 上弧长小于 ego
  的车辆被再次排除；
- 后方只展开相邻 Lane 的 father，不展开当前 Lane father；后车窗口为
  `max(20 m,5*|v_agent|)`，当前 Lane 后车仍显式排除。入选结果同步写 ID、SemanticVehicle
  和 Vehicle 三个容器。

已确认的后续修复/验证点：代码自身已用 `//! bug!` 标记 successor 长度逻辑。相邻 Lane
并不一定与当前 Lane 起点/弧长对齐，却复用 current offset；不同根 Lane 也复用当前 Lane
剩余长度。深层 successor 插入 key_lane_ids 时使用固定 len_sum 而非当前 len_expand，导致
纵距系统性错误；map insert 又不会修正重复 Lane 的更优 offset。前后展开没有 visited/
cycle 防护，`.at` 假设完整拓扑闭合。只展开相邻 Lane 的 predecessor 且排除当前 Lane 后车
会漏掉高速追尾风险；front_range 的固定 +100 m 和硬上下界未按制动/RSS 校准。车辆距离
默认 -1 会通过 `>2 m` 检查，所有值无有限性验证。筛选没有使用类内 rss_checker_，也不
使用 uncertain_vehicle_ids_，与安全/不确定性语义脱节。UpdateSemanticMap 又忽略本函数
错误，使“全部车辆回退”不可观测。M1 应在统一参考 Lane/Frenet 图上计算真实有符号距离，
用 RSS/TTC/可达占用选择关键交互体，并显式标识正常精筛、保守回退和拓扑失败状态。

## 49. M0.3c6f：行为参考 Lane 的拓扑拼接、采样与拟合

- 动态样本路径先投影 state 到目标 Lane。后方不足请求长度时沿 father 逐段扩展，前方沿
  child 扩展；每个分叉默认取 front，若 navi_path 命中则选第一个命中 ID。father 逆序
  收集后反转，再与当前/child 拼成道路前进顺序；
- 原始点拼接只保留第一条非空 Lane 的第 0 点，每个分段随后均从 index 1 开始。先生成
  长 Lane，再按请求前后窗口以 1 m 步长重采样；GetLocalLaneUsingLaneIds 使用 whole
  LaneNet，GetLocalLaneSamplesByState 使用 surrounding LaneNet；
- GetRefLane 先匹配当前 Lane并要求中心线距离不超过 2 m，再把 LK/Undefined 映射当前
  Lane、LCL/LCR 映射可用相邻 Lane。fast LUT 命中时直接返回包含目标 segment 的最小
  local ID 整条 Lane；未命中才动态采样指定长度并重新生成；
- 高质量生成使用累计弦长参数、固定 20 个 breaks 和 `1e6` 正则拟合；普通模式直接用
  LaneGenerator 的样本点生成。SampleLane 输出 `[s0,s1)`，终点不包含。

已确认的后续修复/验证点：所有输出指针、长度、step 和有限性不检查，father/child 遍历
没有 visited，拓扑环可无限循环；分叉只取第一个/导航首命中，无法比较几何连续性或路由
代价。投影和 SampleLane 的 Lane 查询错误均被忽略，原始分段若不严格首尾重合会产生跳点，
只有一个点的后续段被完全丢弃。两个调用点都把未初始化 `acc_dist_tmp` 传给 SampleLane
执行 `+=step`，构成未定义行为；step<=0 还会无限循环。采样终点遗漏、按名义 step 而非
实际弦长累计，裁剪上界也未统一 clamp 到长 Lane 终点。fast LUT 返回围绕 ego 构造的整条
Lane，忽略当前查询 state、navi_path、max_forward/back 和 high_quality；多个路径仅取最小
ID，且可能使用上一帧陈旧 LUT。高质量拟合不检查最少样本/重复点，固定 breaks/正则不随
长度自适应。无效 behavior 在 release 下 assert 消失后可能成功返回未初始化 target ID。
M1 应统一受限图搜索与连续性评分、初始化并验证采样契约，并让 fast LUT 返回可按 state/
路由裁剪的稳定候选而非任意整条 Lane。

## 50. M0.3c6g：Lane 采样式前后车、交通转发与未完成图距离

- 前车查询先投影参考状态，再以 `lat_range/1.4` 为纵向步长向前扫描最多 120 m；每个
  Lane 点线性遍历 VehicleSet，车辆中心进入半径 lat_range 的圆即命中，并返回
  `(120-delta_s)/120` 剩余比例；
- 后车查询向后最多 100 m 且不越过 Lane begin，以相同步长扫描；循环终点额外保留
  `2*lat_range` 裕量。前后车同一采样点有多个候选时都取无序容器遍历首项；
- GetSpeedLimit/GetTrafficStoppingState 只透传 TrafficSignalManager；后者当前仍是成功但
  不写输出的占位接口。IsLocalLaneContainsLane 在 fast LUT 有效时复制 local 对应 segment
  列表并线性查找；未知 local ID 会由 `.at` 抛异常；
- GetDistanceOnLaneNet 仅搭出 visited、优先集合和 child/左右邻边构造框架。它不弹出开放
  节点、不把 successor 加回队列、不累计 cost，也不使用两个 arc_len 或写 dist。

已确认的后续修复/验证点：前后车搜索是 Lane 点圆形采样而非车辆 Frenet/车身间距，可能
命中交叉道路车辆、自车自身或同采样点的非最近哈希项；不排除 reference vehicle。Lane
位置查询错误被忽略，搜索可越过 Lane end；lat_range<=0 会产生零/负步长和潜在死循环。
前向从一个步长后开始，后向又跳过接近 Lane begin 的区间；固定 120/100 m、采样分辨率
和 residual ratio 都不是实际保险杠净距。多个未使用变量表明静态占用/虚拟车逻辑未完成。
交通停车未赋值问题继续向上传播。本地 Lane contains 每次复制 vector，且 has_fast_lut
可能错误地对空/陈旧 LUT 为 true。最严重的是 GetDistanceOnLaneNet：lane0!=lane1 时 pq
永不变化导致无限循环；相同 Lane 虽退出却返回成功且 dist 未初始化。该函数当前无调用，
但属于 M1 必须删除、封禁或完整实现的静态错误。M1 应用一次车辆 Frenet 投影排序获取
真实前后净距，并实现带 arc-length 边界、稳定代价和终止证明的 Dijkstra/A*。

至此 M0.3c 已完成：SemanticMapManager 从 ROS 输入、感知渲染、语义 Lane/车辆、关键体、
预测、碰撞、参考 Lane 到可视化的职责链已经建立中文索引；所有发现的逻辑问题仅登记，
未混入本阶段注释提交，后续统一进入 M1 baseline 修复与回归用例设计。

## 51. M0.4a1：车辆控制器包装层

- `PIDControl` 保存 P/I/D 增益、固定 dt 和最多 1000 个误差历史。每次将
  `desired-true` 追加到 deque，积分项重新遍历全窗口，历史至少三项时用最近两项做后向
  差分微分，最后直接返回三项之和；
- `IntelligentVelocityControl` 每次创建临时 IntelligentDriverModel，写入自车/前车纵向
  位置速度，把自车负速度截为零，odeint 积分 dt 后再次把输出速度截为非负；
- `ContextIntelligentVelocityControl` 同样是无状态单步包装，但额外输入目标位置/速度和
  Context 参数；具体 IDM/上下文融合语义由下层模型决定；
- `PurePursuitControl` 直接计算 `atan2(2*wheelbase*sin(angle_diff),look_ahead_dist)`，不保存
  路径或控制历史。

已确认的后续修复/验证点：所有控制器都不检查输出指针、dt、有限性和参数物理范围。
PID 默认 dt=0.05 s，日志却标为 ms；dt=0 会使微分除零。积分每次 O(N) 重算、无 reset、
抗饱和或输出限幅，且先用 1001 项积分再弹出最旧项；两项历史时仍不计算可用微分。
IDM/CTX 包装只截断自车/输出速度，不验证前车/目标速度、位置间距或 Step 结果，模型异常
仍固定返回成功。Pure Pursuit 不归一化 angle_diff，零/负前视距离仍由 atan2 给出饱和或
反向几何结果，轴距也可非正。M1 应加入统一 Status/输入契约、PID O(1) 积分与 anti-windup、
控制限幅和可复现实验参数，并为零速/零前视/异常间距建立边界测试。

## 52. M0.4a2：odeint 纵向车辆模型

- IntelligentDriverModel 的内部数组为 `[s,v,s_front,v_front]`。Step 用 dt 作为积分终点和
  初始步长调用 odeint，微分算子用 common ACC 公式计算自车加速度，前车保持恒速；积分
  结果同步回公开 State；
- 私有 Linear 备用路径使用 IIDM 加速度和显式匀加速公式，但当前 Step 未调用；它会把
  制动限制为 hard braking 与 `v/dt` 中较小者，并输出两次调试 acc；
- Context 模型扩展为 `[s,v,s_front,v_front,s_target,v_target]`，先计算
  `v_ref=v_target+k_s(s_target-s)`，再用 `k_v(v_ref-v)` 得到并截断至 [-1,1] 的跟踪加速度；
  前车和目标都按恒速运动；
- Context 算子也调用 common IDM 计算 acc_idm，但 IdmState 未从当前六维状态赋值，且最终
  `acc=acc_track`，因此当前模型实际上完全忽略前车和 IDM 参数的控制作用。

已确认的后续修复/验证点：Step 不验证 dt>0/有限性，也不暴露 odeint 失败。更关键的是
odeint 系统函数第三参数是当前积分时间 t，两个 operator 都命名为 dt 并用于 `v/dt` 制动
限制；在 t=0 处存在除零/NaN 风险，且后续限制随绝对积分时间变化而非数值步长。IDM
加速度错误码被忽略，状态/参数无物理范围校验，积分可能产生负速或穿越前车；控制包装只
在最终输出截断，位置已经按异常轨迹推进。Context 的 idm_state 未填充、acc_idm 未使用是
明确的静态逻辑错误，使类名和接口宣称的跟驰能力不存在。头文件全局 using namespace
boost::placeholders 还污染包含者命名空间；空析构和重复状态同步没有必要。M1 应修正
odeint 回调时间语义，用显式步长约束或受控积分器，完整融合 IDM/目标跟踪（含优先级或
安全屏障），并增加跟驰、急刹、零 dt 和目标追踪的解析边界测试。

## 53. M0.4a3：运动学自行车与理想转向模型

- `VehicleModel` 的 odeint 状态为 `[x,y,yaw,steer,velocity]`，输入为前轮转角速度和纵向
  加速度。微分方程使用运动学自行车关系推进平面位置和航向；Step 后归一化 yaw、对转角
  施加对称机械限幅，再由 `tan(steer)/wheelbase` 回写曲率和本周期纵向加速度；
- `IdealSteerModel` 的 odeint 状态为 `[x,y,yaw,velocity,steer]`，上层输入则是目标前轮转角
  和目标速度。Step 先从当前曲率恢复转角，再限制目标非负速度和最大机械转角；
- `TruncateControl` 先按上一周期 acceleration 约束纵向 jerk 与加/减速度并重算可达速度，
  再由 `v^2*tan(steer)/wheelbase` 计算横向加速度，限制横向 jerk/加速度并反解可达转角，
  最后限制转角速度；积分时纵向加速度和转角速度在整个 dt 内保持常值；
- 两个模型都把公开 `common::State` 与固定长度 odeint 数组双向同步，不推进时间戳；时间由
  上层 forward simulator 管理。普通模型不在 set_control 中限幅，理想模型把控制约束集中
  到 Step/TruncateControl。

已确认的后续修复/验证点：`VehicleModel` 默认构造只初始化 2.5 m 轴距，
`max_steering_angle_` 未初始化，随后 Step 的转角比较和截断构成未定义行为。两个模型都不
检查 dt、轴距、状态、控制和参数的有限性/物理范围；dt=0 会使理想模型在加速度、jerk 和
转角速度计算中多次除零，轴距为零会使 yaw rate、曲率或横向加速度除零。普通模型允许
速度积分为负，set_control 的限幅仍是 TODO。理想模型虽然保存 `max_curvature_`，对应限速
逻辑却被注释，因此参数实际无效；低速横向加速度反解只用极小固定分母，可能生成激进转角，
反解和转角速率限制后也没有再次显式应用 `max_steering_angle_`。两个头文件在全局作用域
引入 boost placeholders，污染所有包含者命名空间。M1 应初始化并校验全部参数，统一
dt/有限性错误契约，建立低速稳定的曲率—横向加速度约束，真正应用最大曲率/机械转角，
并增加零 dt、零轴距、低速大转角、负速、jerk 饱和和长时积分边界测试。

## 54. M0.4b1：SSC 地图抽象接口与语义地图适配器

- `SscPlannerMapItf` 把 SSC 核心算法与 ROS/SemanticMapManager 隔离，统一暴露快照时间戳、
  自车/状态、行为参考 Lane、按 ID 查询的语义 Lane、二维障碍 GridMap、世界坐标障碍栅格、
  碰撞检查、离散横向行为，以及候选行为下的自车/周车前向 rollout；
- `SscPlannerAdapter` 持有 `shared_ptr<SemanticMapManager>`。ROS 服务端每个规划周期复制最新
  SemanticMapManager 后调用 set_map，适配器再从同一快照向 SscPlanner 提供数据；
- `GetEgoReferenceLane` 与 `GetLocalReferenceLane` 当前完全相同，都返回
  `ego_behavior().ref_lane` 并要求 Lane 有效；`GetEgoDiscretBehavior` 拒绝 kUndefined；
- 两个 forward trajectory 重载都只用 `forward_behaviors` 非空作为可用条件，三输出版本还
  深拷贝每个候选行为对应的周车 ID—轨迹 map；Lane、SemanticLaneSet、GridMap、障碍集合
  和 SemanticBehavior 均经值拷贝跨越接口边界；
- 碰撞接口委托 SemanticMapManager 的车辆参数/状态碰撞检查，Lane ID 查询先取得完整
  SemanticLaneSet，再只返回命中 SemanticLane 的中心线 Lane。

已确认的后续修复/验证点：抽象基类没有 virtual 析构函数，经基类指针释放派生对象会产生
未定义行为；所有接口均为非常量成员且使用裸输出指针，没有空指针、对象生命周期、快照
一致性或并发读取契约。set_map 接受 nullptr 仍把 is_valid_ 置 true，GetTimeStamp 又完全不
检查状态并直接解引用 map_；is_valid_ 与 shared_ptr 可脱节且没有失效/reset 路径。所有 getter
只检查独立标志，不检查输出指针；缺 Lane、未定义行为和无效适配器都混用 kWrongStatus。
两个参考 Lane 接口重复。forward rollout 只检查行为非空，不校验 behaviors、自车轨迹和
周车轨迹数量、时域、时间戳或车辆 ID 对齐，后续按下标关联可能越界或错配。适配器反复调用
按值 getter，单次请求可能重复深拷贝 SemanticBehavior；Lane ID 查询还复制完整 LaneSet，
障碍地图和预测轨迹的复制开销随场景规模增长。碰撞函数忽略底层 ErrorType 并固定返回成功，
底层失败时 res 可能未定义。接口对 vector/unordered_map 等还依赖传递 include。M1 应加入
virtual 析构、const/noexcept 与非空输出契约，以不可变 shared snapshot/read view 替代独立
有效标志和重复深拷贝，严格传播 Status，并为 rollout 建立候选数、时间轴、ID 和尺寸一致性
校验及空地图/底层碰撞失败测试。

## 55. M0.4b2：SSC Frenet 时空占用栅格与驾驶走廊

- `SscMap::Config` 默认建立 `1000×100×81` 的 `(s,d,t)` 栅格，分辨率分别为
  `0.25 m/0.2 m/0.1 s`，并统一保存纵横向速度/加速度界、单 cube 最大时间跨度及六方向
  膨胀步长。类同时 new 原始占用和按自车足迹膨胀后的两张 GridMap；
- Reset 清走廊/栅格，以 `s_initial-s_back_len`、横向居中位置和初始绝对时间戳设置两张地图
  原点。ConstructSscMap 先把静态 Frenet 障碍点沿全部时间层拉伸，再按每辆周车预测状态的
  时间戳，把车身顶点映射到单层 s/d 图并用 OpenCV fillPoly 写入多边形占用；
- InflateObstacleGrid 全量扫描原始三维栅格，以车辆参考点到车头/车尾距离和横向宽度换算
  s/d 膨胀格数，把非零占用扩张到第二张地图；
- 走廊构建从 initial Frenet state 和首个合法未来轨迹状态开始生成三维 seed。相邻 seed 的
  最小包围盒必须完全空闲，随后 cube 在 s/d 双向和 +t 方向按批次膨胀；后续 seed 若仍在
  当前 cube 内就合并，否则在最后内部 seed 处裁剪时间上界并开始新 cube；
- cube 的 s 膨胀还受初始速度和全局加/减速度估算的可达边界限制，单 cube 的 +t 跨度受
  `kMaxNumOfGridAlongTime` 限制。最终将离散上下界转回 Frenet 指标，并为每个连续 cube
  附加统一纵横向速度/加速度约束，形成优化器使用的 SpatioTemporalSemanticCubeNd。

已确认的后续修复/验证点：默认构造不初始化两张 GridMap 裸指针，带配置构造用 new 分配，
空析构却从不 delete；默认复制/赋值又会浅拷贝指针，形成永久泄漏和共享可变别名。默认两张
地图仅数据区约 16.2 MB。const getter 仍返回可写裸指针，配置和全部 corridor getter 深拷贝。
Config 不验证尺寸、分辨率、动力学界和膨胀步长；零/负分辨率可除零，零/负步长可令四方向
膨胀 while 永不结束。start_time_、map_valid_ 和 inters_for_cube_ 实际未使用，map_valid_
始终为 false；ClearDrivingCorridor 不清 final_corridor/validity，可能暴露陈旧派生结果。
Reset、ConstructSscMap、动态逐车填图、初始 cube 构造、膨胀和 GridMap 查询的大量 ErrorType
被忽略，部分候选碰撞时又保存 invalid corridor 后返回成功，而 seed 不足返回失败，错误契约
不一致。静态填图的地图 t 原点是 initial absolute timestamp，但写入点却使用 `k*resolution`
而未加 start_time，非零绝对时间下静态障碍可能全部落到地图外；静态点和动态多边形又统一
丢弃 `s<=0`，该条件没有相对地图原点/自车定义。动态预测只在离散状态时间层填多边形，不做
时间插值或 swept volume，预测步长大于 0.1 s 时中间层为空；空轨迹错误被上层吞掉，单车
轨迹还按值复制。车辆占用膨胀使用硬编码 `width-0.5 m`、floor 和开区间循环，窄车/低分辨率
下可能不写原始占用或产生非对称欠膨胀，且全图扫描再逐占用扩张复杂度很高，越界写错误被
忽略。seed 不检查初始坐标范围、时间单调、重复或跳变；GetInflationDirections、-t 膨胀和
CorridorRelaxation 主流程均未使用，六方向参数实际只有前五项部分生效。s 可达边界使用
`initial_v*1` 的经验补偿并忽略转换错误；free 检查忽略 GridMap 返回码，is_free 可能未定义。
离散坐标转换得到的是栅格中心而非 cell 外边界，连续 corridor 每侧可能欠半格；首 cube 只
校验初始 d，不校验 s/t/v/a，失败早退还会造成 validity 与 final corridor 数量不一致。M1
应以 RAII/value ownership 重构地图，集中验证 Config 和状态时间基准，用保守栅格边界、连续
扫掠占用和 uncertainty inflation 构图，统一候选级 Status/下标对齐，并用有终止证明的各向
膨胀与可达集约束替代经验补偿；需覆盖非零起始时间、空/稀疏预测、边界 seed、窄车、零步长、
无效 GridMap 查询和多候选部分失败测试。

## 56. M0.4b3：SSC 轨迹规划、Bezier QP 与行为选择

- Init 从 protobuf 文本读取 Planner/MapConfig，把地图尺寸、分辨率、动力学界和六方向膨胀
  参数映射到 SscMap；最小纵向速度被提升到 velocity singularity epsilon，随后 new 内部
  SscMap。Name 固定返回 `ssc_planner`；
- RunOnce 从同一地图快照取得时间、自车、行为参考 Lane、离散行为、障碍地图/点和多候选
  自车/周车 rollout。显式 set_initial_state 只覆盖下一轮起点，否则使用自车当前状态；速度
  高于 low-speed threshold 时使用 s/d 独立 Bezier 轨迹，低速使用 Frenet primitive；
- StateTransformForInputData 按“起始自车—候选自车 rollout—候选周车 rollout—静态障碍点”
  顺序扁平化全部状态和车身顶点，经同一参考 Lane 的 StateTransformer 批量投影，再按状态
  offset 和统一顶点数恢复 FsVehicle 层级。OpenMP 四线程路径由编译宏控制，当前默认关闭；
- 每个行为用专属周车 rollout 重新构造 SSC 占用，再沿对应自车 rollout 生成一个 corridor。
  全部离散 corridor 转为连续 cube 后，RunQpOptimization 为每个有效候选建立 s/d 起点位置、
  速度、加速度约束，以及终点位置/速度约束；完整 rollout 的时间和 s/d 点作为 proximity
  参考，交给 `SplineGenerator<5,2>` 生成五阶二维 Bezier spline；
- 低速模式无论 Bezier 是否成功，都会调用 FrenetPrimitive::Connect 连接初末 Frenet 状态；
  正常速度则丢弃 QP 失败候选。成功候选的 spline、primitive、corridor、参考状态和横向行为
  以相同下标保存；
- 最终选择优先匹配行为层发布的 ego_behavior；没有精确匹配时只尝试 LaneKeeping。trajectory
  getter 按速度模式在堆上复制并返回 FrenetPrimitiveTrajectory 或 FrenetBezierTrajectory。

已确认的后续修复/验证点：默认构造不初始化 map_itf_/p_ssc_map_，RunOnce 开头即解引用
map_itf_，也从不检查 map_valid_、接口 IsValid 或是否 Init；Init 用 new 分配 SscMap，但类无
析构且重复 Init 不释放旧对象。多个 const getter 返回可写裸指针或深拷贝大型 rollout/
corridor，`sur_vehicle_trajs_fs_` 从未填充。ReadConfig 不检查 open 和 TextFormat::Parse 结果，
FileInputStream 没有显式关闭 fd；配置缺字段只 assert，Release 下可能继续使用无效配置，函数
仍固定成功，Init 又忽略其返回值。RunOnce 的 static timer 不是多实例/多线程安全；显式起点
不校验与地图时间/位置的一致性。读取的二维 obstacle GridMap 在规划中完全未使用；Reset 和
多处下游错误被忽略。行为、自车轨迹、周车轨迹及转换后容器只靠隐含下标约定，循环直接用
behavior 数量索引，适配器数据错配时可越界。`is_fitting_only` 会完全跳过障碍填图；正常模式
又为节省时间禁用 ego footprint inflation，使 corridor 只约束参考点而不保证完整自车车身
避碰，是当前最严重的安全缺口。最终 ValidateTrajectory 被 `#if 0` 完全关闭。
批量投影假设所有车辆顶点数等于起始自车 num_v，且总动态点数严格等于状态数×num_v；任何
顶点生成/投影失败都会破坏拆包 offset。单线程转换忽略全部 ErrorType，可能写入未定义 Eigen
点；OpenMP 路径状态失败只保留时间戳、点失败同样被吞掉，却统一返回成功。空候选在打包时
跳过，拆包时仅 assert；Release 下会生成空 Frenet rollout，QP 随后对 `back()` 和 `[-1]`
访问。RunQpOptimization 只检查 cube 数与 behavior 数，不检查 validity、forward_trajs_fs 或
原始 rollout 数量；它强改最后 cube 的 t_ub，却用浮点 `!=` 要求相邻时间精确相等，只检查
时间连接，不检查 cube 时长、s/d 交叠、起终状态、动力学可达或参考时间单调。低速 primitive
Connect 返回值被忽略，候选仍保存默认 Bezier；函数即使零个有效候选也返回成功。行为选择
对重复行为取最后项、不按代价排序；唯一回退是 LaneKeeping，且回退后不更新公开行为语义。
被禁用的 Validate 也只核对起点位置/速度和 `|curvature|<=0.33`，不检查碰撞、corridor、时间、
有限性、速度/加速度/jerk、终端偏差或车辆完整 footprint。M1 应以显式生命周期和状态机封装
Init/Map/Run，严格验证候选 schema 与转换 offset，传播配置/投影/求解错误，恢复连续车身
碰撞和完整轨迹验证；候选选择需返回真实执行行为并按安全、可行性、舒适性和参考偏差排序，
同时加入空候选、错配容器、投影失败、QP infeasible、低速切换、浮点时间容差和降级测试。

## 57. M0.4b4：SSC ROS2 服务端、轨迹双缓冲与可视化

- SscPlannerServer 构造容量 100 的 ReaderWriterQueue，以及语义地图/SSC 两个可视化器；外部
  PushSemanticMap 按值入队，后台线程每周期排空队列并只保留最新 SemanticMapManager，再
  复制 shared snapshot 绑定 MapAdapter；默认 20 Hz，也可由构造参数指定；
- 首次无有效 executing trajectory 时直接 RunOnce，并用地图自车状态初始化重规划/控制历史。
  后续以 global_init_stamp 为时间网格原点，把当前时间向前量化一个周期，从 executing trajectory
  求拼接状态并规划 next trajectory；下一周期将 next 切为 executing，同时继续预生成下一段；
- 极低速奇异过滤以固定 2.85 m 轴距、45° 最大转角和相邻状态 dt 估算允许航向变化，超限时
  把姿态锁到上一历史值。轨迹执行时按 work_rate 向下量化当前时刻，采样状态并编码为 ROS2
  ControlSignal；规划失败保留旧执行轨迹并把 Marker 改黄、显示 Intervention Needed；
- PublishData 先发布 SemanticMapManager/TF 和 SSC 诊断，再发布执行轨迹控制及 Marker。SSC
  Visualizer 使用六个 `/vis/agent_<id>/ssc/*` topic，展示原始占用、自车 Frenet 轮廓、全部
  自车 rollout、第一候选的周车 rollout、DrivingCorridor seed/cube 和 0.02 s 采样的 QP 曲线；
- 时空图统一使用 `ssc_map` frame，大多数对象把绝对时间减 time_origin 映射到 z；动态 Marker
  记录上一帧数量，由 FillHeaderIdInMarkerArray 补删除项。

已确认的后续修复/验证点：Start 创建 detached thread 并捕获 this，类没有 stop/join/析构
协议，对象提前销毁会造成 use-after-free；is_replan_on_/is_map_updated_ 等共享状态不是原子。
work_rate 不检查正值/有限性，零值除零，负值产生非法周期，大于 1000 Hz 时整数毫秒截断为
0 形成忙循环；system_clock 调度又可能受系统时间跳变。队列满时 try_enqueue 失败被静默
丢弃；is_map_updated_ 首次置 true 后永不复位，即使没有新地图也持续按陈旧快照规划。Init/
Start 顺序、node/publisher/map/planner 返回状态均不校验，map_marker_pub_ 从未使用。PublishData
发生在本周期规划/切换前，控制和可视化天然落后一周期；当前时间早于轨迹 begin 时 floor 可
采样到域外，重规划量化时刻也可能超过 executing end。GetState/Filter 失败多数不触发明确
降级；duration 可负，wheel_base 局部变量未使用而公式硬编码 2.85，kBigEPS 阈值使过滤仅在
几乎零速生效。规划失败只置 intervention 并继续旧轨迹，没有最小风险停车轨迹。
可视化无空指针/时间区间/有限性检查，planner getter 反复深拷贝大型数据。QP/自车/前向轨迹
为空时直接 return，不发布 DELETE，RViz 会残留上一帧 Marker。周车只画第一个候选且 unordered
迭代导致 ID 跨帧不稳定；forward/surround 函数的 p_ssc_map 参数未使用。道路 AABB 横向固定
3.5 m，x 中心在已含 s_back 的 origin 上再次减 s_back；cube 长度按栅格中心差计算少一个
resolution。占用 GridMap Marker 未显式减 start_time，而 seed/corridor/QP 已减，可能在 z 轴
错层；corridor Header 又忽略传入 stamp 使用 node 当前时间。M1 应引入可 join 的 jthread/stop
状态机、steady/ROS clock 一致调度、输入新鲜度与丢帧统计、轨迹域 clamp 和安全降级；Marker
应共享稳定 frame/time 变换、稳定 ID、空帧删除和真实地图几何，并补线程销毁、零频率、陈旧
地图、轨迹边界、重规划失败、空候选及 RViz 残留测试。

## 58. M0.4b5：SSC protobuf、运行参数、RViz 与构建边界

- `ssc_config.proto` 使用 proto2 required 字段。PlannerCfg 定义零速奇异保护、2 m/s 高低速
  轨迹模式阈值、参考 rollout proximity 权重和 fitting-only 开关；MapCfg 定义 s/d/t 栅格
  尺寸/分辨率、后向范围、连续动力学界、单 cube 时间跨度和六方向膨胀步数；
- baseline 文本配置建立 `250×71×41` 栅格，分辨率 `1.0 m/0.2 m/0.2 s`，名义覆盖约
  250 m×14.2 m×8.2 s，向后保留 20 m。纵向速度界 0--50 m/s、加速度界 -6--3 m/s²，
  横向速度/加速度绝对界 2.5 m/s 和 1.25 m/s²；单 cube 最多向未来 2 格，每轮 s/d 膨胀
  5 格、+t/-t 各 1 格，且 `is_fitting_only=false`；
- CMake 从 schema 在 binary 目录生成 pb.cc/pb.h。`hkust_pl_ssc` 包含地图适配、SscMap、
  SscPlanner 和生成源码；`ssc_server_ros` 包含服务端/可视化并链接核心库。运行配置安装到
  `share/ssc_planner/config`，Python launch 正是从该包共享目录读取 `ssc_config.pb.txt`；
- RViz 配置以 `ssc_map` 为 Fixed/Target Frame，默认订阅 agent_0 的 ego、周车、自车候选、
  corridor、QP 和 3D GridMap 六个 MarkerArray；semantic voxel display 存在但默认关闭。该文件
  同时保存 Orbit 相机和本机窗口布局；
- `package.xml` 声明 ament_cmake、rclcpp/rclpy、common、SemanticMapManager、vehicle_msgs、
  protobuf 和 glog，并把构建类型导出为 ament_cmake。pb.txt 与 RViz 都是运行时/工具序列化
  数据，本阶段保持原样；参数职责由 schema 注释和本索引统一说明。

已确认的后续修复/验证点：schema 所有字段虽 required，却没有单位、范围、有限性或跨字段
约束；C++ 解析只检查 IsInitialized，不防零/负尺寸分辨率、非正膨胀步、颠倒动力学上下界和
fitting-only 的无障碍风险。name/version/status 不参与算法，baseline version 为空，实验日志
无法据此唯一追踪配置。名义覆盖按 size×resolution 计算，而 GridMap 有效中心跨度实际是
`(size-1)×resolution`，实验文档需区分；当前 8 s 时域也未与行为预测/重规划时域建立校验。
CMake 强制 `CMAKE_BUILD_TYPE=Release` 和全局 `-O3 -Wall`，覆盖用户/多配置生成器选择；设置的
`${PROJECT_SOURCE_DIR}/cmake` 目录不存在。它使用全局 include_directories、硬编码内部 target
名和传递依赖，而不是按 target 声明 public/private ament 依赖。最严重的安装错误是公开头
`ssc_planner.h` 包含生成的 `ssc_config.pb.h`，但安装规则只复制源码 include/，没有安装生成
头；下游在 install space 可能无法编译。RViz 目录和 proto schema 也未安装。CMake 查找并
导出 sensor_msgs/OpenMP/visualization_msgs 等，package.xml 却漏掉 visualization_msgs、
sensor_msgs、OpenMP、tf2、tf2_ros 和 tf2_geometry_msgs；反之 package 声明未使用 rclpy 和
rosidl_default_runtime。许可证仍是 TODO，版本 0.0.0、maintainer 邮箱也是占位值。Glog/Glog
包名大小写和 ament export 名称不统一。RViz topic 固定 agent_0，无法直接复用多车 node_id；
semantic voxel topic 没有对应 publisher，窗口坐标/二进制 QMainWindowState 带机器特定状态，
配置又未安装，用户不能从 install space 直接加载。M1 应增加结构化 ConfigValidator 和配置
hash/schema version，改为现代 target-based CMake，安装生成 protobuf 头、schema、config 和
RViz 资源，补齐 package.xml 依赖/许可证元数据，并为 installed-space 下游 include/link、包
共享路径、非法参数和多 agent RViz topic 建立静态/ROS2 集成检查。

至此 M0.4b 已完成：SSC 从 SemanticMapManager 数据适配、Frenet 三维占用、DrivingCorridor、
Bezier/primitive 轨迹、ROS2 双缓冲执行到配置/可视化的完整职责链均已建立中文索引；本阶段
只登记缺陷，不混入构建或算法修复。

## 59. M0.4c1：playground 场景 JSON/GeoJSON 加载

- `phy_simulator/basics.h` 当前只是 common/Eigen 类型的集中依赖入口，没有定义模块自有类型；
- ArenaLoader 保存 vehicle_set、obstacle map 和 LaneNet 三个路径。VehicleSet 解析遍历
  `vehicles.info`，读取 ID/subclass/type、初始 x/y/yaw/curvature/velocity/acceleration/steer，
  以及车宽、车长、轴距、前后悬、最大转角和纵横向加速度；最大转角由 degree 转 rad，
  `d_cr=length/2-rear_suspension`；
- 障碍地图按 GeoJSON FeatureCollection 读取，只保留 is_valid 非零 Feature；ID/is_spec 写入
  PolygonObstacle，几何从 MultiPolygon `coordinates[0][0]` 取得首个 polygon 外环；
- LaneNet 同样遍历 FeatureCollection。ID/length 来自 properties，dir 固定为 1；child/father
  是逗号分隔字符串，解析后排除 ID 0；left/right ID、换道可用性和 behavior 原样保存；中心线
  只读取 MultiLineString 第一条坐标序列，并以首末样本设置 start/final point；
- 三类结果都用 map insert 按 ID 写入调用者容器，完成后调用 print 输出整个集合。

已确认的后续修复/验证点：默认构造路径为空，三个 Parse 都不检查输出指针、路径、fstream
open/read 状态、JSON parse 异常、schema、类型、有限性或物理范围；缺字段/类型错误会由
nlohmann::json 或 stoi 抛异常，接口没有 catch。VehicleSet 返回 bool，另外两个返回 ErrorType，
但成功/失败契约不统一且当前除异常外始终报告成功；顶层 vehicles.num 被忽略。输出容器不先
清空，insert 遇重复 ID 又不覆盖，重复调用可能混合旧场景并静默保留旧对象。State 时间戳
没有场景字段，车辆 subclass/type 和参数组合不验证，负尺寸/轴距/加速度仍可进入仿真。
GeoJSON CRS/type/name 被忽略；障碍只支持首个 MultiPolygon 外环，丢弃其它 polygon、内洞和
非 MultiPolygon geometry，闭合重复端点原样保留，也不验证最少顶点、自交或 orientation。
Lane 只支持首条 MultiLineString；metadata length 不与几何重算，dir 强制 1，behavior 是未验证
自由字符串。空 child/father token 会 stoi 失败，合法 Lane ID 0 被当哨兵丢弃；左右/父子 ID
存在性、互反性和换道一致性都不检查。空中心线会在 begin/rbegin 解引用时产生未定义行为，
重复 Lane ID 静默丢弃。pedestrian_set_path_ 完全未使用，头文件 guard 仍错误沿用
CORE_SEMANTIC_MAP 命名。M1 应定义版本化 arena schema 和统一 Result/诊断列表，事务式解析到
临时对象后再提交，显式支持/拒绝 GeoJSON geometry 变体，校验 CRS、拓扑、几何和车辆物理
范围，并为缺文件、坏 JSON、空 Lane、重复 ID、多 polygon/洞、ID 0 和重复加载建立测试。

## 60. M0.4c2：多车运动学推进与临时障碍物

- 带路径构造先加载 VehicleSet/ObstacleSet/LaneNet，再为每辆车创建 VehicleModel，使用车辆
  wheelbase/max steering 参数并同步初始 State；Vehicle.type 当前不参与模型选择；
- UpdateSimulatorUsingSignalSet 要求信号数量等于车辆数量。每辆车按 ID 取控制：
  `is_openloop=false` 时用 steer_rate/acc 调用 VehicleModel::Step(dt)，true 时直接以 signal.state
  覆盖模型；最终状态同步回 VehicleSet；
- AddTemporaryObstacle 以输入点为中心生成不重复闭合点的轴对齐方形，type=1，ID 从 10000
  单调递增，同时写入总 ObstacleSet 和临时索引；RemoveTemporaryObstacle 用四顶点平均中心，
  删除到输入点距离小于给定半径的全部临时对象；
- LaneNet、ObstacleSet、VehicleSet 和 vehicle_ids getter 全部按值返回；ID 列表来自车辆
  unordered_map 的遍历顺序。

已确认的后续修复/验证点：默认构造 new 空路径 ArenaLoader 后立即解析，通常会在 JSON 读取
阶段抛异常；参数构造忽略加载/模型 setup 结果。p_arena_loader_ 从不 delete，类默认复制还会
浅拷贝裸指针。GetData/Setup/Update 及公开包装几乎固定返回 true，计时器未使用；assert 在
Release 消失后信号数量错误仍继续执行。Setup 不先清 model map/ID vector，重复调用会保留旧
模型并追加重复 ID。所有车辆无视配置 type 统一使用 VehicleModel，且没有 dt/有限性/控制
限幅、车辆—车辆或车辆—障碍物碰撞、道路边界、时间戳推进和并发保护。信号数量相等不代表
ID 集合相等，`.at(id)` 可抛异常；model find 不检查 end 就解引用。openloop 允许任意状态瞬移，
不验证几何/动力学连续性。临时障碍边长/删除半径不检查正值和有限性，零/负尺寸产生退化或
反向顶点；计数不回收并可能溢出。固定 ID 10000 可能与静态障碍冲突：总集合 insert 失败而
临时集合成功，后续删除会误删同 ID 静态障碍。删除按中心半径批量操作，没有返回实际删除
数量或指定 ID；中心计算也假设顶点非空。getter 深拷贝大型地图/车辆集合。M1 应采用 RAII、
显式不可复制或深复制语义，统一 Status 和事务式初始化；按 Vehicle.type 工厂创建模型，严格
校验 signal ID/dt/state/control 并加入碰撞/边界与时间推进。临时障碍应使用不冲突 ID 分配器、
句柄式增删和几何校验，并覆盖默认构造、重复 setup、错配 ID、缺模型、零 dt、openloop 跳变、
静态 ID 冲突、计数溢出及批量删除测试。

## 61. M0.4c3：物理仿真 ROS2 真值、Marker 与多频率主循环

- RosAdapter 借用 PhySimulation，创建相对 topic `arena_info`、`arena_info_static`、
  `arena_info_dynamic`。Encoder 分别把 LaneNet+VehicleSet+ObstacleSet、LaneNet+ObstacleSet、
  VehicleSet 编成 map frame 的完整/静态/动态 ArenaInfo，publisher 深度均为 10；
- Visualizer 使用固定绝对 topic 发布 VehicleSet、LaneNet 和 ObstacleSet。车辆由公共工具显示
  OBB/速度/转向；每条 Lane 显示中心线、首尾球和起点上方 ID 文本；障碍物统一转 Polygon
  MarkerArray。VisualizeData 使用 node 当前时钟，带 stamp 版本每帧按值取得三类仿真数据；
- planning node 固定以 500 Hz、dt=0.002 s 调用仿真更新；每辆车订阅
  `/ctrl/agent_<vehicle_id>` 并保存最新信号，启动前为全部车辆建立零控制。动态真值 100 Hz、
  静态真值 10 Hz、可视化 20 Hz，均以独立 next publish time 调度；
- `/initialpose` 与 `/move_base_simple/goal` 回调只解析并缓存 x/y/yaw 及接收标志。场景三路径
  作为 ROS 参数读取，launch 正常覆盖源码中的开发机绝对默认值。

已确认的后续修复/验证点：RosAdapter/Visualizer 默认构造不初始化 node、publishers 或
p_phy_sim_，参数构造也不初始化借用指针；所有发布函数直接解引用且不检查 Encoder 返回码。
仿真 getter 每次深拷贝完整地图/车辆。完整 ArenaInfo publisher 在当前 node 中从未调用。
frame 固定 map，QoS/可靠性/瞬态策略不可配置。Visualizer::SendTfWithStamp 只有声明没有定义，
一旦恢复调用会链接失败；当前 TF 调用被注释。可视化绝对 topic 绑定固定节点名，阻碍 namespace
和多实例；Marker ID 依赖 unordered_map 遍历或 7*vehicle_id，无稳定排序、旧 Marker 删除和
负/大 ID 防护。Lane 可视化再次无条件解引用空 lane_points；color_obb 未使用。
node 使用全局可变 signal set/subscription/state，当前单线程 spin_some 尚无数据竞争，但迁移
多线程 executor 会失效。Ctrl callback 用 operator[] 接受未知 ID，没有消息时间戳、新鲜度、
控制超时或有限性检查；丢失控制会永久保持上次值。initialpose/goal 的缓存和 flag 永远不被
读取。参数获取失败后仍以空路径继续；源码绝对默认路径不可移植，也无顶层异常处理。
固定 dt 不使用真实墙钟/ROS 时间差，负载抖动会使仿真时间与消息时间漂移；仿真 State 自身又
不推进时间戳。rclcpp::Rate 与 node clock/use_sim_time 语义未统一。输出调度一次只增加一个
period，严重延迟后会连续多轮追赶；静态大地图仍固定 10 Hz 重复发布。500 Hz 循环每次
spin_some 可能被回调负载拖慢，所有 Update/Publish 返回状态被忽略。M1 应引入有所有权/非空
契约的 adapter，补 SendTf 或删除接口，使用参数化 topic/frame/QoS 和稳定 Marker lifecycle；
主循环需基于统一时钟推进、检测控制 freshness/未知 ID、传播失败并让初始/目标输入具有明确
业务语义，同时覆盖未绑定发布、TF 链接、空 Lane、多实例 topic、控制超时、未知 ID、sim time
暂停/跳变和循环超时测试。

至此 M0.4c 已完成：从场景 JSON/GeoJSON、车辆模型推进、临时障碍物，到 ROS2 真值和 Marker
发布的物理仿真职责链已经建立中文索引；逻辑与构建缺陷留待 M1 集中修复。

## 62. M0.4d1：GeoJSON 局部化工具与物理仿真启动入口

- `proc_geojson.py` 从固定 QGIS 工程目录读取 pt_feat/lane_net/obstacles 三份 GeoJSON，在 point
  features 中查找 name=origin 的坐标，并从每个 Lane/障碍顶点减去该原点；结果写为
  lane_net_norm.json 和 obstacles_norm.json，随后用中心线和半透明障碍 polygon 绘制人工检查图；
- ROS2 `phy_simulator_planning_launch.py` 声明静态/动态 ArenaInfo topic 和 playground 名，
  从 playgrounds 包 share 目录拼接 vehicle_set、obstacles_norm、lane_net_norm 三路径，注入
  phy_simulator_planning_node 并完成真值 topic remap；
- 该 launch 还从 phy_simulator 包 share 目录无条件包含 joy_ctrl_launch.py，后者只启动
  `/dev/input/js0` 的 joy_node；启动前会打印最终 topic、场景和三份资源路径；
- 同目录两个 `.launch` 是使用 `$(find ...)`、`type=` 和私有 remap 语法的 ROS1 遗留等价入口。

已确认的后续修复/验证点：GeoJSON 脚本没有 main/function/CLI，导入即执行；data_folder 依赖
当前工作目录和固定 highway 工程，不能选择输入/输出场景。文件 open/read/write、JSON schema、
origin 是否唯一存在、坐标有限性均不检查；缺 origin 时变量未定义，多 origin 取最后项。与
ArenaLoader 一样只处理首条 MultiLineString 和首个 MultiPolygon 外环，其余 polygon/洞丢失。
输入/输出文件句柄不用 context manager 关闭，json.dump 无缩进/排序/原子替换。大量 matplotlib、
math/pprint import 未使用；随机颜色无 seed，交互 plt.show 阻塞且不适合 CI/headless，Polygon
位置参数还可能受新版 Matplotlib API 变化影响。脚本不校验输出能否被 ArenaLoader 重新加载，
也不保存源 CRS/origin/工具版本 hash 作为实验溯源。
ROS2 仿真 launch 无条件要求 phy_simulator/playgrounds 已正确安装资源；场景名不校验，缺文件
只在节点运行期失败。joy_node 与当前仿真器没有直接订阅链，却总被启动；设备固定 Linux
`/dev/input/js0`，在无 joystick、容器或不同权限环境下产生无关错误，也没有 launch argument
或条件开关。use_sim_time、仿真/发布频率、frame、控制 topic 和 QoS 都不可配置。joy 文件保留
大量未使用模板 import 注释。ROS1 XML 入口在 ROS2 包中不可执行且可能误导；其 remap 使用
`~arena_info_*`，与当前相对 publisher 名也不等价。M1 应把转换器改为可测试 CLI/library，
支持完整 GeoJSON geometry、确定性无头验证图和原子输出，并生成 manifest/hash；ROS2 launch
应提供 joystick 条件、设备/频率/use_sim_time/topic 参数和资源存在性检查，ROS1 文件则迁移
到明确 legacy 目录或删除，并覆盖缺场景、无 origin、多 polygon、headless 和无 joystick 测试。

## 63. M0.4d2：物理仿真构建、安装与综合 RViz 面板

- CMake 使用 C++17/Release/`-O3 -Wall`，查找 ament、rclcpp、visualization/sensor/geometry
  messages、Eigen3、common、vehicle_model 和 vehicle_msgs；
- `phy_simulator_lib` 编译 ArenaLoader/PhySimulation 并链接 dw、common、vehicle_model；
  `phy_simulator_planning_node` 额外编译 main、Visualizer、RosAdapter，链接算法库和消息依赖；
- 库与节点分别安装到 lib 和 `lib/phy_simulator`，公开 include 树与 launch 目录安装到标准
  include/share 路径；ament 导出算法库名、依赖及头路径；
- RViz 使用 map Fixed Frame，启用物理仿真 ObstacleSet/LaneSet，VehicleSet 默认关闭；同时
  集成 agent_0 的语义地图、行为、SSC 执行轨迹、RSS/关键轨迹等面板，以及默认关闭的 agent_9
  debug 组。当前视角是跟随 `ego_vehicle_vis_0` 的 ThirdPersonFollower，TF display 默认关闭；
- package.xml 目前只声明 rclcpp/rclpy、vehicle_msgs、common、vehicle_model 和 ros2launch，
  版本/描述/maintainer/license 保持 baseline 占位值。RViz 是工具生成资源，本阶段保持原样。

已确认的后续修复/验证点：CMake 强制 Release 和全局优化/警告，覆盖用户构建类型；使用全局
include/link 而非 target-based ament_target_dependencies。`${semantics_msgs_INCLUDE_DIRS}` 没有
find_package，nlohmann json 和 backward/dw 也没有显式发现；Linux 专用 dw 阻碍非 Linux 构建。
ROS2 节点直接使用 visualization_msgs、geometry_msgs、sensor_msgs、rclcpp、vehicle_msgs，
却主要依赖传递链接。install 声明 `EXPORT export_phy_simulator`，但没有 ament_export_targets，
该 export set 对下游无效；只导出 phy_simulator_lib，不导出命名空间 target。公开 include 树
包含 ROS adapter/visualizer 头，因此下游还需要 manifest 未完整声明的 ROS 依赖。
package.xml 漏掉 visualization_msgs、sensor_msgs、geometry_msgs、Eigen3、tf2_ros，以及 launch
文件运行时需要的 launch、launch_ros、ament_index_python、joy 和 playgrounds；反而声明未用
rclpy。版本 0.0.0、TODO license/邮箱不满足正式发布要求。CMake 不安装 rviz 和 tools，用户从
install space 无法加载综合面板或复用 GeoJSON 工具。
RViz 资源混合物理仿真、agent_0 规划和 agent_9 debug，topic/agent ID 固定，无法按 ego_id 或
namespace 参数化；AgentDebug 名称与 agent_9 topic 不一致，关键轨迹 topic 还拼成 `critial`。
VehicleSet/TF/Grid 默认关闭，当前 Visualizer 又不发布 TF，ThirdPersonFollower target 可能缺失。
配置包含重复 Help 面板、机器特定窗口几何/QMainWindowState 和大量与 phy_simulator 包本身无关
的规划显示。M1 应采用现代 target/export/install 规则、补全 manifest 和平台条件链接，安装
RViz/tools；把综合面板拆为可参数化的 simulator 与 planning 两套配置，并增加 install-space
find_package/link、ament lint、manifest dependency、RViz topic 存在性和多 agent namespace 检查。

## 64. M0.4d3：四套 playground 场景资源与一致性审计

- playgrounds 是纯 ament 资源包，安装 highway_lite、highway_v1.0、ring_small_v1.0、
  ring_tiny_v1.0 四个目录；每套场景包含 agent_config、vehicle_set、lane_net_norm 和
  obstacles_norm 四份 JSON；
- agent_config 按 agent ID 保存 obstacle GridMap 宽高/分辨率、周边搜索半径、open-loop
  prediction 和 fast Lane LUT 开关；vehicle_set 保存同 ID 车辆的类型、初始 State 和几何/
  动力学参数；后两份是局部坐标 GeoJSON FeatureCollection；
- highway_lite 有 11 辆车/agent、39 条 Lane、39 个有效障碍；highway_v1.0 有 11 辆车/agent、
  107 条 Lane、62 个障碍（1 个 is_valid=false）；ring_small 有 14 辆车/agent、30 条 Lane、
  16 个障碍（3 个无效）；ring_tiny 有 15 辆车、13 个 agent、8 条 Lane、13 个障碍（1 个无效）；
- 本次结构化审计确认四场景 vehicle/agent/Lane/obstacle ID 自身均无重复，Lane 均非空，所有
  child/father/left/right 非零引用都指向现有 Lane；车辆/agent 核心数值均为有限数，地图尺寸、
  分辨率、搜索半径及车宽/长/轴距均为正，静态障碍 ID 均小于临时障碍偏移 10000；
- Lane metadata length 与样本折线长度最大绝对差分别约 0.220/0.345/0.080/0.042 m；当前四套
  障碍 geometry 均为单 polygon、无洞的 MultiPolygon，符合 ArenaLoader 的有限解析能力。

已确认的后续修复/验证点：ring_small 的 agent_config.info 和 vehicle_set.info 实际各 14 项，
两个 num 字段却都为 11；ring_tiny agent 实际 13、vehicle 实际 15，两个 num 仍为 11，且车辆
ID 1002/1003 没有对应 agent_config。ArenaLoader 当前忽略 num，因此仿真仍加载 info 全量，
但任何信任 num 的工具会少分配/漏处理；缺 agent 配置的车辆无法直接创建对应语义地图管理器。
四场景没有 schema/version/单位/坐标原点/生成工具 hash/随机种子 manifest，无法证明数据由
哪次 QGIS/脚本生成。Lane 长度存在小幅 metadata 偏差，未定义允许公差；拓扑只验证“引用存在”，
尚未验证 father-child、left-right 互反、换道标志对称、几何端点连续或 Lane 交叉合法性。车辆
初始位置是否落在可行 Lane、车间/障碍碰撞、agent type 与 vehicle subclass/type 一致性也未
检查。GeoJSON 中无效障碍仍保留在文件，由 loader 运行时过滤；不同消费者可能处理不一致。
CMake 第一条 install 已安装四场景，末尾又重复安装；中间还安装不存在的 `data/` 目录，干净
构建时可能直接报错。资源包设置无意义的 C++17；package version 0.0.0、TODO license/maintainer
和泛化 description 不满足发布/论文归档。M1 应引入 JSON Schema/场景 manifest 和统一 linter，
修正 num/ID 集，验证拓扑互反、几何连续、初始碰撞和 agent-vehicle 对齐，并让 CMake 只安装
存在目录一次；每个实验场景应记录 schema version、生成命令、源 CRS/origin、hash 和用途标签。

## 65. M0.4d4：MPDM/EUDM + SSC 集成入口与公平对照边界

- 两个入口都创建一个 SemanticMapManager/RosAdapter，地图更新回调把快照推给行为服务器，
  行为更新回调再把带决策结果的快照推给 SscPlannerServer；行为与 SSC 工作线程均为 20 Hz，
  main 以 100 Hz spin_some 派发 ROS2 回调；
- MPDM 入口使用 BehaviorPlannerServer，固定 autonomous level 3 并启用 HMI；EUDM 入口使用
  EudmPlannerServer 并额外读取 eudm protobuf 配置。两者共享相同 SemanticMapManager 和 SSC
  运动规划器，是后续算法对照应保持不变的下层边界；
- ROS2 Python launch 均从 playgrounds/ssc_planner/eudm_planner 包 share 路径解析配置，并重映射
  静态/动态 ArenaInfo 输入及 `/ctrl/agent_0` 输出；同目录 XML 是 ROS1 风格历史入口；
- CMake 编译两个 executable，连接对应行为库、ssc、protobuf/glog、OpenMP（MPDM）及
  OOQP/BLAS/MA27/gfortran 数值后端，安装节点到 `lib/planning_integrated`、launch 到 share；
- package.xml 声明 ROS2、common、SemanticMapManager、两个行为规划器、SSC、protobuf/glog 和
  消息依赖；该包不实现算法库或消息接口。

已确认的后续修复/验证点：两个 main 在参数获取失败后只记录错误，仍以空配置路径/默认值
继续；ego_id、desired_vel 和路径不做范围/存在性/配置 ID 一致性校验。launch 传入
`use_sim_state`，main 未 declare，SscPlannerServer::Init 中对应 declare 也被注释；在默认不自动
声明 override 的 ROS2 NodeOptions 下，get_parameter 可能失败或抛 ParameterNotDeclaredException。
全局 server 指针和两个 detached 后台线程没有 join/stop；队列入队失败不可见。异常 catch 直接
return -1 而不调用 rclcpp::shutdown。回调固定返回 0，不能传播 backpressure/处理错误；100 Hz
spin_some 与工作线程状态没有并发契约。
更关键的是默认 Python launch 不能作为公平算法对照：MPDM 使用 highway_lite、desired_vel=60
m/s，EUDM 使用 highway_v1.0、desired_vel=20 m/s；legacy XML 虽都为 20 m/s，场景仍不同。
MPDM 没有显式行为配置路径，且启用 HMI/level 3；EUDM 使用独立 protobuf，除算法外还有配置
和交互模式差异。论文实验必须由统一 scenario manifest 同时启动两者，锁定 ego/traffic seed、
期望速度、地图、SSC、车辆模型、预测时域、计算资源和日志 schema，只改变 behavior algorithm。
CMake 强制 Release/O3、全局 flags/include，并硬编码 Linux dw、OOQP、BLAS、MA27、gfortran；
`if(OPENMP_FOUND)` 大小写变量可能不匹配 FindOpenMP 的 OpenMP_FOUND，EUDM target 又未显式
链接 OpenMP。`ament_export_dependencies(test_ssc_with_eudm ...)` 把 executable 名当依赖包，
而本包是无公共库/头的叶子应用，本不应导出该集合。package.xml 漏掉 OpenMP/数值后端及
launch_ros、ament_index_python、playgrounds 等运行依赖，保留未用 rclpy 和占位许可证/版本。
M1 应建立统一 experiment launcher/config matrix、参数声明和启动前 fail-fast 校验，显式管理
server 线程生命周期与错误传播；构建改为 target-based 条件依赖，并增加 MPDM/EUDM 配置 diff
门禁、相同场景/速度/seed 回归、undeclared 参数、缺配置、节点关闭及 install-space launch 测试。

至此 M0.4 已完成：车辆模型、SSC、物理仿真、playground 数据和 MPDM/EUDM 集成入口均已逐层
建立中文职责与静态风险索引。下一阶段 M0.5 将执行全仓覆盖审计，确认遗漏后再进入 M1 修复。
