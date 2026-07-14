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
- [ ] M0.3c SemanticMapManager 支撑组件与主类。
- [x] M0.3c1 SemanticMapManager 基础配置类型与 JSON ConfigLoader。
- [x] M0.3c2 TrafficSignalManager。
- [ ] M0.3c3 DataRenderer。
- [ ] M0.3c4 ROS adapter。
- [ ] M0.3c5 SemanticMapManager visualizer。
- [ ] M0.3c6 SemanticMapManager 主类分段审计。
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
