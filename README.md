# ns3-olsr-original

单一 contrib 模块 **`etx-olsr`**（不再单独维护 `src/olsr`）：RFC 报文状态与 hop-count 实现在带 `etx-olsr-` 前缀的源文件中；运行时类型仍为 `ns3::olsr::PacketHeader` / `ns3::olsr::RoutingProtocol`（基线）与 `ns3::etxolsr::RoutingProtocol`（ETX 扩展）。

**注意：** 若完整 ns-3 树里仍启用官方内置的 `src/olsr` 模块，会与这里的 `ns3::olsr::*` 实现重复链接；集成时请关闭其一（通常只保留本 contrib）。

```
src/etx-olsr/
├── CMakeLists.txt
├── helper/
│   ├── etx-olsr-helper.{h,cc}           # 安装 ETX 扩展协议
│   ├── etx-olsr-hopcount-helper.{h,cc}   # 安装 hop-count OLSR（原 OlsrHelper）
│   └── control-agent-helper.{h,cc}
└── model/
    ├── etx-olsr-msg-header.{h,cc}        # HELLO/TC/… 报文（原 olsr-header）
    ├── etx-olsr-repositories.h
    ├── etx-olsr-state.{h,cc}
    ├── etx-olsr-hopcount-routing-protocol.{h,cc}
    ├── etx-olsr-routing-protocol.{h,cc}  # ETX 度量路由
    ├── etx-tc-metric-trailer.{h,cc}
    └── control-agent.{h,cc}
```
实现的功能：ETX-OLSR 路由协议
这个模块在 ns-3 仿真平台上实现了 基于 ETX 链路质量度量的 OLSR 路由协议，是对标准 OLSR（RFC 3626）的优化改进。

1. 什么是 ETX？
ETX（Expected Transmission Count，期望传输次数）是一种链路质量度量值：

ETX = 1/PRR（包接收率）
ETX 越小 = 链路质量越好
标准 OLSR 只按跳数选路，ETX-OLSR 按累计链路质量选路
2. 核心功能模块
功能	实现方式
ETX 估算	用 EWMA（指数加权移动平均）实时跟踪每个邻居的包接收率 PRR
缺失 HELLO 惩罚	根据发送方声明的 hTime 推算错过了几个 HELLO，每个缺失算作 PRR=0 的样本
路由计算	Bellman-Ford 风格，以累计 ETX 为代价，替代跳数作为最优路径判断标准
MPR 选择	保留 RFC 3626 标准的多点中继机制（不变）
HNA 支持	保留标准 OLSR 的主机与网络关联公告
完整控制面	HELLO / TC / MID / HNA 消息收发与处理全部实现
3. 本次新增的 etx-olsr-helper.cc
Helper 类提供了对外的用户接口，让上层仿真脚本可以方便地安装协议：

C++
EtxOlsrHelper etxOlsr;
etxOlsr.SetAttribute("EtxAlpha", DoubleValue(0.8));  // 设置 EWMA 平滑系数

Ipv4ListRoutingHelper list;
list.Add(etxOlsr, 10);

InternetStackHelper internet;
internet.SetRoutingHelper(list);
internet.Install(nodes);  // 一键部署到所有节点
它实现了四个方法：

Copy() — 克隆 helper（ns-3 框架要求）
Create(node) — 在指定节点上实例化路由协议对象
SetAttribute() — 设置协议参数（如 EtxAlpha、InitialEtx）
AssignStreams() — 为随机数分配确定性流，保证仿真可重复
4. 可配置参数
参数	默认值	含义
EtxAlpha	0.7	EWMA 平滑系数（0~1），越大历史权重越重
InitialEtx	3.0	未知链路的初始 ETX 估计值
HelloInterval	2s	HELLO 消息发送间隔
TcInterval	5s	TC 消息发送间隔
Willingness	DEFAULT	转发流量的意愿

5. 分布式 ControlAgent（轻量控制面）
- `model/control-agent.{h,cc}`：`ns3::etxolsr::ControlAgent` 应用，**每节点一个**，周期性读取本地 `CollectControlPlaneSignals()`（奖励方差、EWMA 时延、近 1 s 路径切换率），用规则调整 `MabC` 与 `SwitchPenalty`。
- **平滑与记忆**：先由基线 + 规则得到 `targetMabC` / `targetSwitchPen`，再按 `SmoothingBeta`（默认 0.35）做 `out=(1-β)*当前+β*目标`，避免每秒从基线“失忆”重算；`ContinuousSigmaScaling=true`（默认）时在 `SigmaLow`～`SigmaHigh` 之间对 σ 连续插值调节探索强度，减少硬阈值跳变。
- `helper/control-agent-helper.{h,cc}`：`ControlAgentHelper` 批量安装。
- 仿真 `main.cc` 在 `routing==etx` 且 `enableControlAgent==true`（默认）时对所有 UAV 安装；关闭：`--enableControlAgent=false`。
- 路由侧新增 API：`CollectControlPlaneSignals`、`SetAdaptiveHyperparameters`、`GetMabC` / `GetSwitchPenalty`。

---

## 6. 链接 `sim` 时出现 `undefined reference to ControlAgentHelper`

`main.cc` 使用了 `ControlAgentHelper`，实现位于 `src/etx-olsr/helper/control-agent-helper.cc`（随 **`etx-olsr` 模块**一起编译进静态库）。若自定义 CMake 里 **`sim` 未链接该模块**，会在链接阶段报错。

**处理：**

1. 确认 `src/etx-olsr/CMakeLists.txt` 的 `SOURCE_FILES` 包含 `control-agent-helper.cc` 与 `control-agent.cc`（本仓库已包含）。
2. 在定义 `sim` 的 `CMakeLists.txt` 里增加对模块库的链接，例如：

```cmake
target_link_libraries(sim PRIVATE etx-olsr)
```

具体目标名可能是 `etx-olsr`、`ns3-etx-olsr` 等，可在 `build` 目录搜索：`grep -r etx-olsr CMakeFiles/`。更细的说明见 `cmake/link-sim-notes.txt`。

**不想改 CMake 的临时办法**：在 ns-3 里不要单独建 `sim` 目标，改用 `scratch` 或官方推荐方式把 `main.cc` 放进会 **自动链接已启用 contrib 模块** 的构建目标（取决于你的 ns-3 版本与目录布局）。
