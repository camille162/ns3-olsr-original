# ns3-olsr-original
find where to change
src/etx-olsr/
├── CMakeLists.txt                          ✅ 已有
├── helper/
│   ├── etx-olsr-helper.h                  ✅ 已有
│   └── etx-olsr-helper.cc                 ✅ 本次新增（之前缺失）
└── model/
    ├── etx-olsr-routing-protocol.h        ✅ 已有
    └── etx-olsr-routing-protocol.cc       ✅ 已有
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
