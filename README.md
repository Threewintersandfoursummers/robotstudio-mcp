# RobotStudio 6.08 MCP 接入

把 ABB RobotStudio 6.08 接进 WorkBuddy：让助手直接建工站、摆机器人、算正逆解、建 RAPID 轨迹、导出代码、控制仿真。

## 架构

```
WorkBuddy  ──stdio(MCP JSON-RPC)──▶  rs_mcp_server.py
                                        │  loopback TCP 127.0.0.1:47950, 一行一个 JSON
                                        ▼
                            RsMcpBridge.dll（General Add-in，跑在 RobotStudio.exe 进程内）
                                        │  RobotStudio .NET API（UI 线程 marshal）
                                        ▼
                                  工站 / 机器人 / RAPID 模型
```

为什么是 Add-in：RobotStudio 6.x 的 API 只能在进程内使用，没有对外自动化接口。
所以放一个通用 Add-in 到 `Bin/Addins`，由它在进程里开一个回环 socket 当命令总线。

## 文件

| 路径 | 说明 |
|---|---|
| `C:\Users\Administrator\.workbuddy\mcp-servers\robotstudio\addin\` | Add-in 源码（C# / net48 / x64） |
| `...\RobotStudio 6.08\Bin\Addins\RsMcpBridge.dll` | 已部署的 Add-in |
| `...\RobotStudio 6.08\Bin\Addins\RsMcpBridge.rsaddin` | 清单（autoLoad=true, AddInType=General） |
| `C:\Users\Administrator\.workbuddy\mcp-servers\robotstudio\rs_mcp_server.py` | MCP 服务端 |
| `%LOCALAPPDATA%\RsMcpBridge\bridge.log` | Add-in 运行日志 |
| `%LOCALAPPDATA%\RsMcpBridge\port.txt` | 实际监听端口 |

重新编译并部署：

```bash
cd "C:/Users/Administrator/.workbuddy/mcp-servers/robotstudio/addin"
"C:/Program Files/dotnet/dotnet.exe" build -v q --nologo
cp bin/Debug/RsMcpBridge.dll "C:/Program Files (x86)/ABB Industrial IT/Robotics IT/RobotStudio 6.08/Bin/Addins/"
```

改完必须**重启 RobotStudio**（Add-in 只在启动时加载）。

## 用法

1. 打开 RobotStudio 6.08（快捷方式即可），日志出现 `Listening on 127.0.0.1:47950` 说明桥接就绪。
2. 在 WorkBuddy 里重启一次会话，让 `robotstudio` 这组工具生效。
3. 直接用自然语言提要求，或点名工具调用。

## 工具（27 个）

- **链路**：`rs_status`、`rs_launch`（顺带启动 RS / 打开工站）
- **工站**：`rs_new_station`、`rs_open_station`、`rs_save_station`、`rs_station_info`
- **机器人**：`rs_list_robot_library`（341 个型号）、`rs_add_robot`、`rs_list_robots`、`rs_get_joints`、`rs_set_joints`、`rs_home`、`rs_joint_limits`
- **运动学**：`rs_fk`、`rs_ik`、`rs_reach`
- **RAPID**：`rs_list_tasks`、`rs_list_targets`、`rs_add_target`、`rs_add_routine`、`rs_add_move`、`rs_show_code`
- **仿真**：`rs_sim_state`、`rs_sim_control`
- **I/O**：`rs_io_list`、`rs_io_set`
- **逃生口**：`rs_raw`（任意底层命令）、`debug.transform` / `debug.ik` / `debug.render`（诊断）

## 单位约定（实测确认，容易踩坑）

| 位置 | 单位 |
|---|---|
| `RsRobTarget.Frame.X/Y/Z` | mm |
| `RsRobTarget.Frame.RX/RY/RZ` | **弧度**（Euler ZYX），不是度 |
| `Mechanism.GetJointValues()` | 弧度 |
| `Mechanism.CalculateForwardKinematics()` 返回的 `Matrix4.Translation` | **米**（不是 mm） |
| `Matrix4.Quaternion` 索引顺序 | (w, x, y, z) |
| RAPID robtarget 四元数 | (x, y, z, w)，导出时已换序 |

MCP 层面统一对外用 **mm + 度**，转换都在 Add-in 里做完。

## 已知限制

- **没装 RobotWare**：本机 `RobotWare` 未安装，所以没有 Virtual Controller，
  `Simulator.*` 和 `Mechanism.CalculateInverseKinematics`（依赖 controller 的运动学服务）都不可用。
  → 逆解由内置的阻尼最小二乘数值解（`NumericIk.cs`）顶上，用 `CalculateForwardKinematics` 做差分雅可比，
  实测往返精度能精确回到原关节角。
  → 需要真正跑仿真时，装 RobotWare 6.08 并在工站里建 Virtual Controller 即可，接口不用改。
- **RAPID 文本只能读不能写**：RobotStudio 6.x 的离线 API 不给 RAPID 编辑器文本缓冲区，
  所以程序是通过 `RsPathProcedure` / `RsMoveInstruction` 模型构建的，`rs_show_code` 是从模型渲染出来的源码视图，
  不能反向往编辑器里贴文本。
- **新工站名字要保存后才定**：`new Station()` 之后 `Name` 为空，`SaveAs` 的文件名会成为工站名。
