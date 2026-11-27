# Panther 单臂目标点卡片部署记录

## 当前部署

- 机器人：`10.100.129.74`（原生 arm64 构建）
- 容器：`embodied-unixai-panther`
- 镜像：`unixai-panther:point-261127-return`
- 镜像 ID：`sha256:ca1434c78456b2c1dbdd75eaf06362c7bcb6ebf0ccdb47e1ada03002cfad5a24`
- 画布：<https://10.100.129.74:15678>
- MCP：<http://10.100.129.74:15792/mcp>
- 卡片：`arm_point`
- lifting 实际范围：`0.4–0.65 m`；规划读取当前高度，不驱动 lifting。

采用完整的仓库 Panther URDF 做几何计算（live URDF 缺少 `body_joint`），关节限位采用仓库与 live 限位的交集。
目标点是夹爪指尖 frame 的中点；参考系为 `base_link`；目标姿态取自 `start`。
一次发送多点 JointTrajectory，每段关节变化不超过 15°，段时间 1 s。
配置盒：`[-0.20, -0.45, 0.0]` 至 `[0.88, 0.45, 1.20]` m；这是配置值，不是实测可达包络。

## 已验证

- Python 编译与 diff 空白检查通过。
- 命名 `common` 上下文 BuildKit 构建通过。
- 镜像中左右臂 5 cm 目标 IK、关节限位、15° 上限、lifting 高度变化与盒外拒绝测试通过。
- 容器替换后 MCP `tools/list` 包含新卡片；原有 Env 与 Binds 一致。
- 实机左右臂分别执行 `start → preview（当前 x + 0.05 m）→ stop` 成功，最终为 idle。
- 尚未执行实际 move；控制器切换、实机轨迹跟踪、保持与运动中取消仍待现场验证。

## 使用

1. `prepare`：选择 `left` 或 `right`，记录姿态并占用手臂。画布可能隐藏生命周期动作 `start`，因此提供显式 `prepare`；同侧已准备时重复调用不重置姿态。
2. `preview`：输入 `base_link` 下的 x/y/z，单位 m，检查结果。
3. `move`：同一侧与目标，传入 `confirm_motion: true`；当前配置 `allow_motion: true`。
4. 运动完成后保持占用；使用 `cancel` 或 `stop` 恢复原控制器配置并释放占用。切换手臂或重新记录姿态前先取消，再 `prepare`。

卡片只限制夹爪中心工作空间和关节限位，不做整臂碰撞检测；底盘、head 与 lifting 保持静止由操作者保证。

## 构建

机器人源码目录：`/home/unix_ai/panther-point-src`。

```sh
docker buildx build --network=host --load \
  --build-context common=/home/unix_ai/panther-point-src/common \
  -t unixai-panther:point-261127-return \
  /home/unix_ai/panther-point-src/unixai/panther
```

使用 host 构建网络是因为机器人内核缺少 Docker bridge 所需的 iptables raw 表。
Ubuntu 软件源从无法解析的 Tencent 内网域名改为清华镜像。

## 原版本备份与回滚

部署前原镜像：`release.261125.75ffac8`。
原始 inspect、不可变镜像 ID、完整恢复请求保存在机器人：
`/home/unix_ai/panther-point-src/rollback-before-point/`。

在机器人上恢复部署前原版本：

```sh
python3 /home/unix_ai/panther-point-src/local_image.py --rollback \
  /home/unix_ai/panther-point-src/rollback-before-point
```

部署脚本在启动或 MCP 健康检查失败时自动恢复上一容器镜像与参数。
本次最终部署成功，未触发自动回滚；未创建 git commit 或 PR。

## 画布坐标注释

已从线上 tools/list 确认 XYZ 的 description、minimum、maximum，以及 preview/move 的动作 description 已更新。
模型手臂关节全零、夹爪模型张开时：左中心约 `(0.15018, 0.26824, h−0.19931)` m，右中心约 `(0.13923, −0.27576, h−0.19931)` m。
这是模型零位参考，不保证等同于每次开机实际姿态；实际值读取 prepare/info。

当前 URDF 的保守外包球必要条件：
`(x−0.162991)²+(y−y0)²+(z−h−0.498)² ≤ 0.912²`。
左臂 `y0=0.0595`，右臂 `y0=−0.0595`，`h` 为 lifting 高度（m）。
半径由基关节至夹爪中心链条的固定平移长度求和得到（左约 0.910162 m、右约 0.910971 m），统一向上取 0.912 m；不是实测可达半径，也不是新增的运行时球形 barrier。
真正的联合限制仍由限位内 IK、固定目标姿态及路径工作空间检查决定。球内不保证有解。

## 返回选项

新增动作 `preview_return`（只预览）与 `return`（执行），都支持 `return_to` 下拉选项：

- **回到上一位置**：恢复此版本卡片最近一次已接受 `move` 前的关节反馈。记录分左右臂，保留到 cancel/prepare 之后，但重启驱动会清空。旧版本未记录的历史姿态不补猜。lifting 改变时拒绝恢复上一空间位置。
- **回到开机初始位置**：使用 `arm_control.home_pose_deg` 配置，目前是对应手臂八关节全零的下垂参考姿态，不是历史开机反馈快照。lifting 保持当前高度。

返回动作不需要 XYZ，也不锁定当前末端姿态；使用已知关节目标进行插值，检查限位、每段15°上限及1°间隔的夹爪中心工作空间采样。记录的上一姿态若仅有硬限位附近的微小编码器误差，会在既有反馈容差内收敛到限位内目标；不会扩大目标限位。无整臂碰撞检测。

使用：选择 `preview_return`、side 和 return_to，返回 reachable 后选择 `return`，使用同一目标并设置 `confirm_motion=true`。完成后可 cancel 释放手臂；再 prepare 开始新的点位移动。

验证：左右臂离线返回规划与限位拒绝、上一位置缺失拒绝、历史记录跨cancel保留、lifting变化拒绝、并发返回拒绝均通过。部署后基于当前左臂反馈预览返回初始化姿态成功，共10段、约10秒；未执行实机返回运动。
此次部署前的容器备份：`/home/unix_ai/panther-point-src/rollback-before-return/`。
