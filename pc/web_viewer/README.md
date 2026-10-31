# 交互式 3D URDF 查看器

一个本地网页查看器：拖拽旋转、右键平移、滚轮缩放，按 URDF 的关节树摆姿态。
不依赖 ROS，不依赖 npm 构建；three.js / URDFLoader / OrbitControls 已放在 `lib/`。

## 快速开始

实例模型已换成 **reBot Arm B601-DM**（6 轴、STL）：

```bash
cd pc/web_viewer
python3 server.py --open
# 或双击 run_viewer.command
```

模型来自 B601 的 `description/`，只保留每个 link 的主 STL（8 个，约 20 MB），
不是仓库提交内容；`models/rebot/` 已在 `models/.gitignore` 中忽略。

新 clone 后若没有该目录，先运行：

```bash
python3 tools/fetch_rebot.py
```

没有 reBot 目录时会自动回退到 `models/demo`（简化 box/cylinder 臂）。

## 接真实模型

```bash
python3 server.py --model /path/to/your_pkg \
  --urdf relative/path/robot.urdf --open
```

要求：

- `--model` 是 URDF 包的根目录（`package://` 里的包名对应这个目录）；
- `--urdf` 相对该目录；不填会自动找第一个 `.urdf`；
- mesh 支持 **STL / DAE**，路径可以是：
  - `package://pkg/...`（服务端会剥成相对路径）
  - 相对 URDF 的 `../meshes/...`（服务端会解析成模型根目录下的路径）
- 不支持 xacro / GLB；需要先导出成纯 URDF。

## 客户端联动

`client_app.py` 顶部有 **3D 模型** 路径输入框，点 **3D 仿真**：

- 自动启动本地 viewer（端口 8765）；
- 浏览器打开 `?follow=1`；
- 每 100 ms 把客户端 6 个目标角 POST 到 `/angles`；
- 页面滑条同步显示角度，也可取消“跟随客户端角度”手动摆。

`/angles` 接口：

```bash
curl -X POST -H 'Content-Type: application/json' \
  -d '[0.2,-0.6,-0.9,0.3,0.2,-0.4]' http://127.0.0.1:8765/angles
```

角度单位 rad，顺序为 URDF 中可动关节按名称排序（joint1..joint6）。

## 已验证

- B601-DM 的 URDF + STL（见 `docs/b601_lite.png`），6 关节姿态、颜色、随动；
- demo 模型（无外部 mesh）加载；
- `/angles` POST 后页面姿态同步。

B601 模型来源：<https://github.com/Yang-Ci/ReBot_Arm_DigitalTwin_DM>，
CERN-OHL-W-2.0；本仓库不包含它的 STL，只保留截图。
`models/rebot/` 由 `tools/fetch_rebot.py` 本地生成，不提交。

## 已知限制

- B601 全量 URDF 有 34 个重复彩色 mesh（98 MB），加载很慢；验证时只保留每个 link 的主 STL（20 MB）。
- 大模型建议 CAD 只导出每 link 一个 mesh；重复彩色件会拖慢浏览器。
- 关节按名称排序；如果 URDF 的 joint 名称不是 joint1..6，客户端角度顺序需要对齐。

## 第三方

- three.js r128 — MIT
- urdf-loader — MIT
- OrbitControls / STLLoader / ColladaLoader（three.js examples）— MIT
