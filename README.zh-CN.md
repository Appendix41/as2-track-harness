<p align="center">
  <img src="assets/banner.png" alt="AS2 Track Harness —— DeepSeek × VLX，跑在 Unitree AS2 上" width="100%">
</p>

<h1 align="center">AS2 Track Harness</h1>

<p align="center">
  <b>DeepSeek × VLX。</b>把快 VLA 策略和慢 VLM 规划器接到 Unitree AS2 上的部署
  harness（脚手架）。
</p>

<p align="center">
  <a href="LICENSE"><img src="https://img.shields.io/badge/license-Apache--2.0-blue.svg" alt="License"></a>
  <img src="https://img.shields.io/badge/python-3.11%20%7C%203.12-blue.svg" alt="Python">
  <a href="#快速开始无需机器人"><img src="https://img.shields.io/badge/tests-170%20passing-brightgreen.svg" alt="Tests"></a>
  <a href="https://github.com/Appendix41/as2-track-harness/actions/workflows/ci.yml"><img src="https://github.com/Appendix41/as2-track-harness/actions/workflows/ci.yml/badge.svg?branch=main" alt="CI"></a>
  <a href="#安全声明与已知限制"><img src="https://img.shields.io/badge/status-research%20prototype-orange.svg" alt="Status"></a>
</p>

<p align="center">
  <a href="README.md">English</a> | 中文
</p>

---

## 为什么加一个慢模型

VLX-Go 很快，但在我们实验室这种杂物多的环境里，它自己的判断不够用。它只根据看到的
画面给航点，需要理解场景的指令就容易出错：几把黑椅子里哪把更远、目标躲到障碍后面该往
哪走、画面变化后原路线还成不成立。单目投影还会把距离压扁，"去最远的那把黑椅子"恰好是
它不擅长的指令。

慢模型负责这一部分。DeepSeek 看整帧图像和指令，返回一条地面路线；harness 每个任务只问
一次，在机器人出发前完成，所以 10–30 s 的延迟不进 20 Hz 控制回路。这种"慢视觉语言
规划器 + 快 VLA 策略"的分工，参考了银河通用把 Astra 接入 VLA 的做法。

快的一半是 VLX-Go，慢的一半是 DeepSeek，中间是门控。

## 这是什么

两个模型速度差得很远，这个 harness 负责把它们接起来，并且不让慢模型进入控制回路。

- **快回路。** VLX-Go 以约 8 Hz 返回 8×3 机身系航点，提供路径形状。
- **慢回路。** DeepSeek 每个任务只问一次，在机器人出发前返回图像坐标系里的地面路线，
  提供方向。
- **融合。** 路线投影到机身系后冻结。每 20 ms 用当前位姿重新表达它，沿路线计算进度，
  再把计划航向混进 VLX 路径，每个周期最多转 6°。
- **门控。** 只有图像、预测和遥测都新鲜，机器人处于运动模式，没有其他控制者，遥控器空闲，
  速度和倾角都在范围内，指令才会下发。
- **执行器。** 独立进程再校验一遍决策包，做速度与加速度限幅，带 0.2 s 看门狗，结束时
  一定发 `StopMove`。

任务身份是 VLX 的 `task_key`：boot id、流、指令修订号和指令。只有出现新 key 才会重新
规划；任务中途不替换路线。

## 架构

<p align="center">
  <img src="assets/architecture-zh.svg" alt="快 VLX-Go 回路与慢 DeepSeek 规划器进入融合、安全门控和执行器，最后到 AS2" width="600">
</p>

快策略按相机频率跑，慢规划器每任务一次，所有指令都要过同一道门控。

```
空闲 --新 task_key--> 规划中 --计划被接受--> 执行中 --到达 / 超时--> 完成
                        |                                        |
                        +-- 被拒 / 超时 --> 阻塞 <----------------+
```

坐标契约、融合公式和门控顺序见 [架构文档](docs/ARCHITECTURE.md)。

### VLX-Go 模型

快策略是外部模型，本仓库只消费它的 8×3 输出。结构供参考：

<p align="center">
  <a href="assets/vlx-go-architecture.png"><img src="assets/vlx-go-architecture.png" alt="VLX-Go 模型结构：RGB 历史帧与当前帧经冻结的 DINOv3、SigLIP 编码和特征融合，得到 188 个视觉 token，加上文本、时间标记和动作查询 token 送入 Transformer 主干，三层 MLP 航点头输出 8 个机身系路径点" width="100%"></a>
</p>

- 31 帧历史 × 4 个 token（124），加当前帧 64 个 token，共 188 个视觉 token；
  DINOv3 与 SigLIP 特征经对齐、拼接、池化后每个 token 1536 维。
- 共享 Adapter 把 1536 维映射到 1024 维；指令分词后得到 L 个文本 token，再拼上
  时间标记和 1 个 ACT token。
- Qwen3 做一次前向；ACT 位置的输出经三层 MLP（1024 → 24）得到 8×3 机身系路径点。
- 执行器取第 2 个点 ÷ 0.1 s，作为前向、侧向、转向速度。

张量形状和完整流程见 [docs/VLX_GO.md](docs/VLX_GO.md)。图来自上游项目（om-ai-lab，
arXiv:2610.19361），模型和权重不在本仓库。

## 快速开始（无需机器人）

```bash
git clone https://github.com/Appendix41/as2-track-harness.git
cd as2-track-harness
python3.11 -m venv .venv && . .venv/bin/activate
pip install -e '.[vision]'
bash scripts/test.sh
```

不需要相机、API key 或机器人的 mock 回路：

```bash
bash scripts/mock_preview.sh --port 8765          # 终端 A
bash scripts/shadow.sh --config config/shadow.json --duration 15 --port 0   # 终端 B
```

回放一次录制运行：

```bash
PYTHONPATH=src python -m vlx_async.replay \
  --predictions examples/demo_run/predictions.jsonl --max-records 200
```

## 在 AS2 上运行

仓库里没有模型。需要自备：

- 一个 **VLX-Go 预览服务**，在 `127.0.0.1:8765` 提供 `/control.json` 和 `/state.json`；
- **DDS 遥测**（`unitree_sdk2py` + Cyclone DDS，或 `scripts/telemetry_readonly.sh`
  生成的快照）；
- 一个 **控制租约**（可选带手动控制服务），通过 `VLX_CONTROLLER_LOCK` 和
  `VLX_STACK_ROOT` 指向你的部署；
- 一个 **OpenAI 兼容接口**（`base_url`、`model`、`DEEPSEEK_API_KEY`）。

VLX-Go-1.0 和 SigLIP 权重在 Hugging Face 公开，DINOv3 需要申请，推理代码在
`om-ai-lab/VLX-Go`。推理也可以放到另一台机器，适配器留在机器人主机上
（见 [远程推理](docs/REMOTE_INFERENCE.md)）。

```bash
DEEPSEEK_API_KEY=... bash scripts/observe.sh --port 8768   # 观察台，仅 loopback
bash scripts/fusion_trial.sh --duration 10                 # dry-run
bash scripts/fusion_trial.sh --execute --duration 5 \
  --speed 0.05 --lateral-speed 0.03 --yaw-speed 0.10 \
  --linear-acceleration 0.10 --yaw-acceleration 0.20
```

首次实机前读 [SAFETY.md](docs/SAFETY.md)，前置条件和报错原因见
[RUNBOOK.md](docs/RUNBOOK.md)。

## 结果

| 项目 | 结果 | 样本 | 日期 |
| --- | --- | --- | --- |
| 单元测试 | 170 项通过，纯 CPU | 170 | 2026-11-13 |
| 快策略频率 | VLX 约 8.2 Hz，本地回路约 19.9 Hz | 1 次，65 s | 2026-10-25 |
| 规划器延迟 | 每次图像请求 10.3 – 24.0 s | 18 次请求 | 2026-10-25/26 |
| 首次真机短步 | 17 条 `Move`、0.81 s、前进 4.27 cm | 1 次 | 2026-10-25 |
| 受控实机会话 | 每次 2 – 172 条 `Move`，最长 55.6 s | 22 次 | 2026-11-10 |
| mock 影子运行 | 计划被接受，0 条指令 | 1 次 | 2026-11-13 |

首次短步和受控会话都是被 harness 自己的边界停下的（速度门控、路线/行程上限、看门狗、
遥控接管）；到达没有用外部真值测量。未验证：超过数米的连续导航、真机避障（深度默认
关闭）、路线的米制精度、换一台机器人。

## 安全声明与已知限制

> [!WARNING]
> 未认证的研究代码，能让一台 20 kg 以上的四足机器人运动。
>
> - 默认没有障碍感知：`motion_gates.json` 和 `depth.json` 出厂都是 `false`。真正兜底的
>   只有遥控器、看门狗、速度/加速度上限和遥测门控。
> - 门控只是和规划客户端跑在同一台机器上的 Python。进程崩溃、内核卡顿、DDS 丢包或机器人
>   故障都能绕过它。
> - 每任务只规划一次，场景中途变化要发新任务。
> - 不提供担保，不含权重，Apache-2.0 as is。

实机时遥控器拿在手里，旁边再站一个人。细节见 [SAFETY.md](docs/SAFETY.md)。

## 文档

- [架构](docs/ARCHITECTURE.md) —— 数据流、任务生命周期、融合、门控顺序。
- [安全](docs/SAFETY.md) —— 上限、停止行为、系统不提供什么。
- [运行手册](docs/RUNBOOK.md) —— 安装、演示、观察台、实机检查表。
- [配置](docs/CONFIGURATION.md) —— 每个字段和取值范围。
- [远程推理](docs/REMOTE_INFERENCE.md) —— 把规划器放到机器人之外。
- [示例](examples/README.md) —— 录制任务与回放方法。

贡献流程见 [CONTRIBUTING.md](CONTRIBUTING.md)，安全问题走 [SECURITY.md](SECURITY.md)。

## 许可

Apache-2.0。不分发任何模型权重；VLX-Go、DINOv3、SigLIP 遵循各自许可证。第三方归属见
[NOTICE](NOTICE)。

如果这个 harness 对你的工作有帮助：

```bibtex
@software{as2_track_harness_2026,
  title  = {AS2 Track Harness (DeepSeek × VLX): an asynchronous VLA + VLM navigation harness for the Unitree AS2},
  author = {{AS2 Track Harness contributors}},
  year   = {2026},
  url    = {https://github.com/Appendix41/as2-track-harness},
  note   = {Research prototype; not safety certified}
}
```
