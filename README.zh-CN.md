# HORUS

<p align="center">
  <a href="README.md">English</a> ·
  <strong>简体中文</strong> ·
  <a href="README.pt-BR.md">Português (Brasil)</a> ·
  <a href="README.ja.md">日本語</a> ·
  <a href="README.es.md">Español</a> ·
  <a href="README.de.md">Deutsch</a>
</p>

**面向 Rust、Python 和 C++ 的实时分布式中间件。**

> 本文是项目概览的中文翻译。最新、最完整且具有权威性的内容以 [英文 README](README.md) 为准。

[文档](https://docs.horusrobotics.dev) · [快速入门](https://docs.horusrobotics.dev/getting-started/quick-start) · [性能测试](benchmarks/) · [Discord](https://discord.gg/hEZC3ev2Nf)

## 快速开始

```bash
curl -fsSL https://github.com/softmata/horus/raw/main/install.sh | bash
horus new my_robot && cd my_robot && horus run
```

安装脚本会解析最新的发布标签，并从同一个标签取回两部分内容：经该版本 `SHA256SUMS` 校验的 CLI 可执行文件，以及缓存在 `~/.horus/cache/horus@<版本>` 的源码树。两者必须来自同一个标签，因为 `horus run` 会把该源码树作为路径依赖来编译你的项目。

固定到某个版本：

```bash
curl -fsSL https://github.com/softmata/horus/raw/main/install.sh | HORUS_VERSION=v0.4.1 bash
```

也可以手动安装：

```bash
git clone https://github.com/softmata/horus.git
cd horus
./install.sh
```

升级：`horus self update` 会把 CLI 和缓存的源码树一起更新到同一个新标签。卸载：`curl -fsSL https://github.com/softmata/horus/raw/main/uninstall.sh | bash`；脚本会先列出将要删除的内容，并在终端里请求确认。加上 `-s -- --dry-run` 只列出不删除，加上 `-s -- --yes` 则无人值守执行。

Docker：`docker build --target dev -t horus:dev .`。`docker build .` 的默认目标是只含 CLI 的精简镜像，无法构建或运行项目。

C++：链接 `libhorus_cpp`，并包含 `<horus/horus.hpp>`。

Python：`pip install "horus-robotics>=0.4.1"`。PyPI 上的最新版本是 0.1.9，其共享内存格式早于 0.4.x 的 CLI；在 0.4.x 发布之前，请从缓存的源码构建绑定：`pip install ~/.horus/cache/horus@0.4.1/horus_py`。

Rust：没有 crates.io 渠道。`cargo add horus` 和 `cargo install horus` 取到的是同名的无关 crate。

平台支持矩阵、安装脚本的环境变量以及 Docker 细节见 [Install, Upgrade, Uninstall](README.md#install-upgrade-uninstall)。

## 为什么选择 HORUS？

HORUS 用共享内存环形缓冲区和无锁同步替代 DDS，适用于机器人、工业自动化、自动驾驶、交易系统和游戏引擎等对延迟、确定性与安全性要求较高的系统。

| 能力 | HORUS |
|------|-------|
| IPC 延迟 | 跨进程单向 **171 ns**（`cross_process_benchmark`） |
| 调度 | 确定性调度，五种执行类别 |
| 实时支持 | 内置预算、截止时间、CPU 亲和性与看门狗 |
| 安全 | 分级看门狗、安全状态钩子和 BlackBox 飞行记录器 |
| GPU 张量 | 通过 DLPack 与 PyTorch/JAX 零拷贝交换 |
| 语言 | Rust、Python 和 C++ 使用相同的共享内存传输 |
| 配置 | 统一的 `horus.toml` |

性能结果会受到 CPU、操作系统、调度策略、消息大小和拓扑影响。部署前请查看[完整测试方法和结果](benchmarks/README.md)，并在目标硬件上复测。

## 支持的开发方式

### Rust

```rust
use horus::prelude::*;

fn main() -> Result<()> {
    let mut scheduler = Scheduler::new().tick_rate(1000_u64.hz());
    scheduler.add(MyNode::new()?).order(0).rate(1000_u64.hz()).build()?;
    scheduler.run()
}
```

### Python

```python
import horus

def tick(node):
    message = node.recv("sensor.data")
    if message is not None:
        node.send("motor.cmd", message)

horus.run(horus.Node(name="controller", subs=["sensor.data"],
                     pubs=["motor.cmd"], tick=tick, rate=1000))
```

### C++

```cpp
#include <horus/horus.hpp>

int main() {
    horus::Scheduler scheduler;
    scheduler.spin();
}
```

## 主要功能

- 进程内及跨进程的零拷贝发布/订阅
- 多速率确定性调度及实时执行类别
- 服务、动作、参数与坐标变换
- 截止时间策略、故障隔离和安全状态
- 40 多种机器人消息类型
- BlackBox 记录、回放、日志与运行时监控
- 可选的 `net` 功能，用于局域网多机器人主题复制
- Rust、Python 与 C++ 跨语言互操作

## 常用命令

```bash
horus new my_robot --rust   # 创建 Rust 项目
horus new my_robot --python # 创建 Python 项目
horus new my_robot --cpp    # 创建 C++ 项目
horus run                   # 构建并运行
horus topic list            # 列出活动主题
horus node list             # 列出节点
horus monitor               # 启动监控插件
horus doctor                # 检查开发环境
```

## 示例与参与贡献

完整示例位于 [`examples/`](examples/)，涵盖差速驱动、机械臂、传感器导航、多机器人、四足机器人、视觉管线、动作以及记录/回放。

欢迎阅读[贡献指南](CONTRIBUTING.md)、提交 [Issue](https://github.com/softmata/horus/issues)，或加入 [Discord](https://discord.gg/hEZC3ev2Nf)。项目采用 [Apache-2.0](LICENSE) 许可证。
