# HORUS

<p align="center">
  <a href="README.md">English</a> ·
  <a href="README.zh-CN.md">简体中文</a> ·
  <a href="README.pt-BR.md">Português (Brasil)</a> ·
  <strong>日本語</strong> ·
  <a href="README.es.md">Español</a> ·
  <a href="README.de.md">Deutsch</a>
</p>

**Rust、Python、C++ に対応したリアルタイム分散ミドルウェア。**

> これは翻訳された概要です。最新かつ正式な情報は[英語版 README](README.md) を参照してください。

[ドキュメント](https://docs.horusrobotics.dev) · [クイックスタート](https://docs.horusrobotics.dev/getting-started/quick-start) · [ベンチマーク](benchmarks/) · [Discord](https://discord.gg/hEZC3ev2Nf)

## はじめる

```bash
curl -fsSL https://github.com/softmata/horus/raw/release/install.sh | bash
horus new my_robot && cd my_robot && horus run
```

手動インストール：

```bash
git clone https://github.com/softmata/horus.git
cd horus
./install.sh
```

Python：`pip install horus-robotics`。C++：`libhorus_cpp` をリンクし、`<horus/horus.hpp>` をインクルードします。

## HORUS を選ぶ理由

HORUS は DDS の代わりに共有メモリのリングバッファとロックフリー同期を使用します。ロボティクス、産業オートメーション、自律走行など、低遅延・決定性・安全性が重要なシステム向けです。

| 機能 | HORUS |
|------|-------|
| IPC レイテンシ | トポロジーと競合に応じて中央値 **3–304 ns** |
| スケジューリング | 5 種類の実行クラスによる決定論的実行 |
| リアルタイム | バジェット、デッドライン、CPU アフィニティ、ウォッチドッグを内蔵 |
| 安全性 | 段階的ウォッチドッグ、安全状態、BlackBox レコーダー |
| GPU テンソル | DLPack による PyTorch/JAX とのゼロコピー連携 |
| 言語 | Rust、Python、C++ が同じ共有メモリ転送を利用 |
| 設定 | 単一の `horus.toml` |

性能は CPU、OS、スケジューラー、メッセージサイズ、トポロジーによって変化します。[測定方法と結果](benchmarks/README.md)を確認し、対象ハードウェアでも測定してください。

## API

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

## 主な機能

- プロセス内・プロセス間のゼロコピー Pub/Sub
- マルチレートの決定論的スケジューリングとリアルタイム実行クラス
- サービス、アクション、パラメーター、座標変換
- デッドラインポリシー、障害分離、安全状態
- 40 種類以上のロボティクスメッセージ
- BlackBox、記録・再生、ログ、監視
- `net` feature による任意の LAN トピック複製
- Rust、Python、C++ 間の相互運用

## CLI

```bash
horus new my_robot --rust   # Rust プロジェクト
horus new my_robot --python # Python プロジェクト
horus new my_robot --cpp    # C++ プロジェクト
horus run                   # ビルドして実行
horus topic list            # アクティブなトピックを表示
horus node list             # ノードを表示
horus monitor               # 監視プラグインを起動
horus doctor                # 開発環境を診断
```

## サンプルとコミュニティ

[`examples/`](examples/) には差動二輪、ロボットアーム、センサーナビゲーション、マルチロボット、四足歩行、認識、記録・再生のサンプルがあります。

[コントリビューションガイド](CONTRIBUTING.md)を読み、[Issue](https://github.com/softmata/horus/issues) または [Discord](https://discord.gg/hEZC3ev2Nf) から参加してください。[Apache-2.0](LICENSE) ライセンスです。
