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
curl -fsSL https://github.com/softmata/horus/raw/main/install.sh | bash
horus new my_robot && cd my_robot && horus run
```

インストーラーは最新のリリースタグを解決し、そのタグから両方を取得します。ひとつはそのリリースの `SHA256SUMS` で検証された CLI バイナリ、もうひとつは `~/.horus/cache/horus@<バージョン>` に置かれるソースツリーです。`horus run` はこのソースをパス依存としてプロジェクトをビルドするため、両者は必ず同じタグから来る必要があります。

バージョンを固定する場合：

```bash
curl -fsSL https://github.com/softmata/horus/raw/main/install.sh | HORUS_VERSION=v0.4.1 bash
```

手動インストール：

```bash
git clone https://github.com/softmata/horus.git
cd horus
./install.sh
```

更新：`horus self update` は CLI とキャッシュされたソースを同じ新しいタグへまとめて更新します。アンインストール：`curl -fsSL https://github.com/softmata/horus/raw/main/uninstall.sh | bash`。削除対象を一覧表示し、端末で確認を求めます。`-s -- --dry-run` を付けると一覧の表示だけ、`-s -- --yes` を付けると無人実行になります。

Docker：`docker build --target dev -t horus:dev .`。`docker build .` の既定のターゲットは CLI だけを含む軽量イメージで、プロジェクトのビルドも実行もできません。

C++：`libhorus_cpp` をリンクし、`<horus/horus.hpp>` をインクルードします。

Python：`pip install "horus-robotics>=0.4.1"`。PyPI の最新版は 0.1.9 で、0.4.x の CLI とは共有メモリの形式が異なります。0.4.x が公開されるまでは、キャッシュされたソースからビルドしてください：`pip install ~/.horus/cache/horus@0.4.1/horus_py`。

Rust：crates.io のチャンネルはありません。`cargo add horus` と `cargo install horus` は同名の無関係なクレートを取得します。

対応プラットフォーム一覧、インストーラーの環境変数、Docker の詳細：[Install, Upgrade, Uninstall](README.md#install-upgrade-uninstall)。

## HORUS を選ぶ理由

HORUS は DDS の代わりに共有メモリのリングバッファとロックフリー同期を使用します。ロボティクス、産業オートメーション、自律走行など、低遅延・決定性・安全性が重要なシステム向けです。

| 機能 | HORUS |
|------|-------|
| IPC レイテンシ | プロセス間・片道 **171 ns**（`cross_process_benchmark`） |
| スケジューリング | 5 種類の実行クラスによる決定論的実行 |
| リアルタイム | バジェット、デッドライン、CPU アフィニティ、ウォッチドッグを内蔵 |
| 安全性 | 段階的ウォッチドッグ、安全状態、BlackBox レコーダー |
| テンソル | プール管理、NumPy へゼロコピー（ホストメモリ） |
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
