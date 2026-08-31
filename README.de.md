# HORUS

<p align="center">
  <a href="README.md">English</a> ·
  <a href="README.zh-CN.md">简体中文</a> ·
  <a href="README.pt-BR.md">Português (Brasil)</a> ·
  <a href="README.ja.md">日本語</a> ·
  <a href="README.es.md">Español</a> ·
  <strong>Deutsch</strong>
</p>

**Verteilte Echtzeit-Middleware für Rust, Python und C++.**

> Dieses Dokument ist eine übersetzte Übersicht. Die aktuelle und verbindliche Fassung befindet sich in der [englischen README](README.md).

[Dokumentation](https://docs.horusrobotics.dev) · [Schnellstart](https://docs.horusrobotics.dev/getting-started/quick-start) · [Benchmarks](benchmarks/) · [Discord](https://discord.gg/hEZC3ev2Nf)

## Schnellstart

```bash
curl -fsSL https://github.com/softmata/horus/raw/main/install.sh | bash
horus new my_robot && cd my_robot && horus run
```

Das Installationsskript ermittelt das neueste Release-Tag und nimmt beide Hälften daraus: die CLI-Binärdatei, geprüft gegen die `SHA256SUMS` dieses Releases, und den Quellbaum, der unter `~/.horus/cache/horus@<Version>` abgelegt wird. Beide müssen aus demselben Tag stammen, denn `horus run` kompiliert dein Projekt als Pfadabhängigkeit gegen diesen Quellbaum.

Eine bestimmte Version festnageln:

```bash
curl -fsSL https://github.com/softmata/horus/raw/main/install.sh | HORUS_VERSION=v0.4.0 bash
```

Manuelle Installation:

```bash
git clone https://github.com/softmata/horus.git
cd horus
./install.sh
```

Aktualisieren: `horus self update` — hebt CLI und zwischengespeicherten Quellbaum gemeinsam auf dasselbe neue Tag. Deinstallieren: `curl -fsSL https://github.com/softmata/horus/raw/main/uninstall.sh | bash` — das Skript listet auf, was es entfernen wird, und fragt im Terminal nach. Mit `-s -- --dry-run` wird nur aufgelistet, mit `-s -- --yes` läuft es unbeaufsichtigt.

Docker: `docker build --target dev -t horus:dev .`. Das Standardziel von `docker build .` ist das schlanke Image mit ausschließlich der CLI; es kann kein Projekt bauen oder ausführen.

C++: `libhorus_cpp` linken und `<horus/horus.hpp>` einbinden.

Python: `pip install "horus-robotics>=0.4.0"`. Die neueste Version auf PyPI ist 0.1.9 und verwendet ein älteres Shared-Memory-Format als eine 0.4.x-CLI; solange 0.4.x dort nicht veröffentlicht ist, die Bindings aus dem Cache bauen: `pip install ~/.horus/cache/horus@0.4.0/horus_py`.

Rust: es gibt keinen crates.io-Kanal. `cargo add horus` und `cargo install horus` laden eine fremde Crate gleichen Namens.

Plattformmatrix, Umgebungsvariablen des Installers und Docker-Details: [Install, Upgrade, Uninstall](README.md#install-upgrade-uninstall).

## Warum HORUS?

HORUS ersetzt DDS durch Shared-Memory-Ringpuffer und lockfreie Synchronisierung. Es richtet sich an Robotik, Industrieautomatisierung, autonome Fahrzeuge und andere Systeme, bei denen Latenz, Determinismus und Sicherheit entscheidend sind.

| Fähigkeit | HORUS |
|-----------|-------|
| IPC-Latenz | Median **3–304 ns**, abhängig von Topologie und Konkurrenz |
| Scheduling | Deterministisch mit fünf Ausführungsklassen |
| Echtzeit | Budgets, Deadlines, CPU-Affinität und Watchdog integriert |
| Sicherheit | Abgestufter Watchdog, sicherer Zustand und BlackBox-Rekorder |
| GPU-Tensoren | Kopierfreier Austausch mit PyTorch/JAX über DLPack |
| Sprachen | Rust, Python und C++ nutzen denselben Shared-Memory-Transport |
| Konfiguration | Eine zentrale `horus.toml` |

Die Leistung hängt von CPU, Betriebssystem, Scheduler, Nachrichtengröße und Topologie ab. Vor dem Einsatz sollten die [vollständige Methodik](benchmarks/README.md) gelesen und Messungen auf der Zielhardware durchgeführt werden.

## APIs

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

## Kernfunktionen

- Kopierfreies Publish/Subscribe innerhalb und zwischen Prozessen
- Deterministisches Multi-Rate-Scheduling und Echtzeit-Ausführungsklassen
- Services, Actions, Parameter und Koordinatentransformationen
- Deadline-Richtlinien, Fehlerisolation und sichere Zustände
- Mehr als 40 Robotik-Nachrichtentypen
- BlackBox, Aufzeichnung, Wiedergabe, Logging und Überwachung
- Optionale LAN-Replikation von Topics über das Feature `net`
- Interoperabilität zwischen Rust, Python und C++

## CLI

```bash
horus new my_robot --rust   # Rust-Projekt
horus new my_robot --python # Python-Projekt
horus new my_robot --cpp    # C++-Projekt
horus run                   # bauen und ausführen
horus topic list            # aktive Topics anzeigen
horus node list             # Nodes anzeigen
horus monitor               # Monitoring-Plugin starten
horus doctor                # Entwicklungsumgebung prüfen
```

## Beispiele und Community

Unter [`examples/`](examples/) befinden sich Projekte für Differentialantriebe, Roboterarme, Sensornavigation, Multi-Robotik, Quadrupeden, Wahrnehmung sowie Aufzeichnung und Wiedergabe.

Lies den [Beitragsleitfaden](CONTRIBUTING.md), öffne ein [Issue](https://github.com/softmata/horus/issues) oder tritt dem [Discord](https://discord.gg/hEZC3ev2Nf) bei. Lizenziert unter [Apache-2.0](LICENSE).
