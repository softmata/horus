# HORUS

<p align="center">
  <a href="README.md">English</a> ·
  <a href="README.zh-CN.md">简体中文</a> ·
  <a href="README.pt-BR.md">Português (Brasil)</a> ·
  <a href="README.ja.md">日本語</a> ·
  <strong>Español</strong> ·
  <a href="README.de.md">Deutsch</a>
</p>

**Middleware distribuido en tiempo real para Rust, Python y C++.**

> Este documento es una introducción traducida. El [README en inglés](README.md) contiene la versión oficial y más actualizada.

[Documentación](https://docs.horusrobotics.dev) · [Inicio rápido](https://docs.horusrobotics.dev/getting-started/quick-start) · [Benchmarks](benchmarks/) · [Discord](https://discord.gg/hEZC3ev2Nf)

## Primeros pasos

```bash
curl -fsSL https://github.com/softmata/horus/raw/main/install.sh | bash
horus new my_robot && cd my_robot && horus run
```

Instalación manual:

```bash
git clone https://github.com/softmata/horus.git
cd horus
./install.sh
```

Python: `pip install horus-robotics`. C++: enlaza `libhorus_cpp` e incluye `<horus/horus.hpp>`.

## ¿Por qué HORUS?

HORUS sustituye DDS por búferes circulares en memoria compartida y sincronización sin bloqueos. Está diseñado para robótica, automatización industrial, vehículos autónomos y otros sistemas donde importan la latencia, el determinismo y la seguridad.

| Capacidad | HORUS |
|-----------|-------|
| Latencia IPC | **171 ns** unidireccional, entre procesos (`cross_process_benchmark`) |
| Planificación | Determinista, con cinco clases de ejecución |
| Tiempo real | Presupuestos, deadlines, afinidad de CPU y watchdog integrados |
| Seguridad | Watchdog gradual, estado seguro y grabador BlackBox |
| Tensores GPU | Intercambio sin copias con PyTorch/JAX mediante DLPack |
| Lenguajes | Rust, Python y C++ sobre el mismo transporte compartido |
| Configuración | Un único archivo `horus.toml` |

El rendimiento depende de la CPU, el sistema operativo, el planificador, el tamaño del mensaje y la topología. Consulta la [metodología completa](benchmarks/README.md) y mide en el hardware de destino.

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

## Funciones principales

- Publicación y suscripción sin copias dentro y entre procesos
- Planificación determinista multirritmo y clases de ejecución en tiempo real
- Servicios, acciones, parámetros y transformaciones de coordenadas
- Políticas de deadline, aislamiento de fallos y estados seguros
- Más de 40 tipos de mensajes de robótica
- BlackBox, grabación, reproducción, registros y monitorización
- Replicación opcional de tópicos en red local con la feature `net`
- Interoperabilidad entre Rust, Python y C++

## CLI

```bash
horus new my_robot --rust   # proyecto Rust
horus new my_robot --python # proyecto Python
horus new my_robot --cpp    # proyecto C++
horus run                   # compilar y ejecutar
horus topic list            # listar tópicos activos
horus node list             # listar nodos
horus monitor               # iniciar el plugin de monitorización
horus doctor                # comprobar el entorno
```

## Ejemplos y comunidad

Consulta [`examples/`](examples/) para proyectos de conducción diferencial, brazo robótico, navegación, múltiples robots, cuadrúpedos, percepción y grabación/reproducción.

Lee la [guía de contribución](CONTRIBUTING.md), abre una [incidencia](https://github.com/softmata/horus/issues) o únete a [Discord](https://discord.gg/hEZC3ev2Nf). Licencia [Apache-2.0](LICENSE).
