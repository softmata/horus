# HORUS

<p align="center">
  <a href="README.md">English</a> ·
  <a href="README.zh-CN.md">简体中文</a> ·
  <strong>Português (Brasil)</strong> ·
  <a href="README.ja.md">日本語</a> ·
  <a href="README.es.md">Español</a> ·
  <a href="README.de.md">Deutsch</a>
</p>

**Middleware distribuído de tempo real para Rust, Python e C++.**

> Esta é uma visão geral traduzida. O [README em inglês](README.md) é a referência oficial e mais atualizada.

[Documentação](https://docs.horusrobotics.dev) · [Início rápido](https://docs.horusrobotics.dev/getting-started/quick-start) · [Benchmarks](benchmarks/) · [Discord](https://discord.gg/hEZC3ev2Nf)

## Comece agora

```bash
curl -fsSL https://github.com/softmata/horus/raw/main/install.sh | bash
horus new my_robot && cd my_robot && horus run
```

Instalação manual:

```bash
git clone https://github.com/softmata/horus.git
cd horus
./install.sh
```

Python: `pip install horus-robotics`. C++: vincule `libhorus_cpp` e inclua `<horus/horus.hpp>`.

## Por que HORUS?

O HORUS substitui DDS por buffers circulares em memória compartilhada e sincronização sem locks. Ele foi criado para robótica, automação industrial, veículos autônomos e outros sistemas nos quais latência, determinismo e segurança são essenciais.

| Recurso | HORUS |
|---------|-------|
| Latência IPC | Mediana de **3–304 ns**, conforme topologia e contenção |
| Agendamento | Determinístico, com cinco classes de execução |
| Tempo real | Orçamentos, deadlines, afinidade de CPU e watchdog integrados |
| Segurança | Watchdog progressivo, estado seguro e gravador BlackBox |
| Tensores em GPU | Intercâmbio sem cópia com PyTorch/JAX via DLPack |
| Linguagens | Rust, Python e C++ sobre o mesmo transporte compartilhado |
| Configuração | Um único arquivo `horus.toml` |

Os resultados variam conforme CPU, sistema operacional, escalonador, tamanho das mensagens e topologia. Consulte a [metodologia completa](benchmarks/README.md) e teste no hardware de destino.

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

## Principais recursos

- Publicação e assinatura sem cópia, dentro e entre processos
- Agendamento determinístico multirrate e classes de execução em tempo real
- Serviços, ações, parâmetros e transformações de coordenadas
- Políticas de deadline, isolamento de falhas e estados seguros
- Mais de 40 tipos de mensagens para robótica
- BlackBox, gravação, reprodução, logs e monitoramento
- Replicação opcional de tópicos em rede local com a feature `net`
- Interoperabilidade entre Rust, Python e C++

## CLI

```bash
horus new my_robot --rust   # projeto Rust
horus new my_robot --python # projeto Python
horus new my_robot --cpp    # projeto C++
horus run                   # compilar e executar
horus topic list            # listar tópicos ativos
horus node list             # listar nós
horus monitor               # iniciar o plugin de monitoramento
horus doctor                # verificar o ambiente
```

## Exemplos e comunidade

Veja [`examples/`](examples/) para projetos de direção diferencial, braço robótico, navegação, múltiplos robôs, quadrúpedes, percepção e gravação/reprodução.

Leia o [guia de contribuição](CONTRIBUTING.md), abra uma [issue](https://github.com/softmata/horus/issues) ou participe do [Discord](https://discord.gg/hEZC3ev2Nf). Licenciado sob [Apache-2.0](LICENSE).
