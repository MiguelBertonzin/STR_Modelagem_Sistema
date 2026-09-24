# Configuração dos experimentos

As coletas foram executadas em **UNICORE**. A matriz obrigatória combina CUSTOM e DM, preemptivo e cooperativo, nas frequências de 80 e 240 MHz. Os extras acrescentam 160 MHz, RM e time slicing desligado.

Antes de cada cenário, altere a configuração, execute `idf.py fullclean`, recompile, grave a placa e reinicie o roteiro.

## Política de prioridade

Em `main/main.c`, selecione uma das opções:

```c
#define PRIORITY_POLICY POLICY_CUSTOM
// #define PRIORITY_POLICY POLICY_DM
// #define PRIORITY_POLICY POLICY_RM
```

As prioridades implementadas são:

| Política | FS | FUS | CTRL | NAV |
| --- | ---: | ---: | ---: | ---: |
| CUSTOM | 5 | 4 | 3 | 2 |
| DM | 4 | 5 | 5 | 3 |
| RM adaptado | 4 | 5 | 5 | 3 |

CUSTOM dá prioridade máxima à emergência. DM ordena os deadlines de 5, 10 e 20 ms. RM é o experimento extra; na implementação, sua tabela coincide com DM.

## UNICORE e frequência

Execute:

```powershell
idf.py menuconfig
```

Para UNICORE:

```text
Component config
  → FreeRTOS
    → Kernel
      → Run FreeRTOS only on first core
```

Para a frequência:

```text
Component config
  → ESP System Settings
    → CPU frequency
      → 80 MHz, 160 MHz ou 240 MHz
```

O banner do programa deve confirmar `nucleos=1` e a frequência selecionada.

## Preemptivo e cooperativo

No ESP-IDF v6.1 utilizado nas coletas, a configuração foi alterada em:

```text
components/freertos/config/include/freertos/FreeRTOSConfig.h
```

Modo preemptivo:

```c
#define configUSE_PREEMPTION 1
```

Modo cooperativo:

```c
#define configUSE_PREEMPTION 0
```

Depois da alteração, faça `idf.py fullclean` e recompile. O banner deve mostrar `preemption=1`/`PREEMPTIVO` ou `preemption=0`/`COOPERATIVO`.

A configuração cooperativa mantém bloqueios e yields explícitos. O callback touch pode solicitar uma troca na saída da interrupção, portanto o experimento não representa uma execução sem qualquer reescalonamento.

## Time slicing

O extra foi executado somente em modo preemptivo. No mesmo arquivo:

```c
#define configUSE_TIME_SLICING 1  // ligado
#define configUSE_TIME_SLICING 0  // desligado
```

Como FUS e CTRL têm a mesma prioridade em DM/RM, o rodízio por tick pode afetar a alternância quando ambas estão prontas. O valor de slicing não foi impresso nos logs antigos; os arquivos `SLICING_OFF` identificam os dois ensaios em que foi desligado.

## Matriz executada

| Política | Modo | Frequências | Slicing |
| --- | --- | --- | --- |
| CUSTOM | Preemptivo e cooperativo | 80, 160 e 240 MHz | ON |
| DM | Preemptivo e cooperativo | 80, 160 e 240 MHz | ON |
| DM | Preemptivo | 80 e 240 MHz | OFF |
| RM | Preemptivo e cooperativo | 80 e 240 MHz | ON |

## Roteiro físico

1. Reinicie a placa e mantenha os pads livres durante a calibração.
2. Siga as mensagens `GUIA` no monitor.
3. Execute a sequência aceita `C B C A B C B C B C B C D C`.
4. Aguarde a mensagem de sequência concluída e pelo menos 30 s de métricas.
5. Salve o terminal completo antes de iniciar outra configuração.

O firmware aplica debounce de 1.200 ms por canal e exige 500 ms de liberação estável. A ordem guiada padroniza as funções exercitadas, mas o instante exato dos toques em relação à tarefa periódica depende do operador.
