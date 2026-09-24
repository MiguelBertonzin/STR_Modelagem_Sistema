# Drone em ESP32 com FreeRTOS

Trabalho da disciplina **Sistemas em Tempo Real**, desenvolvido para a Avaliação M1 — Modelagem de Sistema, na temática Drone.

O projeto implementa em uma ESP32 um autopiloto didático com quatro funções principais:

- fusão de sinais inerciais simulados a cada 5 ms;
- controle PID de atitude e atualização de quatro motores simulados;
- navegação e telemetria acionadas por touch;
- fail-safe acionado por interrupção, com bloqueio permanente dos motores.

O objetivo experimental é observar deadlines, jitter, latência de ativação e ocupação de CPU sob diferentes políticas de prioridade, modos de escalonamento e frequências. Foram executados 18 cenários em UNICORE, incluindo todos os testes obrigatórios e os extras de RM, 160 MHz e time slicing.

## Documentos principais

- [Relatório completo](relatorio/Relatorio_M1_Drone.pdf)
- [Código principal](main/main.c)
- [Guia das configurações experimentais](docs/CONFIGURACAO_EXPERIMENTOS.md)
- [Validação da entrega](docs/VALIDACAO.md)
- [Resumo dos resultados coletados](resultados/README.md)

O código inicial disponibilizado pelo professor foi usado como referência para a organização das tarefas: [Drone_exemplo_oTask.c](https://github.com/VielF/Real_Time_Systems/blob/main/Assignaments/Drone_exemplo_oTask.c). A implementação deste repositório foi atualizada para a API de touch do ESP-IDF 6.1 e ampliada com simulação funcional, instrumentação, políticas de prioridade e roteiro de coleta.

## Hardware e ambiente

- ESP32 DevKit;
- ESP-IDF v6.1;
- FreeRTOS em modo UNICORE;
- flash configurada para 4 MB;
- Touch A: T0 / GPIO4 — perturbação e carga;
- Touch B: T7 / GPIO27 — navegação;
- Touch C: T4 / GPIO13 — telemetria;
- Touch D: T9 / GPIO32 — emergência.

O projeto usa a API oficial [`driver/touch_sens.h`](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/cap_touch_sens.html). O exemplo oficial consultado está em [`touch_sens_basic`](https://github.com/espressif/esp-idf/tree/master/examples/peripherals/touch_sensor/touch_sens_basic).

## Compilação e execução

Com o ambiente ESP-IDF ativado:

```powershell
idf.py set-target esp32
idf.py build
idf.py -p COMx flash monitor
```

Substitua `COMx` pela porta da placa. Para sair do monitor, use `Ctrl+]`.

O padrão versionado em `sdkconfig.defaults` é **CUSTOM + preemptivo + 240 MHz + UNICORE**. A política é selecionada em `main/main.c` pela macro `PRIORITY_POLICY`. Frequência, preempção e time slicing são explicados em [CONFIGURACAO_EXPERIMENTOS.md](docs/CONFIGURACAO_EXPERIMENTOS.md).

Ao iniciar, mantenha os pads livres durante a calibração. Depois, o terminal apresenta um roteiro de 14 eventos. Cada contato deve ser mantido até o reconhecimento e removido antes do próximo passo.

## Estrutura

```text
.
├── main/                       código da aplicação ESP-IDF
├── docs/                       enunciado e guia de configuração
├── relatorio/                  PDF final e fontes LaTeX
├── resultados/
│   ├── logs/                   18 registros completos dos ensaios
│   ├── dados/                  tabelas derivadas em CSV
│   └── graficos/               figuras em PNG e PDF
├── CMakeLists.txt
└── sdkconfig.defaults
```

Arquivos gerados pelo build não são versionados. Os registros experimentais foram mantidos sem correções retrospectivas; as limitações da instrumentação e o alcance das conclusões são discutidos no relatório.

## Síntese dos resultados

Nas coletas realizadas, CUSTOM e DM preemptivos a 160 e 240 MHz não registraram perdas. A 80 MHz, NAV apresentou perdas em todas as configurações avaliadas. DM e RM preemptivos também registraram atraso na conclusão do job de segurança nessa frequência. Todos os cenários cooperativos apresentaram perdas reportadas da FUS.

Esses valores descrevem as execuções observadas. Uma execução sem misses não constitui, sozinha, garantia formal de pior caso. O relatório apresenta os denominadores, os tempos máximos observados e as limitações da referência temporal utilizada na FUS.

## Autor

Miguel Bertonzin — Engenharia de Computação, Universidade do Vale do Itajaí.
