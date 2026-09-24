# Resultados experimentais

Esta pasta reúne os dados:

- `logs/`: saída serial integral dos 18 cenários aceitos;
- `dados/inventario.csv`: identificação, configuração, duração e SHA-256 de cada log;
- `dados/metricas_todas_janelas.csv`: 488 linhas METRIC com arquivo e linha de origem;
- `dados/metricas_finais_por_tarefa.csv`: último resumo das quatro tarefas em cada cenário;
- `dados/resumo_cenarios.csv`: comparação consolidada;
- `dados/metricas_por_janela.csv`: diferenças entre contadores acumulados;
- `graficos/`


## IDs

- `01` a `08`: matriz CUSTOM/DM, preemptivo/cooperativo e 80/240 MHz;
- `EXTRA_01` a `EXTRA_04`: 160 MHz;
- `EXTRA_05` e `EXTRA_06`: DM preemptivo com slicing OFF;
- `EXTRA_07` a `EXTRA_10`: RM.

Cada execução durou pelo menos 30 s, concluiu a sequência guiada e registrou 12 jobs de NAV e um job de FS. Há apenas uma execução por combinação e um evento de emergência por execução.
