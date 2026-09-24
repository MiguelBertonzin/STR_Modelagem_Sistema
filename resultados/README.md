# Resultados experimentais

Esta pasta reúne os dados utilizados no relatório:

- `logs/`: saída serial integral dos 18 cenários aceitos;
- `dados/inventario.csv`: identificação, configuração, duração e SHA-256 de cada log;
- `dados/metricas_todas_janelas.csv`: 488 linhas METRIC com arquivo e linha de origem;
- `dados/metricas_finais_por_tarefa.csv`: último resumo das quatro tarefas em cada cenário;
- `dados/resumo_cenarios.csv`: comparação consolidada;
- `dados/metricas_por_janela.csv`: diferenças entre contadores acumulados;
- `graficos/`: as 12 figuras do relatório, em PNG/PDF, e um PDF reunindo todas.

Os logs foram mantidos sem edição. As métricas acumuladas usam o último conjunto completo de quatro tarefas para a comparação final. Para taxas por janela, foram calculadas diferenças entre resumos sucessivos, evitando somar repetidamente os mesmos jobs.

## IDs

- `01` a `08`: matriz obrigatória CUSTOM/DM, preemptivo/cooperativo e 80/240 MHz;
- `EXTRA_01` a `EXTRA_04`: 160 MHz;
- `EXTRA_05` e `EXTRA_06`: DM preemptivo com slicing OFF;
- `EXTRA_07` a `EXTRA_10`: RM.

Cada execução durou pelo menos 30 s, concluiu a sequência guiada e registrou 12 jobs de NAV e um job de FS. Há apenas uma execução por combinação e um evento de emergência por execução; os resultados devem ser lidos como observações experimentais.

O campo de execução mede o intervalo decorrido entre início e fim e pode incluir interferência. Seus máximos são valores observados, não limites formais de WCET. A FUS apresenta um desalinhamento entre a grade de ticks e sua referência em microssegundos; o relatório preserva os contadores reportados e descreve essa limitação.
