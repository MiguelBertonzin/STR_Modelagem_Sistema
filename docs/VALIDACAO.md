# Validação da entrega

O código foi compilado a partir de uma configuração limpa com **ESP-IDF v6.1** para o alvo ESP32. O comando `idf.py build` terminou sem erros e gerou o firmware `STR_Modelagem_Sistema.bin`, com 159.632 bytes. O binário ocupa menos de 15% da partição de aplicação de 1 MiB.

A configuração padrão versionada usa:

- ESP32 em modo UNICORE;
- CPU a 240 MHz;
- flash de 4 MB;
- política CUSTOM no código principal;
- estatísticas de tempo de execução habilitadas.

O conjunto experimental contém 18 logs integrais e 488 linhas de métricas. Os 18 hashes SHA-256 registrados em `resultados/dados/inventario.csv` foram comparados com os arquivos publicados e não apresentaram divergências.

O relatório final acompanha o projeto em PDF e também em fontes LaTeX compatíveis com Overleaf. Os dados, tabelas e figuras empregados na análise permanecem disponíveis nas pastas `resultados/` e `relatorio/latex/dados/`.

As alterações de preempção e time slicing usadas na matriz experimental pertencem à configuração do kernel do ESP-IDF instalado. O procedimento de reprodução está documentado em `CONFIGURACAO_EXPERIMENTOS.md`.
