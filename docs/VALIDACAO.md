# Validação da entrega

O código foi compilado a partir de uma configuração limpa com **ESP-IDF v6.1** para o alvo ESP32. O comando `idf.py build` terminou sem erros e gerou o firmware `STR_Modelagem_Sistema.bin`, com 159.632 bytes. O binário ocupa menos de 15% da partição de aplicação de 1 MiB.

A configuração padrão versionada usa:

- ESP32 em modo UNICORE;
- CPU a 240 MHz;
- flash de 4 MB;
- política CUSTOM no código principal;
- estatísticas de tempo de execução habilitadas.
