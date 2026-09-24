# Relatório

O arquivo `Relatorio_M1_Drone.pdf` é a versão compilada do relatório entregue com o projeto.

As fontes estão em `latex/`. Para importá-las no Overleaf, compacte o conteúdo dessa pasta mantendo `main.tex` na raiz do ZIP. O documento utiliza `references.bib`, as imagens vetoriais em `imagens/` e as tabelas já preparadas em `tabelas/`.

Compilação local tradicional:

```text
pdflatex main.tex
bibtex main
pdflatex main.tex
pdflatex main.tex
```
