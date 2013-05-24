#include <stdio.h>
#include <stdlib.h>
#include <string.h>

struct exercicio {
    int tempo;
    int pontos;
};

struct resultado
{
    int tempo;
    int nota;
};

struct exercicio prova[100+1];
struct resultado classe[100+1];

int tabela[100+1][30+1];

#define MAX(A, B) (A>B ? A : B)

/*
 *  IMPLEMENTACAO BASEADA NO PSEUDO CODIGO DO LIVRO
 * THE DESIGN & ANALYSIS OF ALGORITHMS
 * AUTOR: ANANY LEVITIN
 * pp. 301
 */
int dynprog(int i, int j)
{
    int res = 0;
    if(tabela[i][j] < 0) {
        if(j<prova[i-1].tempo)
        {
            res = dynprog(i-1, j);
        }
        else {
            res = MAX(
                dynprog(i-1, j),
                prova[i-1].pontos + dynprog(i-1, j-prova[i-1].tempo)
            );
        }
        tabela[i][j] = res;
    }
    return tabela[i][j];
}

void fazprova(struct resultado* resultado, int questoes)
{
    int i;
    int capacidade = resultado->tempo;

    memset(tabela, -1, sizeof(tabela));
    for(i=0; i<questoes+1; i++) tabela[i][0]=0;
    for(i=0; i<capacidade+1; i++) tabela[0][i]=0;

    resultado->nota = dynprog(questoes, capacidade);
    /*
    int i, j;
    for(i=1; i<questoes+1; i++) {
        for(j=1; j<resultado->tempo+1; j++) {
            if(j < prova[i].tempo) {
                tabela[i][j] =tabela[i-1][j];
            }
            else {
                tabela[i][j] = MAX(tabela[i-1][j], prova[i].pontos + tabela[i-1][j-prova[i].tempo]);
            }
        }
    }
    resultado->nota=tabela[i-1][j-1];
    */
}

int main()
{
#ifdef REDIR
    freopen("input.txt","r",stdin);
#endif
    int questoes, pontos, tempo, alunos, tempoaluno;
    int i;
    double total;

    for(;;) {

        memset(prova, 0, sizeof(prova));
        memset(classe, 0, sizeof(classe));

        scanf("%d", &questoes);
        if(questoes==0) break;
        for(i=0; i<questoes; i++) {
            scanf("%d", &pontos);
            scanf("%d", &tempo);
            prova[i].pontos=pontos;
            prova[i].tempo=tempo;
        }
        scanf("%d", &alunos);
        total = 0;
        for(i=0; i<alunos; i++) {
            scanf("%d", &tempoaluno);
            classe[i].tempo = tempoaluno;

            /*calcula nota do aluno*/
            fazprova(&classe[i], questoes);

            total+=classe[i].nota;
        }

        printf("%.2f\n", total/alunos);

    }
    return 0;
}
