#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MAX 102
#define MIN(a,b) ((a)<(b)?(a):(b))

typedef unsigned int precision;

#define INFINITE 999999999

struct mafioso {
    char nome[22];
    precision dificuldade;
};

struct mafioso mafiosos[MAX];
precision matrix[MAX][MAX];

/*
 *  IMPLEMENTACAO BASEADA NO PSEUDO CODIGO DO LIVRO
 * THE DESIGN & ANALYSIS OF ALGORITHMS
 * AUTOR: ANANY LEVITIN
 * pp. 290
 */
void floyd(int n)
{
    int i, j, k;
    precision a, b;
    for(k=0; k<n; k++) {
        for(i=0; i<n; i++) {
            for(j=0; j<n; j++) {
                if(matrix[i][k]<INFINITE && matrix[k][j]<INFINITE) {
                    a = matrix[i][k]+matrix[k][j];
                }
                else {
                    a = INFINITE;
                }
                b = matrix[i][j];
                matrix[i][j] = MIN(b, a);
            }
        }
    }
}

int compare_dif (const void * a, const void * b)
{
    return  ((struct mafioso*)a)->dificuldade - ((struct mafioso*)b)->dificuldade;
}

int compare_name (const void * a, const void * b)
{
  return  strcmp(((struct mafioso*)a)->nome, ((struct mafioso*)b)->nome);
}

int main()
{
#ifdef REDIR
    freopen("input.txt","r",stdin);
#endif
    int cases, cenario = 0;
    int i, j;
    precision m;
    float inf;
    for(;;) {
        scanf("%d", &cases);
        if (cases==0) break;
        cenario++;
        memset(mafiosos, 0, sizeof(mafiosos));
        memset(matrix, INFINITE, sizeof(matrix));
        for(i=0; i<cases; i++) {
            scanf("%s", &(mafiosos[i].nome[0]));
        }
        for(i=0; i<cases; i++) {
            for(j=i; j<cases; j++) {
                if(i==j) {
                    matrix[i][i] = 0;
                }
                else {
                    scanf("%f", &inf);
                    if(inf>0) {
                        inf = (1.0-inf)+0.000001;
                        matrix[i][j] = (precision)(inf * 10000.0);
                    }
                    else {
                        matrix[i][j] = INFINITE;
                    }
                    matrix[j][i] = matrix[i][j];
                }
            }
        }
        floyd(cases);
        for(i=0; i<cases; i++) {
            for(j=0; j<cases; j++) {
                mafiosos[i].dificuldade+=matrix[i][j];
            }
        }

        qsort(mafiosos, cases, sizeof(struct mafioso), compare_dif);

        j=0;
        m = mafiosos[j].dificuldade;
        j++;
        for(i=1; i<cases; i++) {
            if(mafiosos[i].dificuldade>m) break;
            j++;
        }

        if(j>1) {
            qsort(mafiosos, j, sizeof(struct mafioso), compare_name);
        }

        printf("Cenario #%d:\n", cenario);
        for(i=0; i<j; i++) {
            printf("%s\n", mafiosos[i].nome);
        }
        printf("\n");

    }

    printf("EOR\n");

    return 0;
}
