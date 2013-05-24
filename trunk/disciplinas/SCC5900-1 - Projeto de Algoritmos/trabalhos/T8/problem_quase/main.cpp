#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#define MAX_HANOI 100

int ref_hanoi[MAX_HANOI];
int run_hanoi[MAX_HANOI];

int compare (const void * a, const void * b)
{
    /*ordem decrescente*/
    return ( *(int*)b - *(int*)a );
}

void gira(int g, int size)
{
    int swap;
    int i = g;
    int j = size-1;

    while(i<j) {
        swap=run_hanoi[i];
        run_hanoi[i]=run_hanoi[j];
        run_hanoi[j]=swap;
        i++;
        j--;
    }
    printf(" %d", g);

}

int seek(int start, int value)
{
    int i=start;
    for(; i<MAX_HANOI; i++) {
        if(run_hanoi[i]==value) break;
    }
    return i;
}

int main()
{
#ifdef REDIR
    freopen("input.txt","r",stdin);
#endif

    int n;
    int val;
    int i, k;
    int giro_a, giro_b;
    int girou;
    int caso = 0;

    for(;;) {
        scanf("%d", &n);
        if(n==0) break;

        printf("Caso #%d:", ++caso);
        girou=0;

        for(i=0; i<n; i++) {
            scanf("%d", &val);
            ref_hanoi[i]=val;
            run_hanoi[i]=val;
        }

        /*
        algoritmo
        usa uma torre quasi-hanoi de referencia e tenta alcanca-la com giros que a igualem
        - a estrategia eh executar dois giros em sequencia;
            a) encontra o indice da peca fora de ordem na torre quasi-hanoi
            b) guarda o indice da posicao da referecia que esta fora de ordem
        -> um giro na ultima posicao nao faz sentido, ignora o giro a porque ja esta no topo
        - executa o giro a (para que esse va para a posicao no topo)
        - executa o giro b (para trazer o topo para esta posicao)
        */

        qsort(ref_hanoi, n, sizeof(int), compare);
        k = 0;
        while(k<n)
        {
            for(i=k; i<n; i++) {
                /*ponto fora da ordem*/
                if(run_hanoi[i]!=ref_hanoi[i]) {

                    giro_a = seek(i, ref_hanoi[i]);
                    giro_b = i;

                    if(giro_a!=n-1) {
                        gira(giro_a, n);
                    }

                    gira(giro_b, n);
                    girou = 1;
                    break;
                }
            }
            k=i;
        }
        if(!girou) {
            printf(" em ordem");
        }
        printf(".\n");

    }

    return 0;
}
