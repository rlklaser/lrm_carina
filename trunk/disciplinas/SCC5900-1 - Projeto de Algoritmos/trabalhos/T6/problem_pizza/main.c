#include <stdio.h>
#include <stdlib.h>
#include <string.h>

int compare (const void * a, const void * b)
{
  return ( *(int*)a - *(int*)b );
}

int main()
{
#ifdef REDIR
    freopen("input.txt","r",stdin);
#endif
    int pedidos, queries;
    int i, id;
    int tempo;
    int cliente;
    int clientes[10001];
    int count;
    int last;
    int dia = 0;
    int max, min;

    for(;;)
    {
        scanf("%d", &pedidos);
        scanf("%d", &queries);

        if(pedidos==0 && queries==0) break;

        printf("Dia %d:\n", ++dia);

        memset(clientes, 0, sizeof(clientes));
        max=0;
        min=20000;

        for(i=0; i<pedidos; i++) {
            scanf("%d", &id);
            if(id<min) min = id;
            if(id>max) max = id;
            clientes[id]++;
        }

        count=1;
        for(i=min; i<max; i++) {
            if(clientes[i]!=0) {
                last=clientes[i];
                clientes[i] = count;
                count+=last;
            }
        }
        clientes[max] = count;

        for(i=0; i<queries; i++) {
            scanf("%d", &cliente);
            tempo = clientes[cliente];
            printf("Cliente %d: ", cliente);
            if(tempo==0) {
                printf("quem?");
            }
            else {
                printf("%dmin.", tempo);
            }
            printf("\n");
        }


    }

    return 0;
}
