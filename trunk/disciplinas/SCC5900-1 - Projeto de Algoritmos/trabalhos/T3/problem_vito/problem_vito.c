#include <stdio.h>
#include <stdlib.h>
#include <strings.h>
#include <math.h>

int main()
{
    int values[500];
    int sum_dist[500];

    int cases, elements, min;
    int i, j, k, l;

    //freopen("input.txt","r",stdin);

    scanf("%d", &cases);

    for(k=0; k<cases; k++) {
        scanf("%d", &elements);
        for(l=0; l<elements; l++) {
            scanf("%d", &values[l]);
        }

        memset(sum_dist, 0, sizeof(sum_dist));

        for(i=0;i<elements;i++) {
            for(j=0;j<elements;j++) {
                if(i!=j) {
                   sum_dist[i]+=abs(values[i]-values[j]);
                }
            }
        }

        min = sum_dist[0];
        int imin = 0;
        for(i=1;i<elements;i++) {
            if(sum_dist[i]<min) {
                min=sum_dist[i];
                //imin=i;
            }
        }

        //printf("%d\n", values[imin]);
        printf("%d\n", min);

    }

    return 0;
}
