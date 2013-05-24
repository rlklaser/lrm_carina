#include <stdio.h>
#include <stdlib.h>
#include <strings.h>

typedef struct coord {
    int i;
    int j;
} coord;

int main()
{

    char table[101][101];
    int result[101][101];
    int l, c, i, j, k;
    int field = 0;
    coord co[8];
    char e;

    //FILE* f = freopen("input.txt","r",stdin);

    for(;;)
    {
        memset(table, 0, sizeof(table));
        memset(result, 0, sizeof(result));
        scanf("%d %d", &l, &c);
        if(l==0) break;

        field++;

        for(i=0; i<l; i++) {
            scanf("%s", &table[i+1][1]);
        }

        for(i=0; i<l; i++) {
            for(j=0; j<c; j++) {
                e = table[i+1][j+1];
                if(e=='*') {
                    co[0].i = i-1;
                    co[0].j = j-1;

                    co[1].i = i-1;
                    co[1].j = j;

                    co[2].i = i-1;
                    co[2].j = j+1;

                    co[3].i = i;
                    co[3].j = j-1;

                    co[4].i = i;
                    co[4].j = j+1;

                    co[5].i = i+1;
                    co[5].j = j-1;

                    co[6].i = i+1;
                    co[6].j = j;

                    co[7].i = i+1;
                    co[7].j = j+1;

                    for(k=0;k<8;k++) {
                        result[1+co[k].i][1+co[k].j]++;
                    }

                }
            }
        }

        if(field>1) printf("\n");

        printf("Field #%d:\n", field);
        for(i=0; i<l; i++) {
            for(j=0; j<c; j++) {
                e = table[i+1][j+1];
                if(e!='*') {
                    e = result[i+1][j+1] + 0x30;
                }
                printf("%c", e);
            }
            printf("\n");
        }
        //printf("\n");
    }

    return 0;
}
