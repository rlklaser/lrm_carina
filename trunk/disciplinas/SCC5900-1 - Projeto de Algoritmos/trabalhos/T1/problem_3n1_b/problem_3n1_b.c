#include <stdio.h>
#include <stdlib.h>

int main()
{
    int l, n1, n2, i, j, c, max, t;

    while(scanf("%d %d", &n1, &n2)!=EOF)
    {
        if(n1>n2) {
            i = n2;
            j = n1;
        }
        else {
            i = n1;
            j = n2;
        }
        max = 0;
        for(l=i;l<=j;l++) {

            c = 1;
            t = l;
            while(t!=1) {
                t = (t % 2 == 0) ? t/2 : t*3+1;
                c++;
            }
            if(c>max) max = c;
        }

        printf("%d %d %d\n", n1, n2, max);
    }

    return 0;
}
