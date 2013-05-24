#include <stdio.h>
#include <stdlib.h>

int main()
{
    int m, n;

    while(scanf("%d %d", &m, &n)!=EOF)
    {
        if(m==1 && n==1) break;
        if(m==n) {
            printf("I");
        }
        while (m!=n) {
            if (m<n) {
                printf("L");
                n=n-m;
            }
            else {
                printf("R");
                m=m-n;
            }
        }
        printf("\n");
    }

    return 0;
}
