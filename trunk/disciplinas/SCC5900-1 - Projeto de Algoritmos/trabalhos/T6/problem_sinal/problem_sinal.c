#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

int primes[150000];
int prime_ndx=0;
int prime_max;

/*sieve de Erathostenes*/
void calc_prime(int m)
{
    unsigned long x;
    char sieve[(m)/8+1];
    unsigned long y;
    memset(sieve, 0xFF, sizeof(sieve));
    for(x = 2; x <= m; x++) {
        if(sieve[x/8] & (0x01 << (x % 8))){

            primes[prime_ndx] = x;
            prime_ndx++;
            prime_max = x;

            for(y = 2*x; y <= m; y += x) {
                sieve[y/8] &= ~(0x01 << (y % 8));
            }
        }
    }
}

int main()
{
#ifdef REDIR
    freopen("input.txt","r",stdin);
#endif
    int r, n, m, mm;
    int i, j, k;

    calc_prime(2000000);

    for(;;) {
        r = scanf("%d", &n);
        if(r==-1) break;
        r = scanf("%d", &m);

        if(n==m && n==1) {
            printf("0\n");
            continue;
        }

        if(n>m) {
            mm = m;
            m = n;
            n = mm;
        }

        if(n==m && n==1) {
            k=0;
        }
        else {
            for(i=0; i<prime_ndx; i++) if(primes[i]>=n) break;

            for(j=i; j<prime_ndx; j++) {
                if(primes[j]==m) {
                    k = j-i+1;
                    break;
                }
                if(primes[j]>m) {
                    k = j-i;
                    break;
                }
            }
            if(j==prime_ndx) {
                k=j-i;
            }

        }

        printf("%d\n", k);
    }

    return 0;
}
