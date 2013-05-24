#include <stdio.h>
#include <string.h>
#include <math.h>

int len(char* str)
{
    int j=0;
    char c=str[0];
    while(c!=0) c=str[++j];
    return j;
}

int main()
{

#ifdef REDIR
  freopen("input.txt","r",stdin);
#endif

    int ii, i, j, k, cases, mchar;
    char str[80], strout[80], strout2[80], misses[80];
    long long encoded;
    long long d;
    int total = 26;
    int alpha;
    char mapa[26];
    char remapa[26];

    char c;

    scanf("%d", &cases);
    for(ii=0; ii<cases; ii++) {
        memset(str, 0, sizeof(str));
        memset(strout, 0, sizeof(strout));
        memset(strout2, 0, sizeof(strout2));
        memset(misses, 0, sizeof(misses));
        memset(mapa, 1, sizeof(mapa));
        memset(remapa, 0, sizeof(remapa));

        scanf("%s", str);
        scanf("%s", misses);

        mchar = len(misses);

        for(k=0; k<mchar; k++) {
            mapa[misses[k]-'A']=0;
        }

        j=0;
        for(k=0; k<26; k++) {
            if(mapa[k]) {
                remapa[j++] = 'A'+ k;
            }
        }

        j=len(str);
        k = 0;
        encoded=0;
        while(j!=0) {
            j--;
            c = str[j];
            encoded+=(c-'A') * pow(total, k);
            k++;
        }


        i=0;
        alpha = (total-mchar);
        if(encoded==0) {
            strout[0] = remapa[0];
            i=1;
        }
        while(encoded>0) {
            d = encoded / alpha;
            c = encoded - (d * alpha);
            c = remapa[(int)c];
            strout[i++] = c;
            encoded = d;
        }

        j=0;
        for(k=i;k>0;k--) {
            /*printf("%c", strout[k-1]);*/
            strout2[j++] = strout[k-1];
        }
        printf("%s\n", strout2);

    }

    return 0;
}
