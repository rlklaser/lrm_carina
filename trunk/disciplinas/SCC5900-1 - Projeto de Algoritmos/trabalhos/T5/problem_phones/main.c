#include <stdio.h>
#include <stdlib.h>
#include <string.h>

int map(char c)
{
    int res = 0;

    switch(c) {
        case '0':
            break;
        case '1':
            res = 1;
            break;
        case '2':case 'A':case 'B':case 'C':
            res = 2;
            break;
        case '3':case 'D':case 'E':case 'F':
            res = 3;
            break;
        case '4':case 'G':case 'H':case 'I':
            res = 4;
            break;
        case '5':case 'J':case 'K':case 'L':
            res = 5;
            break;
        case '6':case 'M':case 'N':case 'O':
            res = 6;
            break;
        case '7':case 'P':case 'R':case 'S':
            res = 7;
            break;
        case '8':case 'T':case 'U':case 'V':
            res = 8;
            break;
        case '9':case 'W':case 'X':case 'Y':
            res = 9;
            break;
        default:
            res = -1;
    }

    return res;
}

int compare (const void * a, const void * b)
{
  return ( *(int*)a - *(int*)b );
}

int main()
{
#ifdef REDIR
  freopen("input.txt","r",stdin);
#endif

    int s, n, sets, numbs;
    char number[2][100];
    char nout[10];
    int i, j, k, l;
    char c;
    int numbers[101000];
    int reps;
    int repnum;
    int printed;

    scanf("%d", &sets);
    for(s=0; s<sets; s++) {
        if(s>0) printf("\n");
        scanf("%d", &numbs);
        memset(numbers, 0, sizeof(numbers));
        l=0;
        for(n=0;n<numbs; n++) {
            memset(number, 0, sizeof(number));
            scanf("%s", &number[0][0]);
            i=0;
            k=1;
            number[1][0] = '1';
            do {
                c = number[0][i];
                j=map(c);
                if(j!=-1) {
                    number[1][k++]=j+0x30;
                }
                i++;
            } while (number[0][i]!=0);
            numbers[l++]=atoi(&number[1][0]);
        }
        qsort(numbers, numbs, sizeof(int), compare);
        reps = 1;
        repnum=0;
        printed = 0;
        for(n=0;n<numbs; n++) {
            if(repnum==numbers[n]) {
                reps++;
            }
            else {
                if(reps>1) {
                    memset(nout, 0, sizeof(nout));
                    sprintf(nout, "%d", repnum);
                    printf("%c%c%c-%c%c%c%c %d\n", nout[1],nout[2],nout[3],nout[4],nout[5],nout[6],nout[7],reps);
                    printed = 1;
                }
                reps = 1;
            }
            if(n==numbs-1 && reps>1) {
                memset(nout, 0, sizeof(nout));
                sprintf(nout, "%d", repnum);
                printf("%c%c%c-%c%c%c%c %d\n", nout[1],nout[2],nout[3],nout[4],nout[5],nout[6],nout[7],reps);
                printed = 1;
            }
            repnum=numbers[n];
        }
        if(!printed) {
            printf("No duplicates.\n");
        }
    }

    return 0;
}
