#include <stdio.h>
#include <stdlib.h>

int main()
{
    char c;
    int state = 0;

    //freopen("input.txt", "r", stdin);

    while(state!=4) {
        c = getchar();
        switch(c) {
            case '\n':
                if(state==3)
                    state=4;
                else {
                    state=1;
                    putchar(c);
                }
                break;
            case '4':
                if(state==1)
                    state = 2; //engloe
                else {
                    putchar(c);
                    state = 0;
                }
                break;
            case '2':
                if(state==2)
                    state=3;
                else {
                    putchar(c); //ecoa
                    state = 0;
                }
                break;
            default:
                if(state==2) putchar('4'); //cospe
                if(state==3) {
                    putchar('4');
                    putchar('2');
                }
                state=0;
                putchar(c); //ecoa
                break;
        }
    }

    return 0;
}
