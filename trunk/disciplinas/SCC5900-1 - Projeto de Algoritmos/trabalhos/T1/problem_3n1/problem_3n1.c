#include <iostream>
//#include <string>

using namespace std;

typedef unsigned int uint;

static uint cache[1000000];

inline int calc_cicl(uint val)
{
    uint ciclos=0;
    uint v = val;
    if(cache[val]==0) {
        for(;;) {
            ciclos++;
            //cout << v << " ";
            if(v==1) break;
            if(v%2==1) v=(3*v)+1; else v=(v/2);
        }
        cache[val]=ciclos;
    }
    else {
        ciclos = cache[val];
    }
    return ciclos;
}

int main()
{
    uint i, j, ii, jj;
    uint ciclo;
    uint max;

    //memset(cache, 0, sizeof(cache));
    //cache[] = 0;

    while(!cin.eof()) {
        i=0;
        j=0;
        cin >> i >> j;

        if(i==0) break;

        //swap(i, j);
        if(i>j) {
            ii=j;
            jj=i;
        }
        else {
            ii=i;
            jj=j;
        }
        max = 0;
        for(uint n=ii; n<=jj; n++) {
            //cout << "c:" << n << "=";
            ciclo=calc_cicl(n);
            if (ciclo>max) max=ciclo;
            //cout << "/" << ciclo << endl;
            cout << ciclo << endl;
        }
        cout << i << " " << j << " " << max << endl;
    }

    return 0;
}
