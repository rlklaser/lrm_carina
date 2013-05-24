#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

using namespace std;

#define MAX_N 100
#define INF 99999.0

struct ponto {
    int x;
    int y;
    int z;
    int t;
};

typedef struct ponto ponto;

int N, Nr, Np;

void calc_custo(double custo[][MAX_N], ponto p[], int pr[][2], int pp[][2], double custo_p[][MAX_N]) {
    for (int i = 0; i < N; i++) {
        for (int j = i; j < N; j++) {
            if (i == j) custo[i][j] = 0.0;
            else {
                custo[i][j] = sqrt(pow((p[i].x - p[j].x),2.0) + pow((p[i].y - p[j].y),2.0)
                    + pow((p[i].z - p[j].z),2.0) + pow((p[i].t - p[j].t),2.0));
                custo[j][i] = custo[i][j];
            }
        }
    }

    for (int i = 0; i < Nr; i++) {
        custo[pr[i][0]-1][pr[i][1]-1] = INF;
        custo[pr[i][1]-1][pr[i][0]-1] = INF;
    }

    for (int i = 0; i < Np; i++) {
        custo_p[pp[i][0]-1][pp[i][1]-1] = custo[pp[i][0]-1][pp[i][1]-1];
        custo_p[pp[i][1]-1][pp[i][0]-1] = custo[pp[i][0]-1][pp[i][1]-1];
        custo[pp[i][1]-1][pp[i][0]-1] = 0.0;
        custo[pp[i][0]-1][pp[i][1]-1] = 0.0;
    }
}

void prim_alg(double custo[][MAX_N], int caminho[]) {
    memset(caminho, -1, sizeof(int)*N);
    caminho[0] = 0;

    while (true) {
        double min_cost = INF;
        int v0, w0;
        for (int w = 0; w < N; w++) {
            if (caminho[w] == -1) {
                for (int v = 0; v < N; v++) {
                    if ((caminho[v] != -1) && (min_cost > custo[v][w])) {
                        min_cost = custo[v][w];
                        v0 = v;
                        w0 = w;
                    }
                }
            }
        }

        if (min_cost == INF) break;
        else caminho[w0] = v0;
    }
}

int main() {

#ifdef REDIR
    freopen("input.txt","r",stdin);
#endif

    while (scanf("%d", &N) && (N != 0)) {
        ponto p[MAX_N];
        for (int i = 0; i < N; i++)
            scanf("%d %d %d %d", &p[i].x, &p[i].y, &p[i].z, &p[i].t);

        scanf("%d", &Np);
        int pp[Np][2];
        for (int i = 0; i < Np; i++)
            scanf("%d %d", &pp[i][0], &pp[i][1]);

        scanf("%d", &Nr);
        int pr[Nr][2];
        for (int i = 0; i < Nr; i++)
            scanf("%d %d", &pr[i][0], &pr[i][1]);

        double custo[MAX_N][MAX_N], custo_p[MAX_N][MAX_N];
        calc_custo(custo, p, pr, pp, custo_p);

        int caminho[N];
        prim_alg(custo, caminho);

        double custo_total = 0.0;
        for (int i = 0; i < N; i++)
            custo_total += custo[i][caminho[i]];

        for (int i = 0; i < Np; i++)
            custo_total += custo_p[pp[i][0]-1][pp[i][1]-1];

        printf("%0.2lf\n", custo_total);
    }

    return 0;
}
