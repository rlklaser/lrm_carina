#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <queue>


#define MAX_PONTOS 110

/*#define DEBUG*/
#define DOINT

#ifdef DOINT
 typedef long precision;
 #define POSF sqrt
 #define PREF
#else
 typedef double precision;
 #define POSF
 #define PREF sqrt
#endif

#define INF 99999999
/*maior distancia sera 16000000*/

struct ponto {
    int x;
    int y;
    int z;
    int t;
};

struct item {
    struct ponto p;
#ifdef DEBUG
    int anterior;
#endif
};

struct contexto {
    int total_pontos;
    struct item pontos[MAX_PONTOS+1];
    precision grafo[MAX_PONTOS+1][MAX_PONTOS+1];
    double custo;
    int visitados[MAX_PONTOS];
};

struct contexto ctx;
std::priority_queue<std::pair<int, std::pair<int, int> > >* queue;

precision dist_euclidiana(int p1, int p2)
{
    struct ponto* pt1 = &ctx.pontos[p1].p;
    struct ponto* pt2 = &ctx.pontos[p2].p;
    return PREF(
            (pt1->x-pt2->x)*(pt1->x-pt2->x) +
            (pt1->y-pt2->y)*(pt1->y-pt2->y) +
            (pt1->z-pt2->z)*(pt1->z-pt2->z) +
            (pt1->t-pt2->t)*(pt1->t-pt2->t)
            );
}

void visita(int v)
{
    int j;
    precision dist;
    for(j=1; j<ctx.total_pontos+1; j++) {
        if(ctx.visitados[j]==0 && v!=j) {
            dist = ctx.grafo[v][j];
            if(dist!=INF) {
                queue->push(
                    std::make_pair<precision, std::pair<int, int> >
                    (-dist, std::make_pair<int, int>(v, j)));
            }
        }
    }
    ctx.visitados[v]=1;
}

void conecta(int u, int v)
{
    precision dist = ctx.grafo[u][v];
    if(dist==INF) return; /*nao reconecta ja conectados*/
    if(dist<0) dist = -dist;
    ctx.custo += POSF(dist);
    ctx.grafo[u][v]=INF;
    ctx.grafo[v][u]=INF;
    visita(v);
#ifdef DEBUG
    ctx.pontos[u].anterior=v;
    printf("(%d,%d:%.2f)", u, v, POSF(dist));
#endif
}

int main()
{

#ifdef REDIR
    freopen("input.txt","r",stdin);
#endif

    int pontos;
    int x, y, z, t;
    int i, j, k;
    int rpos, rneg;
    int u, v;
    precision dist;
    std::pair<precision, std::pair<int, int> > item;
    std::pair<int, int> ponte;

    for(;;) {
        scanf("%d", &pontos);
        if(pontos==0) break;

        memset(&ctx, 0, sizeof(struct contexto));
        ctx.total_pontos = pontos;
        queue = new std::priority_queue<std::pair<int, std::pair<int, int> > >();

        for(i=1; i<pontos+1; i++) {
            scanf("%d %d %d %d", &x, &y, &z, &t);
            ctx.pontos[i].p.x=x;
            ctx.pontos[i].p.y=y;
            ctx.pontos[i].p.z=z;
            ctx.pontos[i].p.t=t;
        }

        /*monta o grafo totalmente conexo*/
        for(i=1; i<pontos; i++) {
            for(j=i+1; j<pontos+1; j++) {
                dist=dist_euclidiana(i, j);
                ctx.grafo[i][j]=dist;
                ctx.grafo[j][i]=dist;
            }
        }

        /*le restricoes*/
        scanf("%d", &rpos);
        for(k=0; k<rpos; k++) {
            scanf("%d %d", &i, &j);
            //conecta(i, j);
            ctx.custo+=POSF(ctx.grafo[i][j]);
            ctx.grafo[i][j]=0;
            ctx.grafo[j][i]=0;
        }
        scanf("%d", &rneg);
        for(k=0; k<rneg; k++) {
            scanf("%d %d", &i, &j);
            ctx.grafo[i][j] = INF;
            ctx.grafo[j][i] = INF;
        }

        visita(1);

        while(true) {
            if(queue->empty()) break;
            item = queue->top();
            if(item.first==INF) break;
            ponte = item.second;
            u = ponte.first;
            v = ponte.second;
            queue->pop();
            if(ctx.visitados[u]==1 && ctx.visitados[v]==0) {
                conecta(u, v);
            }
        }

        printf("%.2f\n", ctx.custo);
        delete queue;
    }

    return 0;
}
