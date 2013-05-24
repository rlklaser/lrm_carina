#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <list>

#define INT
#ifdef INT
    typedef unsigned long precision;
    #define FUNC
    #define FINC sqrt
    #define INFINITE ((precision)-1)
#else
    typedef double precision;
    #define FUNC sqrt
    #define FINC
    #define INFINITE ((precision)999999)
#endif

struct ponto {
    int x;
    int y;
    int z;
    int t;
};

struct item {
    int ndx;
    struct ponto p;
    precision distancia;
    char visitado;
    int anterior;
    int restricao;
};

struct ponte {
    int pa;
    int pb;
};

#define MAX_PONTOS 101

struct contexto {
    int tamanho;
    struct item pontos[MAX_PONTOS+1];
    precision grafo[MAX_PONTOS+1][MAX_PONTOS+1];
};

struct contexto ctx;

precision dist_euclidiana(int p1, int p2)
{
    struct ponto* pt1 = &ctx.pontos[p1].p;
    struct ponto* pt2 = &ctx.pontos[p2].p;
    return FUNC(
            pow(pt1->x-pt2->x, 2) +
            pow(pt1->y-pt2->y, 2) +
            pow(pt1->z-pt2->z, 2) +
            pow(pt1->t-pt2->t, 2)
            );
}

/*faz busca sequencial mesmo, nao precisa atualizar pesos*/
std::list<int>* priority_queue;
int priority_remove()
{
    int v = -1;
    int last = -1;
    precision min = INFINITE;

    for(std::list<int>::iterator i = priority_queue->begin(); i != priority_queue->end(); i++) {
        v = (*i);
        if(ctx.pontos[v].distancia<min) {
            min = ctx.pontos[v].distancia;
            last=v;
        }
    }
    if(last==-1) last = v;
    ctx.pontos[last].visitado=1;
    priority_queue->remove(last);
    return last;
}

int main()
{

#ifdef REDIR
    freopen("input.txt","r",stdin);
#endif

    int saltos;
    int x, y, z, t;
    int i, j, k;
    int rpos, rneg;
    int pa, pb;
    int central;
    double custo_total;
    int u, v;
    precision dist_u, dist_v;
    precision alt;

    for(;;) {
        scanf("%d", &saltos);
        if(saltos==0) break;

        /*reinicializa tudo*/
        memset(&ctx, 0, sizeof(struct contexto));
        ctx.tamanho=saltos;
        central=-1;
        custo_total = 0.0;
        priority_queue = new std::list<int>();

        for(i=0; i<saltos; i++) {
            scanf("%d %d %d %d", &x, &y, &z, &t);
            k = i+1;
            ctx.pontos[k].ndx=k;
            ctx.pontos[k].p.x=x;
            ctx.pontos[k].p.y=y;
            ctx.pontos[k].p.z=z;
            ctx.pontos[k].p.t=t;

            /*inicializacao Dijkstra*/
            ctx.pontos[k].distancia=INFINITE;
            ctx.pontos[k].anterior=0;
            ctx.pontos[k].visitado=0;

            if(x==0 && y==0 && z==0) {
                if(t==0) { /*usa o t0 como central sempre*/
                    central = k;
                }
                else if(central==-1) { /*se nao ha central ainda*/
                    central = k;
                }
            }
        }

        if(central==-1) central=1; /*nao definiu central, inicia no primeiro*/

        /*monta o grafo totalmente conexo*/
        for(i=1; i<saltos; i++) {
            for(j=i+1; j<saltos+1; j++) {
                ctx.grafo[i][j]=dist_euclidiana(i, j);
                ctx.grafo[j][i]=ctx.grafo[i][j]; /*espelha - grafo nao orientado*/
            }
        }

        /*le restricoes*/
        scanf("%d", &rpos);
        for(k=0; k<rpos; k++) {
            scanf("%d %d", &pa, &pb);
            i = pa;
            j = pb;
            /*adiciona nos dois sentidos, tanto faz a que cumprir primeiro*/
            ctx.pontos[i].restricao=j;
            ctx.pontos[j].restricao=i;
        }
        scanf("%d", &rneg);
        for(k=0; k<rneg; k++) {
            scanf("%d %d", &pa, &pb);
            i = pa;
            j = pb;
            /*negativa: adiciona custo absurdo para inviabilizar construcao*/
            /*se houver restricao positiva, desconsidera*/
            if(ctx.pontos[i].restricao!=j) {
                ctx.grafo[i][j] = INFINITE;
                ctx.grafo[j][i] = INFINITE;
            }
        }

        /*
        Algoritmo Dijkstra

        pseudo-codigo fonte: Wikipaedia
        */
        /*inicia no ponto considerado central*/
        ctx.pontos[central].distancia=0;
        for(i=1; i<saltos+1; i++) {
            priority_queue->push_back(i);
        }

        while(!priority_queue->empty()) {
            u = priority_remove();
            dist_u = ctx.pontos[u].distancia;

            /*contabiliza o custo do no visitado*/
            j = ctx.pontos[u].anterior;
            if(j>0) {
                custo_total += FINC(ctx.grafo[u][j]);
            }

            /*se tem anterior esquece esse no, continua do menor qualquer*/
            if(dist_u==INFINITE) {
                if(j==0) {
                    /*exesso de restricoes!*/
                    /*conecta ao central*/
                    v = central;
                    alt = dist_euclidiana(u, v);
                    ctx.pontos[u].distancia=alt;
                    ctx.pontos[u].anterior=v;
                    custo_total += FINC(alt);
                }
                continue;
            }

            if(priority_queue->empty()) break; /*se acabou sai*/

            v = ctx.pontos[u].restricao;
            if(v>0) {
                alt = dist_u + ctx.grafo[u][v];
                ctx.pontos[v].distancia=alt;
                ctx.pontos[v].anterior=u;
                ctx.pontos[v].restricao=0; /*restricao cumprida para o outro sentido*/
            }
            else {
                for(v=1; v<saltos+1; v++) {
                    if(v!=u) {
                        if(ctx.pontos[v].visitado==0 && ctx.grafo[u][v]!=INFINITE) {
                            if(ctx.pontos[v].anterior!=0) {
                                alt = ctx.pontos[ctx.pontos[v].anterior].distancia + ctx.grafo[u][v];
                            }
                            else {
                                alt = ctx.grafo[u][v];
                            }
                            dist_v = ctx.pontos[v].distancia;
                            if(alt<dist_v) {
                                ctx.pontos[v].distancia=dist_u + ctx.grafo[u][v];
                                ctx.pontos[v].anterior=u;
                            }
                        }
                    }
                }
            }
        }

        printf("%.2f\n", custo_total);

        delete priority_queue;
    }

    return 0;
}
