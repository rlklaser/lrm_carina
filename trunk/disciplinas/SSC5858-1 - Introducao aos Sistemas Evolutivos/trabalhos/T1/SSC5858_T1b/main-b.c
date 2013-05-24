#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <math.h>

#define MAX_INDIVIDUOS  10
#define MAX_ITERACOES   10000
#define MAX_GERACOES  10
#define MIN_FUNCAO  -10
#define MAX_FUNCAO  10

typedef struct individuo
{
    float x1;
    float x2;
    float fitness;
} individuo;

typedef struct contexto
{
    int inicializado;
    int melhor;
    individuo populacao[MAX_INDIVIDUOS];
    individuo nova_populacao[MAX_INDIVIDUOS];

} *pctx;

void populacao(pctx ctx);
void avaliacao(pctx ctx);
void selecao(pctx ctx);
void crossover(pctx ctx);
void mutacao(pctx ctx);

int main()
{
    struct contexto ctx;
    pctx pctx = &ctx;

    memset(pctx, 0, sizeof(ctx));

    srand(time(NULL)); //semente

    int melhor=-1;
    int melhor_geracoes = 0;
    int geracoes = 0;

    for(;;) {
        geracoes++;

        populacao(pctx);
        avaliacao(pctx);
        selecao(pctx);

        // condicao de saida
        if (melhor==pctx->melhor) {
            melhor_geracoes++;
        }
        else {
            melhor_geracoes = 0;
            melhor=pctx->melhor;
        }


        printf("%2.3f\t%2.3f\t%2.3f\n", ctx.populacao[melhor].x1, ctx.populacao[melhor].x2, ctx.populacao[melhor].fitness);


        if(melhor_geracoes==MAX_GERACOES) break; //se o melhor se mantiver o mesmo por 10 geracoes, finaliza

        crossover(pctx);
        mutacao(pctx);
    }

    return 0;
}

void populacao(pctx ctx)
{
    if(ctx->inicializado==0) { //inicializacao
        for(int i=0; i<MAX_INDIVIDUOS; i++) {
            ctx->populacao[i].x1 = (rand() % (MAX_FUNCAO-MIN_FUNCAO+1)) + MIN_FUNCAO;
            ctx->populacao[i].x2 = (rand() % (MAX_FUNCAO-MIN_FUNCAO+1)) + MIN_FUNCAO;
        }
        ctx->melhor=-1;
        ctx->inicializado = 1;
    }
    else { //nova populacao
        individuo melhor;
        memcpy(&melhor, &ctx->populacao[ctx->melhor], sizeof(individuo));
        memcpy(ctx->populacao, ctx->nova_populacao, sizeof(ctx->populacao));
        memcpy(&ctx->populacao[ctx->melhor], &melhor, sizeof(individuo));

        memset(ctx->nova_populacao, sizeof(ctx->nova_populacao), 0);
    }
}

void avaliacao(pctx ctx)
{
    float x1;
    float x2;
    float y;

    for(int i=0; i<MAX_INDIVIDUOS; i++) {
        x1 = ctx->populacao[i].x1;
        x2 = ctx->populacao[i].x2;

        y = 0.25*pow(x1,4) - 0.5*pow(x1,2) + 0.1*x1 + 0.5*pow(x2,2); //funcao objetivo

        ctx->populacao[i].fitness=y;
    }
}

void selecao(pctx ctx)
{
    float melhor_fitness = 999999999;

    for(int i=0; i<MAX_INDIVIDUOS; i++) {
        if(ctx->populacao[i].fitness<melhor_fitness) {
            melhor_fitness=ctx->populacao[i].fitness;
            ctx->melhor=i;
        }
    }
}

void crossover(pctx ctx)
{
    float x1 = ctx->populacao[ctx->melhor].x1;
    float x2 = ctx->populacao[ctx->melhor].x2;
    for(int i=0; i<MAX_INDIVIDUOS; i++) {
        if(i!=ctx->melhor) {
            ctx->nova_populacao[i].x1 = (x1 + ctx->populacao[i].x1) / 2; //cruzamento;
            ctx->nova_populacao[i].x2 = (x2 + ctx->populacao[i].x2) / 2; //cruzamento;
        }
        else {
            memcpy(&ctx->nova_populacao[i], &ctx->populacao[i], sizeof(individuo)); //preserva melhor
        }
    }
}

void mutacao(pctx ctx)
{
    float taxa;
    for(int i=0; i<MAX_INDIVIDUOS; i++) {
        if(i!=ctx->melhor) {
            taxa = (0.01 * pow(-1, i));
            ctx->nova_populacao[i].x1 = ctx->nova_populacao[i].x1 + taxa;
            ctx->nova_populacao[i].x1 = ctx->nova_populacao[i].x1 + taxa;
        }
    }
}
