#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <math.h>

#define MAX_INDIVIDUOS  10
#define MAX_ITERACOES   10000
#define MAX_GERACOES  10
#define MIN_FUNCAO  0
#define MAX_FUNCAO  20

typedef struct individuo
{
    float valor;
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


        //printf("%d\t%d\t%2.3f\t%2.3f\n", geracoes, melhor, ctx.populacao[melhor].valor, ctx.populacao[melhor].fitness);
        printf("%2.3f\t%2.3f\n", ctx.populacao[melhor].valor, ctx.populacao[melhor].fitness);

        /*
        for(int i=0; i<MAX_INDIVIDUOS; i++) {
            printf("%2.3f ", ctx.populacao[i].fitness);
        }
        printf("\n");
        */

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
            ctx->populacao[i].valor = (rand() % (MAX_FUNCAO-MIN_FUNCAO+1)) + MIN_FUNCAO;
        }
        ctx->melhor=-1;
        ctx->inicializado = 1;

        mutacao(ctx); //adiciona um ruido inicial;
        for(int i=0; i<MAX_INDIVIDUOS; i++) {
            ctx->populacao[i].valor+=ctx->nova_populacao[i].valor;
        }

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
    for(int i=0; i<MAX_INDIVIDUOS; i++) {
        float val = ctx->populacao[i].valor;
        ctx->populacao[i].fitness =
            val<10 ? val : 20 - val; //funcao objetivo
    }
}

void selecao(pctx ctx)
{
    float melhor_fitness = -1;
    for(int i=0; i<MAX_INDIVIDUOS; i++) {
        if(ctx->populacao[i].fitness>melhor_fitness) {
            melhor_fitness=ctx->populacao[i].fitness;
            ctx->melhor=i;
        }
    }
}

void crossover(pctx ctx)
{
    float melhor = ctx->populacao[ctx->melhor].valor;
    for(int i=0; i<MAX_INDIVIDUOS; i++) {
        if(i!=ctx->melhor) {
            ctx->nova_populacao[i].valor = (melhor + ctx->populacao[i].valor) / 2; //cruzamento;
        }
        else {
            memcpy(&ctx->nova_populacao[i], &ctx->populacao[i], sizeof(individuo)); //preserva melhor
        }
    }
}

void mutacao(pctx ctx)
{
    for(int i=0; i<MAX_INDIVIDUOS; i++) {
        if(i!=ctx->melhor) {
                ctx->nova_populacao[i].valor = ctx->nova_populacao[i].valor + (0.01 * pow(-1, i));
        }
    }
}
