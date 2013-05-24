#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <math.h>

#define MAX_INDIVIDUOS  10
#define MAX_ITERACOES   10000
#define MAX_GERACOES  100
#define MIN_FUNCAO  -100
#define MAX_FUNCAO  100

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
void genocidio(pctx ctx);

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


        //printf("%2.3f\t%2.3f\t%2.3f\n", ctx.populacao[melhor].x1, ctx.populacao[melhor].x2, ctx.populacao[melhor].fitness);
        printf("%d\t%8.4f\n", geracoes, ctx.populacao[ctx.melhor].fitness);


        //if(melhor_geracoes==MAX_GERACOES) break; //se o melhor se mantiver o mesmo por 10 geracoes, finaliza
        if(melhor_geracoes==MAX_GERACOES) {
            genocidio(pctx);
        }

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
        memcpy(&ctx->populacao[ctx->melhor], &melhor, sizeof(individuo)); //o melhor da anterior se mantem na nova geracao

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

/*
 * busca o melhor
 */
void selecao(pctx ctx)
{
    float melhor_fitness = ctx->populacao[0].fitness;
    ctx->melhor=0;

    for(int i=1; i<MAX_INDIVIDUOS; i++) {
        if(ctx->populacao[i].fitness<melhor_fitness) {
            melhor_fitness=ctx->populacao[i].fitness;
            ctx->melhor=i;
        }
    }
}

/*
 * combina o melhor com todos
 */
void elitismo(pctx ctx)
{
    float x1 = ctx->populacao[ctx->melhor].x1;
    float x2 = ctx->populacao[ctx->melhor].x2;
    for(int i=0; i<MAX_INDIVIDUOS; i++) {
        if(i!=ctx->melhor) {
            ctx->nova_populacao[i].x1 = (x1 + ctx->populacao[i].x1) / 2.0; //cruzamento;
            ctx->nova_populacao[i].x2 = (x2 + ctx->populacao[i].x2) / 2.0; //cruzamento;
        }
        else {
            memcpy(&ctx->nova_populacao[i], &ctx->populacao[i], sizeof(individuo)); //preserva melhor
        }
    }
}

/*
 * faz sorteio 2 a 2 e escolhe o melhor de cada par
 */
void torneio(pctx ctx)
{
    int match_a;
    int match_b;
    int match_winner[2];
    int j;

    //faz MAX_INDIVIDUOS-1 torneios
    for(int i=0; i<MAX_INDIVIDUOS; i++) {
        if(i!=ctx->melhor) { //preserva a posicao do melhor

            for(j=0;j<2;j++) {
                match_a = (rand() %MAX_INDIVIDUOS) + 1;
                match_b = (rand() %MAX_INDIVIDUOS) + 1;
                match_winner[j] = ctx->populacao[match_a].fitness>ctx->populacao[match_b].fitness ? match_a : match_b;
            }
            //novo individuo
            ctx->nova_populacao[i].x1=(ctx->populacao[match_winner[0]].x1+ctx->populacao[match_winner[1]].x1)/2.0;
            ctx->nova_populacao[i].x2=(ctx->populacao[match_winner[0]].x2+ctx->populacao[match_winner[1]].x2)/2.0;
        }
    }
}

void crossover(pctx ctx)
{
    //elitismo(ctx);
    torneio(ctx);
}

void mutacao(pctx ctx)
{
    float taxa;
    for(int i=0; i<MAX_INDIVIDUOS; i++) {
        if(i!=ctx->melhor) {
            taxa = (0.005 * pow(-1, i));
            ctx->nova_populacao[i].x1 = ctx->nova_populacao[i].x1 + taxa;
            ctx->nova_populacao[i].x1 = ctx->nova_populacao[i].x1 + taxa;
        }
    }
}

void genocidio(pctx ctx)
{
    individuo melhor;
    int i;
    i=ctx->melhor;
    memcpy(&melhor, &ctx->populacao[ctx->melhor], sizeof(individuo)); //preserva melhor
    memset(ctx, sizeof(ctx), 0); //reseta o contexto
    populacao(ctx);
    ctx->melhor=i;
    memcpy(&ctx->populacao[ctx->melhor], &melhor, sizeof(individuo)); //o melhor da anterior se mantem na nova geracao
}
