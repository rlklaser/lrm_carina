#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include <string.h>

#ifndef M_PI
 #define M_PI 3.14159265358979323846
#endif
#define M_PHI 1.618033988749895
#ifndef TRUE
 #define TRUE 1
#endif

#define MAX_DIMENSOES   10
#define MAX_INDIVIDUOS  11
#define MAX_ITERACOES   5000
#define MAX_GERACOES    17
#define NUM_FUNCOES     10


#define NUM_CROSSOVER   3
#define NUM_MUTACAO     3
#define NUM_SELECAO     3


#define MIN_DIST        0.055
#define BEST_OF         2

using namespace std;


typedef struct individuo
{
    double x[MAX_DIMENSOES];
    double fitness;
    int pos;
    int avaliado;
} *pindividuo;

typedef struct objetivo {
    int dimensoes;
    int dominio_min;
    int dominio_max;
    void (*funcao)(pindividuo, struct objetivo*);

    double otimo; //apenas utilizado para avaliar o score
    char* sigla;

} *pobjetivo;

typedef struct geracao
{
    struct individuo populacao[MAX_INDIVIDUOS];
    int melhor;
} *pgeracao;

typedef struct contexto
{
    //buffer circular para n geracoes em memoria
    struct geracao geracoes[MAX_GERACOES]; //MAX_GERACOES minimo 2

    int geracao_atual;
    int quantidade_geracoes;
    int geracoes_melhor;
    int avaliacoes;
    int num_individuos;
    int elitismo; //nao usado, sempre ligado

    //objetivo
    pobjetivo objetivo;

    //funcoes do genetico
    int metodo_selecao;
    int metodo_crossover;
    int metodo_mutacao;

    double distribuicao_selecao[NUM_SELECAO];
    double distribuicao_crossover[NUM_CROSSOVER];
    double distribuicao_mutacao[NUM_MUTACAO];

    int  (*selecao[NUM_SELECAO])    (struct contexto*, pgeracao atual, pgeracao nova, pindividuo melhor);
    void (*crossover[NUM_CROSSOVER])(struct contexto*, pindividuo individuo, pindividuo pai1, pindividuo pai2);
    void (*mutacao[NUM_MUTACAO])    (struct contexto*, pindividuo individuo);

} *pctx;


static void ackley(pindividuo i, pobjetivo obj)
{
    double y, sum1=0, sum2=0;
    for(int n=0;n<obj->dimensoes; n++) {
        sum1+=pow(i->x[n], 2);
        sum2+=cos(2*M_PI*i->x[n]);
    }
    y = -20*exp(-0.02*sqrt((1.0/obj->dimensoes)*sum1)) -
        exp((1.0/obj->dimensoes)*sum2) +
        20  +
        M_E;
    i->fitness = y;
}

static void cosine_mixture(pindividuo i, pobjetivo obj)
{
    double y, x1, x2;
    x1 = i->x[0];
    x2 = i->x[1];
    y = 0.1*(cos(5*M_PI*x1)+cos(5*M_PI*x2)) - (pow(x1, 2)+pow(x2, 2));
    i->fitness = y;
}


static void aluffi_pentini(pindividuo i, pobjetivo obj)
{
    double y, x1, x2;
    x1 = i->x[0];
    x2 = i->x[1];
    y = 0.25*pow(x1, 4) - 0.5*pow(x1, 2) + 0.1*x1 + 0.5*pow(x2, 2);
    i->fitness = y;
}

static void camel_back3(pindividuo i, pobjetivo obj)
{
    double y, x1, x2;
    x1 = i->x[0];
    x2 = i->x[1];
    y = 2*pow(x1, 2) - 1.05*pow(x1, 4) + (1.0/6)*pow(x1, 6) + x1*x2 + pow(x2, 2);
    i->fitness = y;
}

static void camel_back6(pindividuo i, pobjetivo obj)
{
    double y, x1, x2;
    x1 = i->x[0];
    x2 = i->x[1];
    y = 4*pow(x1, 2) - 2.1*pow(x1, 4) + (1.0/3)*pow(x1, 6) + x1*x2  - 4*pow(x2, 2) + 4*pow(x2, 4);
    i->fitness = y;
}

//dominio [-1,2]
static void goldstein_price(pindividuo i, pobjetivo obj)
{
    double y, x1, x2;
    x1 = i->x[0];
    x2 = i->x[1];
    y = (1 + pow(x1 + x2 + 1, 2) *
        (19 - 14*x1 + 3*pow(x1, 2) - 14*x2 + 6*x1*x2 + 3*pow(x2, 2))) *
        (30 + pow(2*x1 - 3*x2, 2) *
        (18 - 32*x1 + 12*pow(x1, 2) + 48*x2 - 36 * x1*x2 + 27*pow(x2, 2)));
    i->fitness = y;
}

//dominio -50 50
static void bohachevsky1(pindividuo i, pobjetivo obj)
{
    double y, x1, x2;
    x1 = i->x[0];
    x2 = i->x[1];
    y = pow(x1, 2)+
        2*pow(x2, 2) -
        0.3*cos(3*M_PI*x1) -
        0.4*cos(4*M_PI*x2) +
        0.7;
    i->fitness = y;
}

static void bohachevsky2(pindividuo i, pobjetivo obj)
{
    double y, x1, x2;
    x1 = i->x[0];
    x2 = i->x[1];
    y = pow(x1, 2)+
        2*pow(x2, 2) -
        0.3*cos(3*M_PI*x1)*cos(4*M_PI*x2) +
        0.3;
    i->fitness = y;
}

//dominio -30 30
static void rosenbrock(pindividuo i, pobjetivo obj)
{
    double y, x1, x2;
    x1 = i->x[0];
    x2 = i->x[1];
    y = pow(1-x1, 2) +
        100*pow(x2 - pow(x1, 2), 2);
    i->fitness = y;
}

void inicializa(pctx ctx, pindividuo individuo)
{
    double range = ctx->objetivo->dominio_max-ctx->objetivo->dominio_min;
    double min = ctx->objetivo->dominio_min;
    double decimais = 100.0;
    int rnd;
    range = (range * decimais)+1;
    min = min * decimais;
    for(int d=0; d<ctx->objetivo->dimensoes; d++) {
        rnd = rand() % (int)range;
        individuo->x[d] = (rnd+min) / decimais;
    }
    individuo->avaliado = 0;
}

void inicializa(pctx ctx, int geracao)
{
    pindividuo individuo;
    for(int i=0; i<ctx->num_individuos; i++) {
        individuo = &ctx->geracoes[geracao].populacao[i];
        individuo->pos = i; //posicao do individuo no vetor
        inicializa(ctx, individuo);
    }
    ctx->geracoes[geracao].melhor=-1;
}

void crossover_media_aritmetica(pctx ctx, pindividuo individuo, pindividuo pai1, pindividuo pai2)
{
    for(int i=0; i<ctx->objetivo->dimensoes; i++)
    {
        individuo->x[i] = (pai1->x[i] + pai2->x[i])/2.0;
    }
    individuo->avaliado = 0; //alterou individuo
}

void crossover_media_geometrica(pctx ctx, pindividuo individuo, pindividuo pai1, pindividuo pai2)
{
    double val;
    int sgn;
    for(int i=0; i<ctx->objetivo->dimensoes; i++)
    {
        val = (pai1->x[i] * pai2->x[i]);
        sgn = val<0?-1:1;
        individuo->x[i] = sqrt(fabs(val)) * sgn;
    }
    individuo->avaliado = 0; //alterou individuo
}

void crossover_blx_alfa(pctx ctx, pindividuo individuo, pindividuo pai1, pindividuo pai2)
{
    //double beta = rand() % (2*PHI+1) - PHI;
    double beta = M_PHI;
    for(int i=0; i<ctx->objetivo->dimensoes; i++)
    {
        individuo->x[i] = pai1->x[i] + beta*(pai2->x[i]-pai1->x[i]);
    }
    individuo->avaliado = 0; //alterou individuo
}

void crossover_linear(pctx ctx, pindividuo individuo, pindividuo pai1, pindividuo pai2)
{
    double ratio_a1;
    double ratio_a2;
    for(int i=0; i<ctx->objetivo->dimensoes; i++)
    {
        ratio_a1 = ((rand() % 3001)-1500)/100.0; //de -1,5 a 1,5
        ratio_a2 = 1 - ratio_a1; //ratio_a1 + ratio_a2 = 1
        individuo->x[i] = ratio_a1*pai1->x[i] + ratio_a2*pai2->x[i];
    }
    individuo->avaliado = 0; //alterou individuo
}

void crossover_aritmetico(pctx ctx, pindividuo individuo, pindividuo pai1, pindividuo pai2)
{
    double beta;
    for(int i=0; i<ctx->objetivo->dimensoes; i++)
    {
        beta = (rand() % 1001)/1000.0; //de 0 a 1
        individuo->x[i] = beta*pai1->x[i] + (1-beta)*pai2->x[i];
    }
    individuo->avaliado = 0; //alterou individuo
}

void crossover_heuristico(pctx ctx, pindividuo individuo, pindividuo pai1, pindividuo pai2)
{
    double r;
    pindividuo p1, p2;

    int maior = (pai1->fitness > pai2->fitness);
    p1 = maior ? pai1 : pai2;
    p2 = maior ? pai2 : pai1;

    for(int i=0; i<ctx->objetivo->dimensoes; i++)
    {
        r = (rand() % 1001)/1000.0; //de 0 a 1
        individuo->x[i] = p1->x[i] + r*(p1->x[i]-p2->x[i]);
    }
    individuo->avaliado = 0; //alterou individuo
}

void mutacao_flutuacao(pctx ctx, pindividuo individuo)
{
    double taxa;
    int pot;

    for(int i=0; i<ctx->objetivo->dimensoes; i++) {
        pot = (rand() % 2) + 1; //1 e 2
        taxa = (0.0005 * pow(-1, pot));
        individuo->x[i] = individuo->x[i] + taxa;
    }
    individuo->avaliado = 0; //alterou individuo
}

void mutacao_percentual(pctx ctx, pindividuo individuo)
{
    int coin;

    for(int i=0; i<ctx->objetivo->dimensoes; i++) {
        coin = (rand() % 2); //0 ou 1
        individuo->x[i] = individuo->x[i] * (coin ? 0.95 : 1.05);
    }
    individuo->avaliado = 0; //alterou individuo
}

void mutacao_agressiva(pctx ctx, pindividuo individuo)
{
    double taxa;
    int pot;

    for(int i=0; i<ctx->objetivo->dimensoes; i++) {
        pot = (rand() % 2) + 1; //1 e 2
        taxa = (0.05 * pow(-1, pot));
        individuo->x[i] = individuo->x[i] + taxa;
    }
    individuo->avaliado = 0; //alterou individuo
}


void avalia(pctx ctx, pindividuo individuo)
{
    if(!individuo->avaliado) { //evita avaliacoes duplicadas
        ctx->avaliacoes++;
        ctx->objetivo->funcao(individuo, ctx->objetivo);
        individuo->avaliado = 1;
    }
}

//faz a selecao dos metodos de forma randomica
void metodos_randomico(pctx ctx)
{
    ctx->metodo_selecao = rand() % NUM_SELECAO;
    ctx->metodo_crossover = rand() % NUM_CROSSOVER;
    ctx->metodo_mutacao = rand() % NUM_MUTACAO;
}

void metodos_roleta(pctx ctx, int melhorou)
{
    //penaliza ou premia os metodos de acordo se melhorou
    double pct;
    //double total;
    double sum;
    double razao;

    int metodo_da_vez = ctx->quantidade_geracoes % 3;


    switch(metodo_da_vez) {

    case 0:

    //selecao
    sum = 0;
    if(!melhorou) {
        razao = 1.0/NUM_SELECAO;
        pct=ctx->distribuicao_selecao[ctx->metodo_selecao];
        ctx->distribuicao_selecao[ctx->metodo_selecao] = pct - (pct*razao);
        for(int i=0; i<NUM_SELECAO; i++) {
            if(i!=ctx->metodo_selecao) {
                sum+=ctx->distribuicao_selecao[i]+=(pct*razao)/(NUM_SELECAO-1);
            }
        }
    } else {
        for(int i=0; i<NUM_SELECAO; i++) {
            if(i!=ctx->metodo_selecao) {
                pct = ctx->distribuicao_selecao[i];
                razao = pct * (1.0/(NUM_SELECAO-1));
                sum+=razao;
                ctx->distribuicao_selecao[i]-=razao;
            }
        }
        ctx->distribuicao_selecao[ctx->metodo_selecao]+=sum;
    }

    break;
    case 1:

    //crossover
    sum = 0;
    if(!melhorou) {
        razao = 1.0/NUM_CROSSOVER;
        pct=ctx->distribuicao_crossover[ctx->metodo_crossover];
        ctx->distribuicao_crossover[ctx->metodo_crossover] = pct - (pct*razao);
        for(int i=0; i<NUM_CROSSOVER; i++) {
            if(i!=ctx->metodo_crossover) {
                sum+=ctx->distribuicao_crossover[i]+=(pct*razao)/(NUM_CROSSOVER-1);
            }
        }
    } else {
        for(int i=0; i<NUM_CROSSOVER; i++) {
            if(i!=ctx->metodo_crossover) {
                pct = ctx->distribuicao_crossover[i];
                razao = pct * (1.0/(NUM_CROSSOVER-1));
                sum+=razao;
                ctx->distribuicao_crossover[i]-=razao;
            }
        }
        ctx->distribuicao_crossover[ctx->metodo_crossover]+=sum;
    }

    break;
    case 2:

    //mutacao
    sum = 0;
    if(!melhorou) {
        razao = 1.0/NUM_MUTACAO;
        pct=ctx->distribuicao_mutacao[ctx->metodo_mutacao];
        ctx->distribuicao_mutacao[ctx->metodo_mutacao] = pct - (pct*razao);
        for(int i=0; i<NUM_MUTACAO; i++) {
            if(i!=ctx->metodo_mutacao) {
                sum+=ctx->distribuicao_mutacao[i]+=(pct*razao)/(NUM_MUTACAO-1);
            }
        }
    } else {
        for(int i=0; i<NUM_MUTACAO; i++) {
            if(i!=ctx->metodo_mutacao) {
                pct = ctx->distribuicao_mutacao[i];
                razao = pct * (1.0/(NUM_MUTACAO-1));
                sum+=razao;
                ctx->distribuicao_mutacao[i]-=razao;
            }
        }
        ctx->distribuicao_mutacao[ctx->metodo_mutacao]+=sum;
    }

    }

    double roleta;
    int metodo;

    //selecao
    roleta = (rand() % 101)/100.0;
    sum = 0;
    for(metodo=0; metodo<NUM_SELECAO; metodo++)
    {
        sum+=ctx->distribuicao_selecao[metodo];
        if(sum>=roleta) {
            ctx->metodo_selecao = metodo;
            break;
        }
    }

    //crossover
    roleta = (rand() % 101)/100.0;
    sum = 0;
    for(metodo=0; metodo<NUM_CROSSOVER; metodo++)
    {
        sum+=ctx->distribuicao_crossover[metodo];
        if(sum>=roleta) {
            ctx->metodo_crossover = metodo;
            break;
        }
    }

    //mutacao
    roleta = (rand() % 101)/100.0;
    sum = 0;
    for(metodo=0; metodo<NUM_MUTACAO; metodo++)
    {
        sum+=ctx->distribuicao_mutacao[metodo];
        if(sum>=roleta) {
            ctx->metodo_mutacao = metodo;
            break;
        }
    }

}

void predacao(pctx ctx)
{
}

int avaliacao(pctx ctx)
{
    int res = 0;
    pgeracao atual = &ctx->geracoes[ctx->geracao_atual];
    pindividuo individuo;
    int melhor = atual->melhor;
    int melhorou = 0;

    double fitness = 0; //vai ser ajustado corretamente na primeira avaliacao
    if(melhor!=-1) {
        fitness = atual->populacao[melhor].fitness;
    }
    else {
        ctx->geracoes_melhor=0;
    }

    for(int i=0; i<ctx->num_individuos; i++) {
        if(i!=melhor) {
            individuo = &atual->populacao[i];

            //funcao de avaliacao
            avalia(ctx, individuo);

            if(melhor==-1 || individuo->fitness<fitness) { //assume sempre minimizacao
                fitness = individuo->fitness;
                melhor = i;
                melhorou = TRUE;
            }
        }
    }

    atual->melhor = melhor;

    //nova geracao
    pindividuo melhor_individuo = &atual->populacao[melhor];
    ctx->quantidade_geracoes++;
    ctx->geracao_atual = ctx->quantidade_geracoes % MAX_GERACOES;
    pgeracao nova = &ctx->geracoes[ctx->geracao_atual];

    //preserva melhor na nova geracao
    nova->melhor = melhor;
    memcpy(&nova->populacao[melhor], melhor_individuo, sizeof(struct individuo));


    if(!melhorou) {
        ctx->geracoes_melhor++;
        //penaliza metodos


        double dist;
        double sum;
        int bestof = 0;

        sum = 0;
        for(int i=0; i<NUM_SELECAO-1; i++) {
            dist = sqrt(pow(ctx->distribuicao_selecao[i]-ctx->distribuicao_selecao[i+1], 2));
            sum+=dist;
        }

        if(sum<MIN_DIST) {
            bestof++;
        }

        sum = 0;
        for(int i=0; i<NUM_CROSSOVER-1; i++) {
            dist = sqrt(pow(ctx->distribuicao_crossover[i]-ctx->distribuicao_crossover[i+1], 2));
            sum+=dist;
        }

        if(sum<MIN_DIST) {
            bestof++;
        }

        sum = 0;
        for(int i=0; i<NUM_MUTACAO-1; i++) {
            dist = sqrt(pow(ctx->distribuicao_mutacao[i]-ctx->distribuicao_mutacao[i+1], 2));
            sum+=dist;
        }

        if(sum<MIN_DIST) {
            bestof++;
        }

        if(bestof>(BEST_OF-1) && ctx->geracoes_melhor>MAX_GERACOES) return 1;

    }
    else {
        ctx->geracoes_melhor = 1;
        //premia metodos
    }

    metodos_roleta(ctx, melhorou);
    //ctx->metodo_crossover = 0;
    //ctx->metodo_mutacao = 0;

    //selecao para nova geracao
    res = ctx->selecao[ctx->metodo_selecao](ctx, atual, nova, melhor_individuo);

    return res;
}

int selecao_melhorcomtodos(pctx ctx, pgeracao atual, pgeracao nova, pindividuo melhor)
{
    int res = 0;
    pindividuo individuo;
    pindividuo novo;

    for(int i=0; i<ctx->num_individuos; i++) {
        if(i!=melhor->pos) { //nao aplica na posicao do melhor
            individuo = &atual->populacao[i];
            novo = &nova->populacao[i];

            ctx->crossover[ctx->metodo_crossover](ctx, novo, melhor, individuo);
            ctx->mutacao[ctx->metodo_mutacao](ctx, novo);
        }
    }

    return res;
}

int selecao_torneio(pctx ctx, pgeracao atual, pgeracao nova, pindividuo melhor)
{
    int res = 0;
    int match_a;
    int match_b;
    int match_winner[2];
    int j;
    pindividuo novo;
    pindividuo pai1;
    pindividuo pai2;

    //faz ctx->num_individuos-1 torneios
    for(int i=0; i<ctx->num_individuos; i++) {
        if(i!=atual->melhor) { //nao aplica na posicao do melhor

            for(j=0;j<2;j++) {
                match_a = rand() % ctx->num_individuos;
                match_b = rand() % ctx->num_individuos;
                match_winner[j] = atual->populacao[match_a].fitness > atual->populacao[match_b].fitness ? match_a : match_b;
            }

            //novo individuo
            novo = &nova->populacao[i];
            pai1 = &atual->populacao[match_winner[0]]; //vencedor torneio 1
            pai2 = &atual->populacao[match_winner[1]]; //vencedor torneio 2

            ctx->crossover[ctx->metodo_crossover](ctx, novo, pai1, pai2);
            ctx->mutacao[ctx->metodo_mutacao](ctx, novo);
        }
    }

    return res;
}

int selecao_truncamento(pctx ctx, pgeracao atual, pgeracao nova, pindividuo melhor)
{
    //gera novos individuos e ordena o N melhores
    int novos = ctx->num_individuos * 1.0/2.0;
    struct geracao geracao;
    struct individuo individuo;

    memcpy(&geracao, atual, sizeof(struct geracao));
    //ordena os individuos
    for(int i=0; i<ctx->num_individuos-1; i++)
    {
        for(int j=i+1; j<ctx->num_individuos; j++)
        {
            if(geracao.populacao[j].fitness<geracao.populacao[i].fitness) {
                memcpy(&individuo, &geracao.populacao[i], sizeof(struct individuo));
                memcpy(&geracao.populacao[i], &geracao.populacao[j], sizeof(struct individuo));
                memcpy(&geracao.populacao[j], &individuo, sizeof(struct individuo));
            }
        }
    }

    pindividuo novo;
    for(int i=0; i<ctx->num_individuos-novos; i++)
    {
        if(nova->melhor!=i) { //pula a posicao do melhor
            novo = &geracao.populacao[i+1 /*pula o 0 que eh o melhor*/];
            novo->pos = i;//acerta a posicao
            memcpy(&nova->populacao[i], novo, sizeof(struct individuo));
        }
    }
    for(int i=ctx->num_individuos-novos; i<ctx->num_individuos; i++)
    {
        if(nova->melhor!=i) { //pula a posicao do melhor
            novo = &nova->populacao[i];
            inicializa(ctx, novo); //gera randomico
            avalia(ctx, novo); //avalia
        }
    }

    return 0;
}

int selecao_roleta(pctx ctx, pgeracao atual, pgeracao nova, pindividuo melhor)
{
    return 0;
}


//inicializa o contexto
void inicializa(pctx ctx)
{
    ctx->num_individuos = MAX_INDIVIDUOS;
    ctx->geracao_atual = 0;
    ctx->quantidade_geracoes = 0;
    ctx->geracoes_melhor = 0;
    ctx->avaliacoes = 0;

    //inicializa os individuos
    //srand(time(NULL));
    for(int g=0; g<MAX_GERACOES; g++) {
        inicializa(ctx, g);
    }

    //preenche as distribuicoes de forma uniforme
    for(int i=0; i<NUM_SELECAO; i++) {
        ctx->distribuicao_selecao[i] = 1.0/NUM_SELECAO;
    }
    for(int i=0; i<NUM_CROSSOVER; i++) {
        ctx->distribuicao_crossover[i] = 1.0/NUM_CROSSOVER;
    }
    for(int i=0; i<NUM_MUTACAO; i++) {
        ctx->distribuicao_mutacao[i] = 1.0/NUM_MUTACAO;
    }

    //inicializa inicialmente de forma randomica os metodos
    metodos_randomico(ctx);
}

void print_traco(pctx ctx)
{
    printf("%4d,%2d,%4d,%2d,%.4f,%6d,",
       ctx->quantidade_geracoes,
       ctx->geracao_atual,
       ctx->geracoes_melhor,
       ctx->geracoes[ctx->geracao_atual].melhor,
       ctx->geracoes[ctx->geracao_atual].populacao[ctx->geracoes[ctx->geracao_atual].melhor].fitness,
       ctx->avaliacoes
       );
}

void print_metodos(pctx ctx)
{
    printf("(%d,%d,%d)",
       ctx->metodo_selecao,
       ctx->metodo_crossover,
       ctx->metodo_mutacao
       );
}

void print_distribuicoes(pctx ctx)
{
    printf("[");
    for(int i=0; i<NUM_SELECAO; i++) {
        if(i!=0) printf(",");
        printf("%1.2f", ctx->distribuicao_selecao[i]);
    }
    printf("][");
    for(int i=0; i<NUM_CROSSOVER; i++) {
        if(i!=0) printf(",");
        printf("%1.2f", ctx->distribuicao_crossover[i]);
    }
    printf("][");
    for(int i=0; i<NUM_MUTACAO; i++) {
        if(i!=0) printf(",");
        printf("%1.2f", ctx->distribuicao_mutacao[i]);
    }
    printf("]");
}

void genetico(pctx ctx, int silent)
{
    int res;

    inicializa(ctx);

    if(!silent) {
        print_metodos(ctx);
        print_distribuicoes(ctx);
        printf("\n");
    }

    for(int t=0; t<MAX_ITERACOES; t++) {
        res = avaliacao(ctx);

        if(!silent) {
            print_traco(ctx);
            print_metodos(ctx);
            print_distribuicoes(ctx);
            printf("\n");
        }

        if(res!=0) break;
    }
}

int main()
{
    srand(time(NULL));

    pctx ctx = new(struct contexto);
    memset(ctx, 0, sizeof(struct contexto));
    pobjetivo pobj[NUM_FUNCOES];
    int f = 0;

    //goldenstein price
    pobj[f] = new(struct objetivo);
    pobj[f]->dimensoes=2;
    pobj[f]->dominio_min=-2;
    pobj[f]->dominio_max=2;
    pobj[f]->otimo = 3.0;
    pobj[f]->sigla = "GP ";
    pobj[f]->funcao = &goldstein_price;
    f++;

    //rosenbrock
    pobj[f] = new(struct objetivo);
    pobj[f]->dimensoes=2;
    pobj[f]->dominio_min=-30;
    pobj[f]->dominio_max=30;
    pobj[f]->otimo = 0.0;
    pobj[f]->sigla = "R2 ";
    pobj[f]->funcao = &rosenbrock;
    f++;

    //ackeley
    pobj[f] = new(struct objetivo);
    pobj[f]->dimensoes=10;
    pobj[f]->dominio_min=-30;
    pobj[f]->dominio_max=30;
    pobj[f]->otimo = 0;
    pobj[f]->sigla = "ACK";
    pobj[f]->funcao = &ackley;
    f++;

    //aluffi pentini
    pobj[f] = new(struct objetivo);
    pobj[f]->dimensoes=2;
    pobj[f]->dominio_min=-10;
    pobj[f]->dominio_max=10;
    pobj[f]->otimo = -0.3523;
    pobj[f]->sigla = "AP ";
    pobj[f]->funcao = &aluffi_pentini;
    f++;

    //bohachevsky 1
    pobj[f] = new(struct objetivo);
    pobj[f]->dimensoes=2;
    pobj[f]->dominio_min=-50;
    pobj[f]->dominio_max=50;
    pobj[f]->otimo = 0.0;
    pobj[f]->sigla = "BF1";
    pobj[f]->funcao = &bohachevsky1;
    f++;

    //bohachevsky 2
    pobj[f] = new(struct objetivo);
    pobj[f]->dimensoes=2;
    pobj[f]->dominio_min=-50;
    pobj[f]->dominio_max=50;
    pobj[f]->otimo = 0.0;
    pobj[f]->sigla = "BF2";
    pobj[f]->funcao = &bohachevsky2;
    f++;

    //gw
    //sin

    //camel back 3
    pobj[f] = new(struct objetivo);
    pobj[f]->dimensoes=2;
    pobj[f]->dominio_min=-5;
    pobj[f]->dominio_max=5;
    pobj[f]->otimo = 0;
    pobj[f]->sigla = "CB3";
    pobj[f]->funcao = &camel_back3;
    f++;

    //camel back 6
    pobj[f] = new(struct objetivo);
    pobj[f]->dimensoes=2;
    pobj[f]->dominio_min=-5;
    pobj[f]->dominio_max=5;
    pobj[f]->otimo = -1.0316;
    pobj[f]->sigla = "CB6";
    pobj[f]->funcao = &camel_back6;
    f++;

    //cosine mixture
    pobj[f] = new(struct objetivo);
    pobj[f]->dimensoes=2;
    pobj[f]->dominio_min=-1;
    pobj[f]->dominio_max=1;
    pobj[f]->otimo = 0.2;
    pobj[f]->sigla = "CM ";
    pobj[f]->funcao = &cosine_mixture;
    f++;

    //ctx->objetivo=pobj[2];


    //ctx->avaliacao = &avaliacao;
    ctx->selecao[0] = &selecao_melhorcomtodos;
    ctx->selecao[1] = &selecao_torneio;
    ctx->selecao[2] = &selecao_truncamento;

    ctx->crossover[0] = &crossover_aritmetico;
    ctx->crossover[1] = &crossover_media_aritmetica;
    ctx->crossover[2] = &crossover_media_geometrica;
    //ctx->crossover[1] = &crossover_blx_alfa;
    //ctx->crossover[3] = &crossover_heuristico;
    //ctx->crossover[4] = &crossover_linear;

    ctx->mutacao[2] = &mutacao_flutuacao;
    ctx->mutacao[1] = &mutacao_agressiva;
    ctx->mutacao[0] = &mutacao_percentual;

    double diff;
    int avaliacoes;
    int corretos;
    int sum_g;

    //ctx->objetivo=pobj[2];
    //genetico(ctx, 0);

    for(int j=0; j<f; j++) {
        ctx->objetivo=pobj[j];
        avaliacoes = 0;
        corretos = 0;
        sum_g = 0;
        for(int i=0; i<100; i++) {
            genetico(ctx, TRUE);

            diff = fabs(ctx->geracoes[ctx->geracao_atual].populacao[ctx->geracoes[ctx->geracao_atual].melhor].fitness - ctx->objetivo->otimo);
            if(diff<0.01) {
                avaliacoes+=ctx->avaliacoes;
                sum_g+=ctx->quantidade_geracoes;
                corretos++;
            }

        }
        printf("%s, %5d, %5d, %3d\n",
               ctx->objetivo->sigla,
               corretos==0? 0 : sum_g/corretos,
               corretos==0? 0 : avaliacoes/corretos,
               corretos
               );
    }

    for(int i=0; i<f; i++) delete(pobj[i]);
    delete(ctx);

    return 0;
}
