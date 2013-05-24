#include <stdio.h>
#include <stdlib.h>
#include <strings.h>


#define MAX_CARDS 52

int win;

typedef struct stack
{
   int values[MAX_CARDS];
   int current;
} *pstack;

inline pstack stack_new()
{
    pstack stack = malloc(sizeof(struct stack));
    memset(stack, 0, sizeof(struct stack));
    stack->current=MAX_CARDS;
    return stack;
}

inline void stack_release(pstack stack)
{
    free(stack);
}

inline int stack_isempty(pstack stack) {
    return (stack->current==MAX_CARDS);
}

inline int stack_isfull(pstack stack) {
    return (stack->current==0);
}

void stack_pop(pstack stack, int value)
{
    if(stack_isfull(stack)) return; //error
    stack->current--;
    stack->values[stack->current]=value;
}

int stack_push(pstack stack)
{
    if(stack_isempty(stack)) return -1; //error
    stack->current++;
    return stack->values[stack->current-1];
}

void pick_table(pstack table, pstack hand)
{
    if(stack_isempty(table)) return;
    pstack temp = stack_new();
    while(!stack_isempty(hand))
    {
        stack_pop(temp, stack_push(hand));
    }
    while(!stack_isempty(table))
    {
        stack_pop(hand, stack_push(table));
    }
    while(!stack_isempty(temp))
    {
        stack_pop(hand, stack_push(temp));
    }
    stack_release(temp);
}


int play(pstack table, pstack hands[])
{
    int gameover = 0;
    int player = 0;
    int pay_count = 1;
    int faced_up = 0;
    int paying_player = -1;
    while(!gameover)
    {
        if(stack_isempty(hands[player])) {
            gameover = 1;
        }
        else {
            pay_count--;
            faced_up = stack_push(hands[player]); //vira do jogador
            stack_pop(table, faced_up); //abre na mesa
            if(faced_up>0) {;
                player = (player+1)%2; //passa jogada
                pay_count = faced_up + 1; //cartas a pagar + jogada
                paying_player = player;//jogador pagante
            }
            else {
                if(pay_count==0) {
                    player = (player+1)%2;
                    pay_count = 1;
                    if(paying_player!=-1) {
                        //o que fez pagar (proximo) pega a mesa
                        pick_table(table, hands[player]);
                        paying_player=-1;
                    }
                }
            }
        }
    }
    return (player+1)%2;
}

int play_until_face(int player, int count, pstack table, pstack hands[])
{
    int i=0, faced_up=-1, looser=-1;
    while(looser==-1 && i!=count) { //enquanto nao houver perdedor
        for(i=0; i<count; i++) {
            if(stack_isempty(hands[player])) {
                looser = player;
                break;
            }
            faced_up = stack_push(hands[player]); //vira do jogador
            stack_pop(table, faced_up); //abre na mesa

            //carta a ser paga
            if(faced_up>0) {
                count = 1;//se esiver pagando, reseta
                looser = play_until_face((player+1)%2, faced_up, table, hands);
                if(stack_isempty(table)) {
                    //desempilhando
                    count=0;
                }
                else {
                    //o ultimo pega a mesa
                    pick_table(table, hands[player]);
                    player++;//o proximo é ele mesmo
                }
            }
        }

        if(count==1 && looser==-1) { //proximo jogador
            looser = play_until_face((player+1)%2, 1, table, hands);
        }
    }
    return looser;
}

int play_game(pstack table, pstack hands[])
{
        int turn = 0, needed = 1, last = 0, faced_up;
        int flag = 1;
        while(flag) {
            while(needed--) {
                if(stack_isempty(hands[turn])) {
                    printf("%d%3d\n", (turn ? 2 : 1), MAX_CARDS-hands[(turn+1)%2]->current-1);
                    flag= 0;
                    break;
                }
                faced_up = stack_push(hands[turn]); //vira do jogador
                stack_pop(table, faced_up); //abre na mesa
                int tn = needed;
                switch(faced_up) {
                    case 4:
                        ++needed;
                    case 3:
                        ++needed;
                    case 2:
                        ++needed;
                    case 1:
                        ++needed;
                        needed -= tn;
                        last= turn+1;
                        turn = (turn+1)%2;
                }
            }
            if(flag) {
                if(last--) {
                    pick_table(table, hands[last]);
                    turn = (last+1)%2;
                }
                last = 0;
                needed = 1;
                turn = (turn+1)%2;
            }
        }
        return 0;
}


int main()
{
    for(;;) {
        pstack hands[2];
        pstack deck = stack_new();
        //pstack hand_dealer = stack_new();
        //pstack hand_first = stack_new();
        pstack table = stack_new();

        hands[0] = stack_new();
        hands[1] = stack_new();

        //pstack dbg_hand_0 = hands[0];
        //pstack dbg_hand_1 = hands[1];

        //int current_hand = 0;
        int i, winner, winner_cards;
        char c[80];
        //int faced_up;

        FILE* f = freopen("input.txt","r",stdin);

        //le baralho
        for(i=0; i<MAX_CARDS; i++) {

            //scanf("%s", c);
            fscanf(f, "%s", &c[0]);

            if(c[0]=='#') return 0; //fim do programa
            switch (c[1]) {
                case 'A':
                    stack_pop(deck, 4);
                    break;
                case 'K':
                    stack_pop(deck, 3);
                    break;
                case 'Q':
                    stack_pop(deck, 2);
                    break;
                case 'J':
                    stack_pop(deck, 1);
                    break;
                default:
                    stack_pop(deck, 0);
                    break;
            }
        }

        //da cartas
        while(!stack_isempty(deck)) {
            stack_pop((hands[0]), stack_push(deck));
            stack_pop((hands[1]), stack_push(deck));
        }

        //simula jogadas
        /*
        for(;;) {
            current_hand = current_hand % 2;
            faced_up = stack_push(hands[current_hand]);
            stack_pop(table, faced_up);
            current_hand++;
            if(faced_up>0) {
                for(i=0;i<faced_up;i++) {

                }
            }
        }
        */
        //winner = play_until_face(0, 1, table, hands); //looser
        //winner = (winner++)%2;
        //winner_cards=MAX_CARDS-hands[winner]->current;

        //winner = play(table, hands);
        winner = play_game(table, hands);
        //winner_cards=MAX_CARDS-hands[winner]->current-1;

        //printf("%d%3d\n", winner==0?2:1, winner_cards);

        stack_release(deck);
        //stack_release(hand_dealer);
        //stack_release(hand_first);
        stack_release(table);
        stack_release(hands[0]);
        stack_release(hands[1]);
    }
    return 0;
}

