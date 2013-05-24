/*problem Great Wall*/
/*
la se vao mais horas de trabalho em vao para nao levar a lugar algum...quero dizer, ao aborrecimento.
*/

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

using namespace std;

//#define DEBUG
#define MAX_LINES   5
#define MAX_COLS    100
//#100 ? trava o debug!!!! sinistro!!!!
#define MAX_ALFAFAS 3
#define INF         -1
#define MAX_FLOW    10000

struct position
{
    int line;
    int column;
    int present;
    int flow;
};

struct graph
{
    int cost;
    struct position neighbours[MAX_LINES];

};

struct graph wall[MAX_LINES][MAX_COLS+2];
struct position alfafas[MAX_ALFAFAS];


void sum_flows(struct position* vertex, int available_flow)
{
    int i;
    struct position* next;
    int cost;
    int flow;

    cost = wall[vertex->line][vertex->column].cost;
    flow = available_flow-cost;
    vertex->flow = flow;
    for(i=0; i<MAX_LINES; i++) {
        next = &wall[vertex->line][vertex->column].neighbours[i];
        if(next->present && flow>next->flow) {
#ifdef DEBUG
            printf("\tL%dC%d -> L%dC%d [label=%d]\n", vertex->line, vertex->column, next->line, next->column, flow);
#endif
            sum_flows(next, flow);
        }
    }
}

void read_input_file(int n);
int max_flow (int source, int sink);

int main()
{
#ifdef REDIR
    freopen("input.txt","r",stdin);
#endif

    int n, i, j;
    char c;
    char line[MAX_COLS+1];
    int alfafa;
    int cost;

    for(;;) {
        scanf("%d", &n);
        if(n==0) break;

        memset(wall, 0, sizeof(wall));
        memset(alfafas, 0, sizeof(alfafas));
        alfafa = 0;

        for(i=0; i<MAX_LINES; i++) {
            memset(line, 0, sizeof(line));
            scanf("%s", line);
            for(j=1; j<n+1; j++) {
                c = line[j-1];
                if(c=='@') {
                    wall[i][j].cost = 0;
                    alfafas[alfafa].line=i;
                    alfafas[alfafa].column=j;

                    wall[0][0].neighbours[alfafa].line = i;
                    wall[0][0].neighbours[alfafa].column = j;
                    wall[0][0].neighbours[alfafa].present = 1;

                    alfafa++;
                }
                else {
                    wall[i][j].cost = c-0x30;
                }

                /*up*/
                wall[i][j].neighbours[0].line = i-1;
                wall[i][j].neighbours[0].column = j;
                wall[i][j].neighbours[0].present = i>0;

                /*down*/
                wall[i][j].neighbours[1].line = i+1;
                wall[i][j].neighbours[1].column = j;
                wall[i][j].neighbours[1].present = i<4;

                /*right*/
                if(j<n) {
                    wall[i][j].neighbours[3].line = i;
                }
                else {
                    wall[i][j].neighbours[3].line = 0;
                }
                wall[i][j].neighbours[3].column = j+1;
                wall[i][j].neighbours[3].present = 1;


            }
        }

        struct position* next;

#ifdef DEBUG
        printf("digraph g {\n");
#endif
        /*acerta fluxos*/
        for(i=0; i<3; i++) {
            next = &wall[0][0].neighbours[i];
#ifdef DEBUG
            printf("\tL%dC%d -> L%dC%d [label=%d]\n", 0, 0, next->line, next->column, MAX_FLOW);
#endif
            sum_flows(next, MAX_FLOW);
        }
#ifdef DEBUG
        printf("}\n");
#endif

        read_input_file(n+2);
        int sink = (n+1)*MAX_LINES;

        cost = max_flow(0, sink);
    }

    return 0;
}




//////////////////////////


#define WHITE 0
#define GRAY 1
#define BLACK 2
#define MAX_NODES 1000
#define oo 1000000000

int nodes, edges;

int capacity[MAX_NODES][MAX_NODES]; // capacity matrix
int flow[MAX_NODES][MAX_NODES];     // flow matrix
int color[MAX_NODES]; // needed for breadth-first search
int pred[MAX_NODES];  // array to store augmenting path

int min (int x, int y) {
    return x<y ? x : y;  // returns minimum of x and y
}

int head,tail;
int q[MAX_NODES+2];

void enqueue (int x) {
    q[tail] = x;
    tail++;
    color[x] = GRAY;
}

int dequeue () {
    int x = q[head];
    head++;
    color[x] = BLACK;
    return x;
}

int bfs (int start, int target) {
    int u,v;
    for (u=0; u<nodes; u++) {
        color[u] = WHITE;
    }
    head = tail = 0;
    enqueue(start);
    pred[start] = -1;
    while (head!=tail) {
        u = dequeue();
        // Search all adjacent white nodes v. If the capacity
        // from u to v in the residual network is positive,
        // enqueue v.
        for (v=0; v<nodes; v++) {
            if (color[v]==WHITE && capacity[u][v]-flow[u][v]>0) {
                enqueue(v);
                pred[v] = u;
            }
        }
    }
    // If the color of the target node is black now,
    // it means that we reached it.
    return color[target]==BLACK;
}


int max_flow (int source, int sink) {

    int u;

    // Initialize empty flow.
    int max_flow = 0;

    memset(flow, 0, sizeof(flow));

    // While there exists an augmenting path,
    // increment the flow along this path.
    while (bfs(source,sink)) {
        // Determine the amount by which we can increment the flow.
        int increment = oo;
        for (u=nodes-1; pred[u]>=0; u=pred[u]) {
            increment = min(increment,capacity[pred[u]][u]-flow[pred[u]][u]);
        }
            // Now increment the flow.
        for (u=nodes-1; pred[u]>=0; u=pred[u]) {
            flow[pred[u]][u] += increment;
            flow[u][pred[u]] -= increment;
        }
        max_flow += increment;
    }

    // No augmenting path anymore. We are done.
    return max_flow;
}

void read_input_file(int n) {
    int a, b, i, j, k;

    // initialize empty capacity matrix
    memset(capacity, 0, sizeof(capacity));

    nodes = ((n-2)*MAX_LINES)+1;
    edges = 0;

    // read edge capacities
    struct position* neigh;
    for(j=0; j<n; j++) {
        for(i=0; i<MAX_LINES; i++) {
            a = j*MAX_LINES + i;
            for(k=0; k<MAX_LINES; k++) {
                neigh = &wall[i][j].neighbours[k];
                if(neigh->present) {
                    b = neigh->column*MAX_LINES + neigh->line;
                    capacity[a][b] = neigh->flow;
                    edges++;
                }
            }
        }
    }
}

