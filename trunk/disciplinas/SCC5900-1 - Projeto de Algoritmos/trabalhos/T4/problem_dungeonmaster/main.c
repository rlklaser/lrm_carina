#include <stdio.h>
#include <string.h>

struct s_coord
{
    char l;
    char r;
    char c;
};

struct s_maze
{
    char ch;
    char v;
    struct s_coord coord;
    int moves;
};

#define MAX 99999999
#define QMAX (30*30*30)

struct s_maze maze[32][32][32];
struct s_maze* moves[QMAX];
int head;
int last;
int count;

void q_init()
{
    head=0;
    count=0;
    last=0;
}

int q_empty()
{
    return (count==0);
}

struct s_maze* q_head()
{
    struct s_maze* res = moves[head];
    head++;
    head = head % QMAX;
    count--;
    return res;
}

void q_enqueue(struct s_maze* i)
{
    int ndx=last % QMAX;
    moves[ndx]=i;
    last++;
    count++;
}

int getout(struct s_coord start_coords, struct s_coord end_coords)
{
    struct s_maze* mz = &maze[(int)start_coords.l][(int)start_coords.r][(int)start_coords.c];
    struct s_maze* mzn;
    q_init();

    mz->v = 1;
    mz->moves = 0;
    q_enqueue(mz);

    while(!q_empty()) {
        mz = q_head();
        if(mz->coord.l==end_coords.l && mz->coord.c==end_coords.c && mz->coord.r==end_coords.r) {
            return mz->moves;
        }
        else {
            mzn = &maze[(int)mz->coord.l+1][(int)mz->coord.r][(int)mz->coord.c];
            if(!mzn->v && mzn->ch!='#') {
                mzn->v = 1;
                mzn->moves=mz->moves+1;
                q_enqueue(mzn);
            }
            mzn = &maze[(int)mz->coord.l-1][(int)mz->coord.r][(int)mz->coord.c];
            if(!mzn->v && mzn->ch!='#') {
                mzn->v = 1;
                mzn->moves=mz->moves+1;
                q_enqueue(mzn);
            }
            mzn = &maze[(int)mz->coord.l][(int)mz->coord.r+1][(int)mz->coord.c];
            if(!mzn->v && mzn->ch!='#') {
                mzn->v = 1;
                mzn->moves=mz->moves+1;
                q_enqueue(mzn);
            }
            mzn = &maze[(int)mz->coord.l][(int)mz->coord.r-1][(int)mz->coord.c];
            if(!mzn->v && mzn->ch!='#') {
                mzn->v = 1;
                mzn->moves=mz->moves+1;
                q_enqueue(mzn);
            }
            mzn = &maze[(int)mz->coord.l][(int)mz->coord.r][(int)mz->coord.c+1];
            if(!mzn->v && mzn->ch!='#') {
                mzn->v = 1;
                mzn->moves=mz->moves+1;
                q_enqueue(mzn);
            }
            mzn = &maze[(int)mz->coord.l][(int)mz->coord.r][(int)mz->coord.c-1];
            if(!mzn->v && mzn->ch!='#') {
                mzn->v = 1;
                mzn->moves=mz->moves+1;
                q_enqueue(mzn);
            }
        }
    }
    return 0;
}

int main()
{
#ifdef REDIR
  freopen("input.txt","r",stdin);
#endif

    int ll, rr, cc;
    int i, j, k;
    char str[80];
    struct s_coord start_coords;
    struct s_coord end_coords;
    char ch;
    int res = 0;

    for(;;)
    {
        memset(maze, '#', sizeof(maze));
        memset(str, 0, sizeof(str));
        memset(moves, 0, sizeof(moves));

        scanf("%d", &ll);
        scanf("%d", &rr);
        scanf("%d", &cc);

        if(ll==0 && rr==0 && cc==0) break;

        for(i=1; i<ll+1; i++) {
            for(j=1; j<rr+1; j++) {
                scanf("%s", str);
                for(k=1; k<cc+1; k++) {
                    ch = str[k-1];
                    if(ch=='S') {
                        start_coords.l=i;
                        start_coords.r=j;
                        start_coords.c=k;
                    }
                    if(ch=='E') {
                        end_coords.l=i;
                        end_coords.r=j;
                        end_coords.c=k;
                    }
                    maze[i][j][k].ch = ch;
                    maze[i][j][k].v = 0;
                    maze[i][j][k].coord.l = i;
                    maze[i][j][k].coord.r = j;
                    maze[i][j][k].coord.c = k;
                }
            }
        }

        res = getout(start_coords, end_coords);

        if(res) {
            printf("Escaped in %d minute(s).\n", res);
        }
        else {
            printf("Trapped!\n");
        }

    }
    return 0;
}
