#include <stdio.h>
#include <string.h>

struct s_coord
{
    int l;
    int r;
    int c;
};

struct s_maze
{
    char ch;
    char v;
};

#define MAX 99999999

struct s_maze maze[32][32][32];

int gmin;

int getout(int l, int r, int c, int walk)
{
    struct s_maze* mz = &maze[l][r][c];
    int res, min = MAX;

    if(mz->v || mz->ch=='#' || walk>gmin) {
        return MAX;
    }

    if(mz->ch=='E') {
        if(walk<gmin) gmin = walk;
        return walk;
    }
    else {
        mz->v = 1;
    }

    res = getout(l+1, r, c, walk+1);
    if(res<min) min = res;

    res = getout(l-1, r, c, walk+1);
    if(res<min) min =res;

    res = getout(l, r+1, c, walk+1);
    if(res<min) min = res;

    res = getout(l, r-1, c, walk+1);
    if(res<min) min = res;

    res = getout(l, r, c+1, walk+1);
    if(res<min) min = res;

    res = getout(l, r, c-1, walk+1);
    if(res<min) min =res;


    if(min<MAX) mz->v = 0;

    return min;
}

int main()
{
#ifdef REDIR
  freopen("input.txt","r",stdin);
#endif

    int l, r, c;
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
        gmin = MAX;
        start_coords.l = MAX;
        end_coords.l = MAX;

        scanf("%d", &l);
        scanf("%d", &r);
        scanf("%d", &c);

        if(l==0 && r==0 && c==0) break;

        for(i=1; i<l+1; i++) {
            for(j=1; j<r+1; j++) {
                scanf("%s", str);
                for(k=1; k<c+1; k++) {
                    ch = str[k-1];
                    if(ch=='S' || ch=='s') {
                        start_coords.l=i;
                        start_coords.r=j;
                        start_coords.c=k;
                        ch = 'S';
                    }
                    if(ch=='E' || ch=='e') {
                        end_coords.l=i;
                        end_coords.r=j;
                        end_coords.c=k;
                        ch = 'E';
                    }
                    maze[i][j][k].ch = ch;
                    maze[i][j][k].v = 0;
                }
            }
        }

        if(start_coords.l<MAX && end_coords.l<MAX) {
            res = getout(start_coords.l, start_coords.r, start_coords.c, 0);
        }
        else {
            res = MAX;
        }

        if(res<MAX) {
            printf("Escaped in %d minute(s).\n", gmin);
        }
        else {
            printf("Trapped!\n");
        }

    }
    return 0;
}
