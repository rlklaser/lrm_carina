/*
Rafael Luiz Klaser
problema A2
*/
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#define MAX 31

int mapa[MAX][MAX];
int visit[MAX];
int min, max;
int total;

int parada()
{
    int i;
    int j;
    i=0;
    for(j=min;j<max+1;j++) {
        i+=visit[j];
    }
    int p = total-i;
    return p;
}

int conta(int p, int m, int nivel)
{
  int t=0;
  visit[p]=1;
  if(m==0) return 0;
  if(nivel==max) {
  	return 0; //nivel==maximo de elementos
  }
  if(parada()==0) {
    return 0; //todos visitados
  }
  int j;
  for(j=min;j<max+1;j++) {
    if(mapa[p][j]==1) {
        t+=conta(j, m-1, nivel+1);
    }
  }
  return t;
}


int main()
{

#ifdef REDIR
  freopen("/home/mgp5900/problem_a2/src/a2.txt", "r", stdin);
#endif

  int count;
  for(;;) {
  scanf("%d", &count);
  if(count==0) return 0;
  int i, j;
  int x, y, p, m;
  min=MAX;
  max=0;
  memset(mapa, 0, sizeof(mapa));
  for(i=0; i<count; i++) {
  	scanf("%d", &x);
	scanf("%d", &y);
        mapa[x][y]=1;
	mapa[y][x]=1;
        mapa[x][x]=1;
	mapa[y][y]=1;
        if(x>max)max=x;
        if(x<min)min=x;
        if(y>max)max=y;
        if(y<min)min=y;
  }
  total = (max-min+1);
  for(;;) {
     scanf("%d", &p);
     scanf("%d", &m);
     if(p==0 && m==0) break;

     int t, n = 0;
     memset(visit, 0, sizeof(visit));
     visit[p]=1;
     t=conta(p, m, 1);
     
     for(i=min;i<max+1;i++) {
        n+=visit[i];
     }
     n = total-n;
     printf("%d\n", n);
  }
  }
}
