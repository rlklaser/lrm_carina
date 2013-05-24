#include <stdio.h>
#include <stdlib.h>
#include <strings.h>


struct point {
    int x;
    int y;
    int invalid;
};

struct point points[2000];
struct point pointsaux[2000];
int hull[2000]; //indices dos pontos do envoltorio


#define RAISEVAL(X, Y) ((X<<16) + Y)

void sort(int max)
{
    int i, j;
    struct point p;
    for(i=0; i<max-1; i++) {
        for(j=i; j<max; j++) {
            if(RAISEVAL(points[j].x, points[j].y)<RAISEVAL(points[i].x, points[i].y))
            {
                memcpy(&p, &points[i], sizeof(struct point));
                memcpy(&points[i], &points[j], sizeof(struct point));
                memcpy(&points[j], &p, sizeof(struct point));
            }
        }
    }
}

inline int isleft(struct point p0, struct point p1, struct point p2)
{
    return (p1.x - p0.x)*(p2.y - p0.y) - (p2.x - p0.x)*(p1.y - p0.y);
}

inline float slope(int p1, int p2)
{
    float v = (points[p1].x - points[p2].x);
    if(v==0) {
        return 1<<31;
    }
    else {
        return (points[p1].y - points[p2].y)/v;
    }
}
//implementacao monotone chain
int convexhull(int n)
{
    int bot=0, top=-1;
    int i;
    int minmin = 0, minmax;
    int xmin = points[0].x;

    for (i=1; i<n; i++) {
        if (points[i].x != xmin) break;
    }

    minmax = i-1;

    if (minmax == n-1) {
        hull[++top] =minmin;
        if (points[minmax].y != points[minmin].y) {
            hull[++top] = minmax;
        }
        hull[++top] = minmin;
        return top+1;
    }

    int maxmin, maxmax = n-1;
    int xmax = points[n-1].x;

    for (i=n-2; i>=0; i--) {
        if (points[i].x != xmax) break;
    }

    maxmin = i+1;
    hull[++top] = minmin;
    i = minmax;

    while (++i <= maxmin) {
        if (isleft( points[minmin], points[maxmin], points[i]) >= 0 && i < maxmin) {
            continue;
        }
        while (top > 0) {
            if (isleft( points[hull[top-1]], points[hull[top]], points[i]) > 0) {
                break;
            }
            else {
                top--;
            }
        }
        hull[++top] = i;
    }

    if (maxmax != maxmin) {
        hull[++top] = maxmax;
    }

    bot = top;
    i = maxmin;

    while (--i >= minmax) {
        if (isleft( points[maxmax], points[minmax], points[i]) >= 0 && i > minmax) {
            continue;
        }
        while (top > bot) {
            if (isleft( points[hull[top-1]], points[hull[top]], points[i]) > 0) {
                break;
            }
            else {
                top--;
            }
        }
        hull[++top] = i;
    }

    if (minmax != minmin) {
        hull[++top] = minmin;
    }

    return top+1;
}

int main()
{
    char* ok = "Take this onion to the lab!\n";
    char* nok = "Do not take this onion to the lab!\n";
    int count, i, j, p, k, l;
    int minx = 2000, miny = 2000;
    float m1, m2;
    int layers;

#ifdef FROMFILE
    //freopen("input.txt","r",stdin); //redireciona stdin para arquivo
    freopen("onion.in.txt","r",stdin); //redireciona stdin para arquivo
#endif

    for(;;) {
        scanf("%d", &count);
        if(count==0) break; //condicao de saida

        memset(points, 0, sizeof(points));
        memset(hull, 0, sizeof(hull));

        //le pontos
        for(p=0; p<count; p++) {
            scanf("%d", &points[p].x);
            scanf("%d", &points[p].y);

            if(points[p].x<minx) minx = points[p].x;
            if(points[p].y<miny) miny = points[p].y;
        }

        //minimo 6 para ter 2 camadas
        //na faz o metodo
        if(count<=5) {
            printf(ok); //impar
            continue;
        }

        //translada os pontos para o quadrante positivo
        if(minx<0) {
            for(p=0; p<count; p++) {
                points[p].x = points[p].x + abs(minx);
            }
        }
        if(miny<0) {
            for(p=0; p<count; p++) {
                points[p].y = points[p].y + abs(miny);
            }
        }

        //ordena por x e y
        sort(count);

        layers=0;
        while(count>5) { //minimo 6 para duas camadas

            i = convexhull(count);

            //invalida os pontos utilizados no envoltorio
            //e os colineares
            for(j=0;j<i-1;j++)
            {
                points[hull[j]].invalid=1;

                m1 = slope(hull[j], hull[j+1]);

                for(l=0; l<count; l++) {
                    if(l!=hull[j]) { //nao eh o proprio ponto
                        m2=slope(hull[j], l);
                        if(m1==m2) {
                            //colinear de uma borda, invalida
                            points[l].invalid=1;
                        }
                    }
                }
            }
            points[hull[i-1]].invalid=1;

            p=0;
            //copia pontos nao invalidados para aux
            memset(pointsaux, 0, sizeof(pointsaux));
            for(k=0;k<count;k++) {
                if(!points[k].invalid) {
                    memcpy(&pointsaux[p], &points[k], sizeof(struct point));
                    p++;
                }
            }
            //utiliza aux como novos pontos
            memcpy(&points[0], &pointsaux[0], sizeof(points));
            //count-=(i-1);
            count=p;
            layers++;
        }
        if(count>2) {
            layers++; //pontos restantes contam como uma camada
        }
        if(layers%2==1) printf(ok); else printf(nok);
    }

    return 0;
}
