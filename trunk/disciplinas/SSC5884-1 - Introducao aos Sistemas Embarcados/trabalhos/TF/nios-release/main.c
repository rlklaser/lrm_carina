#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>


#define X 20
#define R 20
#define PRINT

extern void update_ledg();

static int Lower_Triangular_Solve(double *L, double B[], double x[], int n)
{
   int i, k;

   for (k = 0; k < n; L += n, k++) {
      if (*(L + k) == 0.0) return -1;           // The matrix L is singular
      x[k] = B[k];
      for (i = 0; i < k; i++) x[k] -= x[i] * *(L + i);
      x[k] /= *(L + k);
   }

   return 0;
}

static void Unit_Upper_Triangular_Solve(double *U, double B[], double x[], int n)
{
   int i, k;

   x[n-1] = B[n-1];
   for (k = n-2, U += n * (n - 2); k >= 0; U -= n, k--) {
      x[k] = B[k];
      for (i = k + 1; i < n; i++) x[k] -= x[i] * *(U + i);
   }
}

static int Crout_LU_Decomposition(double *A, int n)
{
   int i, j, k, p;
   double *p_k, *p_row, *p_col;

   for (k = 0, p_k = A; k < n; p_k += n, k++) {
      for (i = k, p_row = p_k; i < n; p_row += n, i++) {
         for (p = 0, p_col = A; p < k; p_col += n, p++)
            *(p_row + k) -= *(p_row + p) * *(p_col + k);
      }
      if ( *(p_k + k) == 0.0 ) {
    	  return -1;
      }
      for (j = k+1; j < n; j++) {
         for (p = 0, p_col = A; p < k; p_col += n,  p++)
            *(p_k + j) -= *(p_k + p) * *(p_col + j);
         *(p_k + j) /= *(p_k + k);
      }
   }
   return 0;
}

static int Crout_LU_Solve(double *LU, double B[], double x[], int n)
{

   if ( Lower_Triangular_Solve(LU, B, x, n) < 0 ) {
	   printf("Lower Triangular Solve error...\n");
	   return -1;
   }

   Unit_Upper_Triangular_Solve(LU, x, x, n);

   return 0;
}
/*
static void transpose(void *dest, void *src, int src_h, int src_w)
{
	int i, j;
	double (*d)[src_h] = dest, (*s)[src_w] = src;
	for (i = 0; i < src_h; i++)
		for (j = 0; j < src_w; j++)
			d[j][i] = s[i][j];
}
*/
static int Solve_Inverse(int print, void* mat, int mat_size)
{
    double I[X][X];
    double B[X][X];
    double LU[X][X];
    int res;
    int i, j, k;

    memset(&B[0][0], 0, sizeof(B));
    memset(&I[0][0], 0, sizeof(I));
    memcpy(&LU[0][0], mat, mat_size);

    //gera identidade
    for(i=0; i<X; i++) {
        I[i][i]=1;
    }


    res = Crout_LU_Decomposition(&LU[0][0], X);

    if(res!=0) {
    	printf("LU Decomposition error...\n");
    	return res;
    }

    for(i=0; i<X; i++) {
        //Recebe LU + vetores coluna I e B
        res = Crout_LU_Solve(&LU[0][0], I[i], B[i], X);
        if(res!=0) {
        	printf("LU Solve error...\n");
        	break;
        }
    }
    //Inv(A) resolvido na transposta -> Inv(A)==Transposta(B)

#ifdef PRINT
    if(print) {
        printf("matriz LU\n");
        for(i=0; i<X; i++) {
            for(j=0; j<X; j++) {
                printf("%.2f;", LU[i][j]);
            }
            printf("\n");
        }

        //transposta
        for (i = 0; i < X; i++)
            for (j = 0; j < X; j++)
                I[j][i] = B[i][j];
        printf("matriz Inv(A)\n");
        for(i=0; i<X; i++) {
            for(j=0; j<X; j++) {
                printf("%.4f;", I[i][j]);
            }
            printf("\n");
        }

        //multiplicacao
        memcpy(&LU[0][0], mat, mat_size);
        double sum;
        for(i=0;i<X;i++)
        {
            for(j=0;j<X;j++)
            {
                sum=0;
                for(k=0;k<X;k++)
                {
                    sum=sum+(LU[i][k]*I[k][j]);
                    B[i][j]=sum;
                }
            }
        }

        printf("matriz A*Inv(A)\n");
        for(i=0; i<X; i++) {
            for(j=0; j<X; j++) {
                printf("%1.1f;", B[i][j]);
            }
            printf("\n");
        }

    }
#endif

    return res;
}

/*
int main()
{
	printf("Hello Board!\n");

	return 0;
}
*/

int main() {
    int i, j;
    double A[X][X];
    int print = 0;
    int res;
#ifdef PRINT
    printf("Hello Board!\n");
    printf("Matrizes %dx%d, inversa %d+1 vezes.\n",X,X,R);
#endif
    update_ledg(1);

    //gera uma matriz XxX com valores aleatoreos de -50 a 50

    srand(23); //usa mesma semente para replicar valores

    for(i=0; i<X; i++) {
        for(j=0; j<X; j++) {
            A[i][j] = rand()%100 - 50;
        }
    }

    //roda n vezez para pegar tempo
    for(i=0;i<R;i++) {
#ifdef PRINT
    	printf("going on %d\n", i);
#endif
    	update_ledg(0);
        res = Solve_Inverse(0, &A[0][0], sizeof(A));
        update_ledg(1);
#ifdef PRINT
        if(res!=0) printf("ERRO\n");
#endif
    }

#ifdef PRINT
    //imprime uma vez os resultados
    if(print) {
        printf("matriz A\n");
        for(i=0; i<X; i++) {
            for(j=0; j<X; j++) {
                printf("%.1f;", A[i][j]);
            }
            printf("\n");
        }
        res = Solve_Inverse(print, &A[0][0], sizeof(A));
        if(res!=0) printf("ERRO\n");
    }
#endif

    #ifdef PRINT
        printf("Goodbye Board!\n");
    #endif
    update_ledg(0);

    //usleep(5000000); //delay para escrever tudo pela serial antes do exit

    return 0;
}
