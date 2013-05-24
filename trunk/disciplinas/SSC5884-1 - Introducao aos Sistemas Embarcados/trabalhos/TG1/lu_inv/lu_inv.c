#include <stdio.h>
#include <stdlib.h>
//#include <math.h>
#include <string.h>
#include <time.h>

int Lower_Triangular_Solve(double *L, double B[], double x[], int n)
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

void Unit_Upper_Triangular_Solve(double *U, double B[], double x[], int n)
{
   int i, k;

   x[n-1] = B[n-1];
   for (k = n-2, U += n * (n - 2); k >= 0; U -= n, k--) {
      x[k] = B[k];
      for (i = k + 1; i < n; i++) x[k] -= x[i] * *(U + i);
   }
}

int Crout_LU_Decomposition(double *A, int n)
{
   int i, j, k, p;
   double *p_k, *p_row, *p_col;

   for (k = 0, p_k = A; k < n; p_k += n, k++) {
      for (i = k, p_row = p_k; i < n; p_row += n, i++) {
         for (p = 0, p_col = A; p < k; p_col += n, p++)
            *(p_row + k) -= *(p_row + p) * *(p_col + k);
      }
      if ( *(p_k + k) == 0.0 ) return -1;
      for (j = k+1; j < n; j++) {
         for (p = 0, p_col = A; p < k; p_col += n,  p++)
            *(p_k + j) -= *(p_k + p) * *(p_col + j);
         *(p_k + j) /= *(p_k + k);
      }
   }
   return 0;
}

int Crout_LU_Solve(double *LU, double B[], double x[], int n)
{

   if ( Lower_Triangular_Solve(LU, B, x, n) < 0 ) return -1;

   Unit_Upper_Triangular_Solve(LU, x, x, n);

   return 0;
}

/*
int Crout_LU_Decomposition_with_Pivoting(double *A, int pivot[], int n)
{
   int row, i, j, k, p;
   double *p_k, *p_row, *p_col;
   double max;

   for (k = 0, p_k = A; k < n; p_k += n, k++) {
      pivot[k] = k;
      max = fabs( *(p_k + k) );
      for (j = k + 1, p_row = p_k + n; j < n; j++, p_row += n) {
         if ( max < fabs(*(p_row + k)) ) {
            max = fabs(*(p_row + k));
            pivot[k] = j;
            p_col = p_row;
         }
      }

      if (pivot[k] != k)
         for (j = 0; j < n; j++) {
            max = *(p_k + j);
            *(p_k + j) = *(p_col + j);
            *(p_col + j) = max;
         }

      if ( *(p_k + k) == 0.0 ) return -1;

      for (j = k+1; j < n; j++) {
         *(p_k + j) /= *(p_k + k);
      }

      for (i = k+1, p_row = p_k + n; i < n; p_row += n, i++)
         for (j = k+1; j < n; j++)
            *(p_row + j) -= *(p_row + k) * *(p_k + j);

   }
   return 0;
}

int Crout_LU_with_Pivoting_Solve(double *LU, double B[], int pivot[], double x[], int n)
{
   int i, k;
   double *p_k;
   double dum;

   for (k = 0, p_k = LU; k < n; p_k += n, k++) {
      if (pivot[k] != k) {dum = B[k]; B[k] = B[pivot[k]]; B[pivot[k]] = dum; }
      x[k] = B[k];
      for (i = 0; i < k; i++) x[k] -= x[i] * *(p_k + i);
      x[k] /= *(p_k + k);
   }

   for (k = n-1, p_k = LU + n*(n-1); k >= 0; k--, p_k -= n) {
      if (pivot[k] != k) {dum = B[k]; B[k] = B[pivot[k]]; B[pivot[k]] = dum; }
      for (i = k + 1; i < n; i++) x[k] -= x[i] * *(p_k + i);
      if (*(p_k + k) == 0.0) return -1;
   }

   return 0;
}
*/

#define N 3
#define X 20

int Solve()
{
    double A[N][N];
    double I[N][N];
    double B[N][N];
    double LU[N][N];
    int res;
    int i;

    A[0][0] = 6; A[0][1] = -2; A[0][2] = 0;
    A[1][0] = 9; A[1][1] = -1; A[1][2] = 1;
    A[2][0] = 3; A[2][1] = 7; A[2][2] = 5;

//I == Transposta(I)
    I[0][0] = 1; I[0][1] = 0; I[0][2] = 0;
    I[1][0] = 0; I[1][1] = 1; I[1][2] = 0;
    I[2][0] = 0; I[2][1] = 0; I[2][2] = 1;

    memset(&B[0][0], 0, sizeof(B));
/*
A = 6 -2 0
    9 -1 1
    3  7 5

L = 6 0 0
    9 2 0
    3 8 1

U = 1 -1/3  0
    0   1  1/2
    0   0   1

LU= 6 -1/3  0
    9   2  1/2
    3   8   1

Inv(A) -1   5/6 -1/6
      -7/2  5/2 -1/2
      11/2  -4    1

*/
    memcpy(&LU[0][0], &A[0][0], sizeof(A));


    res = Crout_LU_Decomposition(&LU[0][0], N);

    for(i=0; i<N; i++) {
        //Recebe LU + vetores coluna I e B
        res = Crout_LU_Solve(&LU[0][0], I[i], B[i], N);
        if(res==-1) {
            printf("error\n");
        }
    }
    //Inv(A) resolvido na transposta -> Inv(A)==Transposta(B)

    //dynamic();

    return 0;
}

void transpose(void *dest, void *src, int src_h, int src_w)
{
	int i, j;
	double (*d)[src_h] = dest, (*s)[src_w] = src;
	for (i = 0; i < src_h; i++)
		for (j = 0; j < src_w; j++)
			d[j][i] = s[i][j];
}


int Solve_Inverse(int print, void* mat, int mat_size)
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

    if(res!=0) return res;

    for(i=0; i<X; i++) {
        //Recebe LU + vetores coluna I e B
        res = Crout_LU_Solve(&LU[0][0], I[i], B[i], X);
        if(res!=0) break;
    }
    //Inv(A) resolvido na transposta -> Inv(A)==Transposta(B)

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
    return res;
}

/*
void dynamic()
{
    int dim;
    int l, c;
    double* matrix;

    printf("informe a dimensao da matriz quadrada A:");
    scanf("%d", &dim);

    matrix = malloc(dim*dim*sizeof(double));

    for(l=0;l<dim;l++) {
        for(c=0;c<dim;c++) {
            printf("informe A[%d][%d]:", l, c);
            scanf("%f", matrix+(l+c));
        }
    }

    free(matrix);
}
*/

int main() {
    int i, j;
    double A[X][X];
    int print = 1;
    int res;

    //gera uma matriz 10x10 com valores aleatoreos de -50 a 50
    srand(time(NULL));
    for(i=0; i<X; i++) {
        for(j=0; j<X; j++) {
            A[i][j] = rand()%100 - 50;
        }
    }

    //roda n vezez para pegar tempo
    for(i=0;i<9999;i++) {
        res = Solve_Inverse(0, &A[0][0], sizeof(A));
        if(res!=0) printf("ERRO\n");
    }

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

    return 0;
}

int main_old()
{
    int i;
    for(i=0; i<9000000; i++) {
        Solve();
    }

    return 0;
}
