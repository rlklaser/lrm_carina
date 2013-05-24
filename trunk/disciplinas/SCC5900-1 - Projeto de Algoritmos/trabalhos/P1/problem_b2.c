/*
Rafael Luiz Klaser
problema B2
*/


#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define MAXA  10000
#define DMIN 10000

int main(int argc, char *argv[])
{

#ifdef REDIR
  freopen("/home/mgp5900/problem_b2/src/b2.txt", "r", stdin);
#endif

  int lugares[MAXA][2];
  int count;
  for(;;) {
	scanf("%d", &count);
	if(count==0) return 0;
	int i, j;
        //int minx1,minx2,miny1,miny2;
	for(i=0; i<count; i++) {
		scanf("%d", &lugares[i][0]);
		scanf("%d", &lugares[i][1]);
	}
	
	double dmin=DMIN;
	double dist;
	//int sim;
	for(i=0;i<count-1;i++) {
	//for(i=count-1;i>0;i--) {
		//sim = 0;
		for(j=i+1;j<count;j++) {
		//for(j=i-1;j>-1;j--) {
			dist = pow(lugares[i][0]-lugares[j][0],2) + pow(lugares[i][1]-lugares[j][1],2);
			dist=sqrt(dist);
			if(dist<dmin) {
				dmin=dist;
				//minx1=lugares[i][0];
				//miny1=lugares[i][1];
				//minx2=lugares[j][0];
				//miny2=lugares[j][1];
			}
			if(dmin==1) {
				break;
			}
		}
		if(dmin==1) {
			break;
		}
	}
	
	if(dmin<DMIN) {
		printf("%.04f\n", dmin);
	}else{
		printf(":(\n");
	}
  }
  return EXIT_SUCCESS;
}
