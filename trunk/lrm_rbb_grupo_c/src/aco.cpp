#include"aco.h"

float delta[TAM_GRID][TAM_GRID]; //matriz auxiliar de feromonio
float feromonio[TAM_GRID][TAM_GRID]; // matriz de feromonio atualizada por iteracao

//================ Function Implementation =========================

float getFeromonio(int posicaoX, int posicaoY){
	return feromonio[posicaoX][posicaoY];
}

void inicializaFeromonio(){
    for(int i = 0; i < TAM_GRID; i++){
        for(int j = 0; j < TAM_GRID; j++){
            feromonio[i][j] = 0;
        }
    }
}

//tmpCode{

void inicializaLixo(Item *lixo){
    for(int i = 0; i < QTD_LIXO; i++){
        lixo[i].x = rand() % 65;
        lixo[i].y = rand() % 65;
    }
}
//endTmpCode

void zeraDelta(){
    for(int i = 0; i < TAM_GRID; i++){
        for(int j = 0; j < TAM_GRID; j++){
            delta[i][j] = 0;
        }
    }
}

bool achouLixo(int x, int y, Item *lixo, veiculo *v, int qualVeiculo){
    for(int i = 0; i < QTD_LIXO; i++){
        if((lixo[i].x == x) && (lixo[i].y == y)){
			if(v[qualVeiculo].temLixo == false){
			    lixo[i].existe = false;
			}
            return true;
        }
    }
    return false;
}


int maiorElementoProx(int x, int y){
    int maior = 0;
    float prox[8];

    if(((x-1) >= 0) && ((y+1) <= TAM_GRID)){
        prox[0] = feromonio[x-1][y+1];
    }else{
        prox[0] = -1;
    }

    if((y+1) <= TAM_GRID){
        prox[1] = feromonio[x][y+1];
    }else{
        prox[1] = -1;
    }

    if(((x+1) <= TAM_GRID) && ((y+1) <= TAM_GRID)){
        prox[2] = feromonio[x+1][y+1];
    }else{
        prox[2] = -1;
    }

    if((x-1) >= 0){
        prox[3] = feromonio[x-1][y];
    }else{
        prox[3] = -1;
    }

    if((x+1) <= TAM_GRID){
        prox[4] = feromonio[x+1][y];
    }else{
        prox[4] = -1;
    }

    if(((x-1) >= 0) && ((y-1) >= 0)){
        prox[5] = feromonio[x-1][y-1];
    }else{
        prox[5] = -1;
    }

    if((y-1) >= 0){
        prox[6] = feromonio[x][y-1];
    }else{
        prox[6] = -1;
    }

    if(((x+1) <= TAM_GRID) && ((y-1) >= 0)){
        prox[7] = feromonio[x+1][y-1];
    }else{
        prox[7] = -1;
    }

    for(int i = 1; i < 8; i++){
        if (prox[i] > prox[maior]){
            maior = i;
        }
    }

    if (prox[maior] == -1) maior = rand() % 8;

    return maior;
}

void atualizaPos(int i, int movX, int movY, veiculo *v){
    if(((v[i].minhaPosicaoX + movX) > 0) && ((v[i].minhaPosicaoX + movX) < TAM_GRID)){
        v[i].destinoX = v[i].minhaPosicaoX + movX; 
		delta[(int)v[i].minhaPosicaoX][(int)v[i].minhaPosicaoY] += QTD_FEROM;

    }
    if(((v[i].minhaPosicaoY + movY) > 0) && ((v[i].minhaPosicaoY + movY) < TAM_GRID)){
        v[i].destinoY = v[i].minhaPosicaoY + movY;
		delta[(int)v[i].minhaPosicaoX][(int)v[i].minhaPosicaoY] += QTD_FEROM;
    }
}

void movimentaFormigas(veiculo *v, Item *lixo){
    for(int i = 0; i < QTD_VEICULOS; i++){
        int next = 0;
		
		if ((v[i].minhaPosicaoX <= DEF_POS_X*2) && (v[i].minhaPosicaoY <= DEF_POS_Y*2)) v[i].temLixo = false;

		if (achouLixo(v[i].minhaPosicaoX, v[i].minhaPosicaoY, lixo, v, i) == 0){
            //caminhar aleatoriamente
            next = rand()%8;
        }else{
            //tendencia para bloco com mais feromonio
			v[i].temLixo = true;
            next = maiorElementoProx(v[i].minhaPosicaoX, v[i].minhaPosicaoY);
        }
        switch (next){
            case 0: atualizaPos(i, -1,  1, v); break;
            case 1: atualizaPos(i,  0,  1, v); break;
            case 2: atualizaPos(i,  1,  1, v); break;
            case 3: atualizaPos(i, -1,  0, v); break;
            case 4: atualizaPos(i,  1,  0, v); break;
            case 5: atualizaPos(i, -1, -1, v); break;
            case 6: atualizaPos(i,  0, -1, v); break;
            case 7: atualizaPos(i,  1, -1, v); break;
        }
    }
}

void atualizaFeromonio(){
    for(int i = 0; i < TAM_GRID; i++){
        for(int j = 0; j < TAM_GRID; j++){
            feromonio[i][j] = (1 - RHO)*feromonio[i][j] + delta[i][j];
        }
    }
}

void atualizaFeromonioInit(){
	for(int i = 0; i < TAM_GRID; i++){
        for(int j = 0; j < TAM_GRID; j++){
            feromonio[i][j] = delta[i][j];
        }
    }
}

