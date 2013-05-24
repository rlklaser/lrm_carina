#ifndef ACO_H
#define ACO_H

#include <iostream>
#include <cstdlib>
#include <ctime>

#include "constantes.h"
#include "item.h"
#include "veiculo.h"


using namespace std;



//--------------functions--------------------------

void inicializaFeromonio();
void zeraDelta();
void atualizaFeromonioInit();
void inicializaLixo(Item *lixo);
float getFeromonio(int posicaoX, int posicaoY);

void atualizaPos(int i, int movX, int movY, veiculo *v); //atualiza posicao do robo e matriz delta
void movimentaFormigas(veiculo *v, Item *lixo); //se nao tem lixo aleatorio, se tem ele inicia a busca do feromonio
void atualizaFeromonio(); // atualiza os feromonios na matriz feromonio

int maiorElementoProx(int x, int y); //acha mais feromonio, quando não tem sorteia um
bool achouLixo(int x, int y, Item *lixo, veiculo *v, int qualVeiculo);  //verifica se tem lixo na posicao

//-------------------------------------------------

#endif
