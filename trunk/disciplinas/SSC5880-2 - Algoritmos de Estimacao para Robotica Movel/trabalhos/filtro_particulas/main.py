'''
Created on Jun 3, 2012

Localizacao Monte Carlo - Filtro de Particulas

@author: Rafael Luiz Klaser

uso:
- o metodo monte_carlo_filter() executa o metodo em um unico passo
- os metodos passo a passo sao:
    step_move()     : gera um deslocamento (positivo ou negativo)
    step_sense()    : executa uma medicao (ex: viu porta)
    step_resample() : reposiciona as particulas de acordo com a crenca gerada por sense

ex execucao:
    step_move(2.4)
    step_sense('porta')
    step_resample()
    
- o metodo show_filter() gera um grafico 'scatter' exibindo a distribuicao
  das particulas em X
  ex:
      executar show_filter() a cada passo para ver as particulas
'''

import random
import matplotlib.pyplot as plt

mundo=['par', 'porta', 'par', 'porta', 'par', 'par', 'par', 'porta', 'par', 'par']

#Monte Carlo
random.seed()

#constantes
min_ambiente = 0.0
max_ambiente = 10.0
num_particulas = 200
base_weight = 1.0/num_particulas

#ruido do deslocamento
max_ruido = 0.17
min_ruido = 0.03

#particulas
_Xt=[0]*num_particulas
_Wt=[0]*num_particulas


'''
Algoritmo MCL
'''     
#- Ut -> Odometria (comando movimento)
#- Zt -> Sensores (observacao)
def monte_carlo_filter(Ut, Zt):
    for m in range(num_particulas):
        _Xt[m] = sample_motion_model(Ut, _Xt[m])
        _Wt[m] = measurement_model(Zt, _Xt[m], m)
    
    _Xt = resample()
    _Wt = [base_weight]*num_particulas

'''
Execucoes passo-a-passo
'''
def step_sense(Zt):
    for m in range(num_particulas):
        _Wt[m] = measurement_model(Zt, _Xt[m], m)
        
def step_move(Ut):
    for m in range(num_particulas):
        _Xt[m] = sample_motion_model(Ut, _Xt[m])

def step_resample():
    Xt = resample()
    for m in range(num_particulas):
        _Xt[m] = Xt[m]
        _Wt[m] = base_weight

def new_particle():
    #gera particulas com 2 casas decimais
    return random.randint(min_ambiente*100, max_ambiente*99) / 100.0

#- distribui as particulas aleatoreamente pelo espaco
#- inicializa distribuicao uniforme
def initialize():
    for i in range(num_particulas):
        _Xt[i] = new_particle()
        _Wt[i] = base_weight
            
def normalize():
    norm=sum(_Wt)
    for j in range(num_particulas):
        _Wt[j]=_Wt[j]/norm
        
def resample():
    '''
    usa os vetores _Xt e _Wt como sendo a p(z|x) X(barra)t
    '''
    #normaliza para fazer o sorteio
    normalize()
            
    #novas particulas
    Xt=[(0)]*num_particulas
        
    for m in range(num_particulas):
        i = draw_with_probability()
        Xt[m]=_Xt[i]

    return Xt
    
def exec_filter(Ut, Zt):
    monte_carlo_filter(Ut, Zt)

def show_exec_filter(Ut, Zt):
    exec_filter(Ut, Zt)
    show_filter()

def show_filter():
    plt.figure(figsize=(12,3))
    plt.scatter(_Xt, _Wt, s=500, marker='|')
    plt.draw()
    
#- retorna um indice baseado no sorteio dentro da
#distribuicao de probabilidade (roleta)
def draw_with_probability():
    roleta = random.random()
    total = 0.0
    ndx = 0
    for i in range(num_particulas):
        total+=_Wt[i]
        if(total>=roleta):
            ndx = i
            break
        
    return ndx

def noise(value):
    sgn = pow(-1, random.randint(1, 2))
    #gera ruido dentro do intervalo
    n = random.randint(int(min_ruido*100), int(max_ruido*100)) / 100.0
    value += (value*n/100) * sgn
    return value
    
#calcula nova posicao dado o comando de deslocamento
def sample_motion_model(Ut, Xt):
    #atua
    motion_pos = Xt+Ut
    #adiciona ruido
    motion_pos=noise(motion_pos)
    
    #valida particula
    #if(motion_pos>=max_ambiente or motion_pos<min_ambiente):
    #    #saiu do ambiente, gera nova particula aleatoria
    #    motion_pos = new_particle()
    
    #cilcula no ambiente
    if(motion_pos>max_ambiente):
        motion_pos = motion_pos - max_ambiente
    if(motion_pos<min_ambiente):
        motion_pos = motion_pos + max_ambiente
    
    return motion_pos

#medicao
def measurement_model(Zt, Xt, m):
    ndx = int(Xt)
    seen = mundo[ndx]
    norm = Xt-ndx
    hit = 0.5
    if(seen==Zt):
        #TODO: correto seria usar a curva gaussiana
        if(norm>=0 and norm<0.1):
            hit=1.20
        if(norm>=0.1 and norm<0.2):
            hit=1.33
        if(norm>=0.2 and norm<0.3):
            hit=1.45
        if(norm>=0.3 and norm<0.4):
            hit=1.70
        if(norm>=0.4 and norm<0.6):
            hit=2.00
        if(norm>=0.6 and norm<0.7):
            hit=1.70   
        if(norm>=0.7 and norm<0.8):
            hit=1.45
        if(norm>=0.8 and norm<0.9):
            hit=1.33
        if(norm>=0.9 and norm<=1):
            hit=1.20
            
    return base_weight*hit


initialize()

#from IPython.core.display import Image
#Image("https://dl.dropbox.com/u/62929183/portas-pacman.png")
