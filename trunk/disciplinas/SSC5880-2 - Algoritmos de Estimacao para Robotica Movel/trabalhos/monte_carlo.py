from collections import namedtuple
import random
import matplotlib

mundo=['par', 'porta', 'par', 'porta', 'par', 'par', 'par', 'porta', 'par', 'par']

#Bayes
loc=[0]*len(mundo)
for i in range (len(mundo)):
    loc[i]=1.0/len(mundo)

z_ok=0.95
z_erro=0.05

u_ok=0.9
u_aprox=0.15

def calc_perc(z):
    for i in range (len(mundo)):
        if (z==mundo[i]):
            loc[i]=loc[i]*z_ok
        else:
            loc[i]=loc[i]*z_erro

    norm=sum(loc)
    for i in range (len(mundo)):
        loc[i]=round(loc[i]/norm,3)


def calc_mov(dist):
    aux=[0]*len(mundo)
    for i in range (len(mundo)):
        aux[i]=loc[i-dist]*u_ok+loc[i-dist-1]*u_aprox+loc[i-dist+1]*u_aprox

    norm=sum(aux)
    for i in range (len(mundo)):
        loc[i]=round(aux[i]/norm,3)

#Monte Carlo
random.seed()

#constantes
min_ambiente = 0.0
max_ambiente = 10.0
max_ruido = 0.60
min_ruido = 0.20 

num_particulas = 100
particulas = [0]*num_particulas
pesos = [0]*num_particulas

Particle = namedtuple('Particle', 'x w')
M = [Particle(0,0)]*num_particulas

posterior = [(0,0)]*num_particulas

def nova_particula():
    #gera particulas com 2 casas decimais
    return random.randint(min_ambiente*100, max_ambiente*100) / 100.0

def inicializa():
    for i in range(num_particulas):
        particulas[i] = nova_particula()
        pesos[i] = 1.0/num_particulas

def add_ruido():
    for i in range(num_particulas):
        #o ruido pode ser positivo ou negativo
        sgn = pow(-1, random.randint(1, 2))
        #gera ruido dentro do intervalo
        ruido = random.randint(min_ruido*100, max_ruido*100) / 10.0
        particulas[i] += (particulas[i]*ruido/100) * sgn
        
def atuador(val):
    for i in range(num_particulas):
        particulas[i]+=val
        #gera nova se saiu do range
        if(particulas[i]>max_ambiente or particulas[i]<min_ambiente):
           particulas[i] = nova_particula()
        
def sensor(val):
    pass

import matplotlib.mlab as mlab
import matplotlib.pyplot as plt

mu, sigma = 100, 15
x = mu + sigma*random.randn(10000)

# the histogram of the data
n, bins, patches = plt.hist(x, 50, normed=1, facecolor='green', alpha=0.75)

# add a 'best fit' line
y = mlab.normpdf( bins, mu, sigma)
l = plt.plot(bins, y, 'r--', linewidth=1)

plt.xlabel('Smarts')
plt.ylabel('Probability')
plt.title(r'$\mathrm{Histogram\ of\ IQ:}\ \mu=100,\ \sigma=15$')
plt.axis([40, 160, 0, 0.03])
plt.grid(True)

plt.show()

