mundo=['par', 'porta', 'par', 'porta', 'par', 'par', 'par', 'porta', 'par', 'par']
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
