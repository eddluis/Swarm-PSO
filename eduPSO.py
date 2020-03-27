# -------------------------------------------------------------------------- #
# Implementação de PSO baseado em artigo escrito por Bratton(2007)
# -------------------------------------------------------------------------- #

import random
import numpy as np
from math import exp
import matplotlib.pyplot as plt


# --- Definicao das particulas
class Particle:
    def __init__(self, initial_pos):
        # campos individuais da particula

        self.position_i = []  # posicao
        self.pos_best_i = []  # melhor posicao

        self.speed_i = []  # velocidade

        self.err_i = -1  # erro/distancia da solucao
        self.err_best_i = -1  # menor erro/distancia da solucao

        for i in range(0, dimensions):
            self.speed_i.append(random.uniform(-1, 1))
            self.position_i.append(initial_pos[i])

    # mede o fitness e verifica se a pos atual é um pBest
    def eval(self, func):
        self.err_i = func(self.position_i)

        if self.err_i < self.err_best_i or self.err_best_i == -1:
            self.pos_best_i = self.position_i
            self.err_best_i = self.err_i

    # atualiza nova particula
    def update_speed(self, g_best, w):
        # inercia
        #w = 0.8
        # constante cognitiva
        c1 = 2.05
        # constante social
        c2 = 2.05

        for i in range(0, dimensions):
            # valores aleatorios para r1 e r2
            r1 = random.random()
            r2 = random.random()

            v_cognitivo = c1 * r1 * (self.pos_best_i[i] - self.position_i[i])
            v_social = c2 * r2 * (g_best[i] - self.position_i[i])
            self.speed_i[i] = w * self.speed_i[i] + v_cognitivo + v_social

    # atualiza a posicao da particula com base na velocidade
    def update_position(self):
        for i in range(0, dimensions):
            self.position_i[i] = self.position_i[i] + self.speed_i[i]


# --- Definicao do PSO
class PSO():
    def __init__(self, func, initial_pos, num_particles, iterations):
        global dimensions
        dimensions = len(initial_pos)
        # menor erro/distancia da solucao
        err_best_g = -1
        # melhor posicao(gbest)
        g_best = []

        # inicializa o swarm
        swarm = []
        for i in range(0, num_particles):
            swarm.append(Particle(initial_pos))

        # mecanismo de otimizacao:
        # loop pelas iteracoes
        # loop pelas particulas no swarm avaliando o fitness
        # compara posicao da particula atual com o gBest
        # atualiza posicoes e velocidades
        i = 0
        fitness_history = []

        while i < iterations:
            for j in range(0, num_particles):
                swarm[j].eval(func)

                # determina se a particula atual tem o gBest
                if swarm[j].err_i < err_best_g or err_best_g == -1:
                    g_best = list(swarm[j].position_i)
                    err_best_g = float(swarm[j].err_i)
            w=0.9
            for j in range(0, num_particles):
                if j > 60000:
                    w=0.8
                elif j >= 120000:
                    w=0.7
                elif j >= 180000:
                    w = 0.6
                elif j >= 240000:
                    w = 0.5
                elif j >= 300000:
                    w = 0.4
                else :
                    w = 0.8

                swarm[j].update_speed(g_best, w)
                swarm[j].update_position()
            i += 1
            fitness_history.append(err_best_g)

        # resultados
        print('RESULTADOS:')
        print("melhor posicao: ", g_best)
        if err_best_g == 0:
            print("fitness: ", 1)
        else:
            print("fitness: ", 1/err_best_g)

        size = range(len(fitness_history))
        #size = len(fitness_history)

        plt.plot(fitness_history, size, '-p', color='gray',
                 markersize=4, linewidth=.5,
                 markerfacecolor='white',
                 markeredgecolor='gray',
                 markeredgewidth=.5)


        # plt.plot(fitness_history, size)
        plt.show()


if __name__ == "__PSO__":
    main()


# --- Funcoes e Parametros

# Funcao a ser minimizada (Sphere)
def function1(x):
    y = 0
    for i in range(len(x)):
        y += x[i] ** 2
    return y

# Funcao a ser minimizada (Rastrigin)
def rastrigin(d):
    sum_i = np.sum([x**2 - 10*np.cos(2 * np.pi * x) for x in d])
    return 10 * len(d) + sum_i

def rastriginFn(d):
    def fn(*args):
        sum_i = sum([args[i]**2 - 10*np.cos(2 * np.pi * args[i]) for i in range(len(args))])
        return 10 * d + sum_i
    return fn

# Funcao a ser minimizada (Rosenbrock)
def rosenbrock(d):
    x = d[0]
    y = d[1]
    a = 1 - x
    b = y - x*x
    return a*a + b*b*100


# parametros ajustaveis
n_dimensions = 30
n_particles = 30
iterations = 30000
# posicao de inicio aleatoria em todas as coordenadas
initial_pos = [random.random()] * n_dimensions

# --- Chamada de inicio
PSO(rosenbrock, initial_pos, n_particles, iterations)

