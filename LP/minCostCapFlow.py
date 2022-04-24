#!/usr/bin/python3
from ortools.linear_solver import pywraplp

def arcCost(source, target, cost, capacity):
    return ((int(source), int(target)), (float(cost), float(capacity)))

#Instancia solver linear
solver = pywraplp.Solver('simple_lp_program', pywraplp.Solver.GLOP_LINEAR_PROGRAMMING)

#Parametrizacao
numNodes = int(input()) #i
numArcs = int(input())
nodesSupplies = [float(input()) for _ in range(numNodes)] #o_i
#matrizes de custo e capacidade, (i,j) -> (c_ij, q_ij) 
arcs = dict(arcCost(*input().split()) for _ in range(numArcs))

#Definicao das variaveis de decisao
Xij = {(i,j) : solver.NumVar(0.0,arcs[i,j][1],'X[%d,%d]'%(i,j)) for i,j in arcs}

#Restricoes
for node in range(numNodes):
    ct = solver.Constraint(nodesSupplies[node], nodesSupplies[node], 'ct%d'%node)
    for i,j in arcs:
        if i == node:
            ct.SetCoefficient(Xij[i,j], 1)
        if j == node:
            ct.SetCoefficient(Xij[i,j], -1)

#Funcao objetivo
objective = solver.Objective()
for i,j in arcs:
    objective.SetCoefficient(Xij[i,j], arcs[i,j][0])
objective.SetMinimization()
solver.Solve()

#Mostra resultado
print('Custo total do sistema = %s'%objective.Value())
for i,j in Xij:
    print(Xij[i,j].name()+" usando "+"{:.2f}".format(Xij[i,j].solution_value())+" da capacidade: "+"{:.2f}".format(Xij[i,j].Ub()))