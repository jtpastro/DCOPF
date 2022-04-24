#!/usr/bin/python3
from ortools.linear_solver import pywraplp

arcCost = lambda source, target, cost, capacity: ((int(source), int(target)), (float(cost), float(capacity)))

#Instancia solver linear
solver = pywraplp.Solver('simple_lp_program', pywraplp.Solver.GLOP_LINEAR_PROGRAMMING)

#Parametrização
numNodes = int(input())
numArcs = int(input())
nodesSupplies = [float(input()) for _ in range(numNodes)]
arcs = dict(arcCost(*input().split()) for _ in range(numArcs))

#Definição das variáveis de decisão
Xij = {(i,j) : solver.NumVar(0.0,arcs[i,j][1],'X[%d,%d]'%(i,j)) for i,j in arcs}


#Restrições
for node in range(numNodes):
    ct = solver.Constraint(nodesSupplies[node], nodesSupplies[node], 'ct%d'%node)
    for i,j in arcs:
        if i == node:
            ct.SetCoefficient(Xij[i,j], 1)
        if j == node:
            ct.SetCoefficient(Xij[i,j], -1)


#Função objetivo
objective = solver.Objective()
for i,j in arcs:
    objective.SetCoefficient(Xij[i,j], arcs[i,j][0])
objective.SetMinimization()
solver.Solve()

#Mostra resultado
print('Custo total do sistema = %s'%objective.Value())
for i,j in Xij:
    print(Xij[i,j].name()+" usando "+"{:.2f}".format(Xij[i,j].solution_value())+" da capacidade: "+"{:.2f}".format(Xij[i,j].Ub()))

        