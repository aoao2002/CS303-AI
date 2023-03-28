import numpy as np
import time
import sys
import re
import copy
import random
import getopt

# file attributes
Name = ""
VERTICES = 0
DEPOT = 0
REQUIRED_EDGES = 0
NON_REQUIRED_EDGES = 0
VEHICLES = 0
CAPACITY = 0
TOTAL_COST_OF_REQUIRED_EDGES = 0
# some constants
SEED = 0
INFINITY = np.inf
NEG_INFINITY = -np.inf
# graph attributes
Nodes = set()  # set of nodes
EDGE_COST = {}  # cost of each edge
DEMAND = {}  # demand of each edge
Shortest_path_matrix = []  # shortest path matrix
Result_path = []  # the final result routing
Total_Cost = 0  # the final result cost
Routing = {}  # the final result routing map
path_cost_map = {}  # each path cost mapping of route
path_load_map = {}  # each path load mapping of route
REQ_ARC = []  # required arcs * 2
# genetic algorithm attributes
POPULATION_SIZE = 1000
population = []
lmd = 0.2
CROSSOVER_RATE = 0.8
alpha = 0.3


def command_line(argv):
    """
    Read the command line arguments and return the filename
    """
    fileName = argv[0]
    Termination, Seed = '', ''
    opts, args = getopt.getopt(argv[1:], "t:s:")
    for opt, arg in opts:
        if opt == '-t':
            Termination = int(arg)
        elif opt == '-s':
            Seed = int(arg)

    return fileName, Termination, Seed


def read_file(filename):
    """
    Read the input file and store the data in the corresponding data structure
    """
    with open(filename, 'r') as f:
        global Name, VERTICES, DEPOT, REQUIRED_EDGES, NON_REQUIRED_EDGES, VEHICLES, CAPACITY, \
            TOTAL_COST_OF_REQUIRED_EDGES
        # Read the first 8 lines
        Name = re.findall(": (.+?)\n", f.readline())[0]
        VERTICES = int(re.findall(": (.+?)\n", f.readline())[0])
        DEPOT = int(re.findall(": (.+?)\n", f.readline())[0])
        REQUIRED_EDGES = int(re.findall(": (.+?)\n", f.readline())[0])
        NON_REQUIRED_EDGES = int(re.findall(": (.+?)\n", f.readline())[0])
        VEHICLES = int(re.findall(": (.+?)\n", f.readline())[0])
        CAPACITY = int(re.findall(": (.+?)\n", f.readline())[0])
        TOTAL_COST_OF_REQUIRED_EDGES = int(re.findall(": (.+?)\n", f.readline())[0])
        # Read the next lines
        read_lines = f.readlines()
        # Preprocess the data
        handle_file_data(read_lines[1:-1])  # pay attention to first line and last line are useless!


def handle_file_data(lines):
    global Shortest_path_matrix, REQ_ARC, Nodes, EDGE_COST, DEMAND
    Nodes = range(1, VERTICES + 1)
    # Initialize the Shortest_path_matrix
    Shortest_path_matrix = np.full((VERTICES + 1, VERTICES + 1), INFINITY)
    np.fill_diagonal(Shortest_path_matrix, 0)
    for line in lines:
        line = line.strip().split()
        node1 = int(line[0])
        node2 = int(line[1])
        cost = int(line[2])
        demand = int(line[3])
        Shortest_path_matrix[node1][node2] = cost
        Shortest_path_matrix[node2][node1] = cost
        EDGE_COST[(node1, node2)] = cost
        EDGE_COST[(node2, node1)] = cost
        if demand:
            REQ_ARC.append((node1, node2))
            REQ_ARC.append((node2, node1))
            DEMAND[(node1, node2)] = demand
            DEMAND[(node2, node1)] = demand


def Floyd():
    """
    Floyd-Warshall algorithm to find the shortest path matrix
    """
    global Shortest_path_matrix
    for k in Nodes:
        for i in Nodes:
            for j in Nodes:
                if Shortest_path_matrix[i][j] > Shortest_path_matrix[i][k] + Shortest_path_matrix[k][j]:
                    Shortest_path_matrix[i][j] = Shortest_path_matrix[i][k] + Shortest_path_matrix[k][j]


def Path_Scanning():
    """
    Path Scanning algorithm to find the required edges
    """
    global Result_path, Total_Cost
    depot = DEPOT
    path = []
    cost = 0
    free = copy.copy(REQ_ARC)  # maybe can use sort optimization
    capacity = CAPACITY
    while free:
        edge = Choose_edge(depot, free, capacity)
        if edge:
            path.append(edge)
            cost += Shortest_path_matrix[depot][edge[0]] + EDGE_COST[edge]
            free.remove(edge)
            free.remove((edge[1], edge[0]))
            capacity -= DEMAND[edge]
            depot = edge[1]
        else:
            path_cost = cost + Shortest_path_matrix[depot][DEPOT]
            path_cost_map[tuple(path)] = path_cost
            path_load_map[tuple(path)] = CAPACITY - capacity
            Total_Cost += path_cost
            Result_path.append(path)
            path = []
            cost = 0
            capacity = CAPACITY
            depot = DEPOT
    if path:
        Result_path.append(path)
        path_cost = cost + Shortest_path_matrix[depot][DEPOT]
        path_cost_map[tuple(path)] = path_cost
        path_load_map[tuple(path)] = CAPACITY - capacity
        Total_Cost += path_cost


def Choose_edge(depot, free, capacity):
    """
    Choose the closest demand edge to the depot
    """
    global Shortest_path_matrix
    min_cost = INFINITY
    min_edges = []
    for edge in free:
        cost_to_depot = Shortest_path_matrix[depot][edge[0]]
        if cost_to_depot == min_cost and DEMAND[edge] <= capacity:
            min_edges.append(edge)
        if cost_to_depot < min_cost and DEMAND[edge] <= capacity:
            min_cost = cost_to_depot
            min_edges = [edge]
    return Five_rules(depot, min_edges, random.randint(1, 5), capacity) if min_edges else None


def Choose_edge_efficient(depot, free, capacity, eff_r):
    """
    Choose the closest demand edge to the depot
    """
    global Shortest_path_matrix
    edges = []
    for edge in free:
        v_i = edge[0]
        v_j = edge[1]
        v_h = depot
        d_i_j = DEMAND[edge]
        SP_h_i = Shortest_path_matrix[v_h][v_i]
        SP_j_0 = Shortest_path_matrix[v_j][DEPOT]
        SP_h_0 = Shortest_path_matrix[v_h][DEPOT]
        c_i_j = EDGE_COST[edge]
        if (SP_h_i + c_i_j + SP_j_0 - SP_h_0) == 0:
            continue
        effRule6 = d_i_j / (SP_h_i + c_i_j + SP_j_0 - SP_h_0)
        if effRule6 >= eff_r and d_i_j <= capacity:
            edges.append(edge)
    return Five_rules(depot, edges, random.randint(1, 5), capacity) if edges else None


def Five_rules(depot, edges, rule, capacity):
    """
    Five rules to choose an edge to go
    """
    #  1. maximize the distance from the task to the depot
    if rule == 1:
        return max(edges, key=lambda x: Shortest_path_matrix[x[1]][depot])
    #  2. minimize the distance from the task to the depot
    if rule == 2:
        return min(edges, key=lambda x: Shortest_path_matrix[x[1]][depot])
    #  3. maximize the term dem(t)/sc(t), where dem(t) and sc(t) are demand and serving cost of task t, respectively
    if rule == 3:
        return max(edges, key=lambda x: DEMAND[x] / EDGE_COST[x])
    #  4. minimize the term dem(t)/sc(t)
    if rule == 4:
        return min(edges, key=lambda x: DEMAND[x] / EDGE_COST[x])
    #  5. use rule 1 if the capacity is greater than half of CAPACITY, otherwise use rule 2
    if rule == 5:
        return max(edges, key=lambda x: Shortest_path_matrix[x[1]][depot]) if capacity > CAPACITY / 2 else min(
            edges, key=lambda x: Shortest_path_matrix[x[1]][depot])
    # #  6. random return
    # if rule == 6:
    #     return random.choice(edges)


def output_result():
    """
    Output the result
    """
    result = "s "
    for path in Result_path:
        result += "0,"
        for edge in path:
            result += "(" + str(edge[0]) + "," + str(edge[1]) + "),"
        result += "0,"
    result = result[:-1]
    result += "\nq " + str(Total_Cost)
    print(result)


def flip(route):
    """
    Flip the edge to find a new routing cost less than origin
    """
    routing = copy.deepcopy(route)
    deta = 0
    for i in range(0, len(routing)):
        path = routing[i]
        for j in range(0, len(path)):
            edge = path[j]
            newEdge = edge[1], edge[0]
            if len(path) == 1:
                deta_cost = Shortest_path_matrix[DEPOT][newEdge[0]] + Shortest_path_matrix[newEdge[1]][DEPOT] \
                            - Shortest_path_matrix[DEPOT][edge[0]] - Shortest_path_matrix[edge[1]][DEPOT]
            else:
                if j == 0:
                    deta_cost = Shortest_path_matrix[DEPOT][newEdge[0]] + Shortest_path_matrix[newEdge[1]][
                        path[j + 1][0]] \
                                - Shortest_path_matrix[DEPOT][edge[0]] - Shortest_path_matrix[edge[1]][path[j + 1][0]]
                elif j == len(path) - 1:
                    deta_cost = Shortest_path_matrix[path[j - 1][1]][newEdge[0]] + Shortest_path_matrix[newEdge[1]][
                        DEPOT] \
                                - Shortest_path_matrix[path[j - 1][1]][edge[0]] - Shortest_path_matrix[edge[1]][DEPOT]
                else:
                    deta_cost = Shortest_path_matrix[path[j - 1][1]][newEdge[0]] + Shortest_path_matrix[newEdge[1]][
                        path[j + 1][0]] \
                                - Shortest_path_matrix[path[j - 1][1]][edge[0]] - Shortest_path_matrix[edge[1]][
                                    path[j + 1][0]]
            if deta_cost < 0:
                # path_origin_cost = path_cost_map[tuple(path)]
                path_origin_load = path_load_map[tuple(path)]
                path[j] = newEdge
                path_load_map[tuple(path)] = path_origin_load
                # path_cost_map[tuple(path)] = path_origin_cost + deta_cost  # path_cost_map maybe have no use?
                deta += deta_cost
    return routing, deta


def Single_Insertion(route):
    """
    Single Insertion to find a new routing cost less than origin
    """
    routing = copy.deepcopy(route)
    deta = 0
    for i in range(0, len(routing)):
        path = routing[i]
        for j in range(0, len(path)):
            edge = path[j]
            for insert_i in range(i, len(routing)):
                insert_path = routing[insert_i]
                for insert_j in range(0, len(insert_path)):
                    insert_edge = insert_path[insert_j]
                    if insert_edge == edge:
                        continue
                    if path == insert_path and insert_j == j - 1:
                        continue
                    if len(path) == 1:
                        if DEMAND[edge] + path_load_map[tuple(insert_path)] > CAPACITY:
                            continue
                        if len(insert_path) == 1:
                            deta_cost = 0
                    else:
                        if path == insert_path:
                            b = 0
                            if j == 0:
                                b = 1
                        else:
                            b = 1
    return routing, deta


def Double_Insertion(route):
    """
    Double Insertion to find a new routing cost less than origin
    """
    routing = copy.deepcopy(route)
    deta = 0
    return routing, deta


def Swap(route):
    """
    Swap the edge to find a new routing cost less than origin
    """
    routing = copy.deepcopy(route)
    deta = 0
    for i in range(0, len(routing)):
        path = routing[i]
        for j in range(0, len(path)):
            edge = path[j]
            for random_i in range(i + 1, len(routing)):
                path_random = routing[random_i]
                for random_j in range(0, len(path_random)):
                    edge_random = path_random[random_j]
                    # while edge_random == edge:
                    #     random_i = random.randint(0, len(routing) - 1)
                    #     path_random = routing[random_i]
                    #     random_j = random.randint(0, len(path_random) - 1)
                    #     edge_random = path_random[random_j]
                    # if edge == edge_random or (random_i == i and random_j < j):
                    #     continue
                    if DEMAND[edge_random] > DEMAND[edge] + CAPACITY - path_load_map[tuple(path)] or \
                            DEMAND[edge] > DEMAND[edge_random] + CAPACITY - path_load_map[tuple(path_random)]:
                        continue
                    if len(path) == 1:
                        deta_cost_p = Shortest_path_matrix[DEPOT][edge_random[0]] + \
                                      Shortest_path_matrix[edge_random[1]][DEPOT] \
                                      + EDGE_COST[edge_random] - Shortest_path_matrix[DEPOT][edge[0]] - \
                                      Shortest_path_matrix[edge[1]][DEPOT] - EDGE_COST[edge]
                    else:
                        if j == 0:
                            deta_cost_p = Shortest_path_matrix[DEPOT][edge_random[0]] + \
                                          Shortest_path_matrix[edge_random[1]][
                                              path[j + 1][0]] \
                                          + EDGE_COST[edge_random] - Shortest_path_matrix[DEPOT][edge[0]] - \
                                          Shortest_path_matrix[edge[1]][path[j + 1][0]] - EDGE_COST[edge]
                        elif j == len(path) - 1:
                            deta_cost_p = Shortest_path_matrix[path[j - 1][1]][edge_random[0]] + Shortest_path_matrix[
                                edge_random[1]][DEPOT] \
                                          + EDGE_COST[edge_random] - Shortest_path_matrix[path[j - 1][1]][edge[0]] - \
                                          Shortest_path_matrix[edge[1]][DEPOT] - EDGE_COST[edge]
                        else:
                            deta_cost_p = Shortest_path_matrix[path[j - 1][1]][edge_random[0]] + Shortest_path_matrix[
                                edge_random[1]][path[j + 1][0]] \
                                          + EDGE_COST[edge_random] - Shortest_path_matrix[path[j - 1][1]][edge[0]] - \
                                          Shortest_path_matrix[edge[1]][path[j + 1][0]] - EDGE_COST[edge]
                    if len(path_random) == 1:
                        deta_cost_r = Shortest_path_matrix[DEPOT][edge[0]] + Shortest_path_matrix[edge[1]][DEPOT] \
                                      + EDGE_COST[edge] - Shortest_path_matrix[DEPOT][edge_random[0]] - \
                                      Shortest_path_matrix[edge_random[1]][DEPOT] - EDGE_COST[edge_random]
                    else:
                        if random_j == 0:
                            deta_cost_r = Shortest_path_matrix[DEPOT][edge[0]] + Shortest_path_matrix[edge[1]][
                                path_random[random_j + 1][0]] \
                                          + EDGE_COST[edge] - Shortest_path_matrix[DEPOT][edge_random[0]] - \
                                          Shortest_path_matrix[edge_random[1]][path_random[random_j + 1][0]] - \
                                          EDGE_COST[edge_random]
                        elif random_j == len(path_random) - 1:
                            deta_cost_r = Shortest_path_matrix[path_random[random_j - 1][1]][edge[0]] + \
                                          Shortest_path_matrix[edge[1]][DEPOT] + EDGE_COST[edge] \
                                          - Shortest_path_matrix[path_random[random_j - 1][1]][edge_random[0]] - \
                                          Shortest_path_matrix[edge_random[1]][DEPOT] - EDGE_COST[edge_random]
                        else:
                            deta_cost_r = Shortest_path_matrix[path_random[random_j - 1][1]][edge[0]] + \
                                          Shortest_path_matrix[edge[1]][path_random[random_j + 1][0]] \
                                          + EDGE_COST[edge] - Shortest_path_matrix[path_random[random_j - 1][1]][
                                              edge_random[0]] - \
                                          Shortest_path_matrix[edge_random[1]][path_random[random_j + 1][0]] - \
                                          EDGE_COST[edge_random]
                    deta_cost = deta_cost_p + deta_cost_r
                    if deta_cost < 0:
                        path_origin_load = path_load_map[tuple(path)]
                        # path_origin_cost = path_cost_map[tuple(path)]
                        path[j] = edge_random
                        path_load_map[tuple(path)] = path_origin_load + DEMAND[edge_random] - DEMAND[edge]
                        # path_cost_map[tuple(path)] = path_origin_cost + deta_cost_p
                        path_origin_load = path_load_map[tuple(path_random)]
                        # path_origin_cost = path_cost_map[tuple(path_random)]
                        path_random[random_j] = edge
                        path_load_map[tuple(path_random)] = path_origin_load + DEMAND[edge] - DEMAND[edge_random]
                        # path_cost_map[tuple(path_random)] = path_origin_cost + deta_cost_r
                        deta += deta_cost
                        edge = edge_random
    return routing, deta


def Two_opt_single_path(route):
    """
    2-opt in one path to find a new routing cost less than origin
    """
    routing = copy.deepcopy(route)
    deta = 0
    for w in range(0, len(routing)):
        path = routing[w]
        if len(path) == 1:
            continue
        for i in range(len(path) - 1):
            for j in range(i + 1, len(path)):
                if j - i == 1:
                    continue
                deta_cost = Shortest_path_matrix[path[i][1]][path[j - 1][1]] + Shortest_path_matrix[path[i + 1][0]][
                    path[j][0]] - Shortest_path_matrix[path[i][1]][path[i + 1][0]] - Shortest_path_matrix[path[
                    j - 1][1]][path[j][0]]
                if deta_cost < 0:
                    path_origin_load = path_load_map[tuple(path)]
                    # the edges between edge i and edge j is reversed , and the endpoint of the edge exchanged
                    path[i + 1:j] = path[j - 1:i:-1]
                    path = [(t[1], t[0]) if k in range(i + 1, j) else t for k, t in enumerate(path)]
                    routing[w] = path
                    path_load_map[tuple(path)] = path_origin_load
                    deta += deta_cost
    return routing, deta


def Two_opt_double_path(routing):
    """
    2-opt in two different paths to find a new routing cost less than origin
    """
    route = copy.deepcopy(routing)
    deta = 0
    for r1 in range(0, len(routing)):
        for r2 in range(r1 + 1, len(routing)):
            path1 = routing[r1]
            path2 = routing[r2]
            if len(path1) == 1 or len(path2) == 1:
                continue
            for i in range(len(path1) - 1):
                for j in range(len(path2) - 1):
                    path1_f = path1[:i + 1]
                    path1_b = path1[i:]
                    path2_f = path2[:j + 1]
                    path2_b = path2[j:]
                    path1_con = path1_f + path2_b
                    path2_con = path2_f + path1_b
                    path1_con_load = calculate_load(path1_con)
                    path2_con_load = calculate_load(path2_con)
                    if path1_con_load > CAPACITY or path2_con_load > CAPACITY:
                        continue
                    deta_cost = calculate_cost(path1_con) + calculate_cost(path2_con) - \
                                calculate_cost(path1) - calculate_cost(path2)
                    if deta_cost < 0:
                        path_load_map[tuple(path1_con)] = path1_con_load
                        path_load_map[tuple(path2_con)] = path2_con_load
                        route[r1] = path1_con
                        route[r2] = path2_con
                        deta += deta_cost
    return route, deta


def verify_cost_and_load(routing):
    """
    verify the cost and load of routing
    """
    cost = 0
    load = 0
    for path in routing:
        for i in range(len(path)):
            if i == 0:
                cost += Shortest_path_matrix[DEPOT][path[i][0]] + EDGE_COST[path[i]]
                load += DEMAND[path[i]]
            else:
                cost += Shortest_path_matrix[path[i - 1][1]][path[i][0]] + EDGE_COST[path[i]]
                load += DEMAND[path[i]]
                if i == len(path) - 1:
                    cost += Shortest_path_matrix[path[i][1]][DEPOT]
    return cost, load


def calculate_load(path):
    """
    calculate the load of path
    """
    load = 0
    for edge in path:
        load += DEMAND[edge]
    return load


def calculate_cost(path):
    """
    calculate the cost of path
    """
    cost = 0
    for i in range(len(path)):
        if i == 0:
            cost += Shortest_path_matrix[DEPOT][path[i][0]] + EDGE_COST[path[i]]
        else:
            cost += Shortest_path_matrix[path[i - 1][1]][path[i][0]] + EDGE_COST[path[i]]
            if i == len(path) - 1:
                cost += Shortest_path_matrix[path[i][1]][DEPOT]
    return cost


def init_population():
    """
    Initialize the population
    """
    global Result_path, Total_Cost, population
    population = []
    for i in range(1, POPULATION_SIZE + 1):
        Result_path = []
        Total_Cost = 0
        Path_Scanning()
        population.append(Result_path)
        if Total_Cost in Routing.keys():
            continue
        Routing[Total_Cost] = Result_path


def crossover():
    """
    crossover
    """
    global population
    new_population = []
    for i in range(0, POPULATION_SIZE):
        new_population.append(population[i])
    for i in range(0, POPULATION_SIZE):
        if random.random() < CROSSOVER_RATE:
            j = random.randint(0, POPULATION_SIZE - 1)
            while j == i:
                j = random.randint(0, POPULATION_SIZE - 1)
            new_population[i] = crossover_path(population[i], population[j])
    population = new_population


def crossover_path(route1, route2):
    """
    crossover two routes
    """
    route1 = copy.deepcopy(route1)
    route2 = copy.deepcopy(route2)
    i = random.randint(0, len(route1))
    j = random.randint(0, len(route2))

    return route1, route2


def local_search():
    """
    local search
    """
    a = 1


def choose_best_opt(route, cost):
    """
    choose the best opt
    """
    r1, deta1 = flip(route)
    r2, deta2 = Swap(route)
    r3, deta3 = Two_opt_single_path(route)
    r4, deta4 = Two_opt_double_path(route)
    min_deta = min(deta1, deta2, deta3, deta4)
    if min_deta == deta1:
        return r1, cost + deta1
    elif min_deta == deta2:
        return r2, cost + deta2
    elif min_deta == deta3:
        return r3, cost + deta3
    else:
        return r4, cost + deta4


def is_clone(r, R):
    for r_ in R:
        if r == r_:
            return True
    return False


if __name__ == "__main__":
    start = time.time()
    file_name, termination, seed = command_line(sys.argv[1:])
    SEED = seed
    read_file(file_name)
    Floyd()
    Path_Scanning()
    # add routing
    Routing[Total_Cost] = Result_path
    # output_result()
    while (time.time() - start) < termination - 1:
        Result_path = []
        Total_Cost = 0
        Path_Scanning()
        Routing[Total_Cost] = Result_path

        # less_routing = Routing[min(Routing.keys())]
        # less_Cost = min(Routing.keys())

        new_routing, new_cost = choose_best_opt(Result_path, Total_Cost)
        Routing[new_cost] = new_routing

        # cost, load = verify_cost_and_load(new_routing)
        # if cost != new_cost:
        #     print(cost, new_cost)
        #     print(load)
    Result_path = Routing[min(Routing.keys())]
    Total_Cost = int(min(Routing.keys()))
    output_result()
