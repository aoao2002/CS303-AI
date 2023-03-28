/*
	Author: Rafael Kendy Arakaki
	Advisor: Prof. Fábio Luiz Usberti
    Institute of Computing - University of Campinas, Brazil

	Title: Hybrid Genetic Algorithm for the CARP (Capacitated Arc Routing Problem)
    Description: A genetic algorithm hybridized with local search routine.
    This algorithm works with a chromosome that represents a non-capacitated route (we call it non-capacitataed chromosome). Solutions are obtained through Split procedure.
    *** We may use a special algorithm (based on feazibilization @OCARP) to fix "good routes" regarding cost and re-optimizing the remaining routes. ***
    **  The criteria for "good routes" can be based on distance from depot node of first/ending tasks AND/OR the capacity usage of the routes. **
    The local search uses a deconstruct/reconstruct approach that improves the solution cost by deconstructing and reconstructing some of the routes of an CARP solution.
	The local search also takes advantage from the Split algorithm.
*/

#ifndef CARP_HGA_MAIN_H
#define CARP_HGA_MAIN_H

#include <iostream>
#include <sstream>
#include <fstream>
#include <climits>
#include <cstring>
#include <ctime>
#include <set>
#include <functional>
#include <algorithm>
#include <map>
#include <queue>
#include <lemon/list_graph.h>
#include <lemon/euler.h>
#include <lemon/dijkstra.h>
#include <iomanip>
#include <unordered_map>

using namespace lemon;
using namespace std;

// ================================================== CONSTANTS AND PARAMETERS ============================================================


// The "DEBUG" flag turns ON debug printfs
#define DEBUG

#ifdef DEBUG
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

// Static sizes
#define GENE_SIZE 500

// ================================================== DATA STRUCTS ============================================================

// Experiments data for an algorithm
typedef struct debugData {
    vector<vector<vector<int>>> sols;
    vector<int> costs, vhcs;
    double best, median, avg, stddev;
    double timeAll, timeBest;
    double top1percent_avg, top1percent_median;
    double best_vhc, avg_vhc, stddev_vhc;
    // double convergenceRatio; // number of similar edges / number of total edges
    clock_t inicio;
} DebugData;

// ================================================== GLOBAL VARIABLES ============================================================

// Instance data
extern int nTasks;
extern int nNodes, nEdges;
extern int nReqEdges;
extern int maxroutes;
extern int capacity;
extern int depotNodeId;

// Graph data
extern ListGraph *g;
extern ListGraph::EdgeMap<int> *custoAresta;
extern ListGraph::EdgeMap<int> *demandaAresta;
extern ListGraph::Node depotNode;

// Preprocessed/Computed graph data
extern vector< vector<int> > spG; // Shortest path on G
extern int **D;
extern int **R;
extern map<int, ListGraph::Edge> edgeFromTask;
extern map<int, int> custo;
extern map<int, int> demanda;
extern int lb0_value;

// Execution parameters
extern int LIMITE_EXECS;

// Literatura
void path_scanning_totalrandom(const vector<int> &vecTasks, vector<vector<int>> &output_sol, int k);
int path_scanning_randomcriteria(const vector<int> &vecTasks, vector<vector<int>> &output_sol, int k);

// Principais
void path_scanning_ellipse (const vector<int> &vecTasks, vector<vector<int>> &output_sol, double alfa); // Santos et al. (2009)
void path_scanning_ellipseEficiencia_TDnear(const vector<int> &vecTasks, vector<vector<int>> &output_sol, double alfa, double beta);
void path_scanning_ellipseEficiencia_TDall(const vector<int> &vecTasks, vector<vector<int>> &output_sol, double alfa);
void path_scanning_ellipseDisparoNovoDefinicaoAntiga (const vector<int> &vecTasks, vector<vector<int>> &output_sol, double alfa, double beta);

// util functions
extern int printsQtde;
int countVehicle(const vector<vector<int>> &sol);

// experiment data functions
void initInfo(DebugData& info);
void updateInfo(DebugData& info, const vector<vector<int>> &sol);
void finishInfo(DebugData& info);
void printInfo(const DebugData& info);
void loopNaoEvolutivo();
void small_framework();

// ================================================== METHODS AND FUNCTIONS ============================================================

// Preprocessing
void pre_processamento();
inline int pos (int task);
inline int sp (int p, int q, int **T = D); // To access shortest path between tasks
inline pair<int,int> getNodes (int task); // To access nodes from tasks
inline pair<int,int> getNodes (ListGraph::Edge e);
inline int costFromDepot(int p, int **T); // To access travel cost from depot to tasks
inline int costToDepot(int p, int **T);

// Objective functions
int objective_function(const vector<vector<int>> solucao );
int obj_function_parcial(const vector<vector<int>> &solucao );
int obj_function_parcial(const vector<const vector<int>*> solucao );

// Reading functions
void readFile(const string filePath, ListGraph &g, ListGraph::EdgeMap<int> &custo, ListGraph::EdgeMap<int> &demanda);

// Random number generators
double frandom(); // Real \in [0, 1]
int irandom(int a); // Integer \in [0, a]


#endif //CARP_HGA_MAIN_H
