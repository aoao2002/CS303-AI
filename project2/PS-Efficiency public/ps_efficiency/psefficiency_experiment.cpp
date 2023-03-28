 /*
	Author: Rafael Kendy Arakaki - rafael.arakaki@ic.unicamp.br
	Advisor: Prof. Fábio Luiz Usberti - fusberti@ic.unicamp.br
    Institute of Computing - University of Campinas, Brazil

    == Attention ==

    This file contains the source codes for the computational experiment presented in [1].

    1) All the source codes are free to use. You may modify it as you want.
    2) The author does not have any responsibility for any consequences of using these codes.
    3) Please provide references to the authors' publications in works using part of these codes.
    4) For clarifications or collaborations please contact us by email.
    5) Thanks.

    [1]: "An efficiency-based path-scanning heuristic for the capacitated arc routing problem.", Arakaki and F. L. Usberti,
    Submitted for Computers and Operations Research, 2018.

	Description: an enchanced path-scanning heuristic is proposed for CARP.
    Based on the PS-ER (path-scanning with ellipse rule) from Santos et al. (2009) we propose an improved heuristic.
    Similar to PS-ER, the restricting rule is activated by a trigger rule when the residual vehicle capacity is low enough.
    After that, the method uses a load-by-distance efficiency rule to decide whether an edge should be serviced or not.
    The rule filters the edges considering the efficiency indices of the current vehicle and the edges.
    The efficiency index of the route is given by the serviced demands over the distance traveled,
    while the efficiency index of an edge is given by its demand over the distance that the current route need to travel in order to service the edge.
    The method reported significantly better results than previous path-scanning heuristics.
*/

// Final version 1.0
#include <queue>
#include <chrono>
#include "psefficiency_experiment.h"

// Instance data
int nTasks;
int nNodes, nEdges;
int nReqEdges;
int maxroutes;
int capacity;
int depotNodeId;

// Graph Data
ListGraph *g = new ListGraph;
ListGraph::EdgeMap<int> *custoAresta;
ListGraph::EdgeMap<int> *demandaAresta;
vector<vector<int> > spG; // Shortest path between arcs
int lb0_value;
int lb_known;
int offset; // some val instances need offset for lowerbound

// Tasks Data (tasks == required arcs)
map<int, ListGraph::Edge> edgeFromTask;
map<int, int> custo;
map<int, int> demanda;
int **D = nullptr;
int **R;

// Execution parameters
int LIMITE_EXECS = 1000;

// Experiment data - algorithms
int NUM_ALGORITHMS;
int NUM_ALGORITHMS_PARAMETERS;
char instance_name[300][60];
int method_nbest[30];
int method_bestResults[30][300];
double method_bestGAPResults[30][300];
double method_top1pctavg[30];
double method_bestGap[30];
double method_medianGap[30];
double method_averageCPU[30];

// Experiment data - instances
vector<pair<double,int>> list_parameters;
char strname[100];
char instancelist[100];
int nInstancesProcessed;

// Usage information
void showUsage (){
    cout << "Usage: ./psefficiency_experiment <Instances FileName> {-k <num_iterations>}" << endl;
    cout << "Default values: -k 1000" << endl;
}

int main(int argc, char *argv[]) {
    cout << "------------" << endl << "Path-scanning with efficiency rule (PS-Efficiency) experiment." << endl
    << "Version: 1.0" << endl << "------------"
    << endl << endl;
    cout << "C++ version: " << __cplusplus << endl;

    // De-activate when debugging
    //srand (time(NULL));

    // Input arguments checking
    if (argc < 1) {
        cout << "Parameters missing." << endl;
        showUsage();
        exit(1);
    }

    // extras zero
    for(int i = 0; i < 30; i++){
        method_top1pctavg[i] = method_bestGap[i] = method_averageCPU[i] = 0.0;
        method_medianGap[i] = 0.0;
        method_nbest[i] = 0;
        for(int j = 0; j < 300; j++){
            method_bestResults[i][j] = INT_MAX;
            method_bestGAPResults[i][j] = INT_MAX;
        }
    }
    nInstancesProcessed = 0;

    // Reading program arguments
    for(int i = 2; i < argc; i++){
        const string arg(argv[i]);
        string next;
        if((i+1) < argc)
            next = string(argv[i+1]);
        else
            next = string("");

        if( arg.find("-k") == 0 && next.size() > 0){
            LIMITE_EXECS = atoi(next.c_str()); i++; continue;
        }
        // Report invalid parameter
        cout << "Invalid parameter: \"" << arg << "\"" << "" << endl;
        showUsage();
        exit(1);
    }

    cout << "*** Path-scanning experiment STARTING *** " << endl << "-------------------------------------------------------" << endl << endl;

    // Date and time
    time_t rawtime;
    struct tm * timeinfo;
    char bufferTimeDate[80];

    time (&rawtime);
    timeinfo = localtime(&rawtime);

    strftime(bufferTimeDate, sizeof(bufferTimeDate), "%d-%m-%Y %I:%M:%S", timeinfo);

    // Data out preparation
    strncpy(instancelist, argv[1], 100);

    // Instance list
    FILE *lista = fopen(argv[1], "r");
    if(lista == NULL){
        cout << "Instance list file not found. File name: " << argv[1] << endl;
        exit(1);
    }
    lb_known = 0;
    offset = 0;
    fscanf(lista, " %s %d %d", strname, &lb_known, &offset);
    while (strcmp(strname, "END") != 0) {
        // debug
        printf("Reading instance %s with %d known lower bound, %d offset.\n", strname, lb_known, offset);

        // Find next instance
        char filepath[100];
        sprintf(filepath, "./instancias-CARP/%s", strname);
        std::string sfile(filepath);
        strncpy(instance_name[nInstancesProcessed], strname, 60);

        // Allocate instance objects
        g = new ListGraph;
        custoAresta = new ListGraph::EdgeMap<int>(*g);
        demandaAresta = new ListGraph::EdgeMap<int>(*g);

        // Instance reading
        readFile(sfile, *g, *custoAresta, *demandaAresta);

        // complete info
        printf("Processing instance %s with %d known lower bound. (|E_R| = %d, M* = %d):\n", strname, lb_known, nReqEdges, maxroutes);
        printf("(Obs: for CARP M* is not used.)\n");

        // Pre-processing (shortest path matrix between required arcs)
        pre_processamento();

        // Experiment framework
        small_framework();

        // Memory deallocation
        for (int i = 0; i <= nTasks; i++){
            delete[] D[i];
            delete[] R[i];
        }
        delete[] D;
        delete[] R;
        nTasks = nNodes = nEdges = nReqEdges = 0;
        delete custoAresta;
        delete demandaAresta;
        delete g;
        edgeFromTask.clear();
        custo.clear();
        demanda.clear();
        
        // Next instance
        lb_known = 0; offset = 0;
        fscanf(lista, " %s %d %d", strname, &lb_known, &offset);
        //break;
    }
    fclose(lista);

    // End of experiment
    printf("***EXPERIMENT EXITING***\n");
    return 0;
}

double TIME_ELAPSED (clock_t after, clock_t before){
    return double(after - before) / CLOCKS_PER_SEC;
}

void small_framework() {
    // Population init
    loopNaoEvolutivo();

    multimap<double, int> placement;

    for (int i = 0; i < NUM_ALGORITHMS; i++) {
        placement.insert(make_pair(method_bestGap[i] / nInstancesProcessed, i));
    }

    // Output experiment data file
    char fileout[100];
    sprintf(fileout, "results_%s.txt", instancelist);
    FILE *out = fopen(fileout, "w");
    if (out == NULL) {
        printf("Could not write the data results file (%s)\n.", fileout);
        exit(1);
    }
    fprintf(out, "Up to instance %s (%d instances processed):\n\n", strname, nInstancesProcessed);

    // Imprime alguns dados extras do loop nao evolutivo
    printf("LoopNaoEvolutivo() Data:\n");
    for (int i = 0; i < NUM_ALGORITHMS; i++) {
        if (i >= 0 && i < NUM_ALGORITHMS_PARAMETERS) {
            printf("Alg%d: MethodID%d(%.2f): n_best %d/%d, avg_gap(best)=%.2f, avg_gap(avg_top10%%)=%.2f,\t avg_gap(median)=%.2f\n",
                   i + 1, list_parameters[i].second, list_parameters[i].first, method_nbest[i], nInstancesProcessed,
                   method_bestGap[i] / nInstancesProcessed, method_top1pctavg[i] / nInstancesProcessed,
                   method_medianGap[i] / nInstancesProcessed);
            fprintf(out,
                    "Alg%d: MethodID%d(%.2f): n_best %d/%d, avg_gap(best)=\t%.2f\t, avg_gap(avg_top10%%)=%.2f, avg_gap(median)=%.2f\n",
                    i + 1, list_parameters[i].second, list_parameters[i].first, method_nbest[i], nInstancesProcessed,
                    method_bestGap[i] / nInstancesProcessed, method_top1pctavg[i] / nInstancesProcessed,
                    method_medianGap[i] / nInstancesProcessed);
        }
    }
    int i1 = -1, i2 = -1;
    for (auto par : placement) {
        if (i1 == -1) { i1 = par.second; }
        else if (i2 == -1) { i2 = par.second; }
        else break;
    }
    printf("\nPrint to table GAPs:\n");
    for (int i = 0; i < NUM_ALGORITHMS; i++) {
        if (i >= 0 && i < NUM_ALGORITHMS_PARAMETERS) {
            printf("%.2f\t", method_bestGap[i] / nInstancesProcessed);
        }
    }
    printf("\n\n");


    printf("\nCPU Times::     ");
    for (int i = 0; i < NUM_ALGORITHMS; i++) {
        if (i >= 0 && i < NUM_ALGORITHMS_PARAMETERS) {
            printf("%.3f\t", method_averageCPU[i] / nInstancesProcessed);
        }
    }
    printf("\t    (in seconds. Sum of all K=%d executions. Averaged over all instances.)\n\n", LIMITE_EXECS);

    printf("\n*** Best methods:\n");
    if( i1 < NUM_ALGORITHMS_PARAMETERS){
    printf("Method%d: MethodID%d(%.2f): n_best %d/%d, avg_gap(best)=%.2f, avg_gap(avg_top10%%)=%.2f,\t avg_gap(median)=%.2f\n",
           i1 + 1, list_parameters[i1].second, list_parameters[i1].first, method_nbest[i1], nInstancesProcessed,
           method_bestGap[i1] / nInstancesProcessed, method_top1pctavg[i1] / nInstancesProcessed,
           method_medianGap[i1] / nInstancesProcessed);
    }
    if( i2 < NUM_ALGORITHMS_PARAMETERS){
    printf("Method%d: MethodID%d(%.2f): n_best %d/%d, avg_gap(best)=%.2f, avg_gap(avg_top10%%)=%.2f,\t avg_gap(median)=%.2f\n",
           i2 + 1, list_parameters[i2].second, list_parameters[i2].first, method_nbest[i2], nInstancesProcessed,
           method_bestGap[i2] / nInstancesProcessed, method_top1pctavg[i2] / nInstancesProcessed,
           method_medianGap[i2] / nInstancesProcessed);
    }
    printf("***\n");

    // All data (each instance)
    fprintf(out, "\nPrinting data from all instances (%d instances) (numExec= %d):\n\n", nInstancesProcessed, LIMITE_EXECS);
    fprintf(out, "Printing Sol-Costs:\n");
    fprintf(out, "Algorithms:\t");
    for (int i = 0; i < NUM_ALGORITHMS; i++) {
        if (i >= 0 && i < NUM_ALGORITHMS_PARAMETERS) {
            fprintf(out, "Alg%d\t", i + 1);
        }
        else{
            fprintf(out, "Met%d\t", i + 1);
        }
    }
    for (int j = 0; j < nInstancesProcessed; j++) {
        fprintf(out, "\n%s:\t", instance_name[j]);
        for (int i = 0; i < NUM_ALGORITHMS; i++) {
            fprintf(out, "%d\t", method_bestResults[i][j]);
        }
    }

    // All data (each instance)
    fprintf(out, "\n\nPrinting GAPs:\n");
    fprintf(out, "Algorithms:\t");
    for (int i = 0; i < NUM_ALGORITHMS; i++) {
        if (i >= 0 && i < NUM_ALGORITHMS_PARAMETERS) {
            fprintf(out, "Alg%d\t", i + 1);
        }
        else{
            fprintf(out, "Met%d\t", i + 1);
        }
    }
    for (int j = 0; j < nInstancesProcessed; j++) {
        fprintf(out, "\n%s:\t", instance_name[j]);
        for (int i = 0; i < NUM_ALGORITHMS; i++) {
            fprintf(out, "%.2f\t", method_bestGAPResults[i][j]);
        }
    }

    // Finish data file
    fclose(out);
}


void loopNaoEvolutivo() {
    // Inicializações para as heurísticas
    vector<int> vectasks;
    double sum_td = 0.0, sum_tc = 0.0;
    for (int i = 1; i <= nReqEdges; i++) {
        vectasks.push_back(i);
        sum_td += demanda.at(i);
        sum_tc += custo.at(i);
    }
    const double init_td = sum_td, init_tc = sum_tc;

    const int NUM_RUNS = LIMITE_EXECS != 0 ? LIMITE_EXECS : 100;
    const int START_ALGO = 0;

    // Parâmetros dos algoritmos executados
    const vector<double> alfa_parameters = { 1.0, 1.5, 2.0, 2.5, 3.0, 3.5 }; // best set = { 1.0, 1.5, 2.0, 2.5, 3.0, 3.5  };
    const vector<int> alg_parameter =  { 1, 2, 3, 4 }; // algorithm "ID"

    list_parameters = vector<pair<double,int>>();
    list_parameters.clear();

    // the code below is used to run each heuristic with varying parameters declared in alfa_paramters.
    //for( auto alg : alg_parameter ){
    //    if( alg <= 10 ){
    //        for( auto alfa : alfa_parameters ){
    //            list_parameters.push_back(make_pair(alfa, alg));
    //        }
    //    }
    //    else{
    //        list_parameters.push_back(make_pair(0.0, alg));
    //    }
    //}

    // Each method with their best-fit value of parameter \alpha
    list_parameters.push_back( make_pair(1.5, 1)); // PS-Ellipse(a=1.5) from Santos et al.
    list_parameters.push_back( make_pair(3.0, 2)); // PS-Efficiency(a=3.0) our heuristic.
    list_parameters.push_back( make_pair(3.0, 3)); // PS-Alt1(a=3.0) alternative 1
    list_parameters.push_back( make_pair(1.0, 4)); // PS-Alt2(a=1.0) alternative 2

    NUM_ALGORITHMS_PARAMETERS = (int) list_parameters.size();
    NUM_ALGORITHMS = (int) list_parameters.size(); //  + 1
    DebugData infoConstructed[NUM_ALGORITHMS];

    const double beta_parameter = 1.0; // used by near() function of 'path_scanning_ellipseEficiencia_TDnear' and 'path_scanning_ellipseDisparoNovoDefinicaoAntiga'.
                                       // not an actual parameter!

    const int K = 1;    // This is a experiment parameter that makes some methods to run 'K' solutions internally and return just the best found solution.
                        // since we are interested in taking each solution costs for statistics we are not using it.
    PRINTF("Running %d solutions. (K=%d)\n", NUM_RUNS, K);
    PRINTF("Running %d algorithms with combination of pre-selected parameters. )\n", (int) list_parameters.size());


    for(int i = START_ALGO; i < NUM_ALGORITHMS; i++){
        initInfo(infoConstructed[i]);

        // deterministic random number generation: the same for each algorithm
        srand(0);

        // Running path scanning methods repeated times
        for(int myCounter = 0; myCounter < NUM_RUNS; myCounter++){
            // Step 1: Construction path scanning
            vector< vector<int>> solucao;
            if( list_parameters.at(i).second == 1 ){
                path_scanning_ellipse(vectasks, solucao, list_parameters.at(i).first);
            }
            else if ( list_parameters.at(i).second == 2 ){
                path_scanning_ellipseEficiencia_TDnear(vectasks, solucao, list_parameters.at(i).first, beta_parameter);
            }
            else if ( list_parameters.at(i).second == 3 ){
                path_scanning_ellipseEficiencia_TDall(vectasks, solucao, list_parameters.at(i).first);
            }
            else if ( list_parameters.at(i).second == 4 ){
                path_scanning_ellipseDisparoNovoDefinicaoAntiga(vectasks, solucao, list_parameters.at(i).first, beta_parameter);
            }
            else if ( list_parameters.at(i).second == 100 ){
                path_scanning_randomcriteria(vectasks, solucao, K);
            }
            else if ( list_parameters.at(i).second == 200 ){
                path_scanning_totalrandom(vectasks, solucao, K);
            }
            else{
                printf("Erro! Algoritmo invalido! (1)\n");
                exit(1);
            }
            // Step 2: Extracting solution generated for statistics
            updateInfo(infoConstructed[i], solucao);
        }
        finishInfo(infoConstructed[i]);
    }

    // Print info
    for(int i = 0; i < NUM_ALGORITHMS; i++){
        if( i >= 0 && i < NUM_ALGORITHMS_PARAMETERS ){
            PRINTF("### Constructed Info %d ### PS-algorithm%d(%.2f)\n", i+1, list_parameters[i].second, list_parameters[i].first);
            printInfo(infoConstructed[i]);
            PRINTF("\n");
        }
        else{
            printf("Algoritmo invalido !!(2)\n");
            exit(1);
        }

        bool flag = true;
        for(int j = 0; j < NUM_ALGORITHMS; j++){
            if( infoConstructed[j].best < infoConstructed[i].best){
                flag = false;
            }
        }
        if( flag ){
            method_nbest[i]++;
        }
        // avg gap of average of top 1% solutions
        method_top1pctavg[i] += 100.00*((double) (infoConstructed[i].top1percent_avg+offset)  - lb_known)/lb_known;
        // avg gap of median cost
        method_medianGap[i] += 100.00*((double) (infoConstructed[i].median+offset)  - lb_known)/lb_known;
        // avg gap of best cost
        method_bestGap[i] +=   100.00*((double) (infoConstructed[i].best+offset)   - lb_known)/lb_known;
        // save best solution
        method_bestResults[i][nInstancesProcessed] = (int) infoConstructed[i].best;
        // save best gap
        method_bestGAPResults[i][nInstancesProcessed] = 100.00*((double) (infoConstructed[i].best+offset)   - lb_known)/lb_known;
        // save average CPU time
        method_averageCPU[i] += infoConstructed[i].timeAll;
    }
    // n instances
    nInstancesProcessed++;
}

// Random a integer random number in [0, a]
 int irandom(int a) {
     if (a <= 0) return 0;
     else return rand() % (a + 1);
 }

// Returns a float random number in [0,1].
 double frandom() {
     return rand() / double(RAND_MAX);
 }

// Calculates objective function (traversal cost) of a partial solution.
 int obj_function_parcial(const vector<vector<int>> &solution) {
     int cost = 0;
     set<int> conj;
     for (int r = 0; r < solution.size(); r++) {
         if( solution[r].size() > 0){
             int sum = 0, cap = 0;
             cap = demanda[solution[r][0]];
             sum = custo[solution[r][0]];
             if (conj.count(solution[r][0]) > 0) {
                 printf("Error: Route %d contains a repeated task! (task: %d)\n", r,
                        solution[r][0]);
                 exit(1);
             }
             conj.insert(solution[r][0]);
             for (int i = 1; i < solution[r].size(); i++) {
                 sum += sp(solution[r][i - 1], solution[r][i], D);
                 int taskid = solution[r][i];
                 //cout << taskid << "(" << nReqEdges << ")" << endl;
                 cap += demanda[taskid];
                 sum += custo[taskid];
                 if (conj.count(solution[r][i]) > 0) {
                     printf("Error: Route %d contains a repeated task! (task: %d)\n", r,
                            solution[r][i]);
                     exit(1);
                 }
                 conj.insert(solution[r][i]);
             }

             // Return cost to depot// Start cost from depot
             int costToAndFromDepot = 0;
             costToAndFromDepot += costFromDepot(solution[r][0], D);
             costToAndFromDepot += costToDepot(solution[r][solution[r].size()-1], D);
             sum += costToAndFromDepot;

             if (cap > capacity) {
                 printf("Error: Capacity exceed in the route (index = %d)\n", r);
                 exit(1);
             }
             cost += sum;
         }
     }

     // Nao da para verificar integridade de conj() aqui porque este metodo é parcial.
     return cost;
 }

// Calculates objective function (traversal cost) of a partial solution.
 int obj_function_parcial(const vector<const vector<int> *> solution) {
     int cost = 0;
     for (int r = 0; r < solution.size(); r++) {
         if(solution[r]->size() > 0){
             int sum = 0, cap = 0;
             cap = demanda[solution[r]->at(0)];
             sum = custo[solution[r]->at(0)];
             for (int i = 1; i < solution[r]->size(); i++) {
                 sum += sp(solution[r]->at((unsigned long) (i - 1)), solution[r]->at((unsigned long) i), D);
                 int taskid = solution[r]->at((unsigned long) i);
                 //cout << taskid << "(" << nReqEdges << ")" << endl;
                 cap += demanda[taskid];
                 sum += custo[taskid];
             }
             sum += costFromDepot(solution[r]->at(0), D);
             sum += costToDepot(solution[r]->at(solution[r]->size()-1), D);
             if (cap > capacity) {
                 printf("Error: Capacity exceed in the route (index = %d)\n", r);
                 exit(1);
             }
             cost += sum;
         }
     }
     return cost;
 }

// Calculates objective function (traversal cost) of a complete CARP solution and verify its feasibility.
 int objective_function(const vector<vector<int>> solution) {
     int cost = 0;
     set<int> conj;
     for (int r = 0; r < solution.size(); r++) {
         if(solution[r].size() > 0) {
             int sum = 0, cap = 0;
             cap = demanda[solution[r][0]];
             if (conj.count(abs(solution[r][0]) ) > 0) {
                 printf("Error: Route %d contains a repeated task/edge! (task: %d)\n", r,
                        solution[r][0]);
                 exit(1);
             }

             conj.insert(abs(solution[r][0]));
             ///printf("%d ", solution[r][0]);///

             for (int i = 1; i < solution[r].size(); i++) {
                 sum += sp(solution[r][i - 1], solution[r][i], D);
                 cap += demanda[solution[r][i]];
                 // ASSERT
                 //printf(" %d.%d, ", sum, cap);//
                 if (conj.count(abs(solution[r][i])) > 0) {
                     printf("Error: Route %d contains a repeated task/edge! (task: %d)\n", r,
                            solution[r][i]);
                     exit(1);
                 }
                 conj.insert(abs(solution[r][i]));//
                 //printf("%d ", solution[r][i]);//
             }

             // Return cost to depot// Start cost from depot
             int costToAndFromDepot = 0;
             costToAndFromDepot += costFromDepot(solution[r][0], D);
             //cout << "soma += " << costFromDepot(solution[r][0]) << endl;
             costToAndFromDepot += costToDepot(solution[r][solution[r].size()-1], D);
             //cout << "soma += " << costToDepot(solution[r][solution[r].size()-1]) << endl;
             sum += costToAndFromDepot;

             /// ASSERT
             if (cap > capacity) {
                 printf("Error: Capacity exceed in the route (index = %d)\n", r);
                 exit(1);
             }

             //cout << "this route has cost = " << sum << endl;
             cost += sum;
         }
     }

     // ASSERT
     if (conj.size() != nReqEdges) {
         printf("Error: The solution does not contain all required edges. %d != %d\n", (int) conj.size(), nReqEdges);
         exit(1);
     }

     for (int i = 1; i <= nReqEdges; i++) {
         if (conj.count(i) + conj.count(-i) < 1) {
             printf("Error: solution does not route the required edge (tasks %d/%d)\n", i, -i);
             exit(1);
         }
     }

     if (lb0_value <= 0) {
         printf("Trivial lower bound not computed or is zero. Please verify lb_0: %d\n", lb0_value);
         exit(1);
     }

     return cost + lb0_value;
 }

int countVehicle(const vector<vector<int>> &sol){
    int c = 0;
    for(auto rota : sol){
        if( rota.size() != 0){
            c++;
        }
    }
    return c;
}

void initInfo(DebugData& info){
    info.best = INT_MAX;
    info.timeBest = 100000;
    info.inicio = clock();
    info.best_vhc = INT_MAX;
}

void updateInfo(DebugData& info, const vector<vector<int>> &sol){
    const int cost = objective_function(sol);
    const int num_vhc = countVehicle(sol);
    info.costs.push_back(cost);
    info.sols.emplace_back(sol);
    if( cost < info.best){
        info.best = cost;
        info.timeBest = TIME_ELAPSED(clock(), info.inicio);
        info.best_vhc = num_vhc;
    }
    info.vhcs.push_back(num_vhc);
}

void finishInfo(DebugData& info){
    int n = (int) info.costs.size();
    if( n == 0){
        printf("Info vazio! %d\n", n);
        exit(1);
    }
    // time all
    info.timeAll = TIME_ELAPSED(clock(), info.inicio);
    // average
    info.avg = 0;
    for(auto cost : info.costs){
        info.avg += cost;
    }
    info.avg /= n;
    // std deviation
    info.stddev = 0;
    for(auto cost : info.costs){
        info.stddev += (cost > info.avg) ? (cost - info.avg) : (info.avg - cost);
    }
    info.stddev /= n;
    // median
    vector<int> sorted = info.costs;
    sort(sorted.begin(), sorted.end());
    if(n % 2 == 0){
        info.median = (double) (sorted[(n/2)-1] + sorted[(n/2)]) / 2.0;
    }
    else{
        info.median = sorted[n/2];
    }
    // top1% avg and median
    info.top1percent_avg = 0.0;
    int c = 0;
    for(int i = 0; i < sorted.size()/10; i++){ // sorted.size()/10 to be top 10% // <10 to be top 10 obviously
        info.top1percent_avg += sorted[i];
        c++;
    }
    info.top1percent_avg /= c;
    info.top1percent_median = sorted[sorted.size()/20]; // sorted.size()/20 to be top 10% // sorted[4] to be top 10
    // vehicle data: avg
    info.avg_vhc = 0;
    for(auto vhc : info.vhcs){
        info.avg_vhc += vhc;
    }
    info.avg_vhc /= n;
    // vehicle data: std deviation
    info.stddev_vhc = 0;
    for(auto vhc : info.vhcs){
        info.stddev_vhc += (vhc > info.avg_vhc) ? (vhc - info.avg_vhc) : (info.avg_vhc - vhc);
    }
    info.stddev_vhc /= n;
    // checking best
    if( info.best != sorted.at(0) ){
        printf("Diferença! ERROR! %f para %d\n", info.best, sorted.at(0));
        exit(1);
    }
    // Algum dia posso fazer: convergenceRatio
    return;
}

void printInfo(const DebugData& info){
    PRINTF("Best = \t%.2f\tMedian =\t%.2f\tAvg = \t%.2f\t(StdDev = \t%.2f) - [TOP-10%%]\tAvg=%.2f\tMedian =\t%.2f\n", info.best, info.median, info.avg, info.stddev, info.top1percent_avg, info.top1percent_median);
    PRINTF("VhcBest = \t%.0f,\tVhcAvg = \t%.2f\t(StdDev = \t%.2f)\n", info.best_vhc, info.avg_vhc, info.stddev_vhc);
    PRINTF("TimeBest = \t%f,\tTimeAll = \t%f\n", info.timeBest, info.timeAll);
}

// Original path-scanning with five criterias. Builds a solution for a specific 'criteria' ranging in [0,4].
void path_scanning_criteria(const vector<int> &vecTasks, vector<vector<int>> &output_sol, int k, const int criteria) {
    vector<vector<int>> bestSol;
    int PSbestCost = INT_MAX;
    for(int q = 0; q < k; q++){
        int solCost;
        vector<vector<int>> currSol;
        vector<bool> vecTasksUnserved(nReqEdges+1, true);

        int lastArc = 0, rvc = capacity;
        int numUnserved = vecTasks.size();
        int veiculoId = 0;

        while( numUnserved != 0){
            vector<int> candidates;
            int minDistance = INT_MAX;

            // Get candidates
            for(int i = 0; i < vecTasks.size(); i++){
                int task = vecTasks[i];
                int edgeID = vecTasks[i] > 0 ? vecTasks[i] : - vecTasks[i];
                if( vecTasksUnserved[edgeID] && demanda.at(task) <= rvc ){

                    //printf("values for task %d = %d,%d\n", task, sp(lastArc, task), sp(lastArc, -task));

                    // Natural order
                    if( sp(lastArc, task) < minDistance ){
                        minDistance = sp(lastArc, task);
                        candidates.clear();
                        candidates.push_back(task);
                        //printf("selected! %d\n", task);
                    }
                    else if (sp(lastArc, task) == minDistance){
                        candidates.push_back(task);
                        //printf("added!! %d\n", task);
                    }
                    // Rerverse order
                    if( sp(lastArc, -task) < minDistance ){
                        minDistance = sp(lastArc, -task);
                        candidates.clear();
                        candidates.push_back(-task);
                        //printf("selected!!! %d\n", -task);
                    }
                    else if ( sp(lastArc, -task) == minDistance){
                        candidates.push_back(-task);
                        //printf("added!! %d\n", -task);
                    }
                }
            }

            // Candidates: edges 'e' with minimum distance sp(lastArc, 'e')
            //printf("candidates size: %d\n", candidates.size());
            if( candidates.size() > 0){
                // Resolve tie by criteria
                int selected = candidates.at(irandom(candidates.size()-1));
                for(auto candidate : candidates){
                    switch(criteria){
                        case 0:
                            if( ( ((double) custo[candidate])/demanda[candidate]) < ( ((double) custo[selected])/demanda[selected]) ){
                                //printf("Analise %.2f (%d/%d) vs %.2f\n", ((double) custo[candidate])/demanda[candidate], custo[candidate], demanda[candidate], ( ((double) custo[selected])/demanda[selected]));
                                selected = candidate;
                            }
                            break;
                        case 1:
                            if( ( ((double) custo[candidate])/demanda[candidate]) > ( ((double) custo[selected])/demanda[selected]) ){
                                //printf("Analise2 %.2f vs %.2f\n", ((double) custo[candidate])/demanda[candidate], ( ((double) custo[selected])/demanda[selected]));
                                selected = candidate;
                            }
                            break;
                        case 2:
                            if( sp(candidate, 0, D) < sp(selected, 0, D) ){
                                //printf("Analise3 %d vs %d\n", sp(candidate, 0, D), sp(selected, 0, D));
                                selected = candidate;
                            }
                            break;
                        case 3:
                            if( sp(candidate, 0, D) > sp(selected, 0, D) ){
                                //printf("Analise4 %d vs %d\n", sp(candidate, 0, D), sp(selected, 0, D));
                                selected = candidate;
                            }
                            break;
                        case 4:
                            if( rvc <= capacity/2.0 ){
                                if( sp(candidate, 0, D) < sp(selected, 0, D) ){
                                    //printf("Analise5.1 %d vs %d\n", sp(candidate, 0, D), sp(selected, 0, D));
                                    selected = candidate;
                                }
                            }
                            else{
                                if( sp(candidate, 0, D) > sp(selected, 0, D) ){
                                    //printf("Analise5.2 %d vs %d\n", sp(candidate, 0, D), sp(selected, 0, D));
                                    selected = candidate;
                                }
                            }
                            break;
                        default:
                            printf("ERROR! Invalid criteria for path-scanning original (Golden et al.). Must be [0,4].\n");
                            exit(1);
                            break;
                    }
                }
                //printf("task selected: %d.\n", selected);

                // Insert into route
                if( currSol.size() == veiculoId ){
                    //printf("new vehicle starting with task == %d\n", selected);
                    vector<int> init;
                    init.push_back(selected);
                    currSol.push_back(init);
                }
                else{
                    currSol[veiculoId].push_back(selected);
                }

                // Update for next iteration
                lastArc = selected;
                vecTasksUnserved[(selected > 0? selected : -selected)] = false;
                numUnserved--;
                rvc -= demanda.at(selected);
            }
            else{
                // The route is ended and we consider a new route
                veiculoId++;
                rvc = capacity;
                lastArc = 0;
            }


        }
        solCost = obj_function_parcial(currSol);
        if (solCost < PSbestCost){
            PSbestCost = solCost;
            bestSol = currSol;
        }
    }
    output_sol = bestSol;
    return;
}

 // Path-scanning with random criteria (from Pearn et al.).
int path_scanning_randomcriteria(const vector<int> &vecTasks, vector<vector<int>> &output_sol, int k){
    int PSbestCost = INT_MAX;
    for(int q = 0; q < k; q++){
        int solCost = 0;
        vector<vector<int>> currSol;
        vector<bool> vecTasksUnserved(nReqEdges+1, true);

        int lastArc = 0, rvc = capacity;
        int numUnserved = (int) vecTasks.size();
        int veiculoId = 0;

        while( numUnserved != 0){
            vector<int> candidates;
            int minDistance = INT_MAX;

            // Get candidates
            for(int i = 0; i < vecTasks.size(); i++){
                int task = vecTasks[i];
                int edgeID = vecTasks[i] > 0 ? vecTasks[i] : - vecTasks[i];
                if( vecTasksUnserved[edgeID] && demanda.at(task) <= rvc ){

                    //printf("values for task %d = %d,%d\n", task, sp(lastArc, task), sp(lastArc, -task));

                    // Natural order
                    if( sp(lastArc, task) < minDistance ){
                        minDistance = sp(lastArc, task);
                        candidates.clear();
                        candidates.push_back(task);
                        //printf("selected! %d\n", task);
                    }
                    else if (sp(lastArc, task) == minDistance){
                        candidates.push_back(task);
                        //printf("added!! %d\n", task);
                    }
                    // Rerverse order
                    if( sp(lastArc, -task) < minDistance ){
                        minDistance = sp(lastArc, -task);
                        candidates.clear();
                        candidates.push_back(-task);
                        //printf("selected!!! %d\n", -task);
                    }
                    else if ( sp(lastArc, -task) == minDistance){
                        candidates.push_back(-task);
                        //printf("added!! %d\n", -task);
                    }
                }
            }

            // Select one of the candidates
            if( candidates.size() > 0){
                // Resolve tie by criteria
                int selected = candidates.at(irandom(candidates.size()-1));
                if( candidates.size() > 1){
                    int criteria = irandom(4);
                    for(auto candidate : candidates){
                        switch(criteria){
                            case 0:
                                if( ( ((double) custo[candidate])/demanda[candidate]) < ( ((double) custo[selected])/demanda[selected]) ){
                                    //printf("Analise %.2f (%d/%d) vs %.2f\n", ((double) custo[candidate])/demanda[candidate], custo[candidate], demanda[candidate], ( ((double) custo[selected])/demanda[selected]));
                                    selected = candidate;
                                }
                                break;
                            case 1:
                                if( ( ((double) custo[candidate])/demanda[candidate]) > ( ((double) custo[selected])/demanda[selected]) ){
                                    //printf("Analise2 %.2f vs %.2f\n", ((double) custo[candidate])/demanda[candidate], ( ((double) custo[selected])/demanda[selected]));
                                    selected = candidate;
                                }
                                break;
                            case 2:
                                if( sp(candidate, 0, D) < sp(selected, 0, D) ){
                                    //printf("Analise3 %d vs %d\n", sp(candidate, 0, D), sp(selected, 0, D));
                                    selected = candidate;
                                }
                                break;
                            case 3:
                                if( sp(candidate, 0, D) > sp(selected, 0, D) ){
                                    //printf("Analise4 %d vs %d\n", sp(candidate, 0, D), sp(selected, 0, D));
                                    selected = candidate;
                                }
                                break;
                            case 4:
                                if( rvc <= capacity/2.0 ){
                                    if( sp(candidate, 0, D) < sp(selected, 0, D) ){
                                        //printf("Analise5.1 %d vs %d\n", sp(candidate, 0, D), sp(selected, 0, D));
                                        selected = candidate;
                                    }
                                }
                                else{
                                    if( sp(candidate, 0, D) > sp(selected, 0, D) ){
                                        //printf("Analise5.2 %d vs %d\n", sp(candidate, 0, D), sp(selected, 0, D));
                                        selected = candidate;
                                    }
                                }
                                break;

                        }
                    }
                }
                //printf("task selected: %d. (dist: %d. numUnserved: %d)\n", selected, sp(lastArc, selected), numUnserved);

                // Insert into route
                if( currSol.size() == veiculoId ){
                    //printf("new vehicle starting with task == %d\n", selected);
                    vector<int> init;
                    init.push_back(selected);
                    currSol.push_back(init);
                }
                else{
                    currSol[veiculoId].push_back(selected);
                }

                // Update for next iteration
                lastArc = selected;
                vecTasksUnserved[(selected > 0? selected : -selected)] = false;
                numUnserved--;
                rvc -= demanda.at(selected);
            }
            else{
                // This route is finished and we consider a new route
                veiculoId++;
                rvc = capacity;
                lastArc = 0;
            }
        }
        solCost = obj_function_parcial(currSol);
        if (solCost < PSbestCost){
            PSbestCost = solCost;
            output_sol = currSol;
        }
    }
    return PSbestCost;
}

 // Path-scanning with random edge (PS-ER): random selection of tied edges and k runs; from Belenguer et al.
void path_scanning_totalrandom(const vector<int> &vecTasks, vector<vector<int>> &output_sol, int k){
    vector<vector<int>> bestSol;
    int PSbestCost = INT_MAX;
    for(int q = 0; q < k; q++){
        int solCost;
        vector<vector<int>> currSol;
        vector<bool> vecTasksUnserved(nReqEdges+1, true);

        int lastArc = 0, rvc = capacity;
        int numUnserved = vecTasks.size();
        int veiculoId = 0;

        while( numUnserved != 0){
            vector<int> candidates;
            int minDistance = INT_MAX;

            // Get candidates
            for(int i = 0; i < vecTasks.size(); i++){
                int task = vecTasks[i];
                int edgeID = vecTasks[i] > 0 ? vecTasks[i] : - vecTasks[i];
                if( vecTasksUnserved[edgeID] && demanda.at(task) <= rvc ){

                    //printf("values for task %d = %d,%d\n", task, sp(lastArc, task), sp(lastArc, -task));

                    // Natural order
                    if( sp(lastArc, task) < minDistance ){
                        minDistance = sp(lastArc, task);
                        candidates.clear();
                        candidates.push_back(task);
                        //printf("selected! %d\n", task);
                    }
                    else if (sp(lastArc, task) == minDistance){
                        candidates.push_back(task);
                        //printf("added!! %d\n", task);
                    }
                    // Rerverse order
                    if( sp(lastArc, -task) < minDistance ){
                        minDistance = sp(lastArc, -task);
                        candidates.clear();
                        candidates.push_back(-task);
                        //printf("selected!!! %d\n", -task);
                    }
                    else if ( sp(lastArc, -task) == minDistance){
                        candidates.push_back(-task);
                        //printf("added!! %d\n", -task);
                    }
                }
            }

            // Candidates: edges 'e' with minimum distance sp(lastArc, 'e')
            //printf("candidates size: %d\n", candidates.size());
            if( candidates.size() > 0){
                // Select random from candidates
                int idx = irandom(candidates.size()-1);
                int task = candidates[idx];
                //printf("task selected: %d.\n", task);

                // Insert into route
                if( currSol.size() == veiculoId ){
                    //printf("new vehicle starting with task == %d\n", task);
                    vector<int> init;
                    init.push_back(task);
                    currSol.push_back(init);
                }
                else{
                    currSol[veiculoId].push_back(task);
                }

                // Update for next iteration
                lastArc = task;
                vecTasksUnserved[(task > 0? task : -task)] = false;
                numUnserved--;
                rvc -= demanda.at(task);
            }
            else{
                // The route is ended and we consider a new route
                veiculoId++;
                rvc = capacity;
                lastArc = 0;
            }


        }
        solCost = obj_function_parcial(currSol);
        if (solCost < PSbestCost){
            PSbestCost = solCost;
            bestSol = currSol;
        }
    }
    output_sol = bestSol;
    return;
}

// Path-scanning with ellipse rule (PS-Ellipse). Random selection of tied edges and k runs with ellipse-rule. Proposed by Santos et al. (2009)
void path_scanning_ellipse (const vector<int> &vecTasks, vector<vector<int>> &output_sol, double alfa){
    vector<vector<int>> bestSol;
    int PSbestCost = INT_MAX;
    double init_td = 0.0;
    double init_tc = 0.0;
    for(int i = 0; i< vecTasks.size(); i++){
        init_td += demanda.at(vecTasks[i]);
        init_tc += custo.at(vecTasks[i]);
    }
    int init_ned = (int) vecTasks.size();

    int solCost;
    vector<vector<int>> currSol;
    vector<bool> vecTasksUnserved((unsigned) nReqEdges+1, true);

    int lastArc = 0, rvc = capacity;
    int numUnserved = (int) vecTasks.size();
    int veiculoId = 0;
    bool ellipse_enforced;

    while( numUnserved != 0){
        vector<int> candidates;
        int minDistance = INT_MAX;

        // Consider trigerring ellipse rule
        ellipse_enforced = false;
        if( rvc <= alfa * (init_td / init_ned) ){
            ellipse_enforced = true;
            //printf("ellipse enforced! %d <= ((%.2f * %.2f) / %d) == %.2f\n", rvc, alfa, td, numUnserved, (alfa * td) / numUnserved);
        }

        // Get candidates
        for(int i = 0; i < vecTasks.size(); i++){
            //printf("checking %d\n", i);
            int task = vecTasks[i];
            int edgeID = vecTasks[i] > 0 ? vecTasks[i] : - vecTasks[i];
            if( vecTasksUnserved[edgeID] && demanda.at(task) <= rvc ){

                if( !ellipse_enforced || (sp(lastArc,task) + custo.at(task) + sp(task, 0) <= (((double)init_tc)/init_ned + sp(lastArc, 0) + 0.001) ) ){

                    if( ellipse_enforced ){
                        //printf("ellipse in task = %d. Porque %d <= %.0f\n", task, sp(lastArc,task) + custo.at(task) + sp(task, 0), (((double)tc)/numUnserved + sp(lastArc, 0) + 0.001));
                    }

                    //printf("values for task %d = %d,%d\n", task, sp(lastArc, task), sp(lastArc, -task));

                    // Natural order
                    if( sp(lastArc, task) < minDistance ){
                        minDistance = sp(lastArc, task);
                        candidates.clear();
                        candidates.push_back(task);
                        //printf("selected! %d\n", task);
                    }
                    else if (sp(lastArc, task) == minDistance){
                        candidates.push_back(task);
                        //printf("added!! %d\n", task);
                    }

                }
                if( !ellipse_enforced || (sp(lastArc,-task) + custo.at(-task) + sp(-task, 0) <= (((double)init_tc)/init_ned + sp(lastArc, 0) + 0.001) ) ) {

                    if( ellipse_enforced ){
                       //printf("ellipse in task = %d. Porque %d <= %.0f\n", -task, sp(lastArc,-task) + custo.at(-task) + sp(-task, 0), (((double)tc)/numUnserved + sp(lastArc, 0) + 0.001));
                    }

                    // Rerverse order
                    if (sp(lastArc, -task) < minDistance) {
                        minDistance = sp(lastArc, -task);
                        candidates.clear();
                        candidates.push_back(-task);
                        //printf("selected!!! %d\n", -task);
                    } else if (sp(lastArc, -task) == minDistance) {
                        candidates.push_back(-task);
                        //printf("added!! %d\n", -task);
                    }
                }
            }
        }

        // Candidates: edges 'e' with minimum distance sp(lastArc, 'e')
        //printf("candidates size: %d\n", candidates.size());
        if( candidates.size() > 0){
            // Select random from candidates
            int idx = irandom((int) candidates.size()-1);
            int task = candidates[idx];
            //printf("task selected: %d.\n", task);

            // Insert into route
            if( currSol.size() == veiculoId ){
                //printf("new vehicle starting with task == %d\n", task);
                vector<int> init;
                init.push_back(task);
                currSol.push_back(init);
            }
            else{
                currSol[veiculoId].push_back(task);
            }

            // Update for next iteration
            lastArc = task;
            vecTasksUnserved[(task > 0? task : -task)] = false;
            numUnserved--;
            rvc -= demanda.at(task);
        }
        else{
            // The route is ended and we consider a new route
            veiculoId++;
            rvc = capacity;
            lastArc = 0;
            //printf("nova rota id: %d\n", veiculoId);
        }
    }
     output_sol = currSol;

    return;
}

int printsQtde;

 // An algorithm combining features from PS-Ellipse and PS-Efficiency. PS-Alt1.
 // (1) Triggering rule of capacity (from Santos et al.).
 // (2) Efficiency rule for restricting candidate edges. (from Arakaki & Usberti).
 void path_scanning_ellipseEficiencia_TDall(const vector<int> &vecTasks, vector<vector<int>> &output_sol, double alfa) {
     double init_td = 0.0, init_tc = 0.0;
     for(int i = 0; i< vecTasks.size(); i++){
         init_td += demanda.at(vecTasks[i]);
         init_tc += custo.at(vecTasks[i]);
     }
     const int init_ned = (int) vecTasks.size();

     int solCost;
     vector<vector<int>> currSol;
     vector<bool> vecTasksUnserved((unsigned) nReqEdges+1, true);

     int lastArc = 0, rvc = capacity;
     int numUnserved = (int) vecTasks.size();
     int veiculoId = 0;
     bool ellipse_enforced = false;

     double td = init_td;
     double tc = init_tc;
     //printf("##### New Algorithm initiated #####\n");

     double distTraveled = 1.0, distTraveled2 = 0.0;
     int numTasksRota = 0, numTasksRota2 = 0;

     L_DOAGAIN:
     while( numUnserved != 0){
         vector<int> listCandidates;
         vector<int> listCandidatesER;
         int minDistance = INT_MAX;
         double demandTraveled = capacity-rvc;
         double eficiencia = (rvc==capacity)?0.0: (distTraveled+sp(lastArc, 0)) /  demandTraveled; // distTraveled /  numTasksRota

         if( !ellipse_enforced && rvc != capacity && rvc <= alfa * (init_td / init_ned) ){
             ellipse_enforced = true;
         }

         // Get listCandidates
         for(int i = 0; i < vecTasks.size(); i++){
             int task = vecTasks[i];
             int edgeID = vecTasks[i] > 0 ? vecTasks[i] : - vecTasks[i];


             if( vecTasksUnserved[edgeID] ){

                 if ( demanda.at(task) <= rvc ) {

                     if (!ellipse_enforced || ((double) ((sp(lastArc, task) + custo.at(task) + sp(task, 0) - sp(lastArc, 0)))) / demanda.at(task) <= eficiencia) {

                         // Natural order
                         if (sp(lastArc, task) < minDistance) {
                             minDistance = sp(lastArc, task);
                             listCandidates.clear();
                             listCandidates.push_back(task);
                         } else if (sp(lastArc, task) == minDistance) {
                             listCandidates.push_back(task);
                         }
                     }
                     if( !ellipse_enforced || ((double)((sp(lastArc,-task) + custo.at(-task) + sp(-task, 0) - sp(lastArc, 0)))) / demanda.at(-task) <= eficiencia ) {

                         // Rerverse order
                         if (sp(lastArc, -task) < minDistance) {
                             minDistance = sp(lastArc, -task);
                             listCandidates.clear();
                             listCandidates.push_back(-task);
                         } else if (sp(lastArc, -task) == minDistance) {
                             listCandidates.push_back(-task);
                         }
                     }
                 }
             }
         }


         // Candidates: edges 'e' with minimum distance sp(lastArc, 'e')
         if( ellipse_enforced && printsQtde-- > 0){
             //printf("listCandidates within ellipse size: %d (dist: %d) [cap: %d]\n", (int) listCandidates.size(), minDistance, rvc);
         }
         if( listCandidates.size() > 0 ){
             // Select random from listCandidates
             int idx = irandom((int)listCandidates.size()-1);
             int task = listCandidates[idx];

             // Insert into route
             if( currSol.size() == veiculoId ){
                 //printf("new vehicle starting with task == %d\n", task);
                 vector<int> init;
                 init.push_back(task);
                 currSol.push_back(init);
             }
             else{
                 currSol[veiculoId].push_back(task);
             }

             // Update for next iteration
             distTraveled += sp(lastArc, task) + custo.at(task);
             distTraveled2 += sp(lastArc, task);
             if( sp(lastArc, task) > 0 ){ numTasksRota2++; }
             lastArc = task;
             vecTasksUnserved[(task > 0? task : -task)] = false;
             numUnserved--;
             rvc -= demanda.at(task);
             td -= demanda.at(task);
             tc -= custo.at(task);
             numTasksRota++;
         }
         else{
             // The route is ended and we consider a new route
             veiculoId++;
             rvc = capacity;
             numTasksRota = 0;
             numTasksRota2 = 0;
             lastArc = 0;
             ellipse_enforced = false;
             distTraveled = 1.0;
             //printf("================ ROTA ENCERRADA. ===========\nnova rota id: %d\n", veiculoId);
         }
     }

     output_sol = currSol;
     return;
 }

 // The new algorithm proposed: PS-Efficiency.
 // (1) Triggering rule of capacity considering only edges close to the current route (from Arakaki & Usberti).
 // (2) Efficiency rule for restricting candidate edges. (from Arakaki & Usberti).
 void path_scanning_ellipseEficiencia_TDnear(const vector<int> &vecTasks, vector<vector<int>> &output_sol, double alfa, double beta) {
     output_sol.clear();
     double init_td = 0.0, init_tc = 0.0;
     for(int i = 0; i< vecTasks.size(); i++){
         init_td += demanda.at(vecTasks[i]);
         init_tc += custo.at(vecTasks[i]);
     }
     const int init_ned = (int) vecTasks.size();

     vector<bool> vecTasksUnserved((unsigned) nReqEdges+1, true);

     int lastArc = 0, rvc = capacity;
     int numUnserved = (int) vecTasks.size();
     int veiculoId = 0;
     bool efficiency_enforced = false;

     double td = init_td;
     double tc = init_tc;

     double distTraveled = 0.0;

     //printf("##### New Algorithm initiated #####\n");
     // L_DOAGAIN: // old place
     while( numUnserved != 0){
         vector<int> listCandidates;
         vector<int> listCandidatesER;
         int minDistance = INT_MAX;
         int minDistanceER = INT_MAX;
         double td_near = 0.0;
         int ned_near = 0;
         double demandTraveled = capacity-rvc;
         double eficiencia = (rvc==capacity)?0.0: (distTraveled+sp(lastArc, 0)) /  demandTraveled; // distTraveled /  numTasksRota


         // Get listCandidates
         L_DOAGAIN:
         for(int i = 0; i < vecTasks.size(); i++){
             int task = vecTasks[i];
             int edgeID = vecTasks[i] > 0 ? vecTasks[i] : - vecTasks[i];

             // Nesse momento tentamos criar uma informação local mais detalhada do que a informação global td/ned.
             double distRota1 = sp(lastArc, task);
             double distRota2 = sp(lastArc, -task);

             // nova ideia é: apenas arestas próximas à rota (chance alta de serem roteadas nas próximas iterações).
             if ( vecTasksUnserved[edgeID] && min(distRota1, distRota2) <= 1.0 * (init_tc/init_ned) ) {
                 td_near += demanda.at(task);
                 ned_near++;
             }

             if( vecTasksUnserved[edgeID] && demanda.at(task) <= rvc){
                 if (!efficiency_enforced || ((double) ((sp(lastArc, task) + custo.at(task) + sp(task, 0) - sp(lastArc, 0)))) / demanda.at(task) <= eficiencia) {
                     // Natural order
                     if (sp(lastArc, task) < minDistance) {
                         minDistance = sp(lastArc, task);
                         listCandidates.clear();
                         listCandidates.push_back(task);
                     } else if (sp(lastArc, task) == minDistance) {
                         listCandidates.push_back(task);
                     }
                 }
                 if( !efficiency_enforced || ((double)((sp(lastArc,-task) + custo.at(-task) + sp(-task, 0) - sp(lastArc, 0)))) / demanda.at(-task) <= eficiencia ) {
                     // Rerverse order
                     if (sp(lastArc, -task) < minDistance) {
                         minDistance = sp(lastArc, -task);
                         listCandidates.clear();
                         listCandidates.push_back(-task);
                     } else if (sp(lastArc, -task) == minDistance) {
                         listCandidates.push_back(-task);
                     }
                 }
             }
         }

         double ratioValue = (ned_near !=0) ? (td_near/ned_near) : (init_td/init_ned);
         //printf("[rvc=%d] [dist=%d] Checking. (tdER=%.1f / %d). <= %.2f [!= %.2f]\n", rvc, sp(lastArc, 0), td_near, ned_near, alfa * ratioValue, alfa * init_td / init_ned);
         if( !efficiency_enforced && rvc != capacity && rvc <= alfa * ratioValue ){
             efficiency_enforced = true;
             listCandidates.clear();
             minDistance = INT_MAX;
             td_near = 0.0;
             ned_near = 0;
             goto L_DOAGAIN; // get new candidate edges that satisfy efficiency rule
         }

         // Candidates: edges 'e' with minimum distance sp(lastArc, 'e')
         if( efficiency_enforced && printsQtde-- > 0){
             //printf("listCandidates within ellipse size: %d (dist: %d) [cap: %d]\n", (int) listCandidates.size(), minDistance, rvc);
         }
         if( listCandidates.size() > 0 ){
             // Select random from listCandidates
             int idx = irandom((int)listCandidates.size()-1);
             int task = listCandidates[idx];

             // Insert into route
             if( output_sol.size() == veiculoId ){
                 vector<int> init;
                 init.push_back(task);
                 output_sol.push_back(init);
             }
             else{
                 output_sol[veiculoId].push_back(task);
             }

             // Update for next iteration
             distTraveled += sp(lastArc, task) + custo.at(task);
             lastArc = task;
             vecTasksUnserved[(task > 0? task : -task)] = false;
             numUnserved--;
             rvc -= demanda.at(task);
             td -= demanda.at(task);
             tc -= custo.at(task);
         }
         else{
             // The route is ended and we consider a new route
             veiculoId++;
             rvc = capacity;
             lastArc = 0;
             efficiency_enforced = false;
             distTraveled = 0.0;
             //printf("================ ROTA ENCERRADA. ===========\nnova rota id: %d\n", veiculoId);
         }
     }
     return;
 }

 // An algorithm combining features of PS-Ellipse and PS-Efficiency
 // (1) Triggering rule of capacity considering only edges close to the current route (from Arakaki & Usberti).
 // (2) Ellipse rule for restricting candidate edges. (from Santos et al.)
 void path_scanning_ellipseDisparoNovoDefinicaoAntiga (const vector<int> &vecTasks, vector<vector<int>> &output_sol, double alfa, double beta){
     double init_td = 0.0, init_tc = 0.0;
     for(int i = 0; i< vecTasks.size(); i++){
         init_td += demanda.at(vecTasks[i]);
         init_tc += custo.at(vecTasks[i]);
     }
     const int init_ned = (int) vecTasks.size();

     vector<vector<int>> currSol;
     vector<bool> vecTasksUnserved((unsigned) nReqEdges+1, true);

     int lastArc = 0, rvc = capacity;
     int numUnserved = (int) vecTasks.size();
     int veiculoId = 0;
     bool ellipse_enforced = false;

     double td = init_td;
     double tc = init_tc;
     //printf("##### New Algorithm initiated #####\n");

     double distTraveled = 1.0, distTraveled2 = 0.0;
     int numTasksRota = 0, numTasksRota2 = 0;

     L_DOAGAIN:
     while( numUnserved != 0){
         vector<int> listCandidates;
         vector<int> listCandidatesER;
         int minDistance = INT_MAX;
         double td_ER = 0.0;
         int size_ER = 0;
         double demandTraveled = capacity-rvc;

         // Get listCandidates
         for(int i = 0; i < vecTasks.size(); i++){
             int task = vecTasks[i];
             int edgeID = vecTasks[i] > 0 ? vecTasks[i] : - vecTasks[i];

             // Nesse momento tentamos criar uma informação local mais detalhada do que a informação global td/ned.
             // Entretanto, não é fácil criar tal informação com robustez para os muitos casos de rotas e locais que podem ocorrer conforme o algoritmo é executado.
             // ::: Arestas de alta chance de serem inclusas na rota (ellipse ou não). :::
             double distRota1 = sp(lastArc, task);
             double distRota2 = sp(lastArc, -task);

             // nova ideia é: apenas arestas próximas à rota (chance alta de serem roteadas nas próximas iterações).
             if ( min(distRota1, distRota2) <= 1.0 * (init_tc/init_ned) ) {
                 if( vecTasksUnserved[edgeID] ){
                     td_ER += demanda.at(task);
                     size_ER++;
                 }
             }

             if( vecTasksUnserved[edgeID] ){
                 if ( demanda.at(task) <= rvc ) {
                     if (!ellipse_enforced || (sp(lastArc,task) + custo.at(task) + sp(task, 0) <= (((double)init_tc)/init_ned + sp(lastArc, 0) + 0.001) )) { // ((double) ((sp(lastArc, task) + sp(task, 0) - sp(lastArc, 0)))) / demanda.at(task) <= eficiencia)

                         // Natural order
                         if (sp(lastArc, task) < minDistance) {
                             minDistance = sp(lastArc, task);
                             listCandidates.clear();
                             listCandidates.push_back(task);
                             //printf("selected! %d\n", task);
                         } else if (sp(lastArc, task) == minDistance) {
                             listCandidates.push_back(task);
                             //printf("added!! %d\n", task);
                         }
                     }
                     if( !ellipse_enforced || (sp(lastArc,-task) + custo.at(-task) + sp(-task, 0) <= (((double)init_tc)/init_ned + sp(lastArc, 0) + 0.001) ) ) { //  beta*( (((double)init_tc)/init_ned) + sp(lastArc, 0) + 0.001) )

                         // Rerverse order
                         if (sp(lastArc, -task) < minDistance) {
                             minDistance = sp(lastArc, -task);
                             listCandidates.clear();
                             listCandidates.push_back(-task);
                             //printf("selected!!! %d\n", -task);
                         } else if (sp(lastArc, -task) == minDistance) {
                             listCandidates.push_back(-task);
                             //printf("added!! %d\n", -task);
                         }
                     }
                 }
             }
         }

         double ratioValue = (size_ER !=0) ? (td_ER/size_ER) : (init_td/init_ned);
         //printf("[rvc=%d] [dist=%d] Checking. (tdER=%.1f / %d). <= %.2f [!= %.2f]\n", rvc, sp(lastArc, 0), td_ER, size_ER, alfa * ratioValue, alfa * init_td / init_ned);
         if( !ellipse_enforced && rvc != capacity && rvc <= alfa * ratioValue ){
             //printf("Triggado! rvc=%d\n",rvc);
             if( rvc <= alfa * (init_td / init_ned) ){
                 //printf("Ambos ! rvc=%d\n", rvc);
             }
             else{
                 //printf("Apenas regra nova ! rvc=%d\n", rvc);
             }
             ellipse_enforced = true;
             goto L_DOAGAIN;
         }

         // Candidates: edges 'e' with minimum distance sp(lastArc, 'e')
         if( ellipse_enforced && printsQtde-- > 0){
             //printf("listCandidates within ellipse size: %d (dist: %d) [cap: %d]\n", (int) listCandidates.size(), minDistance, rvc);
         }
         if( listCandidates.size() > 0 ){
             // Select random from listCandidates
             int idx = irandom((int)listCandidates.size()-1);
             int task = listCandidates[idx];

             // Insert into route
             if( currSol.size() == veiculoId ){
                 //printf("new vehicle starting with task == %d\n", task);
                 vector<int> init;
                 init.push_back(task);
                 currSol.push_back(init);
             }
             else{
                 currSol[veiculoId].push_back(task);
             }

             // Update for next iteration
             distTraveled += sp(lastArc, task) + custo.at(task);
             distTraveled2 += sp(lastArc, task);
             if( sp(lastArc, task) > 0 ){ numTasksRota2++; }
             lastArc = task;
             vecTasksUnserved[(task > 0? task : -task)] = false;
             numUnserved--;
             rvc -= demanda.at(task);
             td -= demanda.at(task);
             tc -= custo.at(task);
             numTasksRota++;
         }
         else{
             // The route is ended and we consider a new route
             veiculoId++;
             rvc = capacity;
             numTasksRota = 0;
             numTasksRota2 = 0;
             lastArc = 0;
             ellipse_enforced = false;
             distTraveled = 1.0;
             //printf("================ ROTA ENCERRADA. ===========\nnova rota id: %d\n", veiculoId);
         }
     }
     output_sol = currSol;
     return;
 }

 // This function encapsulates the complexity to access matrix D (or R).
inline int pos(int task) {
    if (task >= 0)
        return task;
    else
        return nReqEdges + (-task);
}

// This function encapsulates the complexity to access matrix D (or R).
// Gets the shortest path between two tasks (from 'p' to 'q', note that it's different from 'q' to 'p')
inline int sp(int p, int q, int **T) {
    return T[pos(p)][pos(q)];
}

// This function encapsulates the complexity to access vector startDepot (travel cost from depot to task).
inline int costFromDepot(int p, int **T) {
    return T[0][pos(p)];
}

// This function encapsulates the complexity to access vector returnDepot (travel cost from task to depot).
inline int costToDepot(int p, int **T) {
    return T[pos(p)][0];
}

// Get node ids from a task.
inline pair<int, int> getNodes(int task) {
    ListGraph::Edge e = edgeFromTask.at(task);
    if (task > 0) {
        return pair<int, int>((*g).id((*g).u(e)), (*g).id((*g).v(e))); // (u,v)
    }
    else {
        return pair<int, int>((*g).id((*g).v(e)), (*g).id((*g).u(e))); // (v,u)
    }
    // Note that these indexes are from the LEMON indexing, they are not the indexes used in the graph from the instance file.
}

// Preprocessing of the algorithm.
// Calculates the shortest path matrix between required arcs (tasks).
void pre_processamento() {
    // Allocates the matrix
    clock_t iniciopp = clock();
    nTasks = nReqEdges * 2;
    D = new int*[nTasks + 1];
    R = new int*[nTasks + 1];
    for (int i = 0; i <= nTasks; i++) {
        D[i] = new int[nTasks + 1];
        R[i] = new int[nTasks + 1];
    }

    // Fill the matrix

    // 1. All Pairs Shortest Path of G
    spG = vector<vector<int> >(nNodes); // sp[x][y]: shortest path from x to y (obs: sp[x][y] == sp[y][x]).
    for (int i = 0; i < nNodes; i++)
        spG[i] = vector<int>(nNodes);

    // 2. Compute shortest path from node s to each node
    for (ListGraph::NodeIt n1(*g); n1 != INVALID; ++n1) {
        //cout << endl << "Analisando distancias de " << (*g).id(n1) << ":" << endl;
        ListGraph::NodeMap<int> dists(*g);
        dijkstra((*g), (*custoAresta) ).distMap(dists).run(n1);

        for (ListGraph::NodeIt n2(*g); n2 != INVALID; ++n2) {
            //cout << "Para " << (*g).id(n2) << " = " << dists[n2] << ", ";
            spG[(*g).id(n1)][(*g).id(n2)] = dists[n2];
        }
    }

	// 3. To each pair of tasks (inversed or not), calculates the correspondent entries of distances between them
    for (int i = 1; i <= nReqEdges; i++) {
        pair<int, int> nodes_i = getNodes(i);
        for (int j = 1; j <= nReqEdges; j++) {
            pair<int, int> nodes_j = getNodes(j);

            if (i == j) {
                D[pos(i)][pos(j)] = INT_MAX;
                D[pos(i)][pos(-j)] = INT_MAX;
                D[pos(-i)][pos(j)] = INT_MAX;
                D[pos(-i)][pos(-j)] = INT_MAX;
            }
            else {
                // The 4 possibilities: each task inversed or not
                D[pos(i)][pos(j)] = spG[nodes_i.second][nodes_j.first];
                D[pos(i)][pos(-j)] = spG[nodes_i.second][nodes_j.second];
                D[pos(-i)][pos(j)] = spG[nodes_i.first][nodes_j.first];
                D[pos(-i)][pos(-j)] = spG[nodes_i.first][nodes_j.second];
            }
        }
    }

    // Checks the matrix D validity
    for (int i = 1; i <= nReqEdges; i++) {
        for (int j = 1; j <= nReqEdges; j++) {
            if (i != j) {
                if (D[pos(i)][pos(j)] != spG[getNodes(i).second][getNodes(j).first])
                    PRINTF("PROBLEM on matrix! 1\n");
                if (D[pos(-i)][pos(j)] != spG[getNodes(-i).second][getNodes(j).first])
                    PRINTF("PROBLEM on matrix! 2\n");
                if (D[pos(i)][pos(-j)] != spG[getNodes(i).second][getNodes(-j).first])
                    PRINTF("PROBLEM on matrix! 3\n");
                if (D[pos(-i)][pos(-j)] != spG[getNodes(-i).second][getNodes(-j).first])
                    PRINTF("PROBLEM on matrix! 4\n");
            }
        }
    }

    // 4. To each task, compute the distance to/from depot node.
    for (int i = 1; i <= nReqEdges; i++) {
        pair<int, int> nodes_i = getNodes(i);

        D[0][pos(i)]     = spG[depotNodeId-1][nodes_i.first];  // distance do nó depósito até à tarefa 'i'.
        D[pos(i)][0]     = spG[nodes_i.second][depotNodeId-1]; // distance do nó depósito até à tarefa 'i'.

        D[0][pos(-i)]     = spG[depotNodeId-1][nodes_i.second]; // distance do nó depósito até à tarefa '-i'.
        D[pos(-i)][0]     = spG[nodes_i.first][depotNodeId-1];  // distance do nó depósito até à tarefa '-i'.

        //cout << "Distancias para task " << i  << ":\t" << D[0][pos(i)] << "\t/ " << D[pos(i)][0] << endl;
        //cout << "Distancias para task " << -i  << ":\t" << D[0][pos(-i)] << "\t/ " << D[pos(-i)][0] << endl;
    }
    D[0][0] = 0;
    printf("Pre-processing done within %.2f secs\n", TIME_ELAPSED(clock(), iniciopp));
}

// Read the instance file and initialize data
void readFile(const string filepath, ListGraph &g, ListGraph::EdgeMap<int> &custoAresta, ListGraph::EdgeMap<int> &demandaAresta) {
    std::ifstream infile(filepath,  ifstream::in); // -std=c++11
    std::string line;
    nNodes = nTasks = nEdges = nReqEdges = maxroutes = capacity = 0;
    if( !infile.is_open() || !infile.good() ){
        cout << "File could not be opened: " << filepath << endl;
        exit(1);
    }
    int sumCustoRequeridas = 0;
    while (getline(infile, line)) {
        //cout << "Line reading: " << line << endl;
        char ch;
        ch = line.at(0);
        switch (ch) {
            case 'p':
				// Initial instance data
                char type[50];
                sscanf(line.c_str(), "%*s %s", type);
                if( strcmp(type, "CARP") == 0 || strcmp(type, "carp") == 0 ){
                    // carp instances
                    sscanf(line.c_str(), "%*s %*s %d %d %d %d %d", &nNodes, &nEdges, &maxroutes, &capacity, &depotNodeId);
                }
                else{
                    // legacy ocarp instances files (depot node =1)
                    sscanf(line.c_str(), "%*s %*s %d %d %d %d", &nNodes, &nEdges, &maxroutes, &capacity);
                    depotNodeId = 1;
                }
                for (int i = 0; i < nNodes; i++) {
                    g.addNode();
                }
                nReqEdges = 0;
                break;
            case 'e': {
				// Edge reading
                int u, v, c, d;
                ListGraph::Edge edg;
                sscanf(line.c_str(), "%*s %d %d %d %d", &u, &v, &c, &d); // edge (u,v), cost c, demand d.
                edg = g.addEdge(g.nodeFromId(u - 1), g.nodeFromId(v - 1));
                custoAresta[edg] = c;
                demandaAresta[edg] = d;
                if (d > 0){
                    sumCustoRequeridas += c;
                    nReqEdges++;
                    edgeFromTask.insert(pair<int, ListGraph::Edge>(nReqEdges, edg));
                    edgeFromTask.insert(pair<int, ListGraph::Edge>(-nReqEdges, edg));
                    custo.insert(pair<int, int>(nReqEdges, c));
                    custo.insert(pair<int, int>(-nReqEdges, c));
                    demanda.insert(pair<int, int>(nReqEdges, d));
                    demanda.insert(pair<int, int>(-nReqEdges, d));
                    //printf("Task id=%d, arc(%d,%d)\n", nReqEdges, u, v);
                    //printf("Task id=%d, arc(%d,%d)\n", -nReqEdges, v, u);
                }
            }
                break;
            default:
                cout << "Instance reading problem: character unknown:" << ch << endl;
                exit(1);
                break;
        }
    }
	
	// Trivial lower bound: sum of cost of required edges
    lb0_value = sumCustoRequeridas;

    // Asserts
    int nnos = 0, narestas = 0;
    for (ListGraph::NodeIt n(g); n != INVALID; ++n)
        nnos++;
    for (ListGraph::EdgeIt e(g); e != INVALID; ++e)
        narestas++;
    if (nnos != nNodes || narestas != nEdges) {
        cout << "Problema com leitura de nos ou arestas. (" << nnos << "!=" << nNodes << ") ou (" << narestas << "!=" <<
        nEdges << ")" << endl;
        exit(1);
    }

    printf("Instance with capacity %d.\n", capacity);
}
