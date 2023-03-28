#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <algorithm>

using namespace std;

int main(void){
	FILE* aux;
	aux = fopen("grafico.txt", "r");
	if( aux == NULL ){
		printf("Erro ao abrir arquivo de entrada 'grafico.txt'\n");
		exit(1);
	}


	int n, a, s; // n=nr. instances, a= nr. algorithms, s = sampling size (nr. de resultados: 5%, 10%, 15% de instancias)
	if( fscanf( aux, " %d %d %d", &n, &a, &s) != 3){
		printf("Erro ao ler os primeiros 3 numeros do arquivo 'grafico.txt'\n");
		printf("Devem ser:\n");
		printf("<n> <a> <s>\n");
		printf("Onde:\n");
		printf("<n>: numero de instancias\n");
		printf("<a>: numero de algoritmos\n");
		printf("<s>: numero de resultados (ex: se s=20, entao tera resultados para 5%%, 10%%, 15%%, etc. das instancias).\n");
		exit(1);
	}
	vector<double> v[a];

	// leitura dos valores
	for( int i = 0; i < n; i++){
		for(int j = 0; j < a; j++){
			double num;
			fscanf(aux,  " %lf", &num);
			v[j].push_back(num);
		}
	}

	// ordena cada vetor de menor para maior (objective function de minimizacao)
	for(int j =0; j < a; j++){
		sort(v[j].begin(), v[j].end());
	}


	// agora encontra para cada x% de instancias, (ex: 5%, 10%, 15%...), qual o valor de resultado y% obtido por cada algoritmo
	double increaseFloat = 1.0 / ((double) s); // ex: se s=20, então increaseFloat = 0.05, entao teremos resultados para 5%, 10%, 15%, ... das instancias.
	vector<int> garantia[a];
	printf("Processando e imprimindo resultados.\nColuna sao algoritmos e linha sao cada um dos samplings.\n\n");
	for(int k = 0; k < s; k++){
		printf("Para %.3f%% das instancias:\t", 100.00 * increaseFloat * (k+1));
		int posVetor = (int) n * (increaseFloat * (k+1));
		if( k == s-1 ){
			posVetor = n-1; // verificação para que erro numérico não erre excluindo o último valor do vetor, o que seria erro grave nesse caso. (excluir pior resultado do algoritmo!)
		}
		for(int j = 0; j < a; j++){
			garantia[j].push_back(v[j].at(posVetor)); 	// valor em v[j][posVetor] é o MAIOR VALOR registrado em v[j] para um dado intervalo de quantidade de instancias
														// (GAP daquele intervalo garantido ser <= a esse valor).
			printf("%.3f\t", v[j].at(posVetor));
		}
		printf("\n");
	}

	fclose(aux);
	printf("Programa terminado com sucesso. Finalizando.\n");
	return 0;
}