#include <bits/stdc++.h>
using namespace std;

/*
  Projeto: Algoritmo Genetico (esqueleto) para o Problema do Caixeiro Viajante (TSP)
  Objetivo: Gerar instancias (uniforme / circular), criar populacao inicial, avaliar
            e executar uma rotina simples de selecao por geracoes (eliminar piores,
            repor com individuos aleatorios).
  Observacao: Este ficheiro contem apenas a etapa de selecao/visualizacao; operadores
               geneticos (crossover, mutacao) serao integrados posteriormente.
*/

/* ------------------ PARAMETROS (faceis de alterar) ------------------ */
const int CANVAS_W = 140;       // largura do canvas ASCII (colunas)
const int CANVAS_H = 65;        // altura do canvas ASCII (linhas)
const int CITY_BLOCK = 3;       // cada cidade ocupa CITY_BLOCK x CITY_BLOCK no canvas

// Parametros do "GA" / execucao
int POP_SIZE = 6;               // tamanho da populacao (padrao pequeno para teste)
int NUM_GENERATIONS = 5;        // numero de geracoes a executar
double ELIM_FRACTION = 0.5;     // fracao a eliminar por geracao (ex.: 0.5 elimina metade)
/* -------------------------------------------------------------------- */

// Gerador de numeros aleatorios (seed com relogio)
static std::mt19937 rng((unsigned)chrono::high_resolution_clock::now().time_since_epoch().count());

struct Point { double x, y; };

// Distancia euclidiana entre dois pontos
double dist(const Point &a, const Point &b){
    double dx = a.x - b.x;
    double dy = a.y - b.y;
    return sqrt(dx*dx + dy*dy);
}

/*
  Gera n pontos uniformes no quadrado [0,1]x[0,1], garantindo
  unicidade pela discretizacao com 'precision'.
*/
vector<Point> generate_unique_uniform(int n, int precision = 100) {
    uniform_real_distribution<double> U(0.0, 1.0);
    unordered_set<long long> used;
    vector<Point> pts;
    pts.reserve(n);
    while ((int)pts.size() < n) {
        double x = U(rng), y = U(rng);
        int xi = (int)round(x * precision), yi = (int)round(y * precision);
        long long key = (((long long)xi) << 32) ^ (unsigned long long)yi;
        if (used.insert(key).second) pts.push_back({x,y});
    }
    return pts;
}

/*
  Gera n pontos sobre um circulo (benchmark). Os pontos sao criados
  igualmente espaçados e depois embaralhados levemente para evitar
  ordenacao trivial.
*/
vector<Point> generate_unique_circle(int n, double radius = 0.45, double cx = 0.5, double cy = 0.5) {
    vector<Point> pts;
    pts.reserve(n);
    for (int i=0;i<n;i++){
        double theta = 2.0 * M_PI * i / n;
        double x = cx + radius * cos(theta);
        double y = cy + radius * sin(theta);
        pts.push_back({x,y});
    }
    shuffle(pts.begin(), pts.end(), rng);
    return pts;
}

/*
  Converte coordenadas normalizadas [0,1] para coordenadas do canvas ASCII,
  mantendo uma margem para que blocos de cidade nao ultrapassem o limite.
*/
pair<int,int> map_to_canvas(const Point &p){
    int w = CANVAS_W - CITY_BLOCK - 2;
    int h = CANVAS_H - CITY_BLOCK - 2;
    int cx = 1 + (int)round(p.x * w);
    int cy = 1 + (int)round(p.y * h);
    return {cx, cy};
}

/*
  Algoritmo de Bresenham para traçar uma linha (inteira) entre dois pontos no canvas.
  A linha so sobrescreve espacos vazios (' '), preservando blocos de cidade.
*/
void draw_line(vector<string> &canvas, int x0, int y0, int x1, int y1, char ch){
    int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int err = dx + dy;
    int x = x0, y = y0;
    while (true){
        if (x>=0 && x < (int)canvas[0].size() && y>=0 && y < (int)canvas.size()){
            if (canvas[y][x] == ' ') canvas[y][x] = ch;
        }
        if (x == x1 && y == y1) break;
        int e2 = 2*err;
        if (e2 >= dy){ err += dy; x += sx; }
        if (e2 <= dx){ err += dx; y += sy; }
    }
}

/*
  Desenha no terminal um mapa ASCII das cidades; opcionalmente desenha uma rota ciclica.
  Cada cidade e representada por um bloco 'O' e seu indice numerico abaixo.
*/
void print_ascii_map(const vector<Point> &pts, const vector<int> &route = {}){
    vector<string> canvas(CANVAS_H, string(CANVAS_W, ' '));
    vector<pair<int,int>> centers;
    centers.reserve(pts.size());
    for (auto &p : pts) centers.push_back(map_to_canvas(p));

    // desenha blocos de cidade
    for (size_t i=0;i<pts.size();++i){
        int cx = centers[i].first;
        int cy = centers[i].second;
        int half = CITY_BLOCK/2;
        for (int dy=-half; dy<=half; ++dy){
            for (int dx=-half; dx<=half; ++dx){
                int x = cx + dx;
                int y = cy + dy;
                if (x>=0 && x<CANVAS_W && y>=0 && y < CANVAS_H){
                    canvas[y][x] = 'O';
                }
            }
        }
        // indice da cidade sob o bloco (quando cabe)
        string idx = to_string((int)i);
        int px = cx - (int)idx.size()/2;
        int py = cy + half + 1;
        if (py>=0 && py < CANVAS_H){
            for (size_t k=0;k<idx.size();++k){
                int x = px + (int)k;
                if (x>=0 && x<CANVAS_W) canvas[py][x] = idx[k];
            }
        }
    }

    // se rota fornecida, desenha linhas entre centros e redesenha blocos
    if (!route.empty()){
        for (size_t i=0;i<route.size(); ++i){
            int a = route[i];
            int b = route[(i+1) % route.size()];
            int x0 = centers[a].first, y0 = centers[a].second;
            int x1 = centers[b].first, y1 = centers[b].second;
            draw_line(canvas, x0, y0, x1, y1, '+');
        }
        for (size_t i=0;i<pts.size();++i){
            int cx = centers[i].first;
            int cy = centers[i].second;
            int half = CITY_BLOCK/2;
            for (int dy=-half; dy<=half; ++dy){
                for (int dx=-half; dx<=half; ++dx){
                    int x = cx + dx;
                    int y = cy + dy;
                    if (x>=0 && x<CANVAS_W && y>=0 && y < CANVAS_H){
                        canvas[y][x] = 'O';
                    }
                }
            }
        }
    }

    // imprimir com moldura
    string topbot = string(CANVAS_W+2, '-');
    cout << topbot << "\n";
    for (int r=0;r<CANVAS_H;++r){
        cout << '|' << canvas[r] << "|\n";
    }
    cout << topbot << "\n";
}

/*
  Gera um individuo aleatorio (permutacao das cidades).
  Esta funcao e usada temporariamente para repor a populacao apos eliminacao.
*/
vector<int> random_individual(int n){
    vector<int> p(n);
    iota(p.begin(), p.end(), 0);
    shuffle(p.begin(), p.end(), rng);
    return p;
}

/* Calcula o comprimento total de uma rota ciclica (volta a origem). */
double tour_length(const vector<Point> &pts, const vector<int> &route){
    double s = 0.0;
    for (size_t i=0;i<route.size();++i){
        int a = route[i];
        int b = route[(i+1)%route.size()];
        s += dist(pts[a], pts[b]);
    }
    return s;
}

/* Avalia toda a populacao e retorna vector com os comprimentos (mesma ordem). */
vector<double> evaluate_population(const vector<Point> &pts, const vector<vector<int>> &population){
    vector<double> lengths;
    lengths.reserve(population.size());
    for (auto &ind : population) lengths.push_back(tour_length(pts, ind));
    return lengths;
}

/* Converte rota para string compacta, usada para impressao. */
string route_to_string(const vector<int> &route){
    string s = "[";
    for (size_t i=0;i<route.size(); ++i){
        s += to_string(route[i]);
        if (i+1 < route.size()) s += ",";
    }
    s += "]";
    return s;
}

/* --------------------------- FUNCAO PRINCIPAL --------------------------- */
int main(){
    ios::sync_with_stdio(false);
    cin.tie(nullptr);

    cout << fixed << setprecision(3);

    int N = 12; // numero de cidades (>=8)
    cout << "TSP ASCII - demonstracao de selecao\n";
    cout << "Parametros padrao: cidades N=" << N << ", tamanho populacao=" << POP_SIZE
         << ", geracoes=" << NUM_GENERATIONS << ", fracao eliminada=" << ELIM_FRACTION << "\n";

    // prompt para escolher cenario
    cout << "Escolha o cenario: 1 = uniforme aleatorio, 2 = circular (benchmark). Digite 1 ou 2: " << flush;
    int choice = 1;
    cin >> choice;
    if (!cin) choice = 1;
    cout << "\n";

    vector<Point> pts;
    if (choice == 2){
        pts = generate_unique_circle(N);
        cout << "Cenario: circular (benchmark) selecionado.\n";
    } else {
        pts = generate_unique_uniform(N, 200);
        cout << "Cenario: uniforme (aleatorio) selecionado.\n";
    }

    cout << "Mapa inicial (sem rota):\n";
    print_ascii_map(pts);

    // cria populacao inicial
    vector<vector<int>> population;
    population.reserve(POP_SIZE);
    for (int i=0;i<POP_SIZE;i++) population.push_back(random_individual(N));

    // loop de selecao por geracoes
    for (int gen=1; gen<=NUM_GENERATIONS; ++gen){
        cout << "\n" << string(30, '=') << "\n";
        cout << "GERACAO " << gen << "\n";
        cout << string(30, '=') << "\n";

        // avalia populacao
        vector<double> lengths = evaluate_population(pts, population);

        // estatisticas e melhor individuo
        double minL = numeric_limits<double>::infinity();
        double maxL = -numeric_limits<double>::infinity();
        double sumL = 0.0;
        int best_idx = 0;
        for (size_t i=0;i<lengths.size(); ++i){
            double v = lengths[i];
            sumL += v;
            if (v < minL){ minL = v; best_idx = (int)i; }
            if (v > maxL) maxL = v;
        }
        double meanL = (lengths.empty() ? 0.0 : sumL / lengths.size());

        // imprime melhor individuo e mapa
        cout << "Melhor individuo (indice) = " << best_idx << " | comprimento = " << minL << "\n";
        cout << "Rota do melhor individuo: " << route_to_string(population[best_idx]) << "\n";
        cout << "Mapa do melhor individuo:\n";
        print_ascii_map(pts, population[best_idx]);

        // imprime estatisticas
        cout << "Estatisticas da geracao: MIN = " << minL << " | MAX = " << maxL << " | MEDIA = " << meanL << "\n";

        // determina quantos eliminar
        int elim_count = (int)round(POP_SIZE * ELIM_FRACTION);
        if (elim_count < 1) elim_count = 1;
        if (elim_count >= POP_SIZE) elim_count = POP_SIZE - 1; // garante pelo menos 1

        // ordena indices por fitness (ascendente)
        vector<int> idx(population.size());
        iota(idx.begin(), idx.end(), 0);
        sort(idx.begin(), idx.end(), [&](int a, int b){ return lengths[a] < lengths[b]; });

        // mantem os melhores (POP_SIZE - elim_count)
        int keep = POP_SIZE - elim_count;
        vector<vector<int>> newpop;
        newpop.reserve(POP_SIZE);
        for (int i=0;i<keep;++i) newpop.push_back(population[idx[i]]);

        // relatorio de eliminacao
        cout << "Eliminando os " << elim_count << " piores individuos desta geracao.\n";
        cout << "Sobreviventes (indices na populacao anterior): ";
        for (int i=0;i<keep;++i) cout << idx[i] << (i+1<keep? ", " : "\n");

        // repoe populacao com individuos aleatorios (temporario)
        int need = POP_SIZE - (int)newpop.size();
        for (int i=0;i<need; ++i) newpop.push_back(random_individual(N));

        // substitui populacao
        population.swap(newpop);

        // sumario rapido apos refill
        vector<double> lengths_after = evaluate_population(pts, population);
        cout << "Tamanho da populacao apos reposicao: " << population.size() << " | amostras de comprimentos: ";
        for (size_t i=0;i<lengths_after.size(); ++i){
            if (i) cout << ", ";
            cout << lengths_after[i];
            if (i >= 9) { cout << ", ..."; break; }
        }
        cout << "\n";
    } // fim geracoes

    cout << "\nDemonstracao de selecao concluida.\n";
    cout << "Proximos passos: substituir reposicao aleatoria por crossover e mutacao,\n"
         << "implementar criterios de parada mais sofisticados e armazenar historico de melhores por geracao.\n";
    return 0;
}
