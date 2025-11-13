#include <bits/stdc++.h>
using namespace std;

/*
  Algoritmo Genetico - TSP ASCII
  Implementacao:
  - populacao fixa
  - elitismo parcial
  - selecao por rank (peso linear)
  - crossover Order Crossover (OX)
  - mutacao por swap adaptativa (deterministica por individuo)
  - exibicao ASCII do mapa e da melhor rota
*/

/* ------------------ PARAMETROS ------------------ */
const int CANVAS_W = 200;
const int CANVAS_H = 100;
const int CITY_BLOCK = 1;

int pop_size = 100;
int max_generations = 2147483647;
int print_interval = 30;
double mut_range = 0.60;
int mutations_per_gen = 16; // controla quantas passagens completas sobre a populacao
double elitism_rate = 0.05;
double crossover_rate = 0.8;
int max_stagnation = 500;
/* ------------------------------------------------ */

static std::mt19937 rng((unsigned)chrono::high_resolution_clock::now().time_since_epoch().count());

struct Point { double x, y; };

/* Calcula distancia euclidiana entre dois pontos */
double distance_euclid(const Point &a, const Point &b){
    double dx = a.x - b.x, dy = a.y - b.y;
    return sqrt(dx*dx + dy*dy);
}

/* ----------- Geracao de pontos ----------- */
/* Gera n pontos uniformes (garante unicidade discreta com 'precision') */
vector<Point> gen_uniform_points(int n, int precision = 100) {
    uniform_real_distribution<double> U(0.0, 1.0);
    unordered_set<long long> used;
    vector<Point> pts; pts.reserve(n);
    while ((int)pts.size() < n) {
        double x = U(rng), y = U(rng);
        int xi = (int)round(x * precision), yi = (int)round(y * precision);
        long long key = (((long long)xi) << 32) ^ (unsigned long long)yi;
        if (used.insert(key).second) pts.push_back({x,y});
    }
    return pts;
}

/* Gera n pontos distribuídos sobre um circulo e embaralha a ordem */
vector<Point> gen_circle_points(int n, double radius = 0.45, double cx = 0.5, double cy = 0.5) {
    vector<Point> pts; pts.reserve(n);
    for (int i=0;i<n;i++){
        double theta = 2.0 * M_PI * i / n;
        pts.push_back({cx + radius * cos(theta), cy + radius * sin(theta)});
    }
    shuffle(pts.begin(), pts.end(), rng);
    return pts;
}

/* ----------- Funcoes auxiliares de desenho ASCII ----------- */
pair<int,int> to_canvas_coords(const Point &p){
    int w = CANVAS_W - CITY_BLOCK - 2, h = CANVAS_H - CITY_BLOCK - 2;
    int cx = 1 + (int)round(p.x * w);
    int cy = 1 + (int)round(p.y * h);
    return {cx, cy};
}

void draw_line(vector<string> &canvas, int x0, int y0, int x1, int y1, char ch){
    int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int err = dx + dy, x = x0, y = y0;
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

/* Imprime mapa ASCII com pontos; se 'route' fornecido desenha as ligacoes */
void print_ascii_map(const vector<Point> &pts, const vector<int> &route = {}){
    vector<string> canvas(CANVAS_H, string(CANVAS_W, ' '));
    vector<pair<int,int>> centers; centers.reserve(pts.size());
    for (auto &p : pts) centers.push_back(to_canvas_coords(p));

    for (size_t i=0;i<pts.size();++i){
        int cx = centers[i].first, cy = centers[i].second, half = CITY_BLOCK/2;
        for (int dy=-half; dy<=half; ++dy)
            for (int dx=-half; dx<=half; ++dx){
                int x = cx + dx, y = cy + dy;
                if (x>=0 && x<CANVAS_W && y>=0 && y < CANVAS_H) canvas[y][x] = 'O';
            }
        string idx = to_string((int)i);
        int px = cx - (int)idx.size()/2, py = cy + half + 1;
        if (py>=0 && py < CANVAS_H)
            for (size_t k=0;k<idx.size();++k){
                int x = px + (int)k;
                if (x>=0 && x<CANVAS_W) canvas[py][x] = idx[k];
            }
    }

    if (!route.empty()){
        for (size_t i=0;i<route.size(); ++i){
            int a = route[i], b = route[(i+1)%route.size()];
            draw_line(canvas, centers[a].first, centers[a].second, centers[b].first, centers[b].second, '+');
        }
        for (size_t i=0;i<pts.size();++i){
            int cx = centers[i].first, cy = centers[i].second, half = CITY_BLOCK/2;
            for (int dy=-half; dy<=half; ++dy)
                for (int dx=-half; dx<=half; ++dx){
                    int x = cx + dx, y = cy + dy;
                    if (x>=0 && x<CANVAS_W && y>=0 && y < CANVAS_H) canvas[y][x] = 219;
                }
        }
    }

    string topbot = string(CANVAS_W+2, '-');
    cout << topbot << "\n";
    for (int r=0;r<CANVAS_H;++r) cout << '|' << canvas[r] << "|\n";
    cout << topbot << "\n";
}

/* ----------- Utilitarios do GA ----------- */
/* Cria um individuo aleatorio (permutacao 0..n-1) */
vector<int> create_random_individual(int n){
    vector<int> p(n); iota(p.begin(), p.end(), 0);
    shuffle(p.begin(), p.end(), rng); return p;
}

/* Computa o comprimento total de um tour ciclico */
double compute_tour_length(const vector<Point> &pts, const vector<int> &route){
    double s = 0.0;
    for (size_t i=0;i<route.size();++i)
        s += distance_euclid(pts[route[i]], pts[route[(i+1)%route.size()]]);
    return s;
}

/* Avalia toda a populacao retornando vector de comprimentos */
vector<double> evaluate_population_lengths(const vector<Point> &pts, const vector<vector<int>> &population){
    vector<double> lengths; lengths.reserve(population.size());
    for (auto &ind : population) lengths.push_back(compute_tour_length(pts, ind));
    return lengths;
}

/* ----------- Mutacao (swap) ----------- */
/* Executa um swap simples entre duas posicoes aleatorias */
void swap_mutation_once(vector<int> &ind, uniform_int_distribution<int> &dist_pos){
    int i = dist_pos(rng), j = dist_pos(rng);
    while (j == i) j = dist_pos(rng);
    swap(ind[i], ind[j]);
}

/*
  Aplica mutacoes deterministicas:
  - 'mut_passes' indica quantas vezes iteramos por todos os individuos.
  - em cada passagem, para cada individuo calcula-se p_mut = rank_norm * mut_range.
  - 'max_swaps_per_call' define o numero maximo de swaps possiveis (controla agressividade).
  - Esta estrategia garante que todos os individuos sao testados em cada passagem.
*/
void apply_adaptive_mutations_deterministic(vector<vector<int>> &population, const vector<int> &rank_of,
                     int mut_passes, int max_swaps_per_call, double mutation_range){
    int P = (int)population.size();
    uniform_int_distribution<int> dist_pos(0, (int)population[0].size()-1);
    uniform_real_distribution<double> U(0.0, 1.0);

    // mut_passes vezes percorremos toda a populacao, aplicando a probabilidade de mutacao a cada individuo
    for (int pass = 0; pass < mut_passes; ++pass){
        for (int idx = 0; idx < P; ++idx){
            int rank = rank_of[idx];
            double rank_norm = (P > 1) ? (double)rank / (double)(P - 1) : 0.0;
            double p_mut = rank_norm * mutation_range;
            int attempts = 1 + (int)floor(rank_norm * (double)max_swaps_per_call);
            for (int a = 0; a < attempts; ++a){
                if (U(rng) < p_mut) swap_mutation_once(population[idx], dist_pos);
            }
        }
    }
}

/* ----------- Crossover OX ----------- */
vector<int> order_crossover(const vector<int> &p1, const vector<int> &p2){
    int n = p1.size();
    uniform_int_distribution<int> cut(0, n-1);
    int a = cut(rng), b = cut(rng);
    if (a > b) swap(a, b);

    vector<int> child(n, -1);
    unordered_set<int> used;
    for (int i=a;i<=b;i++){ child[i] = p1[i]; used.insert(p1[i]); }

    int idx = (b + 1) % n;
    for (int i=0;i<n;i++){
        int val = p2[(b + 1 + i) % n];
        if (!used.count(val)){
            child[idx] = val;
            idx = (idx + 1) % n;
        }
    }
    return child;
}

/* ----------- PROGRAMA PRINCIPAL ----------- */
int main(){
    ios::sync_with_stdio(false);
    cin.tie(nullptr);
    cout << fixed << setprecision(3);

    int N = 32;
    cout << "TSP ASCII - GA com elitismo e crossover OX\n";
    cout << "Parametros: N=" << N << " | POP=" << pop_size << " | GERACOES=" << max_generations << "\n";
    cout << "ELITISMO=" << elitism_rate << " | CROSSOVER=" << crossover_rate
         << " | MUT_RANGE=" << mut_range << " | MUT_PGEN=" << mutations_per_gen
         << " | ESTAGNACAO=" << max_stagnation << "\n";

    cout << "Escolha o cenario: 1 = uniforme aleatorio, 2 = circular: " << flush;
    int choice = 1;
    if (!(cin >> choice)) choice = 1;
    cout << "\n";

    vector<Point> points = (choice == 2) ? gen_circle_points(N) : gen_uniform_points(N, 200);

    cout << "Mapa inicial (sem rota):\n";
    print_ascii_map(points);

    vector<vector<int>> population;
    for (int i=0;i<pop_size;i++) population.push_back(create_random_individual(N));

    int max_swaps_per_call = max(1, pop_size / 10);

    double best_global = numeric_limits<double>::infinity();
    int stagnant = 0;

    // controla quando o mapa foi impresso por ultimo (somente atualizar quando mapa for mostrado)
    double last_printed_best = numeric_limits<double>::infinity();

    for (int gen=1; gen<=max_generations; ++gen){
        vector<double> lengths = evaluate_population_lengths(points, population);
        vector<int> order(pop_size); iota(order.begin(), order.end(), 0);
        sort(order.begin(), order.end(), [&](int a, int b){ return lengths[a] < lengths[b]; });

        double best_len = lengths[order[0]], worst_len = lengths[order.back()];
        double mean_len = accumulate(lengths.begin(), lengths.end(), 0.0) / lengths.size();

        if (best_len + 1e-9 < best_global) {
            best_global = best_len;
            stagnant = 0;
        } else stagnant++;

        bool show = (gen==1) || (gen==max_generations) || (print_interval && gen%print_interval==0);

        if (show){
            // decide se printar o mapa ou apenas uma mensagem curta
            bool print_map = false;
            if (last_printed_best == numeric_limits<double>::infinity()){
                // forca primeiro print
                print_map = true;
            } else {
                // tolerancia relativa baseada no ultimo melhor impresso
                double eps = 1e-6 * max(1.0, last_printed_best);
                if (best_len + eps < last_printed_best) print_map = true;
            }

            cout << "\n========== GERACAO " << gen << " ==========\n";
            cout << "Melhor rota (idx " << order[0] << ") = " << best_len << "\n";

            if (print_map){
                cout << "Mapa do melhor:\n";
                print_ascii_map(points, population[order[0]]);
                // atualiza marcador do ultimo print (so aqui, quando efetivamente mostramos o mapa)
                last_printed_best = best_len;
            } else {
                // exibicao curta: apenas informa que o mapa foi suprimido por falta de melhora
                cout << "Mapa suprimido - sem melhora significativa desde a ultima exibicao (melhor = "
                     << last_printed_best << ")\n";
            }

            // estatisticas numericas (sempre mostram, evitam duplicacao)
            cout << "MIN=" << best_len << " | MAX=" << worst_len << " | MEDIA=" << mean_len
                 << " | ESTAG=" << stagnant << "/" << max_stagnation << "\n";
        }

        if (stagnant >= max_stagnation) {
            cout << "\nCriterio de parada: estagnacao por " << max_stagnation << " geracoes.\n";
            break;
        }

        /* ----------- GERA NOVA POPULACAO ----------- */
        vector<vector<int>> new_population;
        int num_elite = max(1, (int)(pop_size * elitism_rate));
        for (int i=0;i<num_elite;i++) new_population.push_back(population[order[i]]);

        vector<double> weights(pop_size);
        for (int i=0;i<pop_size;i++) weights[i] = (double)(pop_size - i);
        discrete_distribution<int> parent_dist(weights.begin(), weights.end());
        uniform_real_distribution<double> U(0.0, 1.0);

        // pre-seleciona indices em termos do vetor 'order'; garante que p1 != p2 ao gerar filhos
        while ((int)new_population.size() < pop_size){
            int idx1 = parent_dist(rng);
            int idx2 = parent_dist(rng);
            while (idx2 == idx1) idx2 = parent_dist(rng); // garante pais distintos
            int p1 = order[idx1];
            int p2 = order[idx2];

            if (U(rng) < crossover_rate){
                new_population.push_back(order_crossover(population[p1], population[p2]));
            } else {
                new_population.push_back(population[p1]);
            }
        }

        population = move(new_population);

        // reavalia a populacao atual e cria ranking antes de aplicar mutacoes
        lengths = evaluate_population_lengths(points, population);
        iota(order.begin(), order.end(), 0);
        sort(order.begin(), order.end(), [&](int a, int b){ return lengths[a] < lengths[b]; });

        vector<int> rank_of(pop_size);
        for (int r=0;r<pop_size;r++) rank_of[order[r]] = r;

        // aplica mutacoes: agora deterministica (percorre toda populacao 'mutations_per_gen' vezes)
        apply_adaptive_mutations_deterministic(population, rank_of, mutations_per_gen, max_swaps_per_call, mut_range);
    }

    cout << "\nEvolucao concluida com elitismo e crossover.\n";
    return 0;
}
