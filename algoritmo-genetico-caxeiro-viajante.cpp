#include <bits/stdc++.h>
using namespace std;

/*
  Algoritmo Genético - TSP ASCII (versão com crossover e elitismo)
  - Mantém população fixa (POP_SIZE)
  - Elitismo parcial (mantém ELITISM_RATE dos melhores)
  - Seleção linear por rank
  - Crossover OX (Order Crossover)
  - Mutação adaptativa (mesma do código original)
*/

/* ------------------ PARÂMETROS ------------------ */
const int CANVAS_W = 140;
const int CANVAS_H = 65;
const int CITY_BLOCK = 3;

int POP_SIZE = 16;               // tamanho da população
int NUM_GENERATIONS = 2147483647; // número máximo de gerações
int PRINT_INTERVAL = 10;          // imprime a cada x gerações
double MUT_RANGE = 0.40;          // mutação máxima (pior indivíduo)
int MUTATIONS_PER_GEN = 12;       // mutações por geração
double ELITISM_RATE = 0.10;       // fração de elite mantida (0..1)
double CROSSOVER_RATE = 0.8;      // probabilidade de aplicar crossover
int MAX_STAGNATION = 60;         // gerações sem melhora para parar
/* ------------------------------------------------ */

static std::mt19937 rng((unsigned)chrono::high_resolution_clock::now().time_since_epoch().count());

struct Point { double x, y; };

double dist(const Point &a, const Point &b){
    double dx = a.x - b.x, dy = a.y - b.y;
    return sqrt(dx*dx + dy*dy);
}

/* ----------- Geração de pontos ----------- */
vector<Point> generate_unique_uniform(int n, int precision = 100) {
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

vector<Point> generate_unique_circle(int n, double radius = 0.45, double cx = 0.5, double cy = 0.5) {
    vector<Point> pts; pts.reserve(n);
    for (int i=0;i<n;i++){
        double theta = 2.0 * M_PI * i / n;
        pts.push_back({cx + radius * cos(theta), cy + radius * sin(theta)});
    }
    shuffle(pts.begin(), pts.end(), rng);
    return pts;
}

/* ----------- Funções auxiliares ----------- */
pair<int,int> map_to_canvas(const Point &p){
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

void print_ascii_map(const vector<Point> &pts, const vector<int> &route = {}){
    vector<string> canvas(CANVAS_H, string(CANVAS_W, ' '));
    vector<pair<int,int>> centers; centers.reserve(pts.size());
    for (auto &p : pts) centers.push_back(map_to_canvas(p));

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
                    if (x>=0 && x<CANVAS_W && y>=0 && y < CANVAS_H) canvas[y][x] = 'O';
                }
        }
    }

    string topbot = string(CANVAS_W+2, '-');
    cout << topbot << "\n";
    for (int r=0;r<CANVAS_H;++r) cout << '|' << canvas[r] << "|\n";
    cout << topbot << "\n";
}

/* ----------- GA utilities ----------- */
vector<int> random_individual(int n){
    vector<int> p(n); iota(p.begin(), p.end(), 0);
    shuffle(p.begin(), p.end(), rng); return p;
}

double tour_length(const vector<Point> &pts, const vector<int> &route){
    double s = 0.0;
    for (size_t i=0;i<route.size();++i)
        s += dist(pts[route[i]], pts[route[(i+1)%route.size()]]);
    return s;
}

vector<double> evaluate_population(const vector<Point> &pts, const vector<vector<int>> &pop){
    vector<double> l; l.reserve(pop.size());
    for (auto &ind : pop) l.push_back(tour_length(pts, ind));
    return l;
}

/* ----------- Mutação ----------- */
void swap_mutation_once(vector<int> &ind, uniform_int_distribution<int> &dist_pos){
    int i = dist_pos(rng), j = dist_pos(rng);
    while (j == i) j = dist_pos(rng);
    swap(ind[i], ind[j]);
}

void apply_mutations(vector<vector<int>> &pop, const vector<int> &rank_of,
                     int mutations_per_gen, int max_swaps_per_call, double mut_range){
    int P = (int)pop.size();
    uniform_int_distribution<int> pick_ind(0, P - 1);
    uniform_int_distribution<int> dist_pos(0, (int)pop[0].size()-1);
    uniform_real_distribution<double> U(0.0, 1.0);

    for (int m = 0; m < mutations_per_gen; ++m){
        int idx = pick_ind(rng);
        int rank = rank_of[idx];
        double rank_norm = (P > 1) ? (double)rank / (double)(P - 1) : 0.0;
        double p_mut = rank_norm * mut_range;
        int attempts = 1 + (int)floor(rank_norm * (double)max_swaps_per_call);
        for (int a = 0; a < attempts; ++a)
            if (U(rng) < p_mut) swap_mutation_once(pop[idx], dist_pos);
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

/* ----------- Seleção linear por rank ----------- */
int select_parent(const vector<int> &rank_sorted){
    int P = rank_sorted.size();
    vector<double> weights(P);
    for (int i=0;i<P;i++) weights[i] = (double)(P - i);
    discrete_distribution<int> dist(weights.begin(), weights.end());
    return rank_sorted[dist(rng)];
}

/* ----------- MAIN ----------- */
int main(){
    ios::sync_with_stdio(false);
    cin.tie(nullptr);
    cout << fixed << setprecision(3);

    int N = 16;
    cout << "TSP ASCII - GA com elitismo e crossover OX\n";
    cout << "Parametros: N=" << N << " | POP=" << POP_SIZE << " | GERACOES=" << NUM_GENERATIONS << "\n";
    cout << "ELITISMO=" << ELITISM_RATE << " | CROSSOVER=" << CROSSOVER_RATE
         << " | MUT_RANGE=" << MUT_RANGE << " | MUT_PGEN=" << MUTATIONS_PER_GEN
         << " | ESTAGNACAO=" << MAX_STAGNATION << "\n";

    cout << "Escolha o cenario: 1 = uniforme aleatorio, 2 = circular: " << flush;
    int choice = 1;
    if (!(cin >> choice)) choice = 1;
    cout << "\n";

    vector<Point> pts = (choice == 2) ? generate_unique_circle(N) : generate_unique_uniform(N, 200);

    cout << "Mapa inicial (sem rota):\n";
    print_ascii_map(pts);

    vector<vector<int>> pop;
    for (int i=0;i<POP_SIZE;i++) pop.push_back(random_individual(N));

    int max_swaps_per_call = max(1, POP_SIZE / 10);

    double best_global = numeric_limits<double>::infinity();
    int stagnant = 0;

    for (int gen=1; gen<=NUM_GENERATIONS; ++gen){
        vector<double> L = evaluate_population(pts, pop);
        vector<int> idx(POP_SIZE); iota(idx.begin(), idx.end(), 0);
        sort(idx.begin(), idx.end(), [&](int a, int b){ return L[a] < L[b]; });

        double bestL = L[idx[0]], worstL = L[idx.back()], meanL = accumulate(L.begin(), L.end(), 0.0) / L.size();

        // critério de estagnação
        if (bestL + 1e-9 < best_global) {
            best_global = bestL;
            stagnant = 0;
        } else {
            stagnant++;
        }

        bool show = (gen==1) || (gen==NUM_GENERATIONS) || (PRINT_INTERVAL && gen%PRINT_INTERVAL==0);
        if (show){
            cout << "\n========== GERACAO " << gen << " ==========\n";
            cout << "Melhor rota (idx " << idx[0] << ") = " << bestL << "\n";
            cout << "Mapa do melhor:\n"; print_ascii_map(pts, pop[idx[0]]);
            cout << "MIN=" << bestL << " | MAX=" << worstL << " | MEDIA=" << meanL
                 << " | ESTAG=" << stagnant << "/" << MAX_STAGNATION << "\n";
        }

        if (stagnant >= MAX_STAGNATION) {
            cout << "\nCritério de parada: estagnação por " << MAX_STAGNATION << " gerações.\n";
            break;
        }

        /* ----------- GERA NOVA POPULAÇÃO ----------- */
        vector<vector<int>> new_pop;
        int num_elite = max(1, (int)(POP_SIZE * ELITISM_RATE));
        for (int i=0;i<num_elite;i++) new_pop.push_back(pop[idx[i]]);

        uniform_real_distribution<double> U(0.0, 1.0);
        while ((int)new_pop.size() < POP_SIZE){
            int p1 = select_parent(idx);
            int p2 = select_parent(idx);
            if (U(rng) < CROSSOVER_RATE){
                vector<int> child = order_crossover(pop[p1], pop[p2]);
                new_pop.push_back(child);
            } else {
                new_pop.push_back(pop[p1]);
            }
        }

        // aplica mutação adaptativa
        vector<int> rank_of(POP_SIZE);
        for (int r=0;r<POP_SIZE;r++) rank_of[idx[r]] = r;
        apply_mutations(new_pop, rank_of, MUTATIONS_PER_GEN, max_swaps_per_call, MUT_RANGE);

        pop = std::move(new_pop);
    }

    cout << "\nEvolucao concluida com elitismo e crossover.\n";
    return 0;
}
