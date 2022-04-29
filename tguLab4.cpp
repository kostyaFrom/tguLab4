#include <iostream>
#include <list>
#include <limits.h>
#include <cstring>
#include <iomanip>
#include <sstream>
#include <fstream>
#include <vector>
/* Define Infinite as a large enough
value.This value will be used for
vertices not connected to each other */
//#define INF 99999
#define V 9

int graph[V][V];
using namespace std;

//// ======================== 1. Реализуйте программу, в которой выполняется алгоритм обхода графа на основе поиска в глубину. Depth First Search for a Graph ========================
//class Graph {
//    int numVertices;
//    list<int>* adjLists;
//    bool* visited;
//
//public:
//    Graph(int V);
//    void addEdge(int src, int dest);
//    void DFS(int vertex);
//};
//
//// Initialize graph
//Graph::Graph(int vertices) {
//    numVertices = vertices;
//    adjLists = new list<int>[vertices];
//    visited = new bool[vertices];
//}
//
//// Add edges
//void Graph::addEdge(int src, int dest) {
//    adjLists[src].push_front(dest);
//}
//
//// DFS algorithm
//void Graph::DFS(int vertex) {
//    visited[vertex] = true;
//    list<int> adjList = adjLists[vertex];
//
//    cout << vertex << " ";
//
//    list<int>::iterator i;
//    for (i = adjList.begin(); i != adjList.end(); ++i)
//        if (!visited[*i])
//            DFS(*i);
//}
//// ======================== Depth First Search for a Graph ========================
//
//
//
//
//// ======================== 2. Реализуйте программу, в которой выполняется алгоритм обхода графа на основе поиска в ширину. Breadth First Search for a Graph ========================
//class Graph {
//
//    int V;    // No. of vertices
//
//    // Pointer to an array containing adjacency
//    // lists
//    list<int>* adj;
//
//public:
//
//    Graph(int V);  // Constructor
//
//    // function to add an edge to graph
//    void addEdge(int v, int w);
//
//    // prints BFS traversal from a given source s
//    void BFS(int s);
//};
//
//Graph::Graph(int V) {
//    this->V = V;
//    adj = new list<int>[V];
//}
//
//void Graph::addEdge(int v, int w) {
//
//    adj[v].push_back(w); // Add w to v’s list.
//}
//
//void Graph::BFS(int s) {
//    // Mark all the vertices as not visited
//    bool* visited = new bool[V];
//    for (int i = 0; i < V; i++)
//        visited[i] = false;
//
//    // Create a queue for BFS
//    list<int> queue;
//
//    // Mark the current node as visited and enqueue it
//    visited[s] = true;
//    queue.push_back(s);
//
//    // 'i' will be used to get all adjacent
//    // vertices of a vertex
//    list<int>::iterator i;
//
//    while (!queue.empty()) {
//        // Dequeue a vertex from queue and print it
//        s = queue.front();
//        cout << s << " ";
//        queue.pop_front();
//
//        // Get all adjacent vertices of the dequeued
//        // vertex s. If a adjacent has not been visited,
//        // then mark it visited and enqueue it
//        for (i = adj[s].begin(); i != adj[s].end(); ++i) {
//            if (!visited[*i]) {
//                visited[*i] = true;
//                queue.push_back(*i);
//            }
//        }
//    }
//}
//
//// ======================== END ===== Breadth First Search for a Graph ========================
//
//// =================== 3. Используйте обход графа в ширину для определения всех вершин графа, находящихся на фиксированном расстоянии d от данной вершины.=========================
//vector<vector<int>> g;
//int n;
//vector<bool> used(n);
//
//void bfs(int start) {
//    used.clear();
//
//    queue<int> q;
//    q.push(start);
//    vector<int> d(n), p(n);
//    used[start] = true;
//    p[start] = -1;
//    while (!q.empty()) {
//        int v = q.front();
//        q.pop();
//        for (size_t i = 0; i < g[v].size(); ++i) {
//            int to = g[v][i];
//            if (!used[to]) {
//                used[to] = true;
//                q.push(to);
//                d[to] = d[v] + 1;
//                p[to] = v;
//            }
//        }
//    }
//}
//
//int main() {
//    // ввод графа и остального стафа
//
//    // требуемая вершина
//    int to;
//    cin >> to;
//
//    int ans = 0;
//
//    for (int i = 0; i < n; ++i) {
//        bfs(i);
//
//        if (!used[to])
//            cout << "No path!";
//        else {
//            vector<int> path;
//
//            for (int v = to; v != -1; v = p[v])
//                path.push_back(v);
//
//            if (path.size() == d)
//                ++ans;
//        }
//    }
//}
//
//// =================== 3. Используйте обход графа в ширину для определения всех вершин графа, находящихся на фиксированном расстоянии d от данной вершины. END =========================
//
//
//
//
//
//// ========================== 4. Реализуйте программы, в которых выполняются алгоритм Дейкстры ==================================================
//// A C++ program for Dijkstra's single source shortest path algorithm.
//// The program is for adjacency matrix representation of the graph
//
//
//// A utility function to find the vertex with minimum distance value, from
//// the set of vertices not yet included in shortest path tree
//int minDistance(int dist[], bool sptSet[]) {
//
//    // Initialize min value
//    int min = INT_MAX, min_index;
//
//    for (int v = 0; v < V; v++)
//        if (sptSet[v] == false && dist[v] <= min)
//            min = dist[v], min_index = v;
//
//    return min_index;
//}
//
//// A utility function to print the constructed distance array
//void printSolution(int dist[]) {
//    cout << "Vertex \t Distance from Source" << endl;
//    for (int i = 0; i < V; i++)
//        cout << i << " \t\t" << dist[i] << endl;
//}
//
//// Function that implements Dijkstra's single source shortest path algorithm
//// for a graph represented using adjacency matrix representation
//void dijkstra(int graph[V][V], int src) {
//    int dist[V]; // The output array.  dist[i] will hold the shortest
//    // distance from src to i
//
//    bool sptSet[V]; // sptSet[i] will be true if vertex i is included in shortest
//    // path tree or shortest distance from src to i is finalized
//
//    // Initialize all distances as INFINITE and stpSet[] as false
//    for (int i = 0; i < V; i++)
//        dist[i] = INT_MAX, sptSet[i] = false;
//
//    // Distance of source vertex from itself is always 0
//    dist[src] = 0;
//
//    // Find shortest path for all vertices
//    for (int count = 0; count < V - 1; count++) {
//        // Pick the minimum distance vertex from the set of vertices not
//        // yet processed. u is always equal to src in the first iteration.
//        int u = minDistance(dist, sptSet);
//
//        // Mark the picked vertex as processed
//        sptSet[u] = true;
//
//        // Update dist value of the adjacent vertices of the picked vertex.
//        for (int v = 0; v < V; v++)
//
//            // Update dist[v] only if is not in sptSet, there is an edge from
//            // u to v, and total weight of path from src to  v through u is
//            // smaller than current value of dist[v]
//            if (!sptSet[v] && graph[u][v] && dist[u] != INT_MAX
//                && dist[u] + graph[u][v] < dist[v])
//                dist[v] = dist[u] + graph[u][v];
//    }
//
//    // print the constructed distance array
//    printSolution(dist);
//}
//
//// ========================== алгоритм Дейкстры END ==================================================
//
//// ========================== 4. Реализуйте программы, в которых выполняются алгоритм Флойда Start ==================================================
//
//// Solves the all-pairs shortest path
//// problem using Floyd Warshall algorithm
//void floydWarshall(int graph[][V]) {
//    /* dist[][] will be the output matrix
//    that will finally have the shortest
//    distances between every pair of vertices */
//    int dist[V][V], i, j, k;
//
//    /* Initialize the solution matrix same
//    as input graph matrix. Or we can say
//    the initial values of shortest distances
//    are based on shortest paths considering
//    no intermediate vertex. */
//    for (i = 0; i < V; i++)
//        for (j = 0; j < V; j++)
//            dist[i][j] = graph[i][j];
//
//    /* Add all vertices one by one to
//    the set of intermediate vertices.
//    ---> Before start of an iteration,
//    we have shortest distances between all
//    pairs of vertices such that the
//    shortest distances consider only the
//    vertices in set {0, 1, 2, .. k-1} as
//    intermediate vertices.
//    ----> After the end of an iteration,
//    vertex no. k is added to the set of
//    intermediate vertices and the set becomes {0, 1, 2, ..
//    k} */
//    for (k = 0; k < V; k++) {
//        // Pick all vertices as source one by one
//        for (i = 0; i < V; i++) {
//            // Pick all vertices as destination for the
//            // above picked source
//            for (j = 0; j < V; j++) {
//                // If vertex k is on the shortest path from
//                // i to j, then update the value of
//                // dist[i][j]
//                if (dist[i][j] > (dist[i][k] + dist[k][j])
//                    && (dist[k][j] != INF
//                        && dist[i][k] != INF))
//                    dist[i][j] = dist[i][k] + dist[k][j];
//            }
//        }
//    }
//
//    // Print the shortest distance matrix
//    printSolution(dist);
//}
//
///* A utility function to print solution */
//void printSolution(int dist[][V]) {
//    for (int i = 0; i < V; i++) {
//        for (int j = 0; j < V; j++) {
//            if (dist[i][j] == INF)
//                cout << "INF"
//                << "     ";
//            else
//                cout << dist[i][j] << "     ";
//        }
//        cout << endl;
//    }
//}
//
//// ========================== алгоритм Флойда END ==================================================
//
//// ===================== 5. Реализуйте программу, в которой определяется минимальное остовное дерево графа. Prim’s algorithm =======================
//
//// функция для поиска минимального значения
//int minKey(int key[], bool mstSet[]) {
//    // Initialize min value
//    int min = INT_MAX, min_index;
//
//    for (int v = 0; v < V; v++)
//        if (mstSet[v] == false && key[v] < min)
//            min = key[v], min_index = v;
//
//    return min_index;
//}
//
//// A utility function to print the
//// constructed MST stored in parent[]
//void printMST(int parent[], int graph[V][V]) {
//    cout << "Edge \tWeight\n";
//    for (int i = 1; i < V; i++)
//        cout << parent[i] << " - " << i << " \t" << graph[i][parent[i]] << " \n";
//}
//
//// Function to construct and print MST for
//// a graph represented using adjacency
//// matrix representation
//void primMST(int graph[V][V]) {
//    // Array to store constructed MST
//    int parent[V];
//
//    // Key values used to pick minimum weight edge in cut
//    int key[V];
//
//    // To represent set of vertices included in MST
//    bool mstSet[V];
//
//    // Initialize all keys as INFINITE
//    for (int i = 0; i < V; i++)
//        key[i] = INT_MAX, mstSet[i] = false;
//
//    // Always include first 1st vertex in MST.
//    // Make key 0 so that this vertex is picked as first vertex.
//    key[0] = 0;
//    parent[0] = -1; // First node is always root of MST
//
//    // The MST will have V vertices
//    for (int count = 0; count < V - 1; count++) {
//        // Pick the minimum key vertex from the
//        // set of vertices not yet included in MST
//        int u = minKey(key, mstSet);
//
//        // Add the picked vertex to the MST Set
//        mstSet[u] = true;
//
//        // Update key value and parent index of
//        // the adjacent vertices of the picked vertex.
//        // Consider only those vertices which are not
//        // yet included in MST
//        for (int v = 0; v < V; v++)
//
//            // graph[u][v] is non zero only for adjacent vertices of m
//            // mstSet[v] is false for vertices not yet included in MST
//            // Update the key only if graph[u][v] is smaller than key[v]
//            if (graph[u][v] && mstSet[v] == false && graph[u][v] < key[v])
//                parent[v] = u, key[v] = graph[u][v];
//    }
//
//    // print the constructed MST
//    printMST(parent, graph);
//}
//// ===================== 5. Реализуйте программу, в которой определяется минимальное остовное дерево графа. =======================

// ===================== 6. Чтение из файла =======================


void readGraphFromFile() {   

    string line;    
    
    //файловый поток и связываем его с файлом
    ifstream inFile("C:\\Users\\kostyan\\Development\\graph.txt");

    if (inFile.is_open()) {        

        int j{ 0 };

        while (getline(inFile, line)) {

            for (auto i { 0 }; i < 8; i++) {
                graph[j][i] = line[i];
            }
            j++;           
        }      

        inFile.close();//под конец закроем файла

    } else {
        //Если открытие файла прошло не успешно
        cout << "Файл не найден.";
    }

    for (auto i{ 0 }; i < 8; i++) {
        for (auto j{ 0 }; j < 8; j++) {
            cout << graph[i][j] << endl;
        }        
    }
    
}

int main() {
    readGraphFromFile();

    return 0;
}