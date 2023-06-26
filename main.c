#include <stdio.h>
#include <stdbool.h>
#include <limits.h>
#include <stdlib.h>

#define MAX_VERTICES 1001

typedef struct {
    int vertex;
    int distance;
} Node;

typedef struct {
    Node heap[MAX_VERTICES];
    int size;
} PriorityQueue;

// Function to initialize the priority queue
void initializeQueue(PriorityQueue* pq) {
    pq->size = 0;
}

// Function to swap two nodes
void swapNodes(Node* a, Node* b) {
    Node temp = *a;
    *a = *b;
    *b = temp;
}

// Function to perform heapify operation
void heapify(PriorityQueue* pq, int index) {
    int smallest = index;
    int left = 2 * index + 1;
    int right = 2 * index + 2;

    if (left < pq->size && pq->heap[left].distance < pq->heap[smallest].distance)
        smallest = left;

    if (right < pq->size && pq->heap[right].distance < pq->heap[smallest].distance)
        smallest = right;

    if (smallest != index) {
        swapNodes(&pq->heap[index], &pq->heap[smallest]);
        heapify(pq, smallest);
    }
}

// Function to insert a node into the priority queue
void enqueue(PriorityQueue* pq, int vertex, int distance) {
    Node newNode;
    newNode.vertex = vertex;
    newNode.distance = distance;

    int i = pq->size;
    pq->heap[i] = newNode;

    while (i > 0 && pq->heap[(i - 1) / 2].distance > pq->heap[i].distance) {
        swapNodes(&pq->heap[(i - 1) / 2], &pq->heap[i]);
        i = (i - 1) / 2;
    }

    pq->size++;
}

// Function to extract the node with the minimum distance from the priority queue
Node dequeue(PriorityQueue* pq) {
    Node minNode = pq->heap[0];
    pq->size--;

    pq->heap[0] = pq->heap[pq->size];
    heapify(pq, 0);

    return minNode;
}

// Function to find the minimum distance vertex from the set of vertices not yet included in the shortest path tree
int minDistance(int dist[], bool shortestPathSet[], int vertices) {
    int min = INT_MAX, minIndex;

    for (int v = 0; v < vertices; v++) {
        if (shortestPathSet[v] == false && dist[v] < min) {
            min = dist[v];
            minIndex = v;
        }
    }

    return minIndex;
}

// Function to find the minimum distance needed to travel from city C to city R without passing through city E
int dijkstra(int graph[MAX_VERTICES][MAX_VERTICES], int src, int dest, int avoid, int vertices) {
    int dist[MAX_VERTICES];               // Array to store the shortest distance from src to i
    bool shortestPathSet[MAX_VERTICES];    // Array to track the shortest path tree vertices
    PriorityQueue pq;

    initializeQueue(&pq);

    // Initialize all distances as infinite and shortestPathSet as false
    for (int i = 0; i < vertices; i++) {
        dist[i] = INT_MAX;
        shortestPathSet[i] = false;
    }

    dist[src] = 0; // Distance from source vertex to itself is always 0

    enqueue(&pq, src, dist[src]);

    // Find the shortest path for all vertices
    while (pq.size > 0) {
        Node currentNode = dequeue(&pq);
        int u = currentNode.vertex;
        shortestPathSet[u] = true;

        if (u == dest)
            break;

        for (int v = 0; v < vertices; v++) {
            if (v != avoid && !shortestPathSet[v] && graph[u][v] && dist[u] != INT_MAX && dist[u] + 1 <= dist[v]) {
                dist[v] = dist[u] + 1;
                enqueue(&pq, v, dist[v]);
            }
        }
    }

    return dist[dest] == INT_MAX ? -1 : dist[dest];
}

int main() {
    int vertices, paths;
    int graph[MAX_VERTICES][MAX_VERTICES];

    while (scanf("%d %d", &vertices, &paths) != EOF) {
        // Initialize graph with 0 weights
        for (int i = 0; i < vertices; i++) {
            for (int j = 0; j < vertices; j++) {
                graph[i][j] = 0;
            }
        }

        // Read paths
        for (int i = 0; i < paths; i++) {
            int u, v;
            scanf("%d %d", &u, &v);
            graph[u][v] = 1;
            graph[v][u] = 1;
        }

        int src, dest, avoid;
        scanf("%d %d %d", &src, &dest, &avoid);

        int stepCount = dijkstra(graph, src, dest, avoid, vertices);
        printf("%d\n", stepCount);
    }

    return 0;
}