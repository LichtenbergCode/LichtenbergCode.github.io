
# Graph Theory 

> Author Recognition: William Fiset

Graph theory is the mathematical theory of the properties and applications of graphs (networks). 

## Types of Graphs 

### Undirected graph 
An **undirected graph** is a graph in which edges have no orientation. The edge (u, v) is identical to the edge (v, u). - Wiki 




> In the graph above, the nodes could represent cities and an edge could represent a bidirectional road. 


### Directed Graph (Digraph)
A **directed graph** or **digraph** is a graph in which edges have orientations, For example, the edge (u, v) is the edge from node u to node v. 



> In the graph above, the nodes could represent people and an edge (u, v) could represent that person u bought person v a gift.

### Weighted Graphs 
Many graphs can have edges that contain a certain weight to represent an arbitrary value such as cost, distance, quantity, etc... 

### Trees ! 
A **tree** is an **undirected graph with no cycles**.
Equivalently, it is a connected graph with N nodes and N-1 edges. 

### Rooted Trees ! 
A **rooted tree** is a tree with **a designated root node** where every edge either points away from or towards the root node. When edges point away from the root the graph is called an arborescence (out-tree) and anti-arborescence (in-tree) otherwise. 

### Directed Acyclic Graphs (DAGs)
DAGs are directed graphs with no cycles. These graphs play an important role in representing structures with dependencies. Several efficient algorithms exist to operates on DAGs.

Cool fact: All out-trees are DAGs but not all DAGs are out-trees. 

### Bipartite Graph 
A bipartite graph is one whose vertices can be split into two independent groups U, V such that every edge connects betweens U and V. 
Other definitions exist such as: The graph is two colourable or there is no odd length cycle. 

### Complete graphs 
A complete graph is one where there is a unique edge between every pair of nodes. A complete graph with n vertices is denoted as the graph Kn. 

### Adjacency Matrix 
A **adjacency matrix** m is a very simple way to represent a graph. The idea is that the cell m[i][j] represents the edges weight of going from node i to node j. 

> Note: It is often assumed that the edge of going from a node to itself has a cost of zero. 

## Binary Trees 
A binary tree is a tree data structure in which each node has at most two children, commonly called the left child and the right child. <br>

Terminology: 

* **Tree**: A non-linear data structure where nodes are organized in a hierarchy
* **Leaf node**: A leaf node is anode with no children. 
* **Branch node** (internal node): It ss any node that has at least one child. 
* **Parent** : It is a node that has one or more 
children. 
* **Child**: It is a node that descends directly from a parent. 
* **Siblings**: They are nodes that share the same parent. 
* **Subtree**: A subtree is a node plus all of its descendants. 
* **Size**: The total number of nodes in a tree or subtree. 
* **Depth**: Number of edges from the root to that node. 
* **Height**: Number of edges from that node to the deepest leaf below it. 



