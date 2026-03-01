

# Trees 

A **tree** is a hierarchical, non-linear data structure composed of nodes connected by edges, defined by the following structural properties:
1. There is a single distinguished node called the **root**.
2. Every node (except the root) has exactly one **parent**.
3. Each node may have zero or more **children**.
4. The structure contains **no cycles**.
5. There is exactly one unique path between the root and any node. 
6. A tree with **n nodes** has exactly **n-1 edges** and is a connected acyclic graph. 

> These properties make trees a special case of graphs: they are connected and acyclic. 

## Core Terminology 

### Node, Root and Edge definitions: 

#### Node 
A node is the fundamental unit of a tree. 

Each node typically contains: 

* A **value** (data)
* References (or pointers) to its **children**

in Abstract terms, a node represents a state in a hierarchical relationship. 

In memory, nodes form a recursive structure: 
each node may reference other nodes of the same type.

```python 
class Node: 
    def __init__(self, val, left, right): 
        self.val = val
        self.left = left 
        self.right = right
```

#### Root 
The **root** is the unique top-level node of the tree. 

key properties: 

* It has **no parent**.
* There is exactly one root in a tree. 
* Every other node is reachable from it. 

> Removing the root disconnects the entire structure. 

#### Edge 
An edge is the connection between two nodes. 

In a tree with: 

* **n** nodes
* There are exactly **n - 1** edges

> edges define structure, not just relationships. 

### Parent, Child, Siblings, Leafs and Internal Nodes:

#### Parent and child
If there is an edge from node **A** to node **B**: 
* A is the **parent**
* B is the **Child**

> Every node (except the root) has exactly one parent. 

This ensures:

* No cycles 
* Unique upward traversal 
* A strictly hierarchical structure

This also implies there is exactly one path prom the root to any node.

#### Siblings
Nodes that share the same parent are **siblings**. 

This is often important in: 

* Level-order traversal 
* Multi-branch comparisons
* N-ary tree problems

#### Leaf (External Node)
A **leaf** is a node with no children. 

Properties: 
* It represents a termination point. 
* It defines the boundary of recursion. 
* Many algorithms stop processing at leaves.

Example uses: 
* Counting leaves 
* Computing height 
* Evaluating expression trees

#### Internal Nodes 
An **internal node** is any node that has at least one child. 

Internal nodes are branching points. 

In many algorithms: 

* Leaves define base cases. 
* Internal nodes combine results from subtrees. 


### Path, Degree, Depth and Height


#### Path
A **path** is a sequence of nodes connected by edges. 

Properties in trees: 

* There is exactly one unique simple path between any two nodes. 
* No cycles exist. 

#### Degree 
The degree of a node is the number of its children. 

In a: 
* Binary tree -> degree <= 2
* General tree -> degree can vary

> The degree of a tree is the maximum degree of any node

Example Node Degree 2 and Degree 3
```python
class NodeDegree2: 
    def __init__(self, val = 0, left = None, right = None): 
        self.val = val
        self.left = left # points to the right node 
        self.right = right # points to the left node

class NodeDegree3: 
    def __init__(self, val= 0, left= None, center = None, right = None):
        self.val = val
        self.left = left 
        self.right = right 
        self.center = center 
```

#### Depth 
The depth of a node is the number of edges from the root to that node. 

Formally: 
```bash
depth(root) = 0 
depth(child) = depth(parent) + 1 
```
Depth measures vertical position in the hierarchy. 

It is often computed during DFS. 

#### Height
The height of a node is the the number of edges on the longest downward path from that node to a leaf.

Important distinction: 
* Depth measures distance **upward to root**
* Height measures distance **downward to leaves**

Formally: 
``` bash
height(leaf) = 0
height(node) = 1 + max(heigh(children))
```
> The height of the tree is the height of the root. 

Height determines: 
* Worst-case search complexity 
* Balance properties 
* Recursive stack depth

### Subtree, Ancestors and Descendants 

#### Subtree 
A **subtree** rooted at node v consists of: 

* Node v 
* All of its descendants 

Crucially:
> Every subtree is itself a valid tree

#### Ancestor ans Descendant 
If there is a path from A to node B: 

* A is an ancestor of B 
* B is a descendant of A 

Special case: 

* The root is an ancestor of every node. 
* Every node is a a descendant of itself (depending on definition).

Ancestor relationships are central to: 

* LCA problems
* Path queries 
* Hierarchical reasoning

## Recursive Definition 
A tree can also be defined recursively: 
> A tree is either empty, or it consist of a root node and zero or more subtrees, each of which is itself a tree. 

This recursive structure is fundamental Most tree algorithms (e.g., Depth-First Search) naturally follow this recursive definition.

Example: 
```python
def dib(n): 
    if n < = 1:
        return  
    dib(n-1)
    dib(n-1)
```

## Binary Tree


### Pre-order Traversal 
```python

class Node: 
    def __init__(self, val = 0, left = None, right = None):
        self.val = val 
        self.left = left 
        self.right = right

result = []

def preorder(root: Node):
    if not root: 
        return 

    result.append(root.val)
    preorder(root.left)
    preorder(root.right)
    return result
```

### In-order Traversal
```python
class Node: 
    def __init__(self, val = 0, left = None, right = None): 
        self.val = val 
        self.left = left 
        self.right = right

result = []
def inorder(node: Node):
    if not node: 
        return 
    
    inorder(node.left)
    result.append(node.val)
    inorder(node.right)
    return result
```

### Post-order Traversal
```python
class Node: 
    def __init__(self, val = 0, left = None, right = None): 
        self.val = val 
        self.left = left
        self.right = right
    
    result = []
    def postorder(node): 

        postorder(node.left)
        postorder(node.right)
        result.append(node.val)
```

## Binary Search Tree

## Heap (Priority Queue)

## Trie 

## N-ary Tree

## Segmented Tree