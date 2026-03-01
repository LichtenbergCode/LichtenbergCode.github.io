# Data structures 

Data structures are a way of organizing and storing information in a computer or a program. They help in efficiently and manipulating data. 

## Data structures vs an Algorithm 

**An algorithm** is a set of well defined instructions, a step-by-step procedure designed to solve a specific problem or perform a particular task. 

**A data structure** is a way of *organizing* and *storing* data in a computer or a programming language. It defines the relationship between the data elements, the operations that can be performed on the data, and the rules or constraints for accessing and modifying the data. 

*Data structures provide the organization and representation of information
(the data), and algorithms serve as instructions for transforming that data.*

## Array 

Arrays organize data by holding a collection of elements and making them accessible through an index. 

| action   | time    |
| -------- | ------- |
| lookup   | O(1)    |
| push     | O(1)    | 
| insert   | O(n)    |
| delete  | O(n)    |

*if all you need is to store some data and iterate over it, that is go one by one, step by step this data structure is the best option*

### Static Array

A static array is an array whose size is fixed at compile time (or at creation time) and cannot change during execution. 

* Memory is allocated once 
* Stored in contiguous memory 
* No resizing possible 

**Characteristics**

* Fixed length 
* Very memory-efficient
* No overhead for resizing 
* Used in low-level languages or embedded systems 

```c
int arr[5] = {1, 2, 3, 4, 5}; //c array 
```

When to use: 
* When the size is known in advance 
* Performance-critical systems
* Embedded / real-time systems


### Dynamic Array 
A dynamic array is an array that can grow or shrink at runtime by allocating new memory and copying elements when needed. 

* Still stored in contiguous memory. 
* Automatically resizes.
* Uses extra capacity to optimize insertions. 

**Characteristics**
* flexible size 
* Slight memory overhead 
* Resizing involves copying elements 
* Most high-level languages use this
```python
abc = [1, 2, 3, 4, 5] # Python list (dynamic array)
```

### Key Differences between an Static and a dynamic array
| Feature           | Static Array | Dynamic Array               |
| ----------------- | ------------ | --------------------------- |
| Size              | Fixed        | Resizable                   |
| Memory allocation | Once         | Multiple times              |
| Contiguous memory | Yes          | Yes                         |
| Access time       | O(1)         | O(1)                        |
| Append            | ❌           | O(1) amortized             |
| Memory overhead   | Minimal      | Extra capacity              |
| Example           | `int a[10]`  | Python `list`, C++ `vector` |

### Python Unsorted array 

This code implements an unsorted array data structure with fixed maximum capacity, backed by a typed low-level array. 

Unlike Python lists, this structure does not resize dynamically and does not preserve order, which allows some operations to be faster.



repo_url: 'https://github.com/mlarocca/grokking_data_structures/blob/main/python/arrays/unsorted_array.py'


### Python List operation 

??? example "Python List Operations"
    ```python
    # create a list 
    abc = [i for i in range(1, 6)] # O(n) -> creating a list takes n assignments

    # ===============================
    # ACCESS
    # ===============================

    x = abc[0]                       # O(1) → direct index access
    y = abc[-1]                      # O(1)

    # ===============================
    # SEARCH
    # ===============================

    3 in abc                         # O(n) → linear search
    abc.index(4)                     # O(n)

    # ===============================
    # INSERTION
    # ===============================

    abc.append(6)                    # O(1) amortized → add at the end
    abc.insert(0, 0)                 # O(n) → shifts elements
    abc.extend([7, 8, 9])             # O(k) → k elements added

    # ===============================
    # DELETION
    # ===============================

    abc.pop()                        # O(1) → remove last element
    abc.pop(0)                       # O(n) → shifts elements
    abc.remove(3)                    # O(n) → search + shift
    del abc[2]                       # O(n)

    # ===============================
    # UPDATE
    # ===============================

    abc[1] = 100                     # O(1) → direct replacement

    # ===============================
    # SIZE
    # ===============================

    len(abc)                         # O(1) → Python stores list size

    # ===============================
    # ITERATION
    # ===============================

    for item in abc:                 # O(n)
        pass

    # ===============================
    # COPYING
    # ===============================

    abc_copy = abc.copy()             # O(n)
    abc_copy2 = abc[:]                # O(n)
    abc_copy3 = list(abc)             # O(n)

    # ===============================
    # SORTING
    # ===============================

    abc.sort()                       # O(n log n) → Timsort
    sorted_abc = sorted(abc)         # O(n log n)

    # ===============================
    # REVERSE
    # ===============================

    abc.reverse()                    # O(n)

    # ===============================
    # CONCATENATION
    # ===============================

    new_list = abc + [10, 11]         # O(n + m)

    # ===============================
    # CLEAR
    # ===============================

    abc.clear()                      # O(n) → removes references

    # ===============================
    # COMPARISON
    # ===============================

    [1, 2, 3] == [1, 2, 3]            # O(n) → element-by-element comparison

    # ===============================
    # COUNT
    # ===============================

    abc.count(2)                     # O(n)

    # ===============================
    # MIN / MAX / SUM
    # ===============================

    min(abc)                         # O(n)
    max(abc)                         # O(n)
    sum(abc)                         # O(n)

    ```

### Array Summary 

* Arrays are a way of storing a collection of elements and efficiently accessing them by position. 
* The term array is usually intended as a synonym for a statically-sized array (or "static array" for short), a collection of elements, accessed by an index, where the number of elements is fixed for the entire lifetime of the collection. 
* Dynamically-sized arrays are also possible: they behave like static arrays, except that the number of elements they contain can change. 
* Many programming languages, such as C or Java offer static arrays as built-in feature. 
* Arrays can be initialized at compile time. If a language allows you to skip initialization, then the initial value of the array's elements depends on the language. 
* Arrays can be nested; you can create an array of arrays. For static arrays, we call them multidimensional arrays or matrices. 
* If we do not mind the order of its elements, adding and removing elements to and from an array can be done easily. 
* We can search all (generic) arrays by traversing them until we find what we are looking for. 
* It's possible to use arrays for many applications, for example counting items and computing statistics are perfect use cases for arrays. 

## Sorted Arrays 
A sorted array is an array whose elements are arranged in a specific order, usually ascending or descending. 

**A sorted array is a contiguous structure that sacrifices update speed to gain fast search**

**Order is enforced**: every element knows its position relative to others. 

**Example (ascending)**
```python
[1, 3, 5, 8, 12]
```

**Example Descending:**
```python
[12, 8, 5, 3, 1]
```

### Why sorting matters 
Once an array is sorted, you unlock **faster algorithms**.

**Big win:** <br>
• Binary Search -> O(log n) <br><br>
**Without sorting:** <br>
• Linear search -> O(n) <br><br>

### Time complexity of operations 
| Operation            | Complexity | Why              |
| -------------------- | ---------- | ---------------- |
| Access by index      | O(1)       | Array property   |
| Search (binary)      | O(log n)   | Divide & conquer |
| Insert (keep sorted) | O(n)       | Shift elements   |
| Delete               | O(n)       | Shift elements   |


### Sorted Array vs Unsorted Array
| Feature          | Unsorted   | Sorted       |
| ---------------- | ---------- | ------------ |
| Search           | O(n)       | **O(log n)** |
| Insert           | O(1) (end) | O(n)         |
| Delete           | O(n)       | O(n)         |
| Maintenance cost | Low        | Higher       |

### Typical use cases
* Binary search 
* Range queries 
* Two-pointer techniques
* Merging datasets 
* Deduplication

### Python Sorted Arrays Example: 


### Summary 

* A sorted Array is an array whose elements are kept in order as they change.
* To maintain the elements of an array in order; we need a different approach when inserting and deleting elements. These methods must preserve the order nd therefore require more effort than their counterparts for unsorted arrays. 
* On an already sorted array, we can run **binary search**, a search algorithm that can find a match by looking at fewer elements than **linear search** (which simply scans all elements until it finds a match). 
* With sorted arrays, you have faster search, but you also have an extra cost to keep them sorted: therefore, they are to be preferred when there is a high read/write ratio (many more calls to the binary_search method than to insert and delete).

...

## Hash tables 
In python Hash tables are dictionaries

**key**: <br>
**value**: <br>



## Linked Lists 

A linked list is a linear data structure made of nodes where each node stores data and one or more pointers (references) to other nodes. Unlike arrays, nodes can be scattered in memory and are connected by these pointers. <br>

**O Time Complexity**<br>
prepend -> O(1) <br>
append -> O(1) <br>
lookup -> O(n) <br>
insert -> O(n) <br>
delete -> O(n) <br>


**Node**: typically stores a value and a pointer/reference to another node (or two pointers for doubly linked lists).<br>
**Head**: reference to the first node. <br>
**Tail**: reference to the last node (optional but common).<br>
**Traversal**: starts at head and follow next pointers until you reach the end (None) or come back to head for circular lists.<br> 
**Variants**: 
- Singly linked list: nodes have one pointer (next). Forward-only traversal. 
- Doubly linked list: nodes have next and prev. pointers. Can traverse both directions. 
- Circular linked list: last node points bask to the first (no null terminator). Can be singly or doubly circular. 
- Sentinel (dummy) node: a special node used to simplify edge-case code (insert/remove at ends).


### Singly-linked lists
A singly-linked list is a linked list with a single link per node, pointing to the next element in the list.<br>

The first element of a linked list is called its **head**, and the last element of the lis is called its **tail**. In a singly-linked list, the characteristic of a head node is that no other node points to it, and so we need to store a link to the beginning (aka head) of the list somewhere in a variable. <br>

**Singly Linked List Template**
```python 

class Node: 
    def __init__(self, value): 
        self.value = value
        self.next = None 

class SinglyLinkedList: 
    def __init__(self): 
        self.head = None
        self.tail = None # optional, for O(1) append
    
    def prepend(self, value): 
        node = Node(value)
        node.next = self.head
        self.head = node 
        if self.tail is None: 
            self.tail = node
        
    def append(self, value): 
        node = Noe(value)
        if not self.head: 
            self.head = self.tail = node
        else: 
            self.tail.next = node 
            self.tail = node 
    
    def find(self, value): 
        cur = self.head
        while cur and cur.next: # Reads the value 
            if cur.value == value: 
                return cur 
            cur = cur.next
        return None
    
    def remove(self, value): 
        prev = None
        cur = self.head 
        while cur: 
```
**Singly Linked List Explanation**
* **Node**: simple container holding a value and reference to the next node. 
* **SinglyLinkedList**: a chain of Node objects with references to the head (first) and tail (last) node. It supports O(1) prepend and append (because tail is tracked), searching (find), removal by value, and iteration.

```python 
class Node:  
    def __init_(self, value): 
        self.value = value 
        self.next = None  
```
* Node.value: stores any Python object(number, string, another object, etc). 
* Node.next: references to the next Node in the list (None means end-of-list).
* Nodes themselves are simple and truthy object (useful later when checking prev). 
```python
class SinglyLinkedList: 
    def __init__(self): 
        self.head = None 
        self.tail = None #optional, for O(1) append
```
* head: points to the first Node or None when the list is empty. 
* tail: points to the last Node or None when empty. Maintaining tail lets append run un O(1).<br>

**prepend** (insert at front)
```python
    def prepend(self, value): 
        node = Node(value) # Creating a new Node 
        node.next = self.head #  
        self.head = node #  
        if self.tail is None: #  
            self.tail = node  # 
```
* Create a new node. 
* Make its next point to current head (can be None).
* Update head to the new node. 
* If the 


### Doubly-linked lists
A doubly linked list is a linked data structure where each node stores a value and two pointers: one to the next node and one to the previous node: This allows traversal forward and backward, and enables O(1) removal or insertion at a known node (because you can access both neighbors directly) <br>

**Time Complexity**
* Access by index: O(n) 
* Append (push back): O(1)
* Prepend (push front): O(1)
* Insert after / before a given node: O(1) (given the node)
* Remove a given node: O(1) (given the node)
* Find by value (search): O(n)
* Pop front / pop back: O(1)
* Iterate forward or backward: O(n) 

**Doubly Linked List Template**
```python
class DNode:
    def __init__(self, value):
        self.value = value
        self.prev = None
        self.next = None

class DoublyLinkedList:
    def __init__(self):
        self.head = None
        self.tail = None
        self._size = 0

    def __len__(self):
        return self._size

    def append(self, value):
        node = DNode(value)
        if not self.head:              # empty list
            self.head = self.tail = node
        else:
            node.prev = self.tail
            self.tail.next = node
            self.tail = node
        self._size += 1
        return node

    def prepend(self, value):
        node = DNode(value)
        if not self.head:
            self.head = self.tail = node
        else:
            node.next = self.head
            self.head.prev = node
            self.head = node
        self._size += 1
        return node

    def insert_after(self, node, value):
        if node is None:
            return self.prepend(value)
        if node is self.tail:
            return self.append(value)
        new_node = DNode(value)
        nxt = node.next
        node.next = new_node
        new_node.prev = node
        new_node.next = nxt
        nxt.prev = new_node
        self._size += 1
        return new_node

    def remove_node(self, node):
        if node is None:
            return False
        if node.prev:
            node.prev.next = node.next
        else:
            self.head = node.next
        if node.next:
            node.next.prev = node.prev
        else:
            self.tail = node.prev
        node.prev = node.next = None  # help GC / avoid accidental reuse
        self._size -= 1
        return True

    def find(self, value):
        cur = self.head
        while cur:
            if cur.value == value:
                return cur
            cur = cur.next
        return None

    def pop_front(self):
        if not self.head:
            raise IndexError("pop from empty list")
        val = self.head.value
        self.remove_node(self.head)
        return val

    def pop_back(self):
        if not self.tail:
            raise IndexError("pop from empty list")
        val = self.tail.value
        self.remove_node(self.tail)
        return val

    def __iter__(self):
        cur = self.head
        while cur:
            yield cur.value
            cur = cur.next

    def iter_reverse(self):
        cur = self.tail
        while cur:
            yield cur.value
            cur = cur.prev

    def to_list(self):
        return list(self)
```

### Circular linked lists


## Stacks 
A stack is a linear data structure that follows Last-in, First-Out (LIFO): the last item pushed is the first one popped. Core operations (push, pop, peek) are O(1) time in typical implementations. 

* push (x): place x on top of the stack. 
* pop (): remove and return the top element. 
* Peek/Top(): Read the top element without removing it. 
* is_empty(): test whether the stack has no elements. 
* size(): number of elements.  

**Stacks Template**
```python 
from typing import Iterable, TypeVar

T = TypeVar("T", int, float) # generic variable

class Stack:
    def __init__(self, elements: Iterable[T] | None = None):
        self._stack_elements: list[T] = []
        self._min_elements: list[T] = []
        self._number_of_elements = 0

        if elements:
            self.push_array(elements)

    def push(self, element: T) -> None:
        self._stack_elements.append(element)
        self._number_of_elements += 1

        if not self._min_elements:
            self._min_elements.append(element)
        else:
            self._min_elements.append(min(element, self._min_elements[-1]))

    def push_array(self, arr: Iterable[T]) -> None:
        for i in arr:
            self.push(i)

    def pop(self) -> T | None:
        if not self._stack_elements:
            return None

        self._min_elements.pop()
        self._number_of_elements -= 1
        return self._stack_elements.pop()

    def peek(self) -> T | None:
        if not self._stack_elements:
            return None
        return self._stack_elements[-1]

    def min_element(self) -> T | None:
        if not self._min_elements:
            return None
        return self._min_elements[-1]

    def is_empty(self) -> bool:
        return not self._stack_elements

    def size(self) -> int:
        return self._number_of_elements

```

## Queues 

A queue is a linear data structure which models real world queues by having two primary operations, namely enqueue and dequeue. <br><br>

**Time Complexity**
* Enqueue O(1)
* Dequeue O(1)
* Peeking O(1)
* Contains O(n)
* Removal O(n)
* Is Empty O(1)

### When to use a Queue 
* Any waiting line models a queue, for example a lineup at a movie theatre.
* Can be used to efficiently keep track of x most recently added elements.
* Web server request management where you want first come first serve. 
* Breadth first search (BFS) graph traversal. 

## Binary Trees 

## Heaps 

## Graphs 


