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

