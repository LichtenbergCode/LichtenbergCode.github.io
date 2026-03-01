
# Sorting Algorithms

A sorting algorithm is a procedure that rearranges the elements of all collection (array, list, etc.) into a specific order (usually ascending or descending). 

## Why sorting matters? 
* Sorted data is easier and faster to search, merge, deduplicate, summarize, or present. 
* Many higher-level algorithms rely on sorted input (binary search, set operations, some geometric algorithms, database operation, etc.). 
* Sorting performance often determines overall system performance for data-heavy tasks. 

## Insertion Sort
Insertion sort builds a sorted array (or list) one element at a time by taking the next element and inserting it into the correct position among the already-sorted elements to its left.
```python 
def insertion_sort(A): 
    for i in range(1, len(A)): 
        key = A[i]
        j = i -1
        while j >= 0 and A[j] > key: 
            A[j+1] = A[j]
            j -= 1
        A[j + 1] = key

# Step-by-step example (A = [5, 2, 4, 6, 1, 3])
#
# i = 1, key = 2: shift 5 → place 2 → [2, 5, 4, 6, 1, 3]
# i = 2, key = 4: shift 5 → place 4 → [2, 4, 5, 6, 1, 3]
# i = 3, key = 6: no shifts (6 ≥ 5) → [2, 4, 5, 6, 1, 3]
# i = 4, key = 1: shift 6,5,4,2 → place 1 → [1, 2, 4, 5, 6, 3]
# i = 5, key = 3: shift 6,5,4 → place 3 → [1, 2, 3, 4, 5, 6]
```
### Time Complexity
* Time 
    * Best: O(n) when array is already sorted (only one comparison per element). 
    * Average: O(n^2).
    * Worst: O(n^2) when array is reverse-sorted.
* Space: O(1) extra (in-place). 
* Stability: Stable (equal elements keep original relative order). 
* Adaptive: Yes -- runs faster on nearly sorted data. 
* Comparison sort: Yes (requires element comparisons). 

## Merge Sort 
## Quick Sort 