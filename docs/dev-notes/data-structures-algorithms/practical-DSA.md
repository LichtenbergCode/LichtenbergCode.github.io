

# Practical DSA 

## Array 

Index Access --> Constant Time O(1) <br>
```bash 
+ - +   + - +   + - +   + - +   + - +
| A |   | B |   | C |   | D |   | E |
+ - +   + - +   + - +   + - +   + - +

arr[4] = 'E' # O(1)

```

Appending in the end --> Constant Time O(1)

```bash 
+ - +   + - +   + - +   + - +   
| A |   | B |   | C |   | D |   
+ - +   + - +   + - +   + - +   

Appending "E": 

+ - +   + - +   + - +   + - +   + - +
| A |   | B |   | C |   | D |   | E |
+ - +   + - +   + - +   + - +   + - +

arr.append('E') # O(1)

```
Inserting or Deleting in the middle or the front --> Linear Time O(n)


```bash 
+ - +   + - +   + - +   + - +   
| A |   | C |   | D |   | E |   
+ - +   + - +   + - +   + - +   

Appending "B": 

+ - +   + - +   + - +   + - +   + - +
| A |   | B |   | C |   | D |   | E |
+ - +   + - +   + - +   + - +   + - +

arr.insert('E', 1) # O(1)

```
**Where to use an Array**

* Traverse a structure in order
* Access specific indices
* Compare elements from both ends
* Sliding window, prefix sum, etc. 

 

## Strings 
Arrays of characters <br>
In most languages, strings are immutable, if you change a string , even slightly, you are not really modifying the original. You are creating a new one under the hood. 

Not good implementation $O(n^2)$: 
```python
result = ""
for char in chars: 
    result += char
```

Good implementation $O(n)$:
```python
result = []
for char in chars: 
    result.append(char)
result "".join(result)
```

Strings work largely very similarly to arrays, so we can think the strings as arrays. 

**Where to use an Strings**
* Find the longest substring without repeating characters 
* Check if two strings are anagrams 
* Return all substrings that match a pattern


## Set 
This is the data structure is one of the most useful for time efficiency. A set is basically a collection of unique values. No duplicates and no particular order.<br>

**Valid Set and Invalid Set**
```python
valid_set = {'a', 'b', 'c', 'd', 'e'}
invalid_set = {'a', 'a', 'b', 'b', 'c'}
```

Example, The number is in the list?: 
```python
nums = [1, 2, 3, 4, 5]
seen = set(nums) 
print(3 in Seen) # return True 
```
In the las case the time complexity is constant $O(1)$ but if we use list the time complexity were linear $O(n)$. <br>

**When to use a Set**
Use a set when you take care about: 
* Uniqueness 
* Existence
* Fast membership checks 
* Sliding window


## Control Flow and Looping 
The most problems in Interviews and in CS in general boil down to three things: 
1. A loop
2. SOme conditions 
3. A few variables being updated

What you should know: 
- for i in range(n)
- for val in arr
- while start < end
- if, else, elif 
- break, continue 

## Hash Maps 
A hashmap is a way to store data using a key to find a value. 
Lookup and Insert --> O(1) <br>

**Hash Map Representation:**
```bash
+ - +   + - +   + - +   + - +  
| A |   | B |   | C |   | D | --> Value  
+ - +   + - +   + - +   + - +   
Apple   Orange  Banana  Grape --> Key 
```
**What is Hashable? (What can We use as a Key)**<br>
- Numbers ✓
- Strings ✓
- Tuples ✓
- List ✕
- Dictionaries ✕

**When to use a Hash table to resolve programming problems**<br>
Use a hash table (dict/ map/ unordered_map/ HashMap) when you need fast-average-time lookup, insertion, deletion, or membership test keyed by arbitrary values. Hash tables give amortized O(1) for these operations and are the right tool for many common algorithmic patterns.<br>
**Example:**<br>
**Finding a value that already exists:**<br>
No Hash Map = O(n)<br>
Hash Map = O(1)<br>
**Consider whether you can use a HashTable if you are doing:**
1. Storing something 
2. Looking something up 
3. Updating/initializing values
**Example: 1** <br>

```python 
# Frequency map algorithm
my_map = {}

for item in data: 
    if item not in my_map: 
        my_map[item] = 1
    else: 
        my_map[item] += 1
```
**Example 2** <br>
```python
# Given an array of integers nums and integer target, 
# return the indices of the two numbers such that they add up
# to target. You may assume that each input would have exactly one solution, and you may not use the same element twice. 

def two_sum(arr: List[int], target: int): 
    num_to_index = {} # Creating the Hash Table 
    for i, nums in enumerate(arr): 
        complement = target - num 
        if complement in num_to_index: 
            return [num_to_index[complement], i]
        
        num_to_index[num] = 1
    return []

```

HashMaps are the most flexible structure in your toolbox, you can use 
them literally everything so it's important to understand how to implement hash tables in different situations. 



## Two Pointers 
We have two ways to use Two Pointers, same direction and opposite direction. <br>

### Same Direction 

The same direction pattern usually shows up when you're doing a simple pass over the data, but you need to track a range, not just one element at the time. 

The classic example is with one pointer slow and the other fast, when the slow moves just one the fast will move two so we can identify the half of the array or a circle.

**Example same direction:** 
```bash
  V V                                
  ( )     ( )     ( )     ( )     ( )

           V               V                                
  ( )     ( )     ( )     ( )     ( )

                   V               V 
  ( )     ( )     ( )     ( )     ( )
  
  # identifying the middle of an array
```

### Opposite Direction 
With this implementation one start since the beginning and one start since the end

**Example opposite direction:** 
```bash
   V                               V  
  ( )     ( )     ( )     ( )     ( ) 
```
This two pointers implementation is really usefull because it reduces the number of iterations you need, track a relationship between two places and avoid extra space. 

### Uses of Pointers
- Palindromes
- Reversals 
- Merging sorted data 
- "K" sized comparisons 


**Example:**
```python
# Given a string s, determine if it is a palindrome,
# considering only alphanumeric characters and ignoring
# cases.

def is_palindrome(s:str) -> bool: 
    l, r = 0, len(s)-1
    while l < r: 
        while l < r and not s[l].isalnum(): #Note 1, 2
            l+= 1
            # 0, 1, 2, 3
            # We are using this while as an If but with iteration :0
        while l < r and not s[r].isalnum(): 
            r-=1 
            #9, 8, 7, 6, 5, 4
        
        if s[l].lower() != s[r].lower(): # ignore case
            return False 
        l += 1
        r -= 1
    return True 

if __name__ == '__main__': 
    s = input()
    res = is_palindrome(s)
    print("true" if res == True else "false")
``` 

**Example 2:**

```python
# Find the middle node of a linked list. 
# Example: 
# Input: 0 1 2 3 4
# Output: 2
# If the number of nodes is even, then return the second middle node

class Node: 
    def __init__(self, val, next= None):
        self.val = val 
        self.next = next 
    
def middle_of_linked_list(head: Node) -> int: 
    slow = fast = head
    while fast and fast.next: 
        fast = fast.next.next # Two steps
        slow = slow.next # One step 
    return slow.val
```

## Sliding Window
This is one of the most versatile technique in coding interviews. It is a extension of the two-pointers patterns.<br>

### Fixed Window

With Fixed Window you have an specific Window size, for example:<br>
- "Find the maximum average of any subarray of size k."
- "Return the sum of every k-length block"
- "Find the subarray of length k with the largest/smallest X."
In the last examples the window size is a constant.

```bash
  V              V 
+-------------------+
| () | () | () | () |  ()  ()   
+-------------------+
```
Template for a fixed window. 
```python
# Note: This is pseudo code
def sliding_window_fixed(input, window_size): 
    ans = window = input[0:window_size]
    for right in range(window_size, len(input)): 
        left = right *window_size
        remove input[left] from window
        append input[right] to window
        ans = optimal(ans, window)
    return ans

```

Example Fixed size sliding window:<br>
```python
# Given an array (list) nums consisted of only 
# non negative integers, find the largest sum
# among all subarrays of length k in nums 


def subarray_sum_fixed(nums:list[int], k: int)->:
    window_sum = 0 
    for i in range(k): 
        window_sum += nums[i]
    largest = window_sum
    for right in range(k, len(nums))

```

### Dynamic Window
This is used when the window size is not fixed and you are trying to find the largest, smallest, or optimal range that satisfies some condition, for example:<br>

- "Find the length of the longest substring with at most K unique characters."
- "What's the smallest subarray with a sum greater than a target?"
- "Return the longest window where a certain rule is valid."


```bash
  V              V 
+-------------------+
| () | () | () | () |  ()  ()   
+-------------------+
                  -->
```
Template for a Dynamic Window: 
```python
# Note: This is pseudo code
def sliding_window_flexible_longest(input): 
    initialize window, ans
    left = 0
    for right in range(len(input)):
        append input[right] to window
        while invalid(window): 
            remove input[left] from window
            left += 1 
        ans = max(ans, window)
    return ans 
```

```python
# Longest Substring without Repeating Characters

# Find the length of the substring of a given string without repeating characters. 

from collections import defaultdict 

def longest_substring_without_repeating_characters(s: str) -> int: 
    longest = 0 
    l = 0 
    counter: dict[str, int] = defaultdict(int)
    for r in range(len(s)): 
        counter[s[r]] += 1
        while counter[s[r]] > 1: 
            counter[s[l]] -= 1
            l+=1
        longest = max(longest,r -l +1)
        return longest

```


## Breadth-First Search 

## Depth-First Search 

## Backtracking 

## Heap 

## Binary Search 
Binary search is an efficient algorithm for finding a target value in a sorted array (or for finding a boundary in a monotonic predicate). It repeatedly halves the search interval, discarding the half that cannot contain the target. Time complexity: O(log n). Space: O(1) for iterative, O(log n) for recursive call stack <br>

"Binary Search is so much powerful that just searching for a number".<br> 

**Binary Search Template**<br>
```python
def binary_search(arr: list[int], target: int) -> int: 
    left, right = 0, len(arr) -1 # inclusive left, exclusive right [left, right]
    while left <= right:  # <= For inclusive right [left, right]
                          # < For exclusive right  [left, right)
        mid = (left + right) // 2 
        if arr[mid] == target: 
            return mid
        elif arr[mid] < target: 
            left = mid + 1
        else: 
            right = mid - 1 # Because right is inclusive
            # right = mid # If mid is exclusive
    return -1
```
### Core algorithm steps: 
1. Define boundaries: Initialize left and right pointers to include all possible cases. 
2. Define return values: Determine what to return (index, value, -1, etc.)
3. Define exit condition: Choose appropriate loop condition (<=, < or < -1)
4. Update pointers: Move boundaries based on comparisons with target 

> https://yennj12.js.org/CS_basics/doc/cheatsheet/binary_search.html
> https://leetcode.com/problems/binary-search/solutions/423162/Binary-Search-101-The-Ultimate-Binary-Search-Handbook/

### Monotonic Condition
A monotonic condition (or monotonic predicate) is a boolean test P(x) whose truth value never flips back as x moves in one direction. In other words, once P becomes true it stays true for all larger (or all smaller) x depending on direction. This property is exactly what you need to apply binary search or "search the answer" reliably.

> "The monotonic step is what allows binary search to discard half the space+"


### When to Use Binary Search 
* Sorted arrays: Classic use for finding exact values.
* Monotonic functions: IF condition(k) implies condition(k+1), binary search applies. 
* Search boundaries: Finding first*last ocurrence of a value.
* Optimization problems: Finding minimum/maximum values satisfying constraints.
> https://yennj12.js.org/CS_basics/doc/cheatsheet/binary_search.html
> https://leetcode.com/problems/binary-search/solutions/423162/Binary-Search-101-The-Ultimate-Binary-Search-Handbook/



**Formal / equivalent ways to say it** <br>

* Monotone non‑decreasing predicate (typical for binary search): for all x ≤ y, P(x) ⇒ P(y). That is, P can be False for a while, then True forever after (False...False True...True).<br>
* Monotone non‑increasing predicate: for all x ≤ y, P(y) ⇒ P(x). That is, True...True False...False.<br>
* In math terms, a function f is monotone (nondecreasing or nonincreasing) if it never goes down (or never goes up). For boolean predicates we use the same monotonic idea but with True/False.<br>


**Monotonic and not monotonic condition graphs:**

```bash 
       MONOTONIC (growing)      NO MONOTONIC
      ▲                        ▲
      │       *                │      *     *
      │     *                  │    *   * *
      │   *                    │  *         *
      │ *                      │*
      └──────►                 └────────────►


      Monotonic Growing:        No Monotonic:
      x: 1  2  3  4  5         x: 1  2  3  4  5
      y: 2  4  6  8  10        y: 2  5  3  8  6
         ↑  ↑  ↑  ↑  ↑             ↑  ↓  ↑  ↓
        Always ↑                 Up and Down
```

## Search boundaries

```bash
[  valid  |  Unknown  |  invalid  ]
          ↑           ↑
         left       right
```


```bash
target = 5
len = 15  # 0 - 14 in the function 
left = 0 # Position 0
right = 15 # position 14
middle = left + (right - left)//2  = 0 + (14-0)//2 = 7

[1, 2, 2, 5, 5, 5, 5, 7, 9, 15, 15, 15, 17, 19, 25]
 ↑                    ↑                          ↑
left                middle                      right

right = middle = 7
left = 0
middle = left + (right-left)//2 = 0 + (7-0)//2 = 4

[1, 2, 2, 5, 5, 5, 5, 7, 9, 15, 15, 15, 17, 19, 25]
 ↑           ↑        ↑
 l           m        r

right = middle = 4
left = 0
middle = 0 + (4 - 0)//2 = 2

[1, 2, 2, 5, 5, 5, 5, 7, 9, 15, 15, 15, 17, 19, 25]
 ↑     ↑     ↑
 l     m     r
right = right = 4
left = middle + 1 = 3
middle = 3 +(4-3)//2 = 4
middle = 4
[1, 2, 2, 5, 5, 5, 5, 7, 9, 15, 15, 15, 17, 19, 25]
          ↑  ↑     
          l  mr
right = middle 

```

**lower bound python template**: 
```python
# lower_bound (first >= x)
def lower_bound(ls: List[int], x: int): 
    lo, hi = 0, len(ls)
    while lo < hi: 
        mid = lo + (hi - lo) //2 # Represents 
        if ls[mid] < x: 
            lo = mid +1
        else: 
            hi = mid
    return lo
```

**upper bound python template**:
```python
# Upper_bound (first > X)
```


**Example From LeetCode:**<br>
Given an array of integers ```nums``` sorted in non-decreasing order, find the starting and ending position of a given ```target``` value. <br>

If ```target``` is not found in the array, return ```[-1, -1]```.<br>

You must write an algorithm with ```O(log n)``` runtime complexity. <br>




## Dynamic Programming 

## Recursion
Recursion is a programming technique where a function calls itself to solve smaller instances of the same problem. You solve by reducing it to one oe more simpler subproblems, and you stop when you reach a base case that's trivial to answer. 

```Python
def factorial(n): 
    if n <= 1: # base case 
        return 1
    return n*factorial(n-1) # recursive case
```

### Recursion vs iteration 

* Both can express the same computations. Iterations uses loops; recursion uses self-calls. 
* Recursion can be clearer for problems with natural self-similarity (trees, divide-and-conquer, combinatorics).
* Iterations often uses less call-stack memory and can be faster in languages (like Python) that do not optimize recursion.


## Divide and Conquer 

## Trie 

## Union Find 

## Greedy 

## Sorting Algorithms
Sorting algorithms arrange items into a defined order (e.g., ascending numerically or lexicographically)

### Buble Sort 

### Insertion Sort 

### Selection Sort 

### Merge Sort 

### Quick Sort 
