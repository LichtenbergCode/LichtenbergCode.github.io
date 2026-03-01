# Dynamic Programming

Dynamic Programming (DP) is a problem-solving technique used to solve problems by **breaking them into smaller overlapping subproblems**


## Understanding Decision, Combinatoric, and Optimization Problems

When solving algorithmic problems - especially in dynamic programming and complexity theory  - it is useful to classify them by **the type of question they ask.**

Many seemingly different codding problems fall into one of three fundamental categories: 

1. Decision Problems 
2. Combinatoric (Constructive) Problems
3. Optimization Problems 

### Decision Problems 
A **decision problem** asks: 
> Does a solution exist? 
The output is typically Boolean (True/False, yes/no)

Here the paths: 

### Combinatoric (Constructive) Problems
A **combinatoric** or **constructive problem** asks: 
> What is one valid solution? 
Now existence is not enough. 
You must **construct the actual solution.**

Here the paths:

### Optimization Problems 
An **optimization problem** asks: 
> What is the best possible solution according to some objective? 
Now we introduce a cost function or evaluation criterion

Here the paths:

### The Hierarchical Relationship 
These categories often form a natural progression: 
1. Decision -> Does a solution exist? 
2. Constructive -> What is a valid solution? 
3. Optimization -> What is the best solution? 

In many cases: 
* If you can solve the optimization version, 
* You can solve the constructive version, 
* And therefore the decision version. 

But not necessarily the other way around

### Next Problems: (Delete later)

* canSum -> "Can you do it? yes/no" -> Decision Problem
* howSum -> ""How will you do it?" -> Combinatoric Problem 
* bestSum -> "What is the 'best' way to do it?" -> Optimization Problem



## Memoization 
Memoization is a **technique used to speed up recursive algorithms by storing the results of expensive function calls** and reusing them when the same inputs occur again. 

### Problem: Fibonacci (Memoization)

fib(50) = $2^50 steps$  

``` python
# T: O(2^n)
# S: O(n)
def fib(n): 
    if n <= 2: 
        return 1
    return fib(n-1) + fib(n-2)
```

```python
# T: O(n)
# S: O(n)
def fib_optimized(n, memo: None):
    if not memo: 
        memo = {0: 0, 
                1: 1, 
                2: 1}
    if n in memo: 
        return memo[n]
    memo[n] = fib_optimized(n-1, memo) + fib_optimized(n-2, memo)
    
    return memo[n]

fib_optimized(7) # Output: 13 
```
### Problem: Number of Paths (Memoization)

Problem: 
Say that you are a traveler on a 2D grid. You begin in the top-left corner and your goal is to travel to the bottom-right corner. You may only move down or right.<br>
In how many ways can tou travel to the goal on a grid with dimensions m*n? 

Write a function 'gridTraveler(m, n)' that calculates this. 


```python 
# gridTraveler(2, 3) -> 3
# 1. right, right, down 
# 2. right, down, right 
# 3. down, right, right

# Edge cases: 
# gridTraveler(1, 1) -> 1
# gridTraveler(0, 1) -> 0
# gridTraveler(0, n) -> 1
# gridTraveler(n, 0) -> 0
# gridTraveler(1, 0) -> 0

# T: O(2^(n+m))
# S: O(n+m)

def grid_traveler(m, n): 
    if m == 1 and n == 1: return 1
    if m == 0 or n == 0: return 0 # empty 
    return grid_traveler(m-1, n) + grid_traveler(m, n -1)

```

```python 
# T: O(n+m) 
# S: O(n+m)
def grid_traveler_optimized(m, n, memo= None):
    if memo == None: 
        memo = {}
    
    key = (m, n)
    if key in memo: 
        return memo[key]
    if m == 1 and n == 1: return 1 
    if m == 0 or n == 0: return 0
    memo[key] = grid_traveler_optimized(m-1, n, memo) + grid_traveler_optimized(m, n-1, memo)
    return memo[key]
```

### Memoization Recipe: 
1. Make it work.
    * visualize the problem as a tree
    * implement the tree using recursion 
    * test it  
2. Make it efficient. 
    * add a memo object
    * add a base case to return memo values 
    * store return values into the memo




### Problem: canSum (Memoization): 
Write a function 'canSum(targetSum, numbers)' that takes in a targetSum and an array numbers as arguments. 

The function should return a boolean indicating whether or not it is possible to generate the targetSum using numbers from the array. 

You may use an element of the array as many times as needed. 

You may assume that all input numbers are nonnegative.

Example: 
```bash , 7
canSum(7, [5, 3, 4]) -> True 
canSum(7, [2, 4]) -> False
```

```python 
def can_sum(target:int, numbers:list) -> list: 
    # m = target sum 
    # n = array length 
    # T: O(n^m)
    # S: O(m)
    if target == 0: # It means that we can get to the target 
        return True 

    if target < 0: # It means that we cannot sum the target 
        return False

    for num in numbers: 
        remainder = target -num 
        if can_sum(remainder, numbers) == True: 
            return True 
    return False 
```


```python 
def can_sum_optimized(target:int, numbers:list, memo = None)->list: 
    if memo == None: 
        memo = {}

    if target in memo: 
        return memo[target]

    if target == 0: 
        return True 
    
    if target < 0: 
        return False
    
    for num in numbers: 
        remainder = target - num
        if can_sum_optimized(remainder, numbers, memo): 
            memo[target] = True
            return False 
    
    memo[target] = False
    return False
```

### Problem: howSum (Memoization): 
Write a function 'howSum(target, numbers)' that takes in a targetSum and an array of numbers as arguments. 

The function should return an array containing any combination of elements that add up to exactly the targetSum. If there is no combination that adds up to the targetSum, then return null. 

If there are multiple combinations possible, you may return any single one.

???+ info "Remember"
    The recursive Principle:

    "A recursive function solves a problem by assuming that smaller versions of the same problem are already solved correctly"

    "Each recursive call must be independent on its state." 

Example: 
```bash 
howSum(7, [5, 3, 4, 7]) -> [7] or [3, 4]
howSum(8, [2, 3, 5]) -> [3, 5] or [2, 2, 2, 2]
howSum(0, [1, 2, 3]) -> [] # empty combinations 
```

```python 
def how_sum(target, numbers): 
    if target == 0: 
        return []
    
    if target < 0: 
        return None
    
    for num in numbers: 
        remainder = target - num
        remainder_result = how_sum(remainder, numbers)
        if remainder_result != None: 
            return remainder_result + [num]
    
    return None
```

```python 
def how_sum_optimized(target, numbers, memo = None): 
    if memo == None: 
        memo = dict()
    
    if target in memo: 
        return memo[target]
    
    if target == 0: 
        return []
    
    if target < 0: 
        return None
    
    for num in numbers: 
        remainder = target - num
        remainder_number = how_sum_optimized(remainder, numbers, memo)
        if remainder_number != None: 
            memo[target] = remainder_number + [num]
            return memo[target]
    
    memo[target] = None
    return None
```


### Problem: bestSum (Memoization)
Write a function 'bestSum(targetSum, numbers)' that takes in a targetSum and an array of numbers as arguments. 

The function should return an array containing the **shortest** combination of numbers that add up to exactly the targetSum. 

If there is a tie for the shortest combination, you may return any one of the shortest. 

Example 
```bash 
bestSum(7, [5, 3, 4, 7]) -> [7]
bestSum(8, [2, 3, 5]) -> [3, 5]
```

```python 
def best_sum(target: int, numbers: list):

    if target == 0:  
        return []
    
    if target < 0: 
        return None 
    
    shortest_combination = None # # In the future a list
    
    for number in numbers: 
        remainder = target - number  
        remainder_combination = best_sum(remainder, numbers) # using recursion 
        if remainder_combination != None: 
            combination = reminder_combination + [number]
            if shortest_combination == None or len(combination) < len(shortest_combination): 
                shortest_combination = combination 
    
    return shortest_combination 
```

### canConstruct 
Write a function **'canCOnstruct(target, wordBank)'** that accepts a target and an array of strings. 

The function should return a **boolean** indicating whether or not the 'target' can be constructed by concatenating elements of the 'wordBank' array. 

You may reuse elements of 'wordBank' as many times as needed. 

Example
```bash
canConstruct(abcdef, [ab, abc, cd, def, abcd]) -> True 
canConstruct(skateboard, [bo,rd,ate,t,ska,sk,boar]) -> False
```


### countConstruct 
Write a function **'countConstruct(target, word_bank)'** that accepts a target string and an array of string. 

The function should return the **number of ways** that the **"target"** can be constructed by concatenating elements of the **"word_bank"** array.

You may reuse elements of **"word_bank"** as many times as needed.

Example
```bash 
countConstruct('abcdef', ['ab', 'abc', 'cd', 'def', 'abcd']) -> 1
```

### allConstruct 
Write a function **"alConstruct(target, wordBank)"** that accepts a target string ans an array of strings. 

The function should return a 2D array containing **all of the ways** that the **target** can be constructed by concatenating elements of the **"wordBank"** array. Each element of the 2D array should represent one combination that constructs the **'target'**.

You may reuse elements of **'wordBank'** as many times as needed. 

Example
```bash 

allConstruct('purple', ['purp', 'p', 'ur', 'le', 'purpl']) -> [
    [ purp, le ], 
    [ p, ur, p, le]]

allConstruct('abcde', ['ab', 'abc', 'cd', 'def', 'abcd', 'ef', 'c']) -> [
    ['ab', 'cd', 'ef'], 
    ['ab', 'c', 'def'],
    ['abc', 'def'], 
    ['abcd', 'ef'] 
]

```

## Tabulation





https://www.youtube.com/watch?v=oBt53YbR9Kk&t=2647s