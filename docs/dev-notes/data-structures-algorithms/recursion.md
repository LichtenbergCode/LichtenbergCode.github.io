
# Recursion 
A recursive function is a **function that calls itself**. This technique is commonly used in programming to solve problems that can be broken down into smaller, more manageable **sub-problems**. 

A recursive function breaks down a problem into smaller sub-problems which are easier to solve. 

Recursion Components: 
* A base case to terminate the recursion. 
* A recursive call with a transition that gudes the recursion towards the base case
* The body of the recursive function

**Example 1**: 
```python

# Sums all the numbers from 1 to N for 
# any number N >= 0
# E.g Sum(3) = 1 + 2 + 3 = 6
def sum(n): 
    if n == 0: # Base Case 
        return 0
    return sum(n-1) + n # Recursive call to the same function
    # + n -> The body of the function
    # n = 9
    # -> 9 8 7 6 4 3 2 1 0 
```

**Example 2 Multiplication**
```python
# This is the implementation of the recursive 
# function to compute the product of two numbers 
# when a >= 0.
def multiplication(n, m): 
    if m == 0: 
        return 0
    return (n, m-1) + n
```

**Example 3 Negative Multiplication**
```python
def multiplication2(n, m): 
    if n == 0: 
        return 0
    elif n < 0: 
        return mul(n + 1, m) - m
    return mul(n-1, m) + m
```

**Example 4 Iterating a List**
```python
def list_iteration(ls, n = 0): 
    if n == len(ls): 
        return 0
    print(ls[n])
    return list_iteration(ls, n+1)
```

**Example 5 Sum of the odds**
```python
# Sums the odd numbers in a list
# n - The current index position 
# ls - The list of numbers
def odds_sum(n, ls): 
    if n == len(ls):
        return 0
    value = 0 
    # Check if the current number is odd
    if lst[n] % 2 != 0: 
        value = lst[i]
    return odds_sum(n+1, ls) + value
``` 

**Example 6 Reversing a String**
```python
def reverse(s: List): 
    rev = ""
    for  i in range(len(s)-1, -1, -1): 
        rev += s[i]
    return rev

def reverse2(i: int, s:List[int]): 
    if i == len(s): 
        return = ""
    return reverse2(i+1, s) + s[i]

def copy(i:int, s: List[int]): 
    if i == len(s): 
        return ""
    return s[i] + copy(i+1, s)

def reverse3(i:int, s:List[int]): # Reverse with two opposite pointers
    n = len(s)
    if n<= 1: 
        return s
    left, right = s[0], s[n-1]
    sub = sub[1:n-1]
    return right +reverse3(sub) +left
```

**Example Palindromes**
```python
# A palindrome is a string of characters that reads the same backwards 
# and forwards.
def palindrome(s: List[int]):
    left, right = 0, len(s)-1
    if s[left].lower() != s[right].lower(): 
        return False
    elif right < right: 
        return True
    return palindrome(s[left+1: right-1])
```
## Multiple return values



> https://www.youtube.com/playlist?list=PLDV1Zeh2NRsCmu1lb9grUcljeYJtmgmYc