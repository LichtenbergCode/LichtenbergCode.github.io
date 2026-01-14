# Edge Cases 

**Edge Cases** are inputs or situations that occur at the extremes or boundaries of valid behavior where bugs most often hide. 

## Core Categories of Edge Cases
### 1. Empty or Null Inputs
Inputs that **contain nothing** or don't exist.
> What if there is no data? 

```python
[]
""
None
```

**Why bugs Happen**<br>
Most logic assumes something exists: <br> 
• Accessing index 0 <br>
• Calling methods on None <br>
• Looping without checks <br>

**Example bug**
```python 
arr = []
arr[0] # IndexingError 
```

**Handle it**
```python 
if not arr: 
    return default_value
```
<br><br>

### 2. Minimum/Maximum Boundaries 
Values at the extreme of allowed constraints. 

> What happens at the limit of the system?
> This is **critical in embedded, robotics and systems**. 

```python
INT_MIN = -2^31
INT_MAX = 2^31 -1
```

**Why bugs happen** <br> 
• Overflow <br>
• Underflow <br>
• Precision loss <br> 

**Example bug**
```python 
x * 10 + digit # overflow risk 
```
<br><br>

### 3. Single-Element Inputs
The smallest non-empty input 
> Does my algorithm still work with the smallest valid input? 
```python 
["a"]
[42]
```

**Why bugs happen** <br>
Algorithms often assume: <br> 
• Two pointers <br>
• Comparisons between elements<br> 
• Previous/next elements exist<br>

**Example bug**
```python 
arr[i+ 1]
```
<br><br>

### 4. Off-by-One Errors (Boundaries)
Error at edges of loops or indices. 
> Where does my loop start and stop exactly?
> This is the **#1 cause of production bugs**
```python 
i == 0
i == len(arr) -1
```

**Why bugs happen** <br>
• Inclusive vs exclusive ranges <br> 
• ```<=``` vs ```<``` <br>

**Example bug**
```python 
for i in range(len(arr)):
    arr[i + 1] # crash on last iteration 
```
<br><br>

### 5. All Elements Identical
Input where everything is the same. 
> Does my algorithm depend on change? 

```python 
[0, 0, 0]
["aaa", "aaa"]
```
**Why bugs happen** <br>
• Early exit logic <br>
• Incorrect assumptions about variation<br>

**Example bug**
```python
if arr[i] != arr[i-1]: # Never breaks if all are equal
    break
```
<br><br>

### 6. No-Match/Failure Cases 
Valid input that produces **no meaningful result.**
> What if the answer doesn't exist?

```python 
["dog", "rececar", "car"]
```
**Why bugs happen**<br>
Developers assume: <br>
• A match exists <br>
• A solution is guaranteed<br>

```python
return result # Result never set
```

<br><br>

### 7 Leading/Trailing Noise
Extra characters around meaningful data.

> Is my input sanitized? 
> Very common in real-world data pipelines. 

```python
"  hello  "
```
**Why bugs happen** <br>
Humans ignore spaces but computers don't

**Example bug**
```python 
split(" ") # creates empty tokens 
```
<br><br>

### 8. Negative Values 
Inputs below zero. 
> Does sign change logic?
```python 
-1
-123
```
Why bugs happen 
• % behaves differently  <br>
• Sign must be preserved <br>

**Example bug**
```python
digit = x % 10 # tricky with negatives
```
<br><br>

### 9. Duplicates 
Repeated values. 
> Can identical values break uniqueness assumptions?

```python 
[1, 1, 1]
```

**Why bugs happen** <br>
• Hash collisions <br>
• Infinite loops <br>
• Pointer logic failure <br>

**Example bug**<br>
```python
while nums[l] == nums[r]:
    l += 1 # may skip valid answers
```

<br><br>

### 10. Performance/Scale Edge Cases 
Input size near maximum constraints. 
> Will this survive worst-case input?
```python 
n = 100_000
```
**Why bugs happen** <br>
• Timeouts <br>
• Stack overflow <br>
• Memory exhaustion <br>

**Example bug**
```python
O(n^2) # works locally, fails in prod
```



