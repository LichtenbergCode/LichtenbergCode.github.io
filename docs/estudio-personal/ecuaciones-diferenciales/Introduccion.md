# Ecuación Diferencial vs. Ecuación Algebraica

## Sistema de Ecuaciones Diferenciales

$$
\begin{cases} 
\frac{d^2y}{dx^2} + 2 \frac{dy}{dx} = 3y 
\end{cases}
$$

## Ecuación Algebraica

$$x^2 + 3x + 2 = 0 \quad (x+2)(x+1) = 0$$

$$x = -2 \quad x = -1$$

---

## Solución Funcional

### Funciones y sus derivadas:

$$y_1(x) = e^{-3x} \quad y_2(x) = e^x$$

$$y'_1(x) = -3e^{-3x} \quad y'_2(x) = e^x$$

$$y''_1(x) = 9e^{-3x} \quad y''_2(x) = e^x$$

### Verificación de soluciones:

$$e^x + 2e^x = 3e^x$$

$$9e^{-3x} - 6e^{-3x} = 3e^{-3x}$$

$$3e^{-3x} = 3e^{-3x} \quad \text{✓}$$

---

## Problema de Movimiento de Partícula

Una partícula se mueve a lo largo de una recta. Su velocidad es inversamente proporcional al cuadrado de la distancia, $S$, que ha viajado.

**¿Qué ecuación describe esta relación?**

- $S(t) = \frac{k}{t^2}$  
- $\frac{dS}{dt} = \text{velocidad}$
- $S(t) = \frac{k}{S^2}$  
- $\frac{dS}{dt} = \frac{k}{S^4}$
- $\frac{dS}{dt} = \frac{k}{t^2}$
- $\frac{dS}{dt} = \frac{k}{S^2}$ ✓

---

## Ecuación Diferencial Lineal

$$\frac{dy}{dx} = -2x + 3y - 5 \quad y = mx + b$$

### Resolución:

$$m = -2x + 3(mx + b) - 5 \quad \frac{dy}{dx} = m$$

$$m = -2x + 3mx + 3b - 5 \quad 3m - 2 = 0$$

$$m = (3m - 2)x + 3b - 5 \quad 3m = 2$$

$$m = \frac{2}{3}$$

$$y = \frac{2}{3}x + \frac{17}{9} \quad m = 3b - 5$$

$$\frac{2}{3} = 3b - 5 \quad \Rightarrow \quad 3b = \frac{2}{3} + 5 = \frac{17}{3}$$

$$b = \frac{17}{9}$$