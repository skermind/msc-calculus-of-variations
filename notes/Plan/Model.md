# **Extended Path Functional: Curvature and Length Penalties**

## **Motivation**

The initial path-planning problem considers a curve $\gamma$ connecting two fixed boundary points, with the objective of minimising the accumulated cost associated with the terrain:

# 

# # **$$**

**J_{\text{cost}}(\gamma)**

\int_\gamma c(x,y),ds,  
$$

where $c:\Omega\rightarrow\mathbb{R}$ is the cost field defined over the domain $\Omega$.

While this formulation finds a minimum-cost path, it does not explicitly penalise undesirable geometric properties of the path. In particular, a path may reduce its terrain cost by making sharp turns or by taking an unnecessarily long route.

To investigate these effects, two additional terms can be introduced: a curvature penalty and a length penalty. The resulting functional is

# 

# # 

# # **$$**

**\boxed{**

**J(\gamma)**

\int_\gamma c(x,y),ds  
+  
\lambda\int_\gamma \kappa^2,ds  
+  
\alpha L  
}  
$$

where $\lambda,\alpha\geq0$ are weighting parameters.

The parameters $\lambda$ and $\alpha$ determine the relative importance of smoothness and path length respectively, and can therefore be investigated experimentally.

---

## **Parameterisation of the curve**

Let

$$  
\gamma:[0,1]\rightarrow\mathbb{R}^2  
$$

be an admissible curve satisfying the prescribed boundary conditions, with

$$  
\gamma(t)=(x(t),y(t)).  
$$

The derivative of the curve is

$$  
\gamma’(t)=(x’(t),y’(t)),  
$$

so the differential element of arc length is

# 

# # **$$**

**ds=|\gamma’(t)|,dt**

\sqrt{x’(t)^2+y’(t)^2},dt.  
$$

Consequently, the length of the curve is

# 

# # **$$**

**L(\gamma)**

# 

# # **\int_0^1**

**|\gamma’(t)|,dt**

\int_0^1  
\sqrt{x’(t)^2+y’(t)^2},dt.  
$$

The arc-length parameter is given by

# 

# # **$$**

**s(t)**

\int_0^t  
|\gamma’(\tau)|,d\tau.  
$$

---

## **Curvature**

If the curve is parameterised by arc length $s$, then its curvature can be expressed as

$$  
\kappa(s)=|\gamma’’(s)|.  
$$

However, the original parameterisation $\gamma(t)$ is not necessarily an arc-length parameterisation. For a general parameter $t$, the curvature of a planar curve is

# 

# # 

# # **$$**

**\boxed{**

**\kappa(t)**

\frac{  
|x’(t)y’’(t)-y’(t)x’’(t)|  
}{  
\left(x’(t)^2+y’(t)^2\right)^{3/2}  
}.  
}  
$$

Therefore,

$$  
\int_\gamma\kappa^2,ds  
$$

can be expressed in terms of the original parameterisation if required.

---

# **Reformulation as $y(x)$**

For the numerical path-planning problem, the path is constrained to progress monotonically in the $x$-direction. This allows $x$ to be used as the independent variable.

We therefore write

$$  
\gamma(x)=(x,y(x)),  
\qquad  
x\in[x_0,x_1].  
$$

This gives

$$  
\gamma’(x)=(1,y’(x)),  
$$

and hence

# 

# # **$$**

**ds**

\sqrt{1+(y’)^2},dx.  
$$

This transformation allows the entire problem to be expressed as a one-dimensional variational problem in terms of the function $y(x)$.

---

## **Terrain cost**

The original terrain-cost term becomes

# 

# # **$$**

**\int_\gamma c(x,y),ds**

\int_{x_0}^{x_1}  
c(x,y(x))  
\sqrt{1+(y’)^2},dx.  
$$

---

## **Path length**

Similarly, the length becomes

# 

# # 

# # **$$**

**\boxed{**

**L[y]**

\int_{x_0}^{x_1}  
\sqrt{1+(y’)^2},dx.  
}  
$$

The parameter $\alpha$ therefore controls the contribution of path length to the total objective.

---

## **Curvature penalty**

For a curve represented as $y=y(x)$, the curvature is

# 

# # 

# # **$$**

**\boxed{**

**\kappa(x)**

\frac{|y’’|}  
{\left(1+(y’)^2\right)^{3/2}}.  
}  
$$

Therefore,

# 

# # **$$**

**\kappa^2**

\frac{(y’’)^2}  
{\left(1+(y’)^2\right)^3}.  
$$

Since

$$  
ds=\sqrt{1+(y’)^2},dx,  
$$

the curvature penalty becomes

$$  
\begin{aligned}  
\int_\gamma\kappa^2,ds  
&=  
\int_{x_0}^{x_1}  
\frac{(y’’)^2}  
{\left(1+(y’)^2\right)^3}  
\sqrt{1+(y’)^2},dx\  
&=  
\boxed{  
\int_{x_0}^{x_1}  
\frac{(y’’)^2}  
{\left(1+(y’)^2\right)^{5/2}}  
,dx  
}.  
\end{aligned}  
$$

---

# **Final variational formulation**

Combining the terrain cost, curvature penalty and length penalty gives

# 

# # 

# # **$$**

**\boxed{**

**J[y]**

\int_{x_0}^{x_1}  
\left[  
c(x,y)\sqrt{1+(y’)^2}  
+  
\lambda  
\frac{(y’’)^2}  
{\left(1+(y’)^2\right)^{5/2}}  
+  
\alpha\sqrt{1+(y’)^2}  
\right]dx.  
}  
$$

The functional can equivalently be written as

# 

# # 

# # **$$**

**\boxed{**

**J[y]**

\int_{x_0}^{x_1}  
F(x,y,y’,y’’),dx,  
}  
$$

where

# 

# # 

# # **$$**

**\boxed{**

**F(x,y,y’,y’’)**

(c(x,y)+\alpha)\sqrt{1+(y’)^2}  
+  
\lambda  
\frac{(y’’)^2}  
{\left(1+(y’)^2\right)^{5/2}}.  
}  
$$

Thus, the addition of the curvature term changes the structure of the variational problem from a first-order functional,

$$  
J[y]=\int F(x,y,y’),dx,  
$$

to a second-order functional,

$$  
\boxed{  
J[y]=\int F(x,y,y’,y’’),dx.  
}  
$$

---

# **Higher-order Euler–Lagrange equation**

For a functional of the form

# 

# # **$$**

**J[y]**

\int_{x_0}^{x_1}  
F(x,y,y’,y’’),dx,  
$$

the corresponding Euler–Lagrange equation is

## 

## ## 

## ## **$$**

**\boxed{**

**\frac{\partial F}{\partial y}**

\frac{d}{dx}  
\left(  
\frac{\partial F}{\partial y’}  
\right)  
+  
\frac{d^2}{dx^2}  
\left(  
\frac{\partial F}{\partial y’’}  
\right)  
=0.  
}  
$$

This is the natural extension of the classical Euler–Lagrange equation to functionals depending on the second derivative.

For the present problem,

# 

# # **$$**

**F(x,y,y’,y’’)**

(c(x,y)+\alpha)\sqrt{1+(y’)^2}  
+  
\lambda  
\frac{(y’’)^2}  
{\left(1+(y’)^2\right)^{5/2}}.  
$$

The resulting Euler–Lagrange equation is therefore a higher-order differential equation involving the terrain-cost function $c(x,y)$ and its derivatives.

A useful point for further investigation is that the curvature term introduces dependence on $y’’$. Consequently, the associated Euler–Lagrange equation generally involves derivatives of $y$ of order higher than two.

---

# **Relationship to the numerical method**

The analytical formulation provides the mathematical objective that the simulated annealing algorithm attempts to minimise numerically.

The simulated annealing procedure does not need to explicitly solve the Euler–Lagrange equation. Instead, it searches over a discretised class of admissible curves and evaluates the corresponding discrete approximation of

$$  
J[y].  
$$

The three components of the objective can therefore be interpreted as:

|**Term**|**Interpretation**|
|---|---|
|$\displaystyle\int_\gamma c(x,y),ds$|Cost associated with travelling through the terrain|
|$\displaystyle\lambda\int_\gamma\kappa^2,ds$|Penalises high curvature and therefore encourages smoother paths|
|$\displaystyle\alpha L$|Penalises unnecessarily long paths|

The parameters $\lambda$ and $\alpha$ control the trade-off between these competing objectives.

This provides a direct connection between the continuous variational problem and the stochastic numerical optimisation method used in the computational experiments.
# Extended Path Functional: Curvature and Length Penalties

## Motivation

The initial path-planning problem considers a curve $\gamma$ connecting two fixed boundary points, with the objective of minimising the accumulated cost associated with the terrain:

$$
J_{\mathrm{cost}}(\gamma)=\int_\gamma c(x,y)\,ds,
$$

where $c:\Omega\rightarrow\mathbb{R}$ is the cost field defined over the domain $\Omega$.

This formulation finds a minimum-cost path, but it does not explicitly penalise undesirable geometric properties of the path. In particular, a path may reduce its terrain cost by making sharp turns or by taking an unnecessarily long route.

To investigate these effects, two additional terms can be introduced: a curvature penalty and a length penalty. The resulting functional is

$$
\boxed{
J(\gamma)
=
\int_\gamma c(x,y)\,ds
+ 
\lambda\int_\gamma \kappa^2\,ds
+ 
\alpha L
}
$$

where $\lambda,\alpha\geq0$ are weighting parameters. The parameters $\lambda$ and $\alpha$ determine the relative importance of smoothness and path length and can therefore be investigated experimentally.

## Parameterisation of the curve

Let

$$
\gamma:[0,1]\rightarrow\mathbb{R}^2
$$

be an admissible curve satisfying the prescribed boundary conditions, with

$$
\gamma(t)=(x(t),y(t)).
$$

Then

$$
\gamma'(t)=(x'(t),y'(t)),
$$

and the differential element of arc length is

$$
ds=\|\gamma'(t)\|\,dt
=\sqrt{x'(t)^2+y'(t)^2}\,dt.
$$

Consequently, the length of the curve is

$$
L(\gamma)=\int_0^1\|\gamma'(t)\|\,dt
=\int_0^1\sqrt{x'(t)^2+y'(t)^2}\,dt.
$$

The arc-length parameter is

$$
s(t)=\int_0^t\|\gamma'(\tau)\|\,d\tau.
$$

## Curvature

If the curve is parameterised by arc length $s$, then its curvature can be expressed as

$$
\kappa(s)=\|\gamma''(s)\|.
$$

However, the original parameterisation $\gamma(t)$ is not necessarily an arc-length parameterisation. For a general parameter $t$, the curvature of a planar curve is

$$
\boxed{
\kappa(t)=
\frac{|x'(t)y''(t)-y'(t)x''(t)|}
{\left(x'(t)^2+y'(t)^2\right)^{3/2}}
}
$$

## Reformulation as $y(x)$

For the numerical path-planning problem, the path is constrained to progress monotonically in the $x$-direction. This allows $x$ to be used as the independent variable. We therefore write

$$
\gamma(x)=(x,y(x)),
\qquad x\in[x_0,x_1].
$$

Then

$$
\gamma'(x)=(1,y'(x)),
$$

so

$$
ds=\sqrt{1+(y')^2}\,dx.
$$

This transformation allows the problem to be expressed as a one-dimensional variational problem in terms of the function $y(x)$.

## Terrain cost

The terrain-cost term becomes

$$
\int_\gamma c(x,y)\,ds
=
\int_{x_0}^{x_1}c(x,y(x))\sqrt{1+(y')^2}\,dx.
$$

## Path length

The length becomes

$$
\boxed{
L[y]=\int_{x_0}^{x_1}\sqrt{1+(y')^2}\,dx.
}
$$

Thus $\alpha$ controls the contribution of path length to the total objective.

## Curvature penalty

For a curve represented as $y=y(x)$, the curvature is

$$
\boxed{
\kappa(x)=\frac{|y''|}{\left(1+(y')^2\right)^{3/2}}.
}
$$

Therefore,

$$
\kappa^2=\frac{(y'')^2}{\left(1+(y')^2\right)^3}.
$$

Since

$$
ds=\sqrt{1+(y')^2}\,dx,
$$

the curvature penalty becomes

$$
\begin{aligned}
\int_\gamma\kappa^2\,ds
&=\int_{x_0}^{x_1}
\frac{(y'')^2}{\left(1+(y')^2\right)^3}
\sqrt{1+(y')^2}\,dx\\
&=\boxed{
\int_{x_0}^{x_1}
\frac{(y'')^2}{\left(1+(y')^2\right)^{5/2}}\,dx
}.
\end{aligned}
$$

## Final variational formulation

Combining the terrain cost, curvature penalty and length penalty gives

$$
\boxed{
J[y]=\int_{x_0}^{x_1}
\left[
(c(x,y)+\alpha)\sqrt{1+(y')^2}
+ 
\lambda\frac{(y'')^2}{\left(1+(y')^2\right)^{5/2}}
\right]dx.
}
$$

Equivalently, define

$$
F(x,y,y',y'')
=
(c(x,y)+\alpha)\sqrt{1+(y')^2}
+ 
\lambda\frac{(y'')^2}{\left(1+(y')^2\right)^{5/2}}.
$$

Then

$$
\boxed{
J[y]=\int_{x_0}^{x_1}F(x,y,y',y'')\,dx.
}
$$

Thus, the addition of the curvature term changes the structure of the variational problem from a first-order functional,

$$
J[y]=\int F(x,y,y')\,dx,
$$

to a second-order functional,

$$
\boxed{
J[y]=\int F(x,y,y',y'')\,dx.
}
$$

## Higher-order Euler--Lagrange equation

For a functional of the form

$$
J[y]=\int_{x_0}^{x_1}F(x,y,y',y'')\,dx,
$$

the corresponding higher-order Euler--Lagrange equation is

$$
\boxed{
\frac{\partial F}{\partial y}
-
\frac{d}{dx}\left(\frac{\partial F}{\partial y'}\right)
+ 
\frac{d^2}{dx^2}\left(\frac{\partial F}{\partial y''}\right)
=0.
}
$$

This is the natural extension of the classical Euler--Lagrange equation to functionals depending on the second derivative.

For the present problem,

$$
F(x,y,y',y'')
=
(c(x,y)+\alpha)\sqrt{1+(y')^2}
+ 
\lambda\frac{(y'')^2}{\left(1+(y')^2\right)^{5/2}}.
$$

The curvature term introduces dependence on $y''$, so the associated Euler--Lagrange equation is a higher-order differential equation.

## Relationship to the numerical method

The analytical formulation provides the continuous objective that the simulated annealing algorithm attempts to minimise numerically.

The simulated annealing procedure does not need to explicitly solve the Euler--Lagrange equation. Instead, it searches over a discretised class of admissible curves and evaluates a discrete approximation of $J[y]$.

The three components of the objective are:

| Term | Interpretation |
|---|---|
| $\displaystyle\int_\gamma c(x,y)\,ds$ | Cost associated with travelling through the terrain |
| $\displaystyle\lambda\int_\gamma\kappa^2\,ds$ | Penalises high curvature and encourages smoother paths |
| $\displaystyle\alpha L$ | Penalises unnecessarily long paths |

The parameters $\lambda$ and $\alpha$ therefore control the trade-off between terrain cost, smoothness and path length.

This provides the connection between the continuous variational problem and the stochastic numerical optimisation method used in the computational experiments.