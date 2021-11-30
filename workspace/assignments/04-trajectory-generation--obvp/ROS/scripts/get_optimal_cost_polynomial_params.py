from sympy import symbols
from sympy import Matrix, Transpose
from sympy import Poly
from sympy import simplify, expand, collect
from sympy import diff

# define start state:
p_x_s, p_y_s, p_z_s, v_x_s, v_y_s, v_z_s = symbols('p_x_s, p_y_s, p_z_s, v_x_s, v_y_s, v_z_s')
# define end state:
p_x_e, p_y_e, p_z_e, v_x_e, v_y_e, v_z_e = symbols('p_x_e, p_y_e, p_z_e, v_x_e, v_y_e, v_z_e')
# define duration:
T = symbols('T')

# 
# step 1: solve for coeffs:
#
A = Matrix(
    [
        [-12/(T**3),          0,          0, 6/(T**2),       0,        0],
        [         0, -12/(T**3),          0,       0, 6/(T**2),        0],
        [         0,          0, -12/(T**3),       0,        0, 6/(T**2)],
        [  6/(T**2),          0,          0,    -2/T,        0,        0],
        [         0,   6/(T**2),          0,       0,     -2/T,        0],
        [         0,          0,   6/(T**2),       0,        0,     -2/T],
    ]
)
b = Transpose(
    Matrix(
        [
            [
                p_x_e - v_x_s*T - p_x_s, 
                p_y_e - v_y_s*T - p_y_s, 
                p_z_e - v_z_s*T - p_z_s, 
                          v_x_e - v_x_s, 
                          v_y_e - v_y_s, 
                          v_z_e - v_z_s,
            ]
        ]
    )
)
coeffs = simplify(A*b)

#
# step 2: build cost function
#
M = Matrix(
    [
        [(T**3)/3,        0,        0, (T**2)/2,        0,        0],
        [       0, (T**3)/3,        0,        0, (T**2)/2,        0],
        [       0,        0, (T**3)/3,        0,        0, (T**2)/2],
        [(T**2)/2,        0,        0,        T,        0,        0],
        [       0, (T**2)/2,        0,        0,        T,        0],
        [       0,        0, (T**2)/2,        0,        0,        T],
    ]
)

J = collect(
    expand(
        simplify(
            Transpose(coeffs) * M * coeffs
        )[0] + T
    ),
    syms=T
)

print(J)

#
# step 3: derivative
#
den = T**3
num = den * J

derivative = Poly(
    collect(
        expand(
            diff(num, T)*den - num*diff(den, T)
        ),
        syms=T
    ),
    T
)

#
# done:
#
for order, coeff in zip(
    range(len(derivative.all_coeffs()) - 1, -1, -1),
    derivative.all_coeffs()
):
    print(f"{order}: {coeff}\n")
