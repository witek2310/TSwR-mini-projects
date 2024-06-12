import sympy

# eigenvalus for decentralized controle
l1, l2, l3 = sympy.symbols('l_1 l_2 l_3')
M = sympy.Matrix([[-l1, 1, 0], [-l2, 0, 1], [-l3, 0, 0]])


lam = sympy.symbols("lam")
I = sympy.Matrix([[lam, 0, 0],[0, lam, 0], [0, 0, lam]])

print(sympy.det_quick(I -M).factor())


