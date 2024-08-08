import sympy as sp

# Define the symbols
v_rx, v_ry = sp.symbols('v_rx v_ry')  # Velocity of the robot
v_ox, v_oy = sp.symbols('v_ox v_oy')  # Velocity of the obstacle
u_rx, u_ry = sp.symbols('u_rx u_ry')  # Acceleration of the robot
u_ox, u_oy = sp.symbols('u_ox u_oy')  # Acceleration of the obstacle

# Define the relative velocity and acceleration
v_rox = v_rx - v_ox
v_roy = v_ry - v_oy
u_rox = u_rx - u_ox
u_roy = u_ry - u_oy

# Define the boundary vector components
v_b1x, v_b1y = sp.symbols('v_b1x v_b1y')

# Define the projection function
v_ro = sp.Matrix([v_rox, v_roy])
v_b1 = sp.Matrix([v_b1x, v_b1y])
proj_v_ro_on_v_b1 = (v_ro.dot(v_b1) / v_b1.dot(v_b1)) * v_b1

# Differentiate the projection function with respect to time
t = sp.symbols('t')
v_rx_t = sp.Function('v_rx')(t)
v_ry_t = sp.Function('v_ry')(t)
v_ox_t = sp.Function('v_ox')(t)
v_oy_t = sp.Function('v_oy')(t)
u_rx_t = sp.Function('u_rx')(t)
u_ry_t = sp.Function('u_ry')(t)
u_ox_t = sp.Function('u_ox')(t)
u_oy_t = sp.Function('u_oy')(t)

# Replace the velocity symbols with their time-dependent functions
v_ro_t = sp.Matrix([v_rx_t - v_ox_t, v_ry_t - v_oy_t])
v_b1_t = sp.Matrix([v_b1x, v_b1y])

# Define the projection as a function of time
proj_v_ro_on_v_b1_t = (v_ro_t.dot(v_b1_t) / v_b1_t.dot(v_b1_t)) * v_b1_t

# Differentiate with respect to time
d_proj_v_ro_on_v_b1_t = sp.diff(proj_v_ro_on_v_b1_t, t)

# Simplify the result
d_proj_v_ro_on_v_b1_t_simplified = sp.simplify(d_proj_v_ro_on_v_b1_t)
print(d_proj_v_ro_on_v_b1_t_simplified)
