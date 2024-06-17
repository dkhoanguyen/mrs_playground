import matplotlib.pyplot as plt
import numpy as np

from mr_herding.cbf.constraints import *


# Enable LaTeX for text rendering
plt.rcParams['text.usetex'] = True
plt.rcParams['font.family'] = 'serif'
plt.rcParams['font.serif'] = 'Computer Modern'


def unit_vector(v):
    return v / np.linalg.norm(v)


def plot_line_from_normal(normal, point, ax, v, w, x_range=(-10, 10)):
    point = unit_vector(point) * (np.linalg.norm(point) + 0.4)
    # Normal vector
    a, b = normal

    # Point on the line
    x0, y0 = point

    # Direction vector (perpendicular to the normal vector)
    direction = np.array([-b, a])

    angle = np.degrees(np.arctan2(direction[1], direction[0]))

    # Generate x values
    x_values = np.linspace(x_range[0], x_range[1], 400)

    # Calculate corresponding y values using the point-slope form
    y_values = y0 + (direction[1] / direction[0]) * (x_values - x0)

    # # Plot the line
    # ax.plot(x_values, y_values,
    #         label=f'Line through ({x0}, {y0}) with normal {normal}')
    # Add double-ended arrow
    # mid_idx = len(x_values) // 2
    ax.annotate('', xy=(x_values[0], y_values[0]), xytext=(x_values[-1], y_values[-1]),
                arrowprops=dict(arrowstyle='<->', color='black'))

    start_point = np.array([x_values[0], y_values[0]])
    end_point = np.array([x_values[-1], y_values[-1]])

    shifted_point = point - v

    end_point_shade = end_point + w
    start_point_shade = start_point + w
    print(end_point)

    vertices = np.array([[x_values[0], y_values[0]],
                         [x_values[-1], y_values[-1]],
                         [end_point_shade[0], end_point_shade[1]],
                         [start_point_shade[0], start_point_shade[1]]])

    ax.fill(*zip(*vertices),
            edgecolor='none', facecolor='mistyrose')

    # Add annotation along the line
    annotation_text = r'$\mathit{ORCA}_{i|j}^{\tau}$'
    plt.text(x0+1.05, y0+2, annotation_text, rotation=angle, ha='center', va='center',
             fontsize=16, color='black')


# Enable LaTeX for text rendering
plt.rcParams['text.usetex'] = True
plt.rcParams['font.family'] = 'serif'
plt.rcParams['font.serif'] = 'Computer Modern'

# Parameters for the plot
pA = np.array([0, 0])
pB = np.array([-2., 4.5])
v_A = np.array([1.2, 1.5])
truncated_vA = unit_vector(v_A) * (np.linalg.norm(v_A) - 0.2)
v_B = np.array([2.0, -0.7])
truncated_vB = unit_vector(v_B) * (np.linalg.norm(v_B) - 0.2)
vA_vB = v_A - v_B
truncated_vA_vB = unit_vector(vA_vB) * (np.linalg.norm(vA_vB) - 0.2)

rA = 0.8
rB = 0.8
tau = 2
side_length = 7

# Derived parameters
pB_pA = pB - pA
pB_pA_tau = pB_pA / tau
r_sum = rA + rB

plane = ORCA.construct_orca_plane(xi=pA, xj=pB, vi=vA_vB, vj=np.array(
    [0, 0]), ri=rA, rj=rB, weight=0.5, buffered_r=0.0, time_horizon=2.0)
n = plane.normal.reshape((2,))
w = plane.point.reshape((2,))

# Plotting the vectors and shaded region
fig, ax = plt.subplots()
fig.set_size_inches(8, 7)

# # Shaded region (velocity obstacle)
cone_angle = np.arctan2(pB_pA[1], pB_pA[0])
cone_width_angle = np.arcsin(r_sum/np.linalg.norm(pB_pA))
lower_angle = cone_angle - cone_width_angle
upper_angle = cone_angle + cone_width_angle
left = np.array([side_length * np.cos(upper_angle),
                side_length * np.sin(upper_angle)])

lower_left_length = np.sqrt(np.linalg.norm(pB_pA_tau)**2 - (r_sum/tau)**2)
lower_left = np.array([lower_left_length * np.cos(upper_angle),
                       lower_left_length * np.sin(upper_angle)])
right = np.array([side_length * np.cos(lower_angle),
                 side_length * np.sin(lower_angle)])
lower_right = np.array([lower_left_length * np.cos(lower_angle),
                        lower_left_length * np.sin(lower_angle)])

# For drawing only

pB_pA_tau_offset = unit_vector(
    pB_pA_tau) * (np.linalg.norm(pB_pA_tau) + 0.005)


# Circles representing positions
circleA = plt.Circle(pB_pA_tau, r_sum / tau, edgecolor='black',
                     facecolor='lemonchiffon', linestyle='-', lw=1)


ax.add_patch(circleA)


vertices = np.array([
    [lower_right[0], lower_right[1]],
    [lower_left[0], lower_left[1]],
    [left[0], left[1]],
    [right[0], right[1]]])
# Plot the triangle
ax.fill(*zip(*vertices),
        edgecolor='none', facecolor='lemonchiffon')

# circleC = plt.Circle(pB_pA, r_sum, edgecolor='black',
#                      facecolor='lemonchiffon', linestyle='--', lw=0.5)
# ax.add_patch(circleC)

# plt.plot([0, left[0]], [0, left[1]], 'k', lw=0.75, linestyle='--')
# plt.plot([0, right[0]], [0, right[1]], 'k', lw=0.75, linestyle='--')

plt.arrow(lower_left[0], lower_left[1], left[0] - lower_left[0], left[1] - lower_left[1], head_width=0.1,
          head_length=0.2, fc='k', ec='k', lw=0.75)

plt.arrow(lower_right[0], lower_right[1], right[0] - lower_right[0], right[1] - lower_right[1], head_width=0.1,
          head_length=0.2, fc='k', ec='k', lw=0.75)


plt.plot([v_A[0], vA_vB[0]], [v_A[1], vA_vB[1]],
         'k', lw=0.75, linestyle='dotted')

plt.arrow(0, 0, truncated_vA[0], truncated_vA[1], head_width=0.1,
          head_length=0.2, fc='k', ec='k', lw=0.75)

# For plotting
vA_vB_w = w-vA_vB
half_vA_vB_w = vA_vB_w / 2
truncated_vA_vB_w = unit_vector(vA_vB_w) * (np.linalg.norm(vA_vB_w)-0.2)
truncated_half_vA_vB_w = unit_vector(
    truncated_vA_vB_w) * (np.linalg.norm(truncated_vA_vB_w)-0.2)

plt.arrow(vA_vB[0], vA_vB[1], truncated_vA_vB_w[0], truncated_vA_vB_w[1], head_width=0.1,
          head_length=0.2, fc='k', ec='k', lw=0.75)

point = v_A + truncated_vA_vB_w/2
truncated_point = unit_vector(point) * (np.linalg.norm(point) - 0.2)
plot_line_from_normal(truncated_vA_vB_w, point, ax, v_A,
                      truncated_vA_vB_w/2, (1.05, 3))


radius_angle = (np.pi/2 - cone_width_angle) + cone_angle
radius = np.array([np.cos(radius_angle + 0.3) * (r_sum - 0.2),
                  np.sin(radius_angle + 0.3) * (r_sum - 0.2)])

radius_tau = np.array([np.cos(radius_angle + 1.6) * (r_sum/tau - 0.2),
                       np.sin(radius_angle + 1.6) * (r_sum/tau - 0.2)])

plt.arrow(v_A[0], v_A[1], truncated_half_vA_vB_w[0], truncated_half_vA_vB_w[1], head_width=0.1,
          head_length=0.2, fc='k', ec='k', lw=0.75)

plt.arrow(0, 0, truncated_vA_vB[0], truncated_vA_vB[1], head_width=0.1,
          head_length=0.2, fc='red', ec='red', lw=1.0)


ax.set_aspect('equal', 'box')
ax.set_xlim(-7, 7)
ax.set_ylim(-1, 8)

# Adding arrows for axes
# Adding arrows for axes
plt.arrow(0, 0, 3, 0, head_width=0.05,
          head_length=0.1, fc='k', ec='k', lw=0.5)
plt.arrow(0, 0, 0, 7.0, head_width=0.05,
          head_length=0.1, fc='k', ec='k', lw=0.5)
plt.arrow(0, 0, -3.0, 0, head_width=0.05,
          head_length=0.1, fc='k', ec='k', lw=0.5)
plt.arrow(0, 0, 0, -0.5, head_width=0.05,
          head_length=0.1, fc='k', ec='k', lw=0.5)

ax.plot(*vA_vB, 'ko', markersize=2, label='Agent i')
ax.plot(*v_A, 'ko', markersize=2, label='Agent i')
ax.plot(*[0, 0], 'ko', markersize=2, label='Agent i')

plt.text(2.75, -0.35, r'$\mathbf{v_x}$', fontsize=14)
plt.text(0.1, 6.85, r'$\mathbf{v_y}$', fontsize=14)

# Annotations
ax.annotate(r'$\Delta \mathbf{v}_{ij}$', xy=(vA_vB[0]/2, vA_vB[1]/2),
            xytext=(-15, -10), textcoords='offset points', ha='center', fontsize=14)

ax.annotate(r'$\mathbf{v}_{i}$', xy=(v_A[0]/2, v_A[1]/2),
            xytext=(0, 10), textcoords='offset points', ha='center', fontsize=14)

ax.annotate(r'$\mathbf{w}_{ij}$', xy=(vA_vB[0], vA_vB[1]),
            xytext=(3, 7), textcoords='offset points', ha='center', fontsize=14)

# ax.annotate(r'$\mathbf{w}_{ij}/2$', xy=(v_A[0], v_A[1]),
#             xytext=(-5, 10), textcoords='offset points', ha='center', fontsize=14)

ax.annotate(r'$\mathbf{VO}_{i|j}^{\tau}$', xy=(-1.75, 2.25),
            xytext=(-15, 40), textcoords='offset points', ha='center', fontsize=14)


ax.axis('off')  # Disable the axis
ax.set_aspect('equal', adjustable='box')  # Maintain aspect ratio

# plt.show()
plt.savefig('orca_constraint_plot.pdf', format='pdf')
