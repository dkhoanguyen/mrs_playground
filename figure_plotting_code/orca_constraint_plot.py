import matplotlib.pyplot as plt
import numpy as np

from mr_herding.cbf.constraints import *


# Enable LaTeX for text rendering
plt.rcParams['text.usetex'] = True
plt.rcParams['font.family'] = 'serif'
plt.rcParams['font.serif'] = 'Computer Modern'


def unit_vector(v):
    return v / np.linalg.norm(v)


def plot_line_from_normal(normal, point, ax1, v, w, x_range=(-10, 10)):
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
    # ax2.plot(x_values, y_values,
    #         label=f'Line through ({x0}, {y0}) with normal {normal}')
    # Add double-ended arrow
    # mid_idx = len(x_values) // 2
    ax2.annotate('', xy=(x_values[0], y_values[0]), xytext=(x_values[-1], y_values[-1]),
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

    ax2.fill(*zip(*vertices),
             edgecolor='none', facecolor='mistyrose')

    # Add annotation along the line
    annotation_text = r'$\mathit{ORCA}_{i|j}^{\tau}$'
    ax2.text(x0+0.35, y0+2, annotation_text, rotation=angle, ha='center', va='center',
             fontsize=18, color='black')


# Parameters for the plot
pA = np.array([0, 0])
pB = np.array([-2., 4.5])
v_A = np.array([1.15, 1.58])
truncated_vA = unit_vector(v_A) * (np.linalg.norm(v_A) - 0.2)
v_B = np.array([2.0, -0.7])
truncated_vB = unit_vector(v_B) * (np.linalg.norm(v_B) - 0.2)
vA_vB = v_A - v_B
truncated_vA_vB = unit_vector(vA_vB) * (np.linalg.norm(vA_vB) - 0.2)

rA = 0.75
rB = 0.75
tau = 2
side_length = 6

# Derived parameters
pB_pA = pB - pA
pB_pA_tau = pB_pA / tau
r_sum = rA + rB

plane = ORCA.construct_orca_plane(xi=pA, xj=pB, vi=vA_vB, vj=np.array(
    [0, 0]), ri=rA, rj=rB, weight=0.5, buffered_r=0.0, time_horizon=2.0)
n = plane.normal.reshape((2,))
w = plane.point.reshape((2,))

# Plotting the vectors and shaded region
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(8, 5))
# fig.set_size_inches(15, 7)

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


ax2.add_patch(circleA)


vertices = np.array([
    [lower_right[0], lower_right[1]],
    [lower_left[0], lower_left[1]],
    [left[0], left[1]],
    [right[0], right[1]]])
# Plot the triangle
ax2.fill(*zip(*vertices),
         edgecolor='none', facecolor='lemonchiffon')

# circleC = ax2.Circle(pB_pA, r_sum, edgecolor='black',
#                      facecolor='lemonchiffon', linestyle='--', lw=0.5)
# ax2.add_patch(circleC)

# ax2.plot([0, left[0]], [0, left[1]], 'k', lw=0.75, linestyle='--')
# ax2.plot([0, right[0]], [0, right[1]], 'k', lw=0.75, linestyle='--')

ax2.arrow(lower_left[0], lower_left[1], left[0] - lower_left[0], left[1] - lower_left[1], head_width=0.1,
          head_length=0.2, fc='k', ec='k', lw=0.75)

ax2.arrow(lower_right[0], lower_right[1], right[0] - lower_right[0], right[1] - lower_right[1], head_width=0.1,
          head_length=0.2, fc='k', ec='k', lw=0.75)


ax2.plot([v_A[0], vA_vB[0]], [v_A[1], vA_vB[1]],
         'k', lw=0.75, linestyle='dotted')

ax2.arrow(0, 0, truncated_vA[0], truncated_vA[1], head_width=0.1,
          head_length=0.2, fc='k', ec='k', lw=0.75)

# For plotting
vA_vB_w = w-vA_vB
half_vA_vB_w = vA_vB_w / 2
truncated_vA_vB_w = unit_vector(vA_vB_w) * (np.linalg.norm(vA_vB_w)-0.2)
truncated_half_vA_vB_w = unit_vector(
    truncated_vA_vB_w) * (np.linalg.norm(truncated_vA_vB_w)-0.1)

ax2.arrow(vA_vB[0], vA_vB[1], truncated_vA_vB_w[0], truncated_vA_vB_w[1], head_width=0.1,
          head_length=0.2, fc='k', ec='k', lw=0.75)

point = v_A + truncated_vA_vB_w/2
truncated_point = unit_vector(point) * (np.linalg.norm(point) - 0.2)
plot_line_from_normal(truncated_vA_vB_w, point, ax1, v_A,
                      truncated_vA_vB_w/1.75, (0.75, 2.2))


radius_angle = (np.pi/2 - cone_width_angle) + cone_angle
radius = np.array([np.cos(radius_angle + 0.3) * (r_sum - 0.2),
                  np.sin(radius_angle + 0.3) * (r_sum - 0.2)])

radius_tau = np.array([np.cos(radius_angle + 1.6) * (r_sum/tau - 0.2),
                       np.sin(radius_angle + 1.6) * (r_sum/tau - 0.2)])

ax2.arrow(v_A[0], v_A[1], truncated_half_vA_vB_w[0], truncated_half_vA_vB_w[1], head_width=0.1,
          head_length=0.2, fc='k', ec='k', lw=0.75)

ax2.arrow(0, 0, truncated_vA_vB[0], truncated_vA_vB[1], head_width=0.1,
          head_length=0.2, fc='red', ec='red', lw=1.0)


ax2.set_aspect('equal', 'box')
ax2.set_xlim(-3, 7)
ax2.set_ylim(-1, 7.5)

# Adding arrows for axes
# Adding arrows for axes
ax2.arrow(0, 0, 3, 0, head_width=0.05,
          head_length=0.1, fc='k', ec='k', lw=0.5)
ax2.arrow(0, 0, 0, 7.0, head_width=0.05,
          head_length=0.1, fc='k', ec='k', lw=0.5)
ax2.arrow(0, 0, -3.0, 0, head_width=0.05,
          head_length=0.1, fc='k', ec='k', lw=0.5)
ax2.arrow(0, 0, 0, -0.5, head_width=0.05,
          head_length=0.1, fc='k', ec='k', lw=0.5)

ax2.plot(*vA_vB, 'ko', markersize=2, label='Agent i')
ax2.plot(*v_A, 'ko', markersize=2, label='Agent i')
ax2.plot(*[0, 0], 'ko', markersize=2, label='Agent i')

ax2.text(2.75, -0.35, r'$\mathbf{v}_x$', fontsize=18)
ax2.text(0.1, 6.85, r'$\mathbf{v}_y$', fontsize=18)

# Annotations
ax2.annotate(r'$\Delta \mathbf{v}_{ij}$', xy=(vA_vB[0]/2, vA_vB[1]/2),
             xytext=(-15, -10), textcoords='offset points', ha='center', fontsize=18)

ax2.annotate(r'$\mathbf{v}_{i}$', xy=(v_A[0]/2, v_A[1]/2),
             xytext=(0, 10), textcoords='offset points', ha='center', fontsize=18)

ax2.annotate(r'$\mathbf{p}_{ij}$', xy=(vA_vB[0], vA_vB[1]),
             xytext=(3, 8), textcoords='offset points', ha='center', fontsize=18)

# ax2.annotate(r'$\mathbf{w}_{ij}/2$', xy=(v_A[0], v_A[1]),
#             xytext=(-5, 10), textcoords='offset points', ha='center', fontsize=18)

ax2.annotate(r'$\mathbf{VO}_{i|j}^{\tau}$', xy=(-1.75, 2.25),
             xytext=(-15, 40), textcoords='offset points', ha='center', fontsize=18)


# ax2.axis('off')  # Disable the axis
ax2.set_aspect('equal', adjustable='box')  # Maintain aspect ratio
ax2.set_xlim(-4.2, 3.25)
# ax6.set_ylim(ylim_min, ylim_max * 1.2)

# Positions and radii
pA = [-0.75, -1]
pB = [0.8, 1]
rA = 0.32
rB = 0.32

# Velocities
vA = [0.4, 0.5]
vB = [-0.5, -0.5]

# fig, ax = plt.subplots()

# Plotting the circles
circleA = plt.Circle((pA[0], pA[1]), rA,  facecolor='none',
                     edgecolor='black', lw=1.0)
circleB = plt.Circle((pB[0], pB[1]), rB,  facecolor='none',
                     edgecolor='black', lw=1.0)
ax1.add_artist(circleA)
ax1.add_artist(circleB)

# Plotting the points
ax1.plot(pA[0], pA[1], 'ko')  # Point A
ax1.plot(pB[0], pB[1], 'ko')  # Point B

# Annotating the points
ax1.text(pA[0] + 0.1, pA[1] - 0.2, '  $\mathbf{x}_i$', fontsize=18,
         verticalalignment='bottom', horizontalalignment='right', weight='bold')
ax1.text(pB[0] - 0.05, pB[1], '  $\mathbf{x}_j$', fontsize=18,
         verticalalignment='bottom', horizontalalignment='right', weight='bold')
ax1.text(1.5, -1.9, 'a)', fontsize=20,
         verticalalignment='bottom', horizontalalignment='right', weight='bold')
ax2.text(3.5, -1.72, 'b)', fontsize=20,
         verticalalignment='bottom', horizontalalignment='right', weight='bold')

# Drawing radius lines
ax1.plot([pA[0], pA[0] + rA], [pA[1], pA[1]], 'k-', lw=0.5)
ax1.plot([pB[0], pB[0] + rB], [pB[1], pB[1]], 'k-', lw=0.5)

# Annotating radii
ax1.text(pA[0] + rA/3, pA[1] + 0.05, '$r_i$', fontsize=18,
         verticalalignment='bottom', horizontalalignment='left')
ax1.text(pB[0] + rB/3, pB[1] + 0.05, '$r_j$', fontsize=18,
         verticalalignment='bottom', horizontalalignment='left')

# Adding labels A and B
ax1.text(pA[0], pA[1] - 1.2 * rA, r'Agent i', fontsize=18,
         verticalalignment='top', horizontalalignment='center')
ax1.text(pB[0], pB[1] + 1.2 * rB, r'Agent j', fontsize=18,
         verticalalignment='bottom', horizontalalignment='center')

# Adding velocity arrows
ax1.arrow(pA[0], pA[1], vA[0], vA[1], head_width=0.05,
          head_length=0.1, fc='r', ec='r', lw=0.75)
ax1.arrow(pB[0], pB[1], vB[0], vB[1], head_width=0.05,
          head_length=0.1, fc='r', ec='r', lw=0.75)

# Setting axis limits
ax1.set_xlim(-1.5, 1.5)
ax1.set_ylim(-1.7, 1.7)

ax1.text(1.0, 0.08, r'$\mathbf{x}$', fontsize=18)
ax1.text(0.08, 1.5, r'$\mathbf{y}$', fontsize=18)

# Adding arrows for axes
ax1.arrow(0, 0, 1.0, 0, head_width=0.05,
          head_length=0.1, fc='k', ec='k', lw=0.5)
ax1.arrow(0, 0, 0, 1.5, head_width=0.05,
          head_length=0.1, fc='k', ec='k', lw=0.5)
ax1.arrow(0, 0, -1.0, 0, head_width=0.05,
          head_length=0.1, fc='k', ec='k', lw=0.5)
ax1.arrow(0, 0, 0, -1.5, head_width=0.05,
          head_length=0.1, fc='k', ec='k', lw=0.5)

ax1.text(vB[0]-0.005, vB[1]+0.1, r'$\mathbf{v}_i$', fontsize=18)

ax1.text(vA[0]-0.1, vA[1]-0.15, r'$\mathbf{v}_j$', fontsize=18)

# Hiding grid and axis
ax1.grid(False)
ax1.axis('off')
ax2.grid(False)
ax2.axis('off')

# Setting aspect ratio to be equal
ax1.set_aspect('equal', 'box')
ax2.set_aspect('equal', 'box')


plt.tight_layout()
# plt.show()
plt.savefig('orca_constraint_plot.pdf', format='pdf')
