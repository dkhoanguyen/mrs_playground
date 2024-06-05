import matplotlib.pyplot as plt
import numpy as np

# Enable LaTeX for text rendering
plt.rcParams['text.usetex'] = True
plt.rcParams['font.family'] = 'serif'
plt.rcParams['font.serif'] = 'Computer Modern'


def unit_vector(v):
    return v / np.linalg.norm(v)


# Define the positions and vectors
agent_i_pos = np.array([0, 0])
agent_j_pos = np.array([2, -1])
delta_pij = (agent_i_pos - agent_j_pos) * 0.5
extra_dashed = -(agent_j_pos - 0.3 * delta_pij)
dashed_upper_vert = np.array([1, 2]) * 0.75
dashed_lower_vert = np.array([-1, -2]) * 0.75

delta_vij = np.array([1.25, 0.65])
delta_vij_length = np.linalg.norm(delta_vij)
v_angle = np.arctan2(delta_vij[1], delta_vij[0])

delta_vij_with_arrow = np.array([np.cos(v_angle) * (delta_vij_length + 0.2),
                                 np.sin(v_angle) * (delta_vij_length + 0.2)])

delta_v = np.array([1, -1])

delta_vx = np.dot(
    delta_vij, unit_vector(delta_pij)) * unit_vector(delta_pij)
delta_vx_with_arrow = np.dot(
    delta_vij_with_arrow, unit_vector(delta_pij)) * unit_vector(delta_pij)

delta_vy = np.dot(
    delta_vij, unit_vector(dashed_upper_vert)) * unit_vector(dashed_upper_vert)
delta_vy_with_arrow = np.dot(
    delta_vij_with_arrow, unit_vector(dashed_upper_vert)) * unit_vector(dashed_upper_vert)

delta_vx_angle = np.arctan2(delta_vx[1], delta_vx[0])

# Create a figure and axis
fig, ax = plt.subplots()
fig.set_size_inches(4, 3)

# Plot the agents
ax.plot(*agent_i_pos, 'ko', markersize=2, label='Agent i')
ax.plot(*agent_j_pos, 'ko', markersize=2, label='Agent j')

# Plot the circles around agents
circle_i = plt.Circle(agent_i_pos, 0.2, edgecolor='k', facecolor='none')
circle_j = plt.Circle(agent_j_pos, 0.2, edgecolor='k', facecolor='none')
ax.add_patch(circle_i)
ax.add_patch(circle_j)

# Plot the vectors
ax.arrow(agent_i_pos[0], agent_i_pos[1], delta_pij[0],
         delta_pij[1], head_width=0.1, head_length=0.2, fc='k', ec='k')
ax.arrow(agent_i_pos[0], agent_i_pos[1], delta_vij[0],
         delta_vij[1], head_width=0.1, head_length=0.2, fc='k', ec='k')

ax.arrow(agent_i_pos[0], agent_i_pos[1], delta_vx[0] - np.cos(delta_vx_angle) * 0.085,
         delta_vx[1] - np.sin(delta_vx_angle) * 0.085, head_width=0.1, head_length=0.2, fc='k', ec='k')


ax.plot([agent_j_pos[0], agent_j_pos[0] + extra_dashed[0]],
        [agent_j_pos[1], agent_j_pos[1] + extra_dashed[1]], 'k--', linewidth=1)
ax.plot([agent_i_pos[0], agent_i_pos[0] + dashed_upper_vert[0]],
        [agent_i_pos[1], agent_i_pos[1] + dashed_upper_vert[1]], 'k--', linewidth=1)
ax.plot([agent_i_pos[0], agent_i_pos[0] + dashed_upper_vert[0]],
        [agent_i_pos[1], agent_i_pos[1] + dashed_upper_vert[1]], 'k--', linewidth=1)
ax.plot([agent_i_pos[0], agent_i_pos[0] + dashed_lower_vert[0]],
        [agent_i_pos[1], agent_i_pos[1] + dashed_lower_vert[1]], 'k--', linewidth=1)

ax.plot([delta_vx_with_arrow[0], delta_vx_with_arrow[0] + delta_vy_with_arrow[0]],
        [delta_vx_with_arrow[1], delta_vx_with_arrow[1] + delta_vy_with_arrow[1]], 'k--', linewidth=1)
ax.plot([delta_vy_with_arrow[0], delta_vx_with_arrow[0] + delta_vy_with_arrow[0]],
        [delta_vy_with_arrow[1], delta_vx_with_arrow[1] + delta_vy_with_arrow[1]], 'k--', linewidth=1)

# Add annotations
ax.annotate(r'$\Delta \mathbf{p}_{ij}$', xy=(agent_i_pos[0] + delta_pij[0], agent_i_pos[1] + delta_pij[1]),
            xytext=(-10, 15), textcoords='offset points', ha='center', fontsize=14)

ax.annotate(r'$\Delta \mathbf{v}_{ij}$', xy=(agent_i_pos[0] + delta_vij[0], agent_i_pos[1] + delta_vij[1]),
            xytext=(20, 10), textcoords='offset points', ha='center', fontsize=14)
ax.annotate(r'$\Delta \mathbf{v}$', xy=(agent_j_pos[0] + delta_v[0] / 2, agent_j_pos[1] + delta_v[1] / 2),
            xytext=(10, -20), textcoords='offset points', ha='center', fontsize=14)

ax.annotate(r'$\Delta \bar{v}$', xy=(agent_i_pos[0] + delta_vx_with_arrow[0], agent_i_pos[1] + delta_vx_with_arrow[1]),
            xytext=(0, -20), textcoords='offset points', ha='center', fontsize=14)


ax.annotate('Agent i', xy=agent_i_pos, xytext=(-40, -10),
            textcoords='offset points', ha='center', fontsize=14)
ax.annotate('Agent j', xy=agent_j_pos, xytext=(20, 20),
            textcoords='offset points', ha='center', fontsize=14)

# # Plot dashed lines
# ax.plot([agent_i_pos[0], agent_j_pos[0]], [
#         agent_i_pos[1], agent_j_pos[1]], 'k--', linewidth=1)
# ax.plot([agent_i_pos[0], agent_i_pos[0] + delta_vij[0]],
#         [agent_i_pos[1], agent_i_pos[1] + delta_vij[1]], 'k--', linewidth=1)
# ax.plot([agent_i_pos[0], agent_i_pos[0] + delta_pij[0]],
#         [agent_i_pos[1], agent_i_pos[1] + delta_pij[1]], 'k--', linewidth=1)

# Set equal aspect ratio
ax.set_aspect('equal')

# Hide the axes
ax.axis('off')

# # Save the plot to a PDF file
plt.savefig('min_max_cbf.pdf', format='pdf')
