import matplotlib.pyplot as plt

# Enable LaTeX for text rendering
plt.rcParams['text.usetex'] = True
plt.rcParams['font.family'] = 'serif'
plt.rcParams['font.serif'] = 'Computer Modern'

# Positions and radii
pA = [0.75, -1]
pB = [-0.8, 1]
rA = 0.32
rB = 0.32

# Velocities
vA = [-0.4, 0.5]
vB = [0.5, -0.5]

fig, ax = plt.subplots()

# Plotting the circles
circleA = plt.Circle((pA[0], pA[1]), rA,  facecolor='none',
                     edgecolor='black', lw=1.0)
circleB = plt.Circle((pB[0], pB[1]), rB,  facecolor='none',
                     edgecolor='black', lw=1.0)
ax.add_artist(circleA)
ax.add_artist(circleB)

# Plotting the points
plt.plot(pA[0], pA[1], 'ko')  # Point A
plt.plot(pB[0], pB[1], 'ko')  # Point B

# Annotating the points
plt.text(pA[0] + 0.1, pA[1] - 0.2, '  $\mathbf{p}_i$', fontsize=14,
         verticalalignment='bottom', horizontalalignment='right', weight='bold')
plt.text(pB[0] - 0.05, pB[1], '  $\mathbf{p}_j$', fontsize=14,
         verticalalignment='bottom', horizontalalignment='right', weight='bold')

# Drawing radius lines
plt.plot([pA[0], pA[0] + rA], [pA[1], pA[1]], 'k-', lw=0.5)
plt.plot([pB[0], pB[0] + rB], [pB[1], pB[1]], 'k-', lw=0.5)

# Annotating radii
plt.text(pA[0] + rA/3, pA[1] + 0.05, '$r_i$', fontsize=14,
         verticalalignment='bottom', horizontalalignment='left')
plt.text(pB[0] + rB/3, pB[1] + 0.05, '$r_j$', fontsize=14,
         verticalalignment='bottom', horizontalalignment='left')

# Adding labels A and B
plt.text(pA[0], pA[1] - 1.2 * rA, r'Agent i', fontsize=16,
         verticalalignment='top', horizontalalignment='center')
plt.text(pB[0], pB[1] + 1.2 * rB, r'Agent j', fontsize=16,
         verticalalignment='bottom', horizontalalignment='center')

# Adding velocity arrows
plt.arrow(pA[0], pA[1], vA[0], vA[1], head_width=0.05,
          head_length=0.1, fc='r', ec='r', lw=0.75)
plt.arrow(pB[0], pB[1], vB[0], vB[1], head_width=0.05,
          head_length=0.1, fc='r', ec='r', lw=0.75)

# Setting axis limits
plt.xlim(-2, 2)
plt.ylim(-2, 2)

plt.text(1.15, 0.08, r'$\mathbf{x}$', fontsize=14)
plt.text(0.08, 1.7, r'$\mathbf{y}$', fontsize=14)

# Adding arrows for axes
plt.arrow(0, 0, 1.2, 0, head_width=0.05,
          head_length=0.1, fc='k', ec='k', lw=0.5)
plt.arrow(0, 0, 0, 1.75, head_width=0.05,
          head_length=0.1, fc='k', ec='k', lw=0.5)
plt.arrow(0, 0, -1.2, 0, head_width=0.05,
          head_length=0.1, fc='k', ec='k', lw=0.5)
plt.arrow(0, 0, 0, -1.75, head_width=0.05,
          head_length=0.1, fc='k', ec='k', lw=0.5)

plt.text(vB[0]-0.2, vB[1]+0.15, r'$v_i$', fontsize=14)

plt.text(vA[0], vA[1]-0.25, r'$v_j$', fontsize=14)

# Hiding grid and axis
ax.grid(False)
ax.axis('off')

# Setting aspect ratio to be equal
ax.set_aspect('equal', 'box')


plt.savefig('orca_plot.pdf', format='pdf')
