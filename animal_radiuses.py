import matplotlib
import matplotlib.pyplot as plt
import numpy as np

# Enable LaTeX for text rendering
plt.rcParams['text.usetex'] = True
plt.rcParams['font.family'] = 'serif'
plt.rcParams['font.serif'] = 'Computer Modern'

# Example usage
center = (0, 0)
height = 4
base = 3


def plot_triangle(center, height, base, angle, color, ax):
    # Calculate the vertices of the triangle before rotation
    half_base = base / 2
    vertices = np.array([
        [-half_base, -height / 3],
        [half_base, -height / 3],
        [0, 2*height / 3]
    ])

    # Create a rotation matrix
    angle_rad = np.radians(angle)
    rotation_matrix = np.array([
        [np.cos(angle_rad), -np.sin(angle_rad)],
        [np.sin(angle_rad), np.cos(angle_rad)]
    ])

    # Rotate the vertices
    rotated_vertices = np.dot(vertices, rotation_matrix)

    # Translate vertices to the center
    translated_vertices = rotated_vertices + np.array(center)

    triangle = plt.Polygon(translated_vertices,
                           edgecolor=color, facecolor=color)

    # Plot the triangle
    ax.fill(*zip(*translated_vertices),
            edgecolor=color, facecolor=color)

    return triangle


def plot_robot(center, radius, color):
    return plt.Circle(center, radius, edgecolor=color, facecolor=color)


fig, ax = plt.subplots()
fig.set_size_inches(7, 6)


R_o = 7
R_avoid = 22.5
R_s = 30
head_length = 2
circle = plt.Circle(center, R_s, edgecolor='black',
                    facecolor='none', linestyle='--')
ax.add_patch(circle)
circle = plt.Circle(center, R_avoid, edgecolor='black',
                    facecolor='lightyellow', linestyle='--')
ax.add_patch(circle)
circle = plt.Circle(center, R_o, edgecolor='black',
                    facecolor='peachpuff', linestyle='--')
ax.add_patch(circle)

main_sheep = plot_triangle(center, height, base, 75, 'red', ax)
ax.add_patch(main_sheep)
sheep1 = plot_triangle((15, -12), height, base, 123, 'red', ax)
ax.add_patch(sheep1)
sheep2 = plot_triangle((24, -16), height, base, 43, 'red', ax)
ax.add_patch(sheep2)
sheep3 = plot_triangle((19, -5), height, base, 3, 'red', ax)
ax.add_patch(sheep3)
sheep4 = plot_triangle((27, -8), height, base, 94, 'red', ax)
ax.add_patch(sheep4)
sheep5 = plot_triangle((4, -20), height, base, 67, 'red', ax)
ax.add_patch(sheep5)
sheep6 = plot_triangle((33, -1), height, base, 60, 'red', ax)
ax.add_patch(sheep5)

# Robot
robot = plot_robot((-24, 13), 1.5, 'green')
ax.add_patch(robot)

angle = np.deg2rad(145)
x_end = center[0] + (R_o-head_length) * np.cos(angle)
y_end = center[1] + (R_o-head_length) * np.sin(angle)

# Draw the arrow
ax.arrow(center[0], center[1], x_end - center[0], y_end - center[1],
         head_width=1, head_length=head_length, fc='k', ec='k')
annotation = r'$\mathrm{R_{separation}}$'
ax.annotate(annotation, (x_end, y_end), textcoords="offset points",
            xytext=(-31, 7.5), ha='center')

angle = np.deg2rad(75)
x_end = center[0] + (R_avoid-head_length) * np.cos(angle)
y_end = center[1] + (R_avoid-head_length) * np.sin(angle)

# Draw the arrow
ax.arrow(center[0], center[1], x_end - center[0], y_end - center[1],
         head_width=1, head_length=head_length, fc='k', ec='k')

annotation = r'$\mathrm{R_{influence}}$'
ax.annotate(annotation, (x_end, y_end), textcoords="offset points",
            xytext=(10, 15), ha='center')

angle = np.deg2rad(35)
x_end = center[0] + (R_s-head_length) * np.cos(angle)
y_end = center[1] + (R_s-head_length) * np.sin(angle)

annotation = r'$\mathrm{R_{sensing}}$'
ax.annotate(annotation, (x_end, y_end), textcoords="offset points",
            xytext=(25, 10), ha='center')

# Draw the arrow
ax.arrow(center[0], center[1], x_end - center[0], y_end - center[1],
         head_width=1, head_length=head_length, fc='k', ec='k')

ax.axis('off')  # Disable the axis
ax.set_aspect('equal', adjustable='box')  # Maintain aspect ratio

plt.savefig('animal_radiuses.pdf', format='pdf')
