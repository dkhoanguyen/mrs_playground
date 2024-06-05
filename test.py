import matplotlib.pyplot as plt

plt.rcParams['text.usetex'] = True
plt.rcParams['font.family'] = 'serif'
plt.rcParams['font.serif'] = 'Computer Modern Roman'

fig, ax = plt.subplots()
ax.text(0.5, 0.5, r'$\alpha + \beta = \gamma$', fontsize=20, ha='center')

ax.set_xlim(0, 1)
ax.set_ylim(0, 1)

plt.show()
