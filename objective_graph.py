import numpy as np
import matplotlib.pyplot as plt

# Enable LaTeX for text rendering
plt.rcParams['text.usetex'] = True
plt.rcParams['font.family'] = 'serif'
plt.rcParams['font.serif'] = 'Computer Modern'
# 1D case
x = np.linspace(-35, 35, 3000)
y = np.zeros_like(x)
y_int = np.zeros_like(x)

a = 1
c = 1
d = 2

for i in range(x.shape[0]):
    n_xij_d = np.abs(x[i]) - d
    fx = a*(1 - np.exp(-n_xij_d/c))**3

    exp_term = np.exp(-n_xij_d)
    factor = (1 - exp_term)**2
    dfx = 3 * ((1 - exp_term)**2) * exp_term

    gx = -(x[i] * (-np.abs(x[i]) - np.exp(3 * (d - np.abs(x[i]))) / 3 + (3 *
           np.exp(2 * (d - np.abs(x[i])))) / 2 - 3 * np.exp(d - np.abs(x[i])) + d)) / np.abs(x[i]) - 1.82
    y[i] = fx
    y_int[i] = dfx

fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(8, 4))
ax1.plot(x, y, linewidth=2)

# Xlim Ylim
ax1.set_xlim([0, 7])
ax1.set_ylim([-5, 2])

ax1.axhline(y=0, color='black', linestyle='--', linewidth=0.75)
ax1.axvline(x=0, color='black', linestyle='--', linewidth=0.75)

# Customize tick appearance
ax1.tick_params(axis='both', which='both', direction='inout')

# # Set custom ticks if needed
# ax1.set_xticks(np.arange(-, 8, 1))
# ax2.set_yticks(np.arange(-7, 3, 1))

# Labels
ax1.set_xlabel(r"$\|\Delta \mathbf{x}\|$", fontsize=18)
ax1.set_ylabel(r"$f$", fontsize=18)

ax1.plot([d, -d], [0, 0], 'o')
ax1.annotate(r"$R_{{influence}}$", xy=[
             0, 0], xytext=[2, -0.5], fontsize=16)

ax2.plot(x, y_int, linewidth=2)
# Xlim Ylim
# ax2.set_xlim([0, 7])
# ax2.set_ylim([-1, 7])
ax2.axhline(y=0, color='black', linestyle='--', linewidth=0.75)

ax2.set_xlabel(r"$\|\Delta \mathbf{x}\|$", fontsize=18)
ax2.set_ylabel(r"$P$", fontsize=18)

ax2.plot([d, -d], [0, 0], 'o')
ax2.annotate(r"$R_{{influence}}$", xy=[
             0, 0], xytext=[1.35, 0.75], fontsize=18)
ax1.tick_params(axis='both', which='major', labelsize=16)
ax2.tick_params(axis='both', which='major', labelsize=16)

# plt.show()
plt.tight_layout()
plt.show()
# plt.savefig('force_func.pdf')
