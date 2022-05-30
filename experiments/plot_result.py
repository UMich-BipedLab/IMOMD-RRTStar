import pandas as pd
import matplotlib.pyplot as plt
import os

# Data Load
dir_path = os.path.dirname(os.path.realpath(__file__))
folder = "bugtrap/sanfrancisco/"

files = ["imomd_8", "bi_astar_8", "ana_star_8"]
algorithms = ["IMOMD", "Bi-A*", "ANA*"]
path_colors = ["-b", "-r", "-g"]
size_colors = ["--b", "--r", "--g"]

log_data = {}
for file_name, algorithm in zip(files, algorithms):
    log_data[algorithm] = pd.read_csv(os.path.join(dir_path, folder, file_name +
        ".csv"), sep=';')

# Plot
plt.rcParams.update({'font.size': 40})
fig, ax1 = plt.subplots()

ax1.set_xlabel("Elapsed Time [s]", labelpad=10, size=50)
ax1.set_ylabel("Path Cost [km]", labelpad=10, size=50)
for algorithm, color in zip(algorithms, path_colors):
    ax1.step(log_data[algorithm].CPU_time, log_data[algorithm].path_cost/1000,
            color, where='post', label=algorithm + "(Path Cost)", linewidth=5)
lgd1 = ax1.legend(loc=9, bbox_to_anchor=(0.25, 1.0))
# ax1.legend()

ax2 = ax1.twinx()
ax2.set_ylabel("Total Size of Trees [k]", labelpad=10, size=50)
for algorithm, color in zip(algorithms, size_colors):
    ax2.plot(log_data[algorithm].CPU_time, log_data[algorithm].tree_size/1000,
            color, label=algorithm + "(Tree Size)", linewidth=5)

lgd2 = ax2.legend(loc=9, bbox_to_anchor=(0.55, 1.0))
# ax2.legend()
# fig.tight_layout()

plt.xlim(0, 25)
# plt.ylim(0, 1000)
plt.show()
# fig.savefig('samplefigure', bbox_inches='tight')
