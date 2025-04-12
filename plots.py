import pandas as pd
import matplotlib.pyplot as plt

# Load data
df = pd.read_csv("bereshit_data.csv")

# Define all variables to plot
variables = ["vs", "hs", "dist", "alt", "ang", "weight", "acc"]

# Create subplots
fig, axes = plt.subplots(nrows=2, ncols=4, figsize=(12, 6))

# Flatten axes array for easy iteration
axes = axes.flatten()

# Plot each variable
for i, var in enumerate(variables):
    axes[i].plot(df["time"], df[var])
    axes[i].set_title(var)
    axes[i].set_xlabel("Time")
    axes[i].set_ylabel(var)
    axes[i].set_xlim(0, 500)  

# Hide last empty subplot
axes[-1].axis("off")

# Adjust layout
plt.tight_layout()
plt.show()
