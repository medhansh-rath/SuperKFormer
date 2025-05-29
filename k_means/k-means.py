import numpy as np
import matplotlib.pyplot as plt
from sklearn.datasets import make_blobs

# Step 1: Generate data
X, _ = make_blobs(n_samples=300, centers=3, cluster_std=1.0, random_state=42)

# Step 2: Initialize centroids randomly
k = 3
np.random.seed(42)
initial_centroids = X[np.random.choice(X.shape[0], k, replace=False)]

def assign_clusters(X, centroids):
    """Assigns each point to the nearest centroid"""
    distances = np.linalg.norm(X[:, np.newaxis] - centroids, axis=2)
    return np.argmin(distances, axis=1)

def update_centroids(X, labels, k):
    """Recomputes centroids as the mean of points in each cluster"""
    return np.array([X[labels == i].mean(axis=0) for i in range(k)])

# Step 3: First assignment
labels_step1 = assign_clusters(X, initial_centroids)
centroids_step2 = update_centroids(X, labels_step1, k)

# Step 4: Second assignment (final)
labels_final = assign_clusters(X, centroids_step2)
centroids_step3 = update_centroids(X, labels_final, k)

# Plotting all steps
fig, axs = plt.subplots(2, 2, figsize=(12, 12))

colors = ['red', 'green', 'blue']
titles = ['Initial Random Centroids', 'First Assignment', 'Updating Centroids and Second Assignment', 'Updating Centroids and Final Clusters']

# # Plot 0: Plot dataset
# axs[0, 0].scatter(X[:, 0], X[:, 1], c='gray', alpha=0.5)
# axs[0, 0].set_title(titles[0])
# axs[0, 0].grid(True)

# Plot 1: Initial centroids
axs[0, 0].scatter(X[:, 0], X[:, 1], c='gray', alpha=0.5)
axs[0, 0].scatter(initial_centroids[:, 0], initial_centroids[:, 1], c='black', marker='X', s=200)
axs[0, 0].set_title(titles[0])
axs[0, 0].grid(True)

# Plot 2: First assignment
for i in range(k):
    axs[0, 1].scatter(X[labels_step1 == i][:, 0], X[labels_step1 == i][:, 1], c=colors[i], alpha=0.6, label=f'Cluster {i+1}')
axs[0, 1].scatter(initial_centroids[:, 0], initial_centroids[:, 1], c='black', marker='X', s=200)
axs[0, 1].set_title(titles[1])
axs[0, 1].legend()
axs[0, 1].grid(True)

# Plot 2: First assignment
for i in range(k):
    axs[1, 0].scatter(X[labels_final == i][:, 0], X[labels_final == i][:, 1], c=colors[i], alpha=0.6, label=f'Cluster {i+1}')
axs[1, 0].scatter(centroids_step2[:, 0], centroids_step2[:, 1], c='black', marker='X', s=200)
axs[1, 0].set_title(titles[2])
axs[1, 0].legend()
axs[1, 0].grid(True)

# Plot 3: Final assignment
for i in range(k):
    axs[1, 1].scatter(X[labels_final == i][:, 0], X[labels_final == i][:, 1], c=colors[i], alpha=0.6, label=f'Cluster {i+1}')
axs[1, 1].scatter(centroids_step3[:, 0], centroids_step3[:, 1], c='black', marker='X', s=200)
axs[1, 1].set_title(titles[3])
axs[1, 1].legend()
axs[1, 1].grid(True)

# Correctly set labels for all subplots
for ax_row in axs:
    for ax in ax_row:
        ax.set_xlabel('Feature 1')
        ax.set_ylabel('Feature 2')

plt.tight_layout()
plt.savefig('k_means.png')
#plt.show()