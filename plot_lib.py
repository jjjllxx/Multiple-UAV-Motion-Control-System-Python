import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import imageio
import structs

def save_traj_gif(trajs, world:structs.World):
    time_limit = len(trajs[0])

    for traj in trajs:
        time_limit = min(time_limit, len(traj))

    time_limit = time_limit + 2
    
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlim(0, world.width)
    ax.set_ylim(0, world.length)
    ax.set_zlim(0, world.height)

    frames = []

    for t in range(time_limit):
        ax.cla()
        ax.set_xlim(0, world.width)
        ax.set_ylim(0, world.length)
        ax.set_zlim(0, world.height)

        for traj in trajs:
            idx = min(t, len(traj))
            ax.plot(traj[idx].x, traj[idx].y, traj[idx].z, 'o', markersize=4)
            ax.plot([pt.x for pt in traj], [pt.y for pt in traj], [pt.z for pt in traj], lw=1)
        
        plt.draw()
        fig.canvas.draw()
        frame = np.frombuffer(fig.canvas.tostring_rgb(), dtype=np.uint8)
        frame = frame.reshape(fig.canvas.get_width_height()[::-1] + (3,))
        frames.append(frame)

        plt.pause(0.5)

    with imageio.get_writer('trajectories.gif', mode='I', duration=0.5) as writer:
        for frame in frames:
            writer.append_data(frame)

    plt.show()

# Work similarly as comet3 function in matlab
def comet3(x, y, z, interval=100):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    ax.plot(x, y, z, label='3D Line')

    marker, = ax.plot([], [], [], 'ro')

    ax.set_xlim(min(x), max(x))
    ax.set_ylim(min(y), max(y))
    ax.set_zlim(min(z), max(z))
    ax.set_xlabel('X-axis')
    ax.set_ylabel('Y-axis')
    ax.set_zlabel('Z-axis')

    # Function to update the marker position
    def update(frame):
        marker.set_data(x[frame:frame+1], y[frame:frame+1])
        marker.set_3d_properties([z[frame]])
        return marker,

    ani = FuncAnimation(fig, update, frames=len(x), interval=interval, blit=True)

    plt.show()

def plot_key_pts(key_pts_all):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    for i in range(len(key_pts_all)):
        ax.plot([pt.x for pt in key_pts_all[i]], [pt.y for pt in key_pts_all[i]], [pt.z for pt in key_pts_all[i]], label=f'Path {i+1}')

    ax.legend(loc='best')
    ax.set_title('RRT Path Extraction')
    plt.show()

def plot_traj_xyz(traj_all):
    fig, axes = plt.subplots(3, 1, figsize=(8, 6))

    for i in range(len(traj_all)):
        axes[0].plot([i + 1 for i in range(len(traj_all[i]))], [pt.x for pt in traj_all[i]], label=f'traj{i+1}-x')

    axes[0].legend(loc='best')
    axes[0].set_xlabel('time (s)')
    axes[0].set_ylabel(f'x-location')
    axes[0].set_title(f'Time-based trajectory (x-axis)')

    for i in range(len(traj_all)):
        axes[1].plot([i + 1 for i in range(len(traj_all[i]))], [pt.z for pt in traj_all[i]], label=f'traj{i+1}-y')

    axes[1].legend(loc='best')
    axes[1].set_xlabel('time (s)')
    axes[1].set_ylabel(f'y-location')
    axes[1].set_title(f'Time-based trajectory (y-axis)')

    for i in range(len(traj_all)):
        axes[2].plot([i + 1 for i in range(len(traj_all[i]))], [pt.z for pt in traj_all[i]], label=f'traj{i+1}-z')

    axes[2].legend(loc='best')
    axes[2].set_xlabel('time (s)')
    axes[2].set_ylabel(f'z-location')
    axes[2].set_title(f'Time-based trajectory (z-axis)')

    plt.tight_layout()
    plt.show()

def show_old_new(traj1, traj2, idx):
    t1 = list(range(len(traj1)))
    t2 = list(range(len(traj2)))
    
    fig, axs = plt.subplots(3, 1, figsize=(8, 10))
    
    axs[0].plot(t1, [pt.x for pt in traj1], label='Old Traj - x')
    axs[0].plot(t2, [pt.x for pt in traj2], label='New Traj - x')
    axs[0].set_title(f'Comparison of Old and New Trajectory {idx} (x-axis)')
    axs[0].set_xlabel('Time (s)')
    axs[0].set_ylabel('X-location')
    axs[0].legend(loc='best')
    axs[0].grid(True)
    
    axs[1].plot(t1, [pt.y for pt in traj1], label='Old Traj - y')
    axs[1].plot(t2, [pt.y for pt in traj2], label='New Traj - y')
    axs[1].set_title(f'Comparison of Old and New Trajectory {idx} (y-axis)')
    axs[1].set_xlabel('Time (s)')
    axs[1].set_ylabel('Y-location')
    axs[1].legend(loc='best')
    axs[1].grid(True)
    
    axs[2].plot(t1, [pt.z for pt in traj1], label='Old Traj - z')
    axs[2].plot(t2, [pt.z for pt in traj2], label='New Traj - z')
    axs[2].set_title(f'Comparison of Old and New Trajectory {idx} (z-axis)')
    axs[2].set_xlabel('Time (s)')
    axs[2].set_ylabel('Z-location')
    axs[2].legend(loc='best')
    axs[2].grid(True)
    
    plt.tight_layout()
    plt.show()

def plot_result(world: structs.World, path, tree = None):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    ax.set_xlim(0, world.width)
    ax.set_ylim(0, world.length)
    ax.set_zlim(0, world.height)

    ax.set_xlabel('X axis')
    ax.set_ylabel('Y axis')
    ax.set_zlabel('Z axis')
    ax.set_title('RRT Algorithm (3D)')

    # Plot obstacles
    for obs in world.obstacles:
        u, v = np.mgrid[0:2 * np.pi:10j, 0:np.pi:10j]
        X = obs.radius * np.sin(v) * np.cos(u)
        Y = obs.radius * np.sin(v) * np.sin(u)
        Z = obs.radius * np.cos(v)
        ax.plot_surface(X + obs.x, Y + obs.y, Z + obs.z, color=[0.5, 0.2, 0.3])

    if tree:
        for idx in range(len(tree) - 1, -1, -1):
            branch = []
            node = tree[idx]  # Current node
            branch.append(node)  # Add current node to the branch
            
            parent_id = node.parent
            while parent_id > 0:  # Iterate until reaching the root node
                branch.append(tree[parent_id])  # Add parent node
                parent_id = tree[parent_id].parent  # Get the parent of the current node

            ax.plot([node.pt.x for node in branch], 
                    [node.pt.y for node in branch], 
                    [node.pt.z for node in branch], 
                    color='red', linewidth=0.5, marker='.', markeredgecolor='green')

    # Plot the path
    ax.plot([pt.x for pt in path], [pt.y for pt in path], [pt.z for pt in path], color='black', linewidth=3)

    ax.set_xlabel('X axis')
    ax.set_ylabel('Y axis')
    ax.set_zlabel('Z axis')
    ax.set_title('Expanded Tree (3D)')
    plt.grid(True)
    plt.show()