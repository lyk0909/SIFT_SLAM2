#!/usr/bin/env python3
# SIFT-SLAM Trajectory Visualization Tool

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import sys

def read_trajectory(filename):
    """读取TUM格式的轨迹文件"""
    try:
        data = np.loadtxt(filename)
        timestamps = data[:, 0]
        positions = data[:, 1:4]   # tx, ty, tz
        quaternions = data[:, 4:8] # qx, qy, qz, qw
        return timestamps, positions, quaternions
    except Exception as e:
        print(f"Error reading file: {e}")
        sys.exit(1)

def calculate_statistics(timestamps, positions):
    """计算轨迹统计信息"""
    # 计算总距离
    distances = np.sqrt(np.sum(np.diff(positions, axis=0)**2, axis=1))
    total_distance = np.sum(distances)
    
    # 计算平均速度
    time_diff = np.diff(timestamps)
    velocities = distances / time_diff
    avg_velocity = np.mean(velocities)
    max_velocity = np.max(velocities)
    
    # 轨迹范围
    x_range = positions[:, 0].max() - positions[:, 0].min()
    y_range = positions[:, 1].max() - positions[:, 1].min()
    z_range = positions[:, 2].max() - positions[:, 2].min()
    
    print("=" * 60)
    print("📊 SIFT-SLAM Trajectory Statistics")
    print("=" * 60)
    print(f"Number of keyframes: {len(positions)}")
    print(f"Duration: {timestamps[-1] - timestamps[0]:.2f} seconds")
    print(f"Total distance traveled: {total_distance:.2f} meters")
    print(f"Average velocity: {avg_velocity:.2f} m/s")
    print(f"Max velocity: {max_velocity:.2f} m/s")
    print(f"\nTrajectory bounds:")
    print(f"  X range: {x_range:.2f} m (min: {positions[:, 0].min():.2f}, max: {positions[:, 0].max():.2f})")
    print(f"  Y range: {y_range:.2f} m (min: {positions[:, 1].min():.2f}, max: {positions[:, 1].max():.2f})")
    print(f"  Z range: {z_range:.2f} m (min: {positions[:, 2].min():.2f}, max: {positions[:, 2].max():.2f})")
    print(f"\nStart position: ({positions[0, 0]:.3f}, {positions[0, 1]:.3f}, {positions[0, 2]:.3f})")
    print(f"End position: ({positions[-1, 0]:.3f}, {positions[-1, 1]:.3f}, {positions[-1, 2]:.3f})")
    print("=" * 60)

def plot_trajectory_3d(positions, save_path='trajectory_3d.png'):
    """3D轨迹可视化"""
    fig = plt.figure(figsize=(14, 10))
    ax = fig.add_subplot(111, projection='3d')
    
    # 绘制轨迹 - 根据时间着色
    n_points = len(positions)
    colors = plt.cm.viridis(np.linspace(0, 1, n_points))
    
    # 绘制彩色轨迹线
    for i in range(n_points - 1):
        ax.plot(positions[i:i+2, 0], positions[i:i+2, 1], positions[i:i+2, 2],
                color=colors[i], linewidth=2, alpha=0.8)
    
    # 标记起点和终点
    ax.scatter(positions[0, 0], positions[0, 1], positions[0, 2],
               c='green', marker='o', s=200, label='Start', edgecolors='black', linewidths=2)
    ax.scatter(positions[-1, 0], positions[-1, 1], positions[-1, 2],
               c='red', marker='s', s=200, label='End', edgecolors='black', linewidths=2)
    
    # 每10个关键帧标记一个点
    step = max(1, len(positions) // 10)
    ax.scatter(positions[::step, 0], positions[::step, 1], positions[::step, 2],
               c='blue', marker='.', s=50, alpha=0.5, label='Keyframes')
    
    ax.set_xlabel('X (m)', fontsize=12, fontweight='bold')
    ax.set_ylabel('Y (m)', fontsize=12, fontweight='bold')
    ax.set_zlabel('Z (m)', fontsize=12, fontweight='bold')
    ax.set_title('SIFT-SLAM 3D Trajectory (Color: Time Progress)', fontsize=14, fontweight='bold')
    ax.legend(fontsize=10)
    ax.grid(True, alpha=0.3)
    
    # 设置相等的坐标轴比例
    max_range = np.array([
        positions[:, 0].max() - positions[:, 0].min(),
        positions[:, 1].max() - positions[:, 1].min(),
        positions[:, 2].max() - positions[:, 2].min()
    ]).max() / 2.0
    
    mid_x = (positions[:, 0].max() + positions[:, 0].min()) * 0.5
    mid_y = (positions[:, 1].max() + positions[:, 1].min()) * 0.5
    mid_z = (positions[:, 2].max() + positions[:, 2].min()) * 0.5
    
    ax.set_xlim(mid_x - max_range, mid_x + max_range)
    ax.set_ylim(mid_y - max_range, mid_y + max_range)
    ax.set_zlim(mid_z - max_range, mid_z + max_range)
    
    plt.tight_layout()
    plt.savefig(save_path, dpi=300, bbox_inches='tight')
    print(f"✅ Saved: {save_path}")

def plot_trajectory_2d(positions, save_path='trajectory_2d.png'):
    """2D多视角轨迹"""
    fig, axes = plt.subplots(2, 2, figsize=(16, 12))
    
    n_points = len(positions)
    colors = plt.cm.viridis(np.linspace(0, 1, n_points))
    
    # XY平面（俯视图）
    for i in range(n_points - 1):
        axes[0, 0].plot(positions[i:i+2, 0], positions[i:i+2, 1],
                        color=colors[i], linewidth=2, alpha=0.8)
    axes[0, 0].scatter(positions[0, 0], positions[0, 1], c='green', s=150, marker='o', 
                       edgecolors='black', linewidths=2, label='Start', zorder=5)
    axes[0, 0].scatter(positions[-1, 0], positions[-1, 1], c='red', s=150, marker='s',
                       edgecolors='black', linewidths=2, label='End', zorder=5)
    axes[0, 0].set_xlabel('X (m)', fontweight='bold')
    axes[0, 0].set_ylabel('Y (m)', fontweight='bold')
    axes[0, 0].set_title('Top View (XY Plane)', fontweight='bold', fontsize=12)
    axes[0, 0].legend()
    axes[0, 0].grid(True, alpha=0.3)
    axes[0, 0].axis('equal')
    
    # XZ平面（侧视图）
    for i in range(n_points - 1):
        axes[0, 1].plot(positions[i:i+2, 0], positions[i:i+2, 2],
                        color=colors[i], linewidth=2, alpha=0.8)
    axes[0, 1].scatter(positions[0, 0], positions[0, 2], c='green', s=150, marker='o',
                       edgecolors='black', linewidths=2, zorder=5)
    axes[0, 1].scatter(positions[-1, 0], positions[-1, 2], c='red', s=150, marker='s',
                       edgecolors='black', linewidths=2, zorder=5)
    axes[0, 1].set_xlabel('X (m)', fontweight='bold')
    axes[0, 1].set_ylabel('Z (m)', fontweight='bold')
    axes[0, 1].set_title('Side View (XZ Plane)', fontweight='bold', fontsize=12)
    axes[0, 1].grid(True, alpha=0.3)
    axes[0, 1].axis('equal')
    
    # YZ平面（正视图）
    for i in range(n_points - 1):
        axes[1, 0].plot(positions[i:i+2, 1], positions[i:i+2, 2],
                        color=colors[i], linewidth=2, alpha=0.8)
    axes[1, 0].scatter(positions[0, 1], positions[0, 2], c='green', s=150, marker='o',
                       edgecolors='black', linewidths=2, zorder=5)
    axes[1, 0].scatter(positions[-1, 1], positions[-1, 2], c='red', s=150, marker='s',
                       edgecolors='black', linewidths=2, zorder=5)
    axes[1, 0].set_xlabel('Y (m)', fontweight='bold')
    axes[1, 0].set_ylabel('Z (m)', fontweight='bold')
    axes[1, 0].set_title('Front View (YZ Plane)', fontweight='bold', fontsize=12)
    axes[1, 0].grid(True, alpha=0.3)
    axes[1, 0].axis('equal')
    
    # 轨迹长度随时间变化
    cumulative_dist = np.zeros(n_points)
    for i in range(1, n_points):
        dist = np.sqrt(np.sum((positions[i] - positions[i-1])**2))
        cumulative_dist[i] = cumulative_dist[i-1] + dist
    
    axes[1, 1].plot(range(n_points), cumulative_dist, 'b-', linewidth=2)
    axes[1, 1].set_xlabel('Keyframe Index', fontweight='bold')
    axes[1, 1].set_ylabel('Cumulative Distance (m)', fontweight='bold')
    axes[1, 1].set_title('Traveled Distance vs Keyframes', fontweight='bold', fontsize=12)
    axes[1, 1].grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig(save_path, dpi=300, bbox_inches='tight')
    print(f"✅ Saved: {save_path}")

def plot_position_vs_time(timestamps, positions, save_path='position_vs_time.png'):
    """位置分量随时间变化"""
    time_s = timestamps - timestamps[0]
    
    fig, axes = plt.subplots(3, 1, figsize=(14, 10))
    
    axes[0].plot(time_s, positions[:, 0], 'r-', linewidth=2, label='X')
    axes[0].set_ylabel('X (m)', fontweight='bold', fontsize=11)
    axes[0].set_title('Position Components vs Time', fontweight='bold', fontsize=13)
    axes[0].grid(True, alpha=0.3)
    axes[0].legend(fontsize=10)
    
    axes[1].plot(time_s, positions[:, 1], 'g-', linewidth=2, label='Y')
    axes[1].set_ylabel('Y (m)', fontweight='bold', fontsize=11)
    axes[1].grid(True, alpha=0.3)
    axes[1].legend(fontsize=10)
    
    axes[2].plot(time_s, positions[:, 2], 'b-', linewidth=2, label='Z')
    axes[2].set_ylabel('Z (m)', fontweight='bold', fontsize=11)
    axes[2].set_xlabel('Time (s)', fontweight='bold', fontsize=11)
    axes[2].grid(True, alpha=0.3)
    axes[2].legend(fontsize=10)
    
    plt.tight_layout()
    plt.savefig(save_path, dpi=300, bbox_inches='tight')
    print(f"✅ Saved: {save_path}")

def plot_velocity(timestamps, positions, save_path='velocity.png'):
    """速度分析"""
    time_s = timestamps - timestamps[0]
    
    # 计算速度
    distances = np.sqrt(np.sum(np.diff(positions, axis=0)**2, axis=1))
    time_diff = np.diff(timestamps)
    velocities = distances / time_diff
    time_vel = (time_s[:-1] + time_s[1:]) / 2
    
    fig, axes = plt.subplots(2, 1, figsize=(14, 8))
    
    # 速度曲线
    axes[0].plot(time_vel, velocities, 'b-', linewidth=2, alpha=0.7)
    axes[0].axhline(y=np.mean(velocities), color='r', linestyle='--', 
                    linewidth=2, label=f'Average: {np.mean(velocities):.2f} m/s')
    axes[0].set_ylabel('Velocity (m/s)', fontweight='bold', fontsize=11)
    axes[0].set_title('Camera Velocity over Time', fontweight='bold', fontsize=13)
    axes[0].grid(True, alpha=0.3)
    axes[0].legend(fontsize=10)
    
    # 速度直方图
    axes[1].hist(velocities, bins=30, color='skyblue', edgecolor='black', alpha=0.7)
    axes[1].axvline(x=np.mean(velocities), color='r', linestyle='--', 
                    linewidth=2, label=f'Mean: {np.mean(velocities):.2f} m/s')
    axes[1].axvline(x=np.median(velocities), color='g', linestyle='--',
                    linewidth=2, label=f'Median: {np.median(velocities):.2f} m/s')
    axes[1].set_xlabel('Velocity (m/s)', fontweight='bold', fontsize=11)
    axes[1].set_ylabel('Frequency', fontweight='bold', fontsize=11)
    axes[1].set_title('Velocity Distribution', fontweight='bold', fontsize=12)
    axes[1].grid(True, alpha=0.3, axis='y')
    axes[1].legend(fontsize=10)
    
    plt.tight_layout()
    plt.savefig(save_path, dpi=300, bbox_inches='tight')
    print(f"✅ Saved: {save_path}")

if __name__ == "__main__":
    print("\n" + "=" * 60)
    print("🗺️  SIFT-SLAM Trajectory Visualization Tool")
    print("=" * 60 + "\n")
    
    # 读取轨迹文件
    trajectory_file = "KeyFrameTrajectory.txt"
    
    print(f"📂 Reading trajectory from: {trajectory_file}")
    timestamps, positions, quaternions = read_trajectory(trajectory_file)
    print(f"✅ Loaded {len(positions)} keyframes\n")
    
    # 计算统计信息
    calculate_statistics(timestamps, positions)
    print()
    
    # 生成可视化
    print("🎨 Generating visualizations...\n")
    
    plot_trajectory_3d(positions, 'trajectory_3d.png')
    plot_trajectory_2d(positions, 'trajectory_2d.png')
    plot_position_vs_time(timestamps, positions, 'position_vs_time.png')
    plot_velocity(timestamps, positions, 'velocity.png')
    
    print("\n" + "=" * 60)
    print("✨ All visualizations generated successfully!")
    print("=" * 60)
    print("\nGenerated files:")
    print("  📊 trajectory_3d.png - 3D trajectory view")
    print("  📊 trajectory_2d.png - Multi-view 2D projections")
    print("  📊 position_vs_time.png - Position components over time")
    print("  📊 velocity.png - Velocity analysis")
    print("\n💡 Tip: Use 'eog *.png' or image viewer to open all images")
    print("=" * 60 + "\n")
    
    # 显示图像
    plt.show()
