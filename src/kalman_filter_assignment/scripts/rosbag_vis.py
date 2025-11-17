import os
import math
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# Simple visualization of exported bag CSVs


def plotter(cmd_path, fgps_path, ke_path, odom_path, odom1_path):
    def load_csv(path):
        if not os.path.exists(path):
            print(f"[warn] File not found: {path}")
            return None
        df = pd.read_csv(path)
        if 'time' in df.columns:
            df = df.sort_values('time')
            df['t_rel'] = df['time'] - df['time'].iloc[0]
        return df

    def quat_to_yaw(qx, qy, qz, qw):
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        return math.atan2(siny_cosp, cosy_cosp)

    def extract_xy(df):
        for px, py in [
            ('pose_pose_position_x', 'pose_pose_position_y'),
            ('pose_position_x', 'pose_position_y'),
            ('position_x', 'position_y'),
            ('x', 'y'),
        ]:
            if df is not None and px in df.columns and py in df.columns:
                return df[px].to_numpy(), df[py].to_numpy()
        return None, None

    def extract_yaw(df):
        if df is None:
            return None
        qcols = [
            'pose_pose_orientation_x',
            'pose_pose_orientation_y',
            'pose_pose_orientation_z',
            'pose_pose_orientation_w',
        ]
        if all(c in df.columns for c in qcols):
            qx = df[qcols[0]].to_numpy()
            qy = df[qcols[1]].to_numpy()
            qz = df[qcols[2]].to_numpy()
            qw = df[qcols[3]].to_numpy()
            return np.array([quat_to_yaw(a, b, c, d) for a, b, c, d in zip(qx, qy, qz, qw)])
        for c in ['yaw', 'pose_yaw', 'theta']:
            if c in df.columns:
                return df[c].to_numpy()
        return None

    def plot_xy(ax, df, label, style='-'):
        x, y = extract_xy(df)
        if x is not None and y is not None:
            ax.plot(x, y, style, label=label, linewidth=1.5)

    def plot_xy_series(ax_x, ax_y, df, label, color):
        x, y = extract_xy(df)
        if x is not None and y is not None and df is not None and 't_rel' in df.columns:
            ax_x.plot(df['t_rel'], x, color=color, label=label)
            ax_y.plot(df['t_rel'], y, color=color, label=label)

    def plot_yaw_series(ax, df, label, color):
        yaw = extract_yaw(df)
        if yaw is not None and df is not None and 't_rel' in df.columns:
            ax.plot(df['t_rel'], yaw, color=color, label=label)

    def maybe_legend(ax):
        handles, labels = ax.get_legend_handles_labels()
        if handles:
            ax.legend(loc='best')

    df_gps = load_csv(fgps_path)
    df_ke = load_csv(ke_path)
    df_odom = load_csv(odom_path)
    df_odom1 = load_csv(odom1_path)

    fig = plt.figure(figsize=(12, 12))
    gs = fig.add_gridspec(2, 1, height_ratios=[1.1, 1.8], hspace=0.35)

    ax_xy = fig.add_subplot(gs[0])
    plot_xy(ax_xy, df_odom, 'odom_gt', style='k--')
    plot_xy(ax_xy, df_odom1, 'odom1 (noisy)', style='r:')
    plot_xy(ax_xy, df_gps, 'fake_gps', style='g^')
    plot_xy(ax_xy, df_ke, 'kalman_estimate', style='b-')
    ax_xy.set_xlabel('x [m]')
    ax_xy.set_ylabel('y [m]')
    ax_xy.set_title('Trajectories')
    ax_xy.axis('equal')
    ax_xy.grid(True)
    maybe_legend(ax_xy)

    gs_states = gs[1].subgridspec(3, 1, hspace=0.05)
    ax_x = fig.add_subplot(gs_states[0])
    ax_y = fig.add_subplot(gs_states[1], sharex=ax_x)
    ax_yaw = fig.add_subplot(gs_states[2], sharex=ax_x)

    plot_xy_series(ax_x, ax_y, df_odom, 'odom', 'k')
    plot_xy_series(ax_x, ax_y, df_odom1, 'odom1', 'r')
    plot_xy_series(ax_x, ax_y, df_gps, 'fake_gps', 'g')
    plot_xy_series(ax_x, ax_y, df_ke, 'kalman_estimate', 'b')
    plot_yaw_series(ax_yaw, df_odom1, 'odom1', 'r')
    plot_yaw_series(ax_yaw, df_ke, 'kalman_estimate', 'b')

    ax_x.set_ylabel('x [m]')
    ax_y.set_ylabel('y [m]')
    ax_yaw.set_ylabel('yaw [rad]')
    ax_yaw.set_xlabel('time [s]')
    for axi in (ax_x, ax_y, ax_yaw):
        axi.grid(True)
    maybe_legend(ax_x)
    maybe_legend(ax_y)
    maybe_legend(ax_yaw)
    plt.setp(ax_x.get_xticklabels(), visible=False)
    plt.setp(ax_y.get_xticklabels(), visible=False)

    fig.suptitle('ROS Bag Visualization', fontsize=14)
    fig.tight_layout(rect=(0, 0, 1, 0.98))
    plt.show()


if __name__ == "__main__":
    v = "v3/"
    plotter(
        cmd_path=v + "cmd_vel.csv",
        fgps_path=v + "fake_gps.csv",
        ke_path=v + "kalman_estimate.csv",
        odom_path=v + "odom.csv",
        odom1_path=v + "odom1.csv",
    )