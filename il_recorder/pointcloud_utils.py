import time

import fpsample
import numpy as np
from scipy.spatial.transform import Rotation

"""

Utility functions for handling pointcloud data in il_recorder.

ros2_numpy had strange behaviour, thus we reimplement ROS PointCloud2 to numpy conversion here.

"""


def flat_pc_from_ros(msg, remove_nans=True):
    """
    Input: sensor_msgs/msg/PointCloud2
    Output: (N, 6) numpy array [x, y, z, r, g, b] (normalized 0-1)

    Args:
        msg: The ROS message
        remove_nans: If True, rows containing NaNs in x, y, or z are removed.
    """
    n_points = msg.width * msg.height

    # 1. Get raw byte data
    raw_data = np.frombuffer(msg.data, dtype=np.uint8)

    # 2. Extract Field Offsets
    field_offsets = {f.name: f.offset for f in msg.fields}
    stride = msg.point_step

    def get_column(name, dtype):
        if name not in field_offsets:
            return None
        offset = field_offsets[name]
        # Create a view of the column (strided access)
        col_bytes = np.lib.stride_tricks.as_strided(
            raw_data[offset:], shape=(n_points, 4), strides=(stride, 1)
        )
        # Copy to contiguous array and cast
        return col_bytes.copy().view(dtype).ravel()

    # 3. Extract Geometry
    x = get_column("x", np.float32)
    y = get_column("y", np.float32)
    z = get_column("z", np.float32)

    # 4. Extract Color (Packed float32)
    rgb_packed = get_column("rgb", np.float32)
    if rgb_packed is None:
        rgb_packed = get_column("rgba", np.float32)

    # 5. Filter NaNs
    # We do this before unpacking colors to save processing time
    if remove_nans:
        # standard "isfinite" checks for NaN and Inf
        mask = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)

        x = x[mask]
        y = y[mask]
        z = z[mask]
        if rgb_packed is not None:
            rgb_packed = rgb_packed[mask]

    # 6. Unpack RGB
    if rgb_packed is not None:
        # Cast float32 -> uint32 (bits) -> uint8 (bytes)
        # Layout: [Blue, Green, Red, Alpha] (Little Endian standard)
        rgb_uint8 = rgb_packed.view(np.uint32).view(np.uint8).reshape(-1, 4)

        # Normalize 0-255 -> 0.0-1.0
        b = rgb_uint8[:, 0].astype(np.float32) / 255.0
        g = rgb_uint8[:, 1].astype(np.float32) / 255.0
        r = rgb_uint8[:, 2].astype(np.float32) / 255.0

        # Stack: X, Y, Z, R, G, B
        return np.stack([x, y, z, r, g, b], axis=1)

    # Fallback if no color
    return np.stack([x, y, z], axis=1)


# NOTE: below from: https://github.com/YanjieZe/Improved-3D-Diffusion-Policy/blob/main/Improved-3D-Diffusion-Policy/diffusion_policy_3d/common/multi_realsense.py
def grid_sample_pcd(point_cloud, grid_size=0.005):
    """
    A simple grid sampling function for point clouds.

    Parameters:
    - point_cloud: A NumPy array of shape (N, 3) or (N, 6), where N is the number of points.
                   The first 3 columns represent the coordinates (x, y, z).
                   The next 3 columns (if present) can represent additional attributes like color or normals.
    - grid_size: Size of the grid for sampling.

    Returns:
    - A NumPy array of sampled points with the same shape as the input but with fewer rows.
    """
    coords = point_cloud[:, :3]  # Extract coordinates
    scaled_coords = coords / grid_size
    grid_coords = np.floor(scaled_coords).astype(int)

    # Create unique grid keys
    keys = grid_coords[:, 0] + grid_coords[:, 1] * 10000 + grid_coords[:, 2] * 100000000

    # Select unique points based on grid keys
    _, indices = np.unique(keys, return_index=True)

    # Return sampled points
    return point_cloud[indices]


def idp3_preprocess_point_cloud(points, num_points=1024, near=0.1, far=1.5):
    # NOTE: This function is inspired by the IDP3 realsense module preprocessing steps.
    # It performs clipping based on distance, grid sampling, and uniform sampling to the desired size.

    # Clip points based on depth
    mask = (points[:, 2] < far) & (points[:, 2] > near)
    points = points[mask]

    # Apply grid sampling
    points = grid_sample_pcd(points, grid_size=0.005)

    # Uniformly sample or pad to match the desired number of points
    if points.shape[0] > num_points:
        selected_idx = np.random.choice(points.shape[0], num_points, replace=False)
        points = points[selected_idx]
    else:
        num_pad = num_points - points.shape[0]
        pad_points = np.zeros((num_pad, points.shape[1]))
        points = np.vstack((points, pad_points))

    # Shuffle the points
    # NOTE: np.random.shuffle was VERY slow here, so replaced by permutation.
    indices = np.random.permutation(len(points))
    points = points[indices]

    return points


def dp3_preprocess_point_cloud(
    points,
    num_points=1024,
    workspace=None,
    rpy=[0, 0, 0],
):
    # NOTE: rewrite of DP3 preprocessing, which is essentially just workspace cropping and farthest point sampling.

    dim = points.shape[1]

    rot_matrix = Rotation.from_euler("xyz", rpy, degrees=True).as_matrix()

    # We only rotate the first 3 columns (XYZ)
    # The [:, :3] slice ensures we don't try to rotate RGB values
    points[:, :3] = points[:, :3] @ rot_matrix.T

    # 2. Workspace Cropping
    if workspace is not None:
        mask = (
            (points[:, 0] > workspace[0][0])
            & (points[:, 0] < workspace[0][1])
            & (points[:, 1] > workspace[1][0])
            & (points[:, 1] < workspace[1][1])
            & (points[:, 2] > workspace[2][0])
            & (points[:, 2] < workspace[2][1])
        )
        points = points[mask]

    if points.shape[0] == 0:
        return np.zeros((num_points, dim))

    # 3. Farthest Point Sampling
    if points.shape[0] > num_points:
        xyz_contiguous = np.ascontiguousarray(points[:, :3])
        sample_indices = fpsample.bucket_fps_kdline_sampling(
            xyz_contiguous, num_points, h=5
        )

        points = points[sample_indices]
    else:
        # Padding logic
        num_pad = num_points - points.shape[0]
        pad = np.zeros((num_pad, dim))
        points = np.vstack((points, pad))

    return points
