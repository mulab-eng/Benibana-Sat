from scipy.spatial.transform import Rotation as R
import numpy as np


def multiply_quaternions(q2, q1):
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2

    return np.array([
        w2 * x1 + x2 * w1 + y2 * z1 - z2 * y1,
        w2 * y1 - x2 * z1 + y2 * w1 + z2 * x1,
        w2 * z1 + x2 * y1 - y2 * x1 + z2 * w1,
        w2 * w1 - x2 * x1 - y2 * y1 - z2 * z1,
    ])


def main():
    # z軸まわりに90度回転するクォータニオンを作る
    rotation = R.from_euler("z", 30, degrees=True)
    print(rotation)

    # SciPyのクォータニオン順序は [x, y, z, w]
    quaternion = rotation.as_quat()
    print("quaternion [x, y, z, w] =", quaternion)

    # 回転行列へ変換する
    rotation_matrix = rotation.as_matrix()
    print("rotation matrix =")
    print(rotation_matrix)

    # ベクトルを回転させる
    vector = np.array([1.0, 0.0, 0.0])
    rotated_vector = rotation.apply(vector)
    print("rotated vector =", rotated_vector)

    # 2つの回転を合成する
    rotation_x = R.from_euler("x", 30, degrees=True)
    rotation_y = R.from_euler("y", 45, degrees=True)
    combined_rotation = rotation_y * rotation_x
    quaternion_x = rotation_x.as_quat()
    quaternion_y = rotation_y.as_quat()
    combined_quaternion = multiply_quaternions(quaternion_y, quaternion_x)
    combined_quaternion_norm = np.linalg.norm(combined_quaternion)

    print("quaternion_x [x, y, z, w] =", quaternion_x)
    print("quaternion_y [x, y, z, w] =", quaternion_y)
    print("combined quaternion by multiplication [x, y, z, w] =", combined_quaternion)
    print("norm of combined quaternion =", combined_quaternion_norm)
    print("combined quaternion [x, y, z, w] =", combined_rotation.as_quat())
    print("same rotation =", np.allclose(combined_quaternion, combined_rotation.as_quat()))


if __name__ == "__main__":
    main()