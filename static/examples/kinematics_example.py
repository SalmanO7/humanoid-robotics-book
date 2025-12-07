# Sample Python code for kinematics calculations
import numpy as np

def dh_transform(a, alpha, d, theta):
    """Calculate Denavit-Hartenberg transformation matrix"""
    sa = np.sin(alpha)
    ca = np.cos(alpha)
    st = np.sin(theta)
    ct = np.cos(theta)

    transform = np.array([
        [ct, -st*ca, st*sa, a*ct],
        [st, ct*ca, -ct*sa, a*st],
        [0, sa, ca, d],
        [0, 0, 0, 1]
    ])
    return transform

# Example usage
if __name__ == "__main__":
    # Simple 2-DOF planar manipulator
    T1 = dh_transform(a=1.0, alpha=0, d=0, theta=np.pi/4)
    T2 = dh_transform(a=1.0, alpha=0, d=0, theta=np.pi/6)

    T_total = T1 @ T2
    print("End-effector position:", T_total[:3, 3])