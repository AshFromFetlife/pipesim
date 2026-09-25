"""Right-handed Z-up geometry. Authoring uses mm and extrinsic XYZ degrees."""
from __future__ import annotations
import numpy as np
from scipy.spatial.transform import Rotation

def vec(value):
    result = np.asarray(value, dtype=float)
    if result.shape != (3,) or not np.isfinite(result).all():
        raise ValueError(f"Expected three finite coordinates, got {value!r}")
    return result

def unit(value):
    value = vec(value)
    norm = np.linalg.norm(value)
    if norm < 1e-12:
        raise ValueError("An axis must have nonzero length")
    return value / norm

def rotation(euler=(0, 0, 0)):
    return Rotation.from_euler("xyz", vec(euler), degrees=True).as_matrix()

def transform(pose=None):
    pose = pose or {}
    result = np.eye(4)
    result[:3, :3] = rotation(pose.get("rotation_deg", [0, 0, 0]))
    result[:3, 3] = vec(pose.get("position_mm", [0, 0, 0]))
    return result

def pose_of(matrix):
    import warnings
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", UserWarning)
        angles = Rotation.from_matrix(matrix[:3, :3]).as_euler("xyz", degrees=True)
    return {"position_mm": np.round(matrix[:3, 3], 8).tolist(), "rotation_deg": np.round(angles, 8).tolist()}

def point(matrix, value):
    return matrix[:3, :3] @ vec(value) + matrix[:3, 3]

def axis_frame(axis):
    z = unit(axis)
    ref = np.array([0., 1., 0.]) if abs(z[1]) < .95 else np.array([1., 0., 0.])
    x = unit(np.cross(ref, z))
    return np.column_stack((x, np.cross(z, x), z))

def align_axis(source, target):
    source, target = unit(source), unit(target)
    cross = np.cross(source, target)
    dot = float(np.clip(source @ target, -1, 1))
    if np.linalg.norm(cross) < 1e-10:
        return np.eye(3) if dot > 0 else Rotation.from_rotvec(axis_frame(source)[:, 0]*np.pi).as_matrix()
    return Rotation.from_rotvec(unit(cross)*np.arccos(dot)).as_matrix()

def skew(v):
    x,y,z = v
    return np.array([[0,-z,y],[z,0,-x],[-y,x,0]], dtype=float)

def segment_distance(p1, q1, p2, q2):
    """Closest points on finite segments, including parallel and zero-length cases."""
    p1,q1,p2,q2 = map(vec, (p1,q1,p2,q2))
    d1,d2,r = q1-p1,q2-p2,p1-p2
    a,e = d1@d1,d2@d2
    f = d2@r
    if a < 1e-12 and e < 1e-12:
        return float(np.linalg.norm(r)), p1, p2
    if a < 1e-12:
        s,t = 0., np.clip(f/e, 0, 1)
    else:
        c = d1@r
        if e < 1e-12:
            t,s = 0., np.clip(-c/a, 0, 1)
        else:
            b = d1@d2
            denom = a*e-b*b
            s = np.clip((b*f-c*e)/denom, 0, 1) if abs(denom) > 1e-12 else 0.
            t = (b*s+f)/e
            if t < 0:
                t,s = 0.,np.clip(-c/a,0,1)
            elif t > 1:
                t,s = 1.,np.clip((b-c)/a,0,1)
    first,second = p1+d1*s,p2+d2*t
    return float(np.linalg.norm(first-second)),first,second

class UnionFind:
    def __init__(self, values):
        self.parent = {v:v for v in values}
    def find(self, value):
        if self.parent[value] != value:
            self.parent[value] = self.find(self.parent[value])
        return self.parent[value]
    def union(self, a, b):
        self.parent[self.find(b)] = self.find(a)
    def groups(self):
        result = {}
        for item in self.parent:
            result.setdefault(self.find(item), []).append(item)
        return list(result.values())
