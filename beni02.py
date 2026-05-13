# 3軸にトルクを与えて回転させる

# 古いパソコンの場合，つぎの2行が必要
from panda3d.core import loadPrcFileData
loadPrcFileData("", "load-display pandadx9")

from direct.showbase.ShowBase import ShowBase
from direct.showbase import ShowBaseGlobal
from panda3d.core import LineSegs, NodePath, WindowProperties, Quat, Vec3, Mat3
import math
from panda3d.core import Geom, GeomNode, GeomVertexData, GeomVertexFormat, GeomVertexWriter, GeomTriangles
import numpy as np

#window
class Window:
    def __init__(self, title):
        if ShowBaseGlobal.base is None:
            raise RuntimeError("ShowBase is not initialized. Create CubeApp() before Window().")
        base = ShowBaseGlobal.base
        self.props = WindowProperties()
        self.props.setTitle(title)
        self.props.setSize(800, 600)
        base.win.requestProperties(self.props)
        base.setBackgroundColor(0.1, 0.1, 0.3) 


class Cube(NodePath):
    def __init__(self, size=0.1, name="cube"):
        super().__init__(name)

        ls = LineSegs()
        ls.setThickness(2.0)
        ls.setColor(0.5, 0.5, 0.5, 1.0)

        s = float(size)
        verts = [
            (-s, -s, -s),
            (s, -s, -s),
            (s, s, -s),
            (-s, s, -s),
            (-s, -s, s),
            (s, -s, s),
            (s, s, s),
            (-s, s, s),
        ]

        edges = [
            (0, 1), (1, 2), (2, 3), (3, 0),
            (4, 5), (5, 6), (6, 7), (7, 4),
            (0, 4), (1, 5), (2, 6), (3, 7),
        ]

        for a, b in edges:
            ls.moveTo(*verts[a])
            ls.drawTo(*verts[b])

        node = ls.create()
        geom_np = NodePath(node)
        geom_np.reparentTo(self)


class Arrow(NodePath):
    def __init__(self, start=(0, 0, 0), end=(0, 0, 1), color=(1, 0, 0, 1), name="arrow"):
        super().__init__(name)
        sx, sy, sz = start
        ex, ey, ez = end
        dx, dy, dz = ex - sx, ey - sy, ez - sz
        length = math.sqrt(dx * dx + dy * dy + dz * dz) or 1.0

        head_len = 0.03
        shaft_len = max(0.001, length - head_len)
        shaft_radius = 0.002
        head_radius = 0.008

        # build cylinder for shaft along +Y from 0..shaft_len
        cyl_node = self._make_cylinder(shaft_radius, shaft_len, color=color, slices=24)
        cyl_np = NodePath(cyl_node)
        # place cylinder so its base is at the node origin (y=0..shaft_len)
        cyl_np.setPos(0, 0, 0)
        cyl_np.reparentTo(self)
        cyl_np.setTwoSided(True)

        # build cone for head: base at y=0, apex at y=head_len (cone points +Y)
        cone_node = self._make_cone(head_radius, head_len, color=color, slices=24)
        cone_np = NodePath(cone_node)
        cone_np.setPos(0, shaft_len, 0)
        cone_np.reparentTo(self)
        cone_np.setTwoSided(True)

        # position and orient the arrow: parent will set it into the scene
        self.setPos(sx, sy, sz)
        # point +Y to end point
        self.lookAt(ex, ey, ez)

    def _make_cylinder(self, radius, height, color=(1, 0, 0, 1), slices=16):
        fmt = GeomVertexFormat.getV3c4()
        vdata = GeomVertexData('cyl', fmt, Geom.UHStatic)
        vwriter = GeomVertexWriter(vdata, 'vertex')
        cwriter = GeomVertexWriter(vdata, 'color')

        tris = GeomTriangles(Geom.UHStatic)

        # side vertices (cross-section in XZ plane, axis along +Y)
        idx = []
        for y in (0.0, height):
            for i in range(slices):
                theta = 2.0 * math.pi * i / slices
                x = math.cos(theta) * radius
                z = math.sin(theta) * radius
                vwriter.addData3(x, y, z)
                cwriter.addData4f(*color)
                idx.append(vwriter.getWriteRow() - 1)

        # build quads as two tris
        for i in range(slices):
            i0 = i
            i1 = (i + 1) % slices
            i2 = i + slices
            i3 = ((i + 1) % slices) + slices
            tris.addVertices(i0, i1, i2)
            tris.addVertices(i2, i1, i3)

        geom = Geom(vdata)
        geom.addPrimitive(tris)
        node = GeomNode('cylinder')
        node.addGeom(geom)
        return node

    def _make_cone(self, radius, height, color=(1, 0, 0, 1), slices=16):
        fmt = GeomVertexFormat.getV3c4()
        vdata = GeomVertexData('cone', fmt, Geom.UHStatic)
        vwriter = GeomVertexWriter(vdata, 'vertex')
        cwriter = GeomVertexWriter(vdata, 'color')

        tris = GeomTriangles(Geom.UHStatic)

        # apex (along +Y axis)
        vwriter.addData3(0, height, 0)
        cwriter.addData4f(*color)
        apex_idx = 0

        # base circle vertices
        base_indices = []
        for i in range(slices):
            theta = 2.0 * math.pi * i / slices
            x = math.cos(theta) * radius
            z = math.sin(theta) * radius
            vwriter.addData3(x, 0.0, z)
            cwriter.addData4f(*color)
            base_indices.append(vwriter.getWriteRow() - 1)

        # triangles from apex to base edges
        for i in range(slices):
            i0 = apex_idx
            i1 = base_indices[i]
            i2 = base_indices[(i + 1) % slices]
            tris.addVertices(i0, i1, i2)

        geom = Geom(vdata)
        geom.addPrimitive(tris)
        node = GeomNode('cone')
        node.addGeom(geom)
        return node


def vector_arrow(start, vector, color=(1, 0, 0, 1), name="arrow"):
    sx, sy, sz = start
    dx, dy, dz = vector
    ex, ey, ez = sx + dx, sy + dy, sz + dz
    return Arrow(start=(sx, sy, sz), end=(ex, ey, ez), color=color, name=name)


class Scene:
    def __init__(self, render):
        # highlight the face orthogonal to +y (face index 3) in green
        # and the face orthogonal to +z (face index 1) in blue
        # highlight the face orthogonal to +x (face index 5) in red
        self.cube = Cube(size=0.1, name="cube1")
        self.cube.reparentTo(render)  # cubeはrenderの子ノードとして追加

        self.arrow_x = Arrow(start=(0, 0, 0), end=(0.35, 0, 0), color=(1, 1, 1, 1), name="axis_x0")
        self.arrow_x.reparentTo(render)
        self.arrow_y = Arrow(start=(0, 0, 0), end=(0, 0.35, 0), color=(1, 1, 1, 1), name="axis_y0")
        self.arrow_y.reparentTo(render)
        self.arrow_z = Arrow(start=(0, 0, 0), end=(0, 0, 0.35), color=(1, 1, 1, 1), name="axis_z0")
        self.arrow_z.reparentTo(render) # 慣性座標系はrenderに合わせて回転するので、矢印もrenderの子ノードとして追加

        self.arrow_x = vector_arrow(start=(0, 0, 0), vector=(0.2, 0, 0), color=(1, 0, 0, 1), name="axis_x")
        self.arrow_x.reparentTo(self.cube)
        self.arrow_y = vector_arrow(start=(0, 0, 0), vector=(0, 0.2, 0), color=(0, 1, 0, 1), name="axis_y")
        self.arrow_y.reparentTo(self.cube)
        self.arrow_z = vector_arrow(start=(0, 0, 0), vector=(0, 0, 0.2), color=(0, 0, 1, 1), name="axis_z")
        self.arrow_z.reparentTo(self.cube)  # 機体座標系はcubeに合わせて回転するので、矢印もcubeの子ノードとして追加



class CubeApp(ShowBase):
    def __init__(self):
        super().__init__()
        self.disableMouse()
        self.scene = Scene(self.render)
        props = WindowProperties()
        props.setTitle("Satellite")
        props.setSize(900, 700)
        self.win.requestProperties(props)
        self.setBackgroundColor(0.1, 0.1, 0.3, 1)
        self.cam.setPos(0.5, 1.5, 0.5)
        self.cam.lookAt(0, 0, 0)
        self.deltaT = 0.01
        self._sim_time = 0.0
        self._accumulator = 0.0
        self.taskMgr.add(self.update, "update")
        self.mass = 1.0
        self.side = 2.0
        # self.inertia = (1.0 / 6.0) * self.mass * (0.5 ** 2)
        self.inertia = 1.5
        self.J = np.array([
            [self.inertia, 0.0, 0.0],
            [0.0, self.inertia, 0.0],
            [0.0, 0.0, self.inertia],
        ])
        self.J_inv = np.array([
            [1.0 /self.inertia, 0.0, 0.0],
            [0.0, 1.0 /self.inertia, 0.0],
            [0.0, 0.0, 1.0/self.inertia],
        ])
        self.R = np.array([
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
        ], dtype=float)
        self.omega = np.array([0.0, 0.0, 0.0])   # 角速度ベクトル
        self.tau=np.array([0.0, 0.0, 0.0]) # トルクベクトル
        self.alpha = np.array([0.0, 0.0, 0.0]) # 角加速度ベクトル

        self.disp_omega = Vec3(0, 0, 0) #描画用に大きさを調整した角速度データ
        self.disp_torque = Vec3(0, 0, 0) #描画用に大きさを調整したトルクデータ
               
    def update(self, task):
        dt = globalClock.getDt()
        self._accumulator += dt
        while self._accumulator >= self.deltaT:
            self._simulate_step(self.deltaT)
            self._accumulator -= self.deltaT
        return task.cont


    def _simulate_step(self, dt):
        self._sim_time += dt
        t = self._sim_time

        # 物理計算
        # 時間に応じたトルクの切り替え
        tx=np.array([1.0, 0.0, 0.0]) # x軸周りのトルク
        ty=np.array([0.0, 1.0, 0.0]) # y軸周りのトルク
        tz=np.array([0.0, 0.0, 1.0]) # z軸周りのトルク
        if t < 3:
            self.torque = 1*tx+0*ty+0*tz
        elif t>3 and t<6:
            self.torque = -1*tx+0*ty+0*tz
        elif t>6 and t<9:
            self.torque = 0*tx+1*ty+0*tz
        elif t>9 and t<12:
            self.torque = 0*tx-1*ty+0*tz
        elif t>12 and t<14:
            self.torque = 0*tx+0*ty+1*tz
        elif t>14 and t<18:
            self.torque = 0*tx+0*ty-1*tz
        elif t>18 and t<21:
             self.torque = 1*tx+1*ty+0*tz
        elif t>21 and t<24:
             self.torque = -1*tx-1*ty+0*tz
        #     torque = np.array([0, 1, 0])
        # elif t>20 and t<25:
        #     torque = np.array([0, 0, 0])
        # elif t>25 and t<30:
        #     torque = np.array([0, -1, 0])
        # elif t>30 and t<35:
        #     torque = np.array([0, 0, 1])
        else:
            self.torque = np.array([0, 0, 0])

        Jw=self.J @ self.omega                        # (3,1)   w
        Jw_cross = np.cross(Jw, self.omega)     # (3,1) Jw x w
        self.alpha = self.J_inv @ (Jw_cross+self.torque)            # J^-1 * (Jw x w + tau) -> 角加速度ベクトル
        self.omega = self.omega + self.alpha * dt          # 角速度ベクトル w の更新
        # print(f"omega: {self.omega[0]:.6f}, {self.omega[1]:.6f}, {self.omega[2]:.6f}")
        Omega_cross = np.array([[0, -self.omega[2], self.omega[1]],
                            [self.omega[2], 0, -self.omega[0]],
                            [-self.omega[1], self.omega[0], 0]]) # (3,3) Omegaの反対称行列     
        Rdot = self.R @ Omega_cross                        # (3,3) @ (3,3) -> (3,3) Rの時間微分
        self.R += Rdot * dt                               # 回転行列の更新 R
        # Orthonormalize R to prevent numerical drift これがないと、回転行列の数値誤差が蓄積して，表示がおかしくなる
        try:
            u, s, vh = np.linalg.svd(self.R)
            self.R = (u @ vh)
        except Exception:
            pass
        # axis = [1, 0, 0]
        # theta = np.pi / 6
        # # 回転オブジェクトの作成（回転ベクトル = 単位ベクトル * 角度）
        # rotation = Rsci.from_rotvec(np.array(axis) * theta)
        # self.R = rotation.as_matrix()

        # 角速度ベクトル描画  機体座標系の角速度を慣性座標系に変換して描画するため、Rをかける
        self.disp_omega = 0.3 * self.R @ self.omega #描画用の角速度データ
        omega_mag = np.linalg.norm(self.disp_omega)
        if omega_mag < 1e-8:
            self.disp_omega = np.array([0, 0, 0.01])  # 正規化してスケール調整
        if hasattr(self.scene, 'arrow_omega') and not self.scene.arrow_omega.isEmpty():
            try:
                self.scene.arrow_omega.removeNode()
            except Exception:
                pass
        self.scene.arrow_omega = vector_arrow(start=(0, 0, 0), vector=self.disp_omega, 
                    color=(1, 1, 0, 1), name="dispomega")
        self.scene.arrow_omega.reparentTo(render) # 慣性座標系に合わせて回転するので、矢印もrenderの子ノードとして追加

        # # トルクベクトル描画
        # self.disp_torque =self.R @ self.torque #描画用のトルクデータ
        # torque_mag = np.linalg.norm(self.disp_torque)
        # if torque_mag < 1e-8:
        #     self.disp_torque = np.array([0, 0, 0.01])  # 正規化してスケール調整
        # if hasattr(self.scene, 'arrow_torque') and not self.scene.arrow_torque.isEmpty():
        #     try:
        #         self.scene.arrow_torque.removeNode()
        #     except Exception:
        #         pass
        # self.scene.arrow_torque = vector_arrow(start=(0, 0, 0), vector=self.disp_torque, 
        #                         color=(1, 0, 1, 1), name="disptorque")
        # self.scene.arrow_torque.reparentTo(render) # 慣性座標系に合わせて回転するので、矢印もrenderの子ノードとして追加


        Rt = self.R.T
        mat = Mat3(
            float(Rt[0, 0]), float(Rt[0, 1]), float(Rt[0, 2]),
            float(Rt[1, 0]), float(Rt[1, 1]), float(Rt[1, 2]),
            float(Rt[2, 0]), float(Rt[2, 1]), float(Rt[2, 2]),
        )
        self.scene.cube.setMat(self.render, mat)

        # self.scene.text1.setText(f"t: {t}")
        # self.scene.text2.setText(f"Trq: {self.torque}")
        # self.scene.text3.setText(f"J: {self.J}")
        # self.scene.text4.setText(f"J_inv: {self.J_inv}")
        # self.scene.text5.setText(f"omega: {self.omega}")
        # self.scene.text6.setText(f"omegadot: {self.omegadot}")


if __name__ == "__main__":
    app = CubeApp()
    app.run()
