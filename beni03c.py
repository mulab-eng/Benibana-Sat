# 磁気トルカを用いた姿勢制御
# （角速度ベクトルが測定できる場合の回転の抑制制御）

# 古いパソコンの場合，つぎの2行が必要
from panda3d.core import loadPrcFileData
loadPrcFileData("", "load-display pandadx9")

from direct.showbase.ShowBase import ShowBase
from panda3d.core import LineSegs, NodePath, WindowProperties, Quat, Vec3, Mat3, TextNode
from direct.gui.OnscreenText import OnscreenText
import math
from panda3d.core import Geom, GeomNode, GeomVertexData, GeomVertexFormat, GeomVertexWriter, GeomTriangles
import numpy as np



def angle_between(v1, v2):
    v1_u = v1 / np.linalg.norm(v1)
    v2_u = v2 / np.linalg.norm(v2)
    
    dot = np.dot(v1_u, v2_u)
    cross_norm = np.linalg.norm(np.cross(v1_u, v2_u))
    
    # atan2(sin_component, cos_component)
    angle = np.arctan2(cross_norm, dot)
    if math.fabs(angle) < 0.01:
        angle=0.01
    return angle # ラジアンを度に変換する場合

def rotation_matrix_from_axis_angle(axis, angle):
    axis = axis / np.linalg.norm(axis)
    x, y, z = axis
    c = math.cos(angle)
    s = math.sin(angle)
    C = 1 - c
    R = np.array([
        [c + x*x*C, x*y*C - z*s, x*z*C + y*s],
        [y*x*C + z*s, c + y*y*C, y*z*C - x*s],
        [z*x*C - y*s, z*y*C + x*s, c + z*z*C]
    ])
    return R



#window
class Window:
    def __init__(self, title):
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
        self.cube = Cube(size=0.1, name="cube1")
        self.cube.reparentTo(render)  # cubeはrenderの子ノードとして追加

        self.arrow_magnet= Arrow(start=(0.3, -0.3, 0), end=(-0.3, 0.3, 0), color=(0, 1, 1, 1), name="axis_mag")
        self.arrow_magnet.reparentTo(render)     

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
        self.paused = False
        self.accept('s', self.toggle_pause)
        # on-screen pause indicator (hidden initially)
        self.pause_text = OnscreenText(text="Paused (press 's' to resume)", pos=(0, 0.9), scale=0.06, fg=(1, 1, 1, 1), align=TextNode.ACenter)
        self.pause_text.hide()
        # on-screen torque display (top-left)
        self.torque_text = OnscreenText(text="Trq: [0.0000, 0.0000, 0.0000]", pos=(-1.15, 0.9), scale=0.05, fg=(1, 1, 1, 1), align=TextNode.ALeft)
        # on-screen error display (top-left)
        self.error_text = OnscreenText(text="Error: 0.0000 rad", pos=(-1.15, 0.8), scale=0.05, fg=(1, 1, 1, 1), align=TextNode.ALeft)
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
        self.omega = np.array([0.0, 0.0, 0.3])   # 角速度ベクトル
        self.tau=np.array([0.0, 0.0, 0.0]) # トルクベクトル
        self.alpha = np.array([0.0, 0.0, 0.0]) # 角加速度ベクトル

        self.disp_omega = Vec3(0, 0, 0) #描画用に大きさを調整した角速度データ
        self.disp_torque = Vec3(0, 0, 0) #描画用に大きさを調整したトルクデータ

        self.magnetic=np.array([0.0, 0.0, 0.0]) # 地磁気（機体座標系）

        self.toruca_x=np.array([0.0, 0.0, 0.0]) # 磁気トルカ(機体座標系）
        self.toruca_y=np.array([0.0, 0.3, 0.0]) # 磁気トルカ(機体座標系）
        self.toruca_z=np.array([0.0, 0.0, 0.0]) # 磁気トルカ(機体座標系）
        self.flag=1
        self.gain=0.3
               
    def update(self, task):
        dt = globalClock.getDt()
        if self.paused:
            return task.cont
        self._accumulator += dt
        while self._accumulator >= self.deltaT:
            self._simulate_step(self.deltaT)
            self._accumulator -= self.deltaT
        return task.cont
    
    def bdot_controller(self, omega, B, K_gain=0.3):
        B_dot = np.cross(omega, B)
        m = K_gain * B_dot
        return m

    def toggle_pause(self):
        self.paused = not self.paused
        if self.paused:
            print("Simulation paused")
        else:
            # reset accumulator to avoid large catch-up on unpause
            self._accumulator = 0.0
            print("Simulation resumed")

    def _simulate_step(self, dt):
        self._sim_time += dt
        t = self._sim_time

        # 地磁気を固定し，磁気トルカと地磁気の外積をトルクとする．
        self.magnetic = self.R.T @ np.array([-0.1, 0.1, 0.0]) # 地磁気ベクトルを機体座標系に変換
        torque_x = np.cross(self.toruca_x, self.magnetic) # 磁気トルクの計算 tau = m x B
        magtoruca_y = self.bdot_controller(self.omega, self.magnetic, K_gain=10) # B-dot制御則で磁気トルカを計算
        torque_y = np.cross(magtoruca_y, self.magnetic) # 磁気トルクの計算 tau = m x B
        torque_z = np.cross(self.toruca_z, self.magnetic) # 磁気トルクの計算 tau = m x B
        self.torque=torque_x+torque_y+torque_z

        Jw=self.J @ self.omega                        # (3,1)   w
        Jw_cross = np.cross(Jw, self.omega)     # (3,1) Jw x w
        self.alpha = self.J_inv @ (Jw_cross+self.torque)            # J^-1 * (Jw x w + tau) -> 角加速度ベクトル
        self.omega = self.omega + self.alpha * dt          # 角速度ベクトル w の更新
        # print(f"toruca_y: {self.toruca_y[0]:.6f}, {self.toruca_y[1]:.6f}, {self.toruca_y[2]:.6f}")
        # print(f"magnetic: {self.magnetic[0]:.6f}, {self.magnetic[1]:.6f}, {self.magnetic[2]:.6f}")
        # print(f"torque: {self.torque[0]:.6f}, {self.torque[1]:.6f}, {self.torque[2]:.6f}")
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

        # # 角速度ベクトル描画  機体座標系の角速度を慣性座標系に変換して描画するため、Rをかける
        # disp_omega = 0.7 * self.R @ self.omega #描画用の角速度データ
        # omega_mag = np.linalg.norm(disp_omega)
        # if omega_mag < 1e-8:
        #     self.disp_omega = np.array([0, 0, 0.01])  # ほぼ0のベクトル
        # if hasattr(self.scene, 'arrow_omega') and not self.scene.arrow_omega.isEmpty():
        #     try:
        #         self.scene.arrow_omega.removeNode()
        #     except Exception:
        #         pass
        # self.scene.arrow_omega = vector_arrow(start=(0, 0, 0), vector=disp_omega, 
        #             color=(1, 1, 0, 1), name="dispomega")
        # self.scene.arrow_omega.reparentTo(render) # 慣性座標系に合わせて回転するので、矢印もrenderの子ノードとして追加


        # 磁気モーメントベクトル描画  機体座標系で座標を指定
        disp_toruca_y = 0.4 * self.toruca_y #描画用の角速度データ
        toruca_y_mag = np.linalg.norm(disp_toruca_y)
        if toruca_y_mag < 1e-2:
            try:
               self.scene.arrow_toruca_y.removeNode()
            except Exception:
                pass
        else:
            if hasattr(self.scene, 'arrow_toruca_y') and not self.scene.arrow_toruca_y.isEmpty():
                try:
                    self.scene.arrow_toruca_y.removeNode()
                except Exception:
                    pass
            self.scene.arrow_toruca_y = vector_arrow(start=(0.1, -0.05, 0), vector=disp_toruca_y,  color=(1, 0, 1, 1), name="disptoruca_y")
            self.scene.arrow_toruca_y.reparentTo(self.scene.cube) # 慣性座標系に合わせて回転するのでcubeの子ノードとして追加



        # トルクベクトル描画
        disp_torque =5.0*self.R @ self.torque #描画用のトルクデータ
        torque_mag = np.linalg.norm(disp_torque)
        if torque_mag < 1e-8:
            disp_torque = np.array([0, 0, 0.01])  # 正規化してスケール調整
        if hasattr(self.scene, 'arrow_torque') and not self.scene.arrow_torque.isEmpty():
            try:
                self.scene.arrow_torque.removeNode()
            except Exception:
                pass
        self.scene.arrow_torque = vector_arrow(start=(0, 0, 0), vector=disp_torque, 
                                color=(0.7, 0.6, 0, 1), name="disptorque")
        self.scene.arrow_torque.reparentTo(render) # 慣性座標系に合わせて回転するので、矢印もrenderの子ノードとして追加


        Rt = self.R.T
        mat = Mat3(
            float(Rt[0, 0]), float(Rt[0, 1]), float(Rt[0, 2]),
            float(Rt[1, 0]), float(Rt[1, 1]), float(Rt[1, 2]),
            float(Rt[2, 0]), float(Rt[2, 1]), float(Rt[2, 2]),
        )
        self.scene.cube.setMat(self.render, mat)

        try:
            self.torque_text.setText(f"Trq: [{self.torque[0]:.4f}, {self.torque[1]:.4f}, {self.torque[2]:.4f}]")
        except Exception:
            pass
        try:
            self.error_text.setText(f"error: {error_angle:.4f} rad ")
        except Exception:
            pass

if __name__ == "__main__":
    app = CubeApp()
    app.run()
