import mujoco
import numpy as np
from mujoco.glfw import glfw
import os
from mujoco.viewer import launch

# URDFファイルのパスを指定
urdf_path = "/home/hori/mujoco/urdf/roborecipe4_go2.urdf"

# URDFファイルからMuJoCoモデルを作成
model = mujoco.MjModel.from_xml_path(urdf_path)
data = mujoco.MjData(model)

"""
# ビジュアライザの設定
def render_model(model, data):
    window = glfw.create_window(800, 600, "URDF Viewer", None, None)
    glfw.make_context_current(window)
    glfw.swap_interval(1)
    
    # ビューワーの初期化
    opt = mujoco.MjvOption()  # オプションオブジェクトを作成
    
    cam = mujoco.MjvCamera()    
    cam.distance = 1.0  # カメラの距離を調整
    cam.azimuth = 90    # カメラの方位角を調整
    cam.elevation = -20 # カメラの仰角を調整
    
    scene = mujoco.MjvScene(model, maxgeom=10000)
    context = mujoco.MjrContext(model, mujoco.mjtFontScale.mjFONTSCALE_150)
    
    # perturbオブジェクトの作成（必要なければNoneでも可）
    pert = mujoco.MjvPerturb()
    
    # メインループ
    while not glfw.window_should_close(window):
        viewport = mujoco.MjrRect(0, 0, 1200, 900)
        
        # データの更新
        mujoco.mj_step(model, data)
        
        # シーンの更新 - pert引数を追加
        mujoco.mjv_updateScene(model, data, opt, pert, cam, mujoco.mjtCatBit.mjCAT_ALL, scene)
        
        # レンダリング
        mujoco.mjr_render(viewport, scene, context)
        
        # ウィンドウの更新
        glfw.swap_buffers(window)
        glfw.poll_events()
    
    glfw.terminate()

# グラフィカルインターフェースの初期化
glfw.init()
render_model(model, data)
"""

launch(model, data)