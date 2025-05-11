import mujoco
from mujoco.viewer import launch

# XMLモデルファイルを読み込む
#model = mujoco.MjModel.from_xml_path('/opt/mujoco/model/humanoid/humanoid.xml')
#model = mujoco.MjModel.from_xml_path('/home/hori/mujoco/urdf/roborecipe4_go2_with_motors2.xml')
model = mujoco.MjModel.from_xml_path('/home/hori/mujoco/urdf/scene.xml')

data = mujoco.MjData(model)


# ビューアを起動
launch(model, data)