import mujoco
import numpy as np
import os
from mujoco.viewer import launch
import tempfile
import xml.etree.ElementTree as ET

# URDFファイルのパスを指定
urdf_path = "/home/hori/mujoco/urdf/roborecipe4_go2.urdf"

# 指定されたディレクトリパス
urdf_directory_path = "/home/hori/mujoco/urdf"

# URDFファイル名を取得
urdf_filename = os.path.basename(urdf_path)
urdf_name_without_ext = os.path.splitext(urdf_filename)[0]

# 一時ファイルを作成してMuJoCo XMLを保存
with tempfile.NamedTemporaryFile(suffix='.xml', delete=False) as tmp_file:
    temp_xml_path = tmp_file.name
    
    # URDFファイルからMuJoCoモデルを作成し、一時XMLファイルに保存
    model = mujoco.MjModel.from_xml_path(urdf_path)
    mujoco.mj_saveLastXML(temp_xml_path, model)
    
    print(f"Temporary MuJoCo XML saved to: {temp_xml_path}")

# 一時XMLファイルを読み込み
with open(temp_xml_path, 'r') as f:
    xml_content = f.read()

# XMLをパース
root = ET.fromstring(xml_content)

# 全ての関節を見つける
joints = root.findall(".//joint")

# 関節にモータを追加
for joint in joints:
    # 関節の名前を取得
    joint_name = joint.get('name')
    
    # 関節の種類を確認（回転関節のみにモータを追加）
    joint_type = joint.get('type')
    
    if joint_type in ['hinge', None]:  # None はデフォルトでhingeとみなす
        # すでにactuatorセクションがあるか確認
        actuator_section = root.find('.//actuator')
        if actuator_section is None:
            # actuatorセクションがなければ作成
            actuator_section = ET.SubElement(root, 'actuator')
        
        # モータ要素を作成
        motor = ET.SubElement(actuator_section, 'motor')
        motor.set('name', f'motor_{joint_name}')
        motor.set('joint', joint_name)
        motor.set('gear', '100')  # ギア比の設定（適宜調整）
        motor.set('ctrlrange', '-1 1')  # 制御範囲の設定

# 更新したXMLを文字列に変換
updated_xml = ET.tostring(root, encoding='utf-8').decode('utf-8')

# 指定されたディレクトリに保存
output_filename = f"{urdf_name_without_ext}_with_motors.xml"
output_path = os.path.join(urdf_directory_path, output_filename)
with open(output_path, 'w') as f:
    f.write(updated_xml)

# 一時ファイルを削除
os.unlink(temp_xml_path)

print(f"Modified XML saved to: {output_path}")

# 新しいXMLファイルを読み込む
new_model = mujoco.MjModel.from_xml_path(output_path)
new_data = mujoco.MjData(new_model)

# コントロールパネルを表示するようにオプションを設定してビューワーを起動
print("Launching viewer with the modified model...")
launch(new_model, new_data)