"""MJCFファイルのボディ質量を実サーボ重量に基づいて調整するスクリプト。

melissa_mjcf_base.xml（CADから書き出したままの、質量がプレースホルダー値の
生エクスポート）を入力とし、以下のルールで各ボディの mass と fullinertia
を書き換えて melissa_mjcf.xml（シミュレーションで実際に使うファイル）を
生成する。

  - <joint> を直接持つボディ（＝サーボモータで駆動される可動部）
      -> --servo-mass （実測サーボ重量）
  - <freejoint /> を持つボディ（＝ルート/胴体基部フレーム、モータなし）
      -> --waist-mass （仮定値）
  - それ以外（カバー・装飾メッシュなど、関節を持たないボディ）
      -> --cosmetic-mass （仮定値）

fullinertia は各ボディごとに (新質量 / 元の質量) の比率で一律スケーリング
し、元データの形状に基づく相対的な慣性分布を維持する。

melissa_mjcf_base.xml がCAD側で再エクスポートされ、ボディ構成や個数が
変わった場合でも、このスクリプトを再実行すれば同じルールで
melissa_mjcf.xml を再生成できる。入力ファイル（*_base.xml）は書き換えず、
常に出力ファイルに新規生成するため、何度実行しても安全（冪等）。

使用例:
    # デフォルト（melissa: KRS-4033HV 61.4g x 20個）で再生成
    python adjust_mjcf_mass.py

    # サーボが変わった場合
    python adjust_mjcf_mass.py --servo-mass 0.0415 --servo-count 22

    # 別モデルに適用する場合
    python adjust_mjcf_mass.py --input foo_mjcf/foo_mjcf_base.xml --output foo_mjcf/foo_mjcf.xml
"""
import argparse
import re
import sys


def adjust_mass(input_path, output_path, servo_mass, servo_count, waist_mass, cosmetic_mass):
    with open(input_path, encoding="utf-8", errors="surrogateescape") as f:
        text = f.read()

    # <compiler> に balanceinertia="true" が無ければ追加する。
    # CAD由来のfullinertiaは境界的な値を含むことがあり、質量スケーリング時の
    # 丸め誤差で物理的妥当性チェック（A+B>=C）にわずかに抵触することがあるため。
    if 'balanceinertia' not in re.search(r'<compiler\b[^>]*/?>', text).group(0):
        text = re.sub(
            r'(<compiler\b[^>]*?)\s*/>',
            r'\1 balanceinertia="true" />',
            text,
            count=1,
        )

    body_open_re = re.compile(r'<body\s+name="([^"]+)"')
    freejoint_re = re.compile(r'<freejoint\s*/>')
    joint_re = re.compile(r'<joint\b')
    inertial_re = re.compile(r'(<inertial\s+pos="[^"]*"\s+mass=")([0-9.eE+-]+)("\s+fullinertia=")([^"]*)(")')

    lines = text.split("\n")
    # 各ボディの種別を「直近で開いたbody名」と、そのbody名を含む行から
    # 次のbody開始行までの間に joint/freejoint が現れたかをワンパスで判定する
    # （inertialは各ボディの冒頭付近、次の子bodyが開く前に必ず現れる構造を前提とする）。
    current_body = None
    body_has_joint = {}
    body_has_freejoint = {}
    for line in lines:
        m = body_open_re.search(line)
        if m:
            current_body = m.group(1)
            body_has_joint.setdefault(current_body, False)
            body_has_freejoint.setdefault(current_body, False)
            continue
        if current_body is None:
            continue
        if freejoint_re.search(line):
            body_has_freejoint[current_body] = True
        elif joint_re.search(line):
            body_has_joint[current_body] = True

    actuated_bodies = {b for b, v in body_has_joint.items() if v}
    waist_bodies = {b for b, v in body_has_freejoint.items() if v}

    if len(actuated_bodies) != servo_count:
        print(
            f"[WARN] detected {len(actuated_bodies)} actuated bodies, "
            f"but --servo-count={servo_count}. joint_names側の定義と齟齬がないか確認してください。",
            file=sys.stderr,
        )

    current_body = None
    out_lines = []
    changed = []

    def repl(match):
        old_mass = float(match.group(2))
        if current_body in actuated_bodies:
            new_mass = servo_mass
        elif current_body in waist_bodies:
            new_mass = waist_mass
        else:
            new_mass = cosmetic_mass
        scale = new_mass / old_mass
        old_vals = [float(x) for x in match.group(4).split()]
        new_vals = [v * scale for v in old_vals]
        new_inertia_str = " ".join(f"{v:.8g}" for v in new_vals)
        changed.append((current_body, old_mass, new_mass))
        return f"{match.group(1)}{new_mass:.4f}{match.group(3)}{new_inertia_str}{match.group(5)}"

    for line in lines:
        m = body_open_re.search(line)
        if m:
            current_body = m.group(1)
        out_lines.append(inertial_re.sub(repl, line))

    new_text = "\n".join(out_lines)
    with open(output_path, "w", encoding="utf-8", errors="surrogateescape") as f:
        f.write(new_text)

    total = sum(m for _, _, m in changed)
    print(f"{input_path} -> {output_path}")
    print(f"  actuated bodies: {len(actuated_bodies)} x {servo_mass:.4f} kg")
    print(f"  waist bodies:    {len(waist_bodies)} x {waist_mass:.4f} kg")
    print(f"  cosmetic bodies: {len(changed) - len(actuated_bodies) - len(waist_bodies)} x {cosmetic_mass:.4f} kg")
    print(f"  total bodies changed: {len(changed)}")
    print(f"  total mass: {total:.4f} kg")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--input', default='melissa_mjcf_base.xml',
                         help='生エクスポートのMJCFファイル（質量プレースホルダー入り、変更しない）')
    parser.add_argument('--output', default='melissa_mjcf.xml',
                         help='質量調整後に書き出すMJCFファイル（シミュレーションで使用）')
    parser.add_argument('--servo-mass', type=float, default=0.0614,
                         help='サーボ1個の実測重量[kg]（例: KRS-4033HV=0.0614）')
    parser.add_argument('--servo-count', type=int, default=20,
                         help='サーボ個数（検出された可動ボディ数との整合チェックに使用）')
    parser.add_argument('--waist-mass', type=float, default=0.010,
                         help='ルート(freejoint)ボディ＝胴体基部フレームの仮定質量[kg]')
    parser.add_argument('--cosmetic-mass', type=float, default=0.003,
                         help='関節を持たない外装カバー類1個あたりの仮定質量[kg]')
    args = parser.parse_args()

    adjust_mass(args.input, args.output, args.servo_mass, args.servo_count,
                args.waist_mass, args.cosmetic_mass)


if __name__ == '__main__':
    main()
