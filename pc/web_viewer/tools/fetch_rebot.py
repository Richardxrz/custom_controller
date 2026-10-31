"""下载 reBot Arm B601-DM 并生成轻量模型（每个 link 只保留主 STL）。

用法：
    python3 tools/fetch_rebot.py            # 首次下载
    python3 tools/fetch_rebot.py --force    # 重新生成

产物：models/rebot/model.urdf + meshes/*.STL
来源：https://github.com/Yang-Ci/ReBot_Arm_DigitalTwin_DM
许可：CERN-OHL-W-2.0（随模型复制 LICENSE，不提交到本仓库）
"""

from __future__ import annotations

import argparse
import os
import shutil
import subprocess
import tempfile
import xml.etree.ElementTree as ET

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(HERE, "models", "rebot")
REPO = "https://github.com/Yang-Ci/ReBot_Arm_DigitalTwin_DM.git"
DESC = "reBotArm_ros2_DM/src/rebotarm_bringup/description"
KEEP = {
    "base_link": "base_link.STL",
    "link1": "link1.STL",
    "link2": "link2.STL",
    "link3": "link3.STL",
    "link4": "link4.STL",
    "link5": "link5.STL",
    "link6": "link6.STL",
    "end_link": "gripper_base.stl",
}


def build(src: str, tmp_out: str) -> None:
    os.makedirs(os.path.join(tmp_out, "meshes"))
    tree = ET.parse(os.path.join(src, "urdf", "ReBot_Arm_DM.urdf"))
    root = tree.getroot()
    for j in list(root.findall("joint")):
        if j.get("name") in ("finger_left", "finger_right"):
            root.remove(j)
    for link in list(root.findall("link")):
        if link.get("name") in ("finger_left_link", "finger_right_link"):
            root.remove(link)
    for link in root.findall("link"):
        name = link.get("name")
        if name not in KEEP:
            continue
        fname = KEEP[name]
        shutil.copyfile(os.path.join(src, "meshes", fname),
                        os.path.join(tmp_out, "meshes", fname))
        visuals = link.findall("visual")
        chosen = None
        for v in visuals:
            m = v.find("geometry/mesh")
            if m is not None and os.path.basename(m.get("filename")) == fname:
                chosen = v
                break
        for v in visuals:
            link.remove(v)
        for c in link.findall("collision"):
            link.remove(c)
        if chosen is None:
            chosen = ET.SubElement(link, "visual")
            g = ET.SubElement(chosen, "geometry")
            ET.SubElement(g, "mesh", filename=f"meshes/{fname}")
        else:
            link.append(chosen)
            chosen.find("geometry/mesh").set("filename", f"meshes/{fname}")
    tree.write(os.path.join(tmp_out, "model.urdf"), encoding="utf-8", xml_declaration=True)
    license_src = os.path.join(src, "..", "..", "..", "..", "LICENSE")
    if os.path.isfile(license_src):
        shutil.copyfile(license_src, os.path.join(tmp_out, "LICENSE"))


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--force", action="store_true")
    args = ap.parse_args()
    if os.path.isdir(OUT) and not args.force:
        print(f"已存在：{OUT}（--force 重新生成）")
        return 0
    tmp = tempfile.mkdtemp(prefix="rebot_src_")
    tmp_out = tempfile.mkdtemp(prefix="rebot_lite_")
    try:
        print("克隆 B601-DM description（约 100 MB）…")
        subprocess.check_call(["git", "clone", "--depth", "1", "--filter=blob:none",
                               "--sparse", REPO, tmp])
        subprocess.check_call(["git", "-C", tmp, "sparse-checkout", "set", DESC])
        src = os.path.join(tmp, DESC)
        build(src, tmp_out)
        if os.path.isdir(OUT):
            shutil.rmtree(OUT)
        shutil.move(tmp_out, OUT)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
        shutil.rmtree(tmp_out, ignore_errors=True)
    print(f"完成：{OUT}")
    print("启动：python3 server.py --open")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
