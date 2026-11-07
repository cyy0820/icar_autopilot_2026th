#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
校验 include/utils/tools.hpp 里的 LABEL_* 索引与模型 label_list.txt 的行号是否一致。
换模型后必须先跑这个：索引错位不会报错，只会让所有 FSM 认错标志。

用法:  python3 check_labels.py [模型目录]
默认:  res/models/yolov3_mobilenet_v1
"""

import json
import os
import re
import sys

BASE = os.path.dirname(os.path.abspath(__file__))

# LABEL_* -> label_list.txt 里应有的名字
EXPECT = {
    "LABEL_CONE": "cone",
    "LABEL_PERSON": "person",
    "LABEL_BUSY": "busy",
    "LABEL_LIMIT": "limit",
    "LABEL_UNLIMIT": "unlimit",
    "LABEL_STOP": "stop",
    "LABEL_PARK": "park",
    "LABEL_GATE": "gate",
    "LABEL_CROSS": "cross",
    "LABEL_FORK": "fork",
    "LABEL_LEFT": "left",
    "LABEL_CHOICE": "choice",
    "LABEL_YLEFT": "yleft",
    "LABEL_YRIGHT": "yright",
}
SKIP = {"LABEL_STATION"}  # 别名，跟着别的类走


def model_dir():
    if len(sys.argv) > 1:
        d = sys.argv[1]
    else:
        d = "../res/models/yolov3_mobilenet_v1"
        try:
            with open(os.path.join(BASE, "res/config.json"), encoding="utf-8") as f:
                d = json.load(f)["通用配置参数"]["model"]
        except Exception:
            pass
    d = d.replace(".." + os.sep, "").replace("../", "").lstrip("/\\")
    return d if os.path.isabs(d) else os.path.join(BASE, d)


def parse_defines(path):
    raw = {}
    with open(path, encoding="utf-8") as f:
        for m in re.finditer(r"#define\s+(LABEL_\w+)\s+([\w]+)", f.read()):
            raw[m.group(1)] = m.group(2)

    def val(name, depth=0):
        v = raw[name]
        assert depth < 8, "别名循环: " + name
        return val(v, depth + 1) if v.startswith("LABEL_") else int(v)

    return {k: val(k) for k in raw}


def main():
    labels = parse_defines(os.path.join(BASE, "include/utils/tools.hpp"))
    mdir = model_dir()
    with open(os.path.join(mdir, "label_list.txt"), encoding="utf-8") as f:
        names = [x.strip() for x in f.read().replace("\r", "\n").split("\n") if x.strip()]

    print("模型: %s  (%d 类)" % (mdir, len(names)))
    bad = 0
    for const, want in sorted(EXPECT.items(), key=lambda kv: labels.get(kv[0], 999)):
        if const not in labels:
            continue
        if want not in names:          # 本模型没有这个类（如 12 类模型没有 yleft/yright）
            print("  %-15s 跳过（模型无 %s 类）" % (const, want))
            continue
        idx = labels[const]
        got = names[idx] if idx < len(names) else "<越界>"
        ok = got == want
        bad += 0 if ok else 1
        print("  %-15s = %2d -> %-8s %s" % (const, idx, got, "OK" if ok else "错误，应为 " + want))

    if bad:
        print("\n%d 处索引错位，改 include/utils/tools.hpp 的 LABEL_* 定义" % bad)
        sys.exit(1)
    print("\n全部一致")


if __name__ == "__main__":
    main()
