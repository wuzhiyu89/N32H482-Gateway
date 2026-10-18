"""检查 .ps1 是否会踩"注释吞掉下一行"的编码坑。

背景
----
Windows PowerShell 5.1 对**没有 BOM** 的 .ps1 按 ANSI/GBK 解码。脚本里只要
有中文注释，且注释行最后一个字符是全角标点（。，、：等），它的 UTF-8 末字节
（0x82/0x8C/0x81/0x9A ...）就会落在 GBK 前导字节区间 0x81-0xFE 内，解码器
会把紧跟的 CR/LF 当成这个双字节字符的第二个字节吃掉 —— 下一行整行被并进注释。

2026-10-02 加 test_md5 套件时踩到：`name = 'test_md5'` 被上一行注释吞掉，
套件名变成空，测试照跑，日志里只显示 "=== build & run  ==="。

判据
----
  · 文件有 UTF-8 BOM      -> 安全（PowerShell 会正确解码）
  · 无 BOM 且某行行尾字节 >= 0x80 -> 危险，那一行会吞掉下一行

用法（在 demo 目录下）
---------------------
    python _host_test/check_ps1.py
退出码 0 = 全部安全；1 = 有危险行。
"""
import glob
import os
import sys

BOM = b"\xef\xbb\xbf"


def scan(path):
    """返回该文件的危险行列表 [(行号, 行尾字节, 行内容)]。"""
    data = open(path, "rb").read()
    if data.startswith(BOM):
        return None  # 有 BOM，安全
    bad = []
    for i, raw in enumerate(data.split(b"\n")):
        line = raw.rstrip(b"\r")
        if line and line[-1] >= 0x80:
            bad.append((i + 1, line[-1], raw.decode("utf-8", "replace")))
    return bad


def main():
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    os.chdir(root)

    files = sorted(glob.glob("_host_test/*.ps1")) + sorted(glob.glob("*.ps1"))
    dangerous = 0
    for f in files:
        bad = scan(f)
        if bad is None:
            print("  ok    : %s (有 BOM)" % f)
            continue
        if not bad:
            print("  ok    : %s (无 BOM, 但行尾全是 ASCII)" % f)
            continue
        for ln, byte, text in bad:
            dangerous += 1
            print("  !!    : %s:%d 行尾字节 0x%02X 会吞掉下一行" % (f, ln, byte))
            print("          %s" % text.strip()[:88])
        print("          -> 修法: 把该文件另存为 UTF-8 with BOM")

    print("\n危险行数 = %d" % dangerous)
    return 1 if dangerous else 0


if __name__ == "__main__":
    sys.exit(main())
