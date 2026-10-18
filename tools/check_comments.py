"""列出 C 文件里【包含 U+FFFD 的注释块】及其精确起止行。

为什么需要它：编码事故会把多行注释压成一行，按行号或关键词替换很容易
切进代码。正确做法是整块替换 —— 本工具给出每块的 (start,end)，替换时
用整块文本即可，代码行不会被碰到。

用法:
    python tools/check_comments.py app/cloud/cloud_uplink.c
退出码: 0 = 没有损坏注释; 1 = 有
"""
import sys

BOM = b"\xef\xbb\xbf"


def comment_blocks(path):
    text = open(path, encoding="utf-8").read()
    blocks = []           # (start_line, end_line, body)
    in_block = False
    start = 0
    buf = []
    for no, line in enumerate(text.splitlines(), 1):
        i = 0
        while i < len(line):
            if not in_block:
                if line.startswith("/*", i):
                    in_block, start, buf = True, no, []
                    i += 2
                    continue
                if line.startswith("//", i):
                    break                      # 行注释: 整行余下都是注释
                i += 1
            else:
                end = line.find("*/", i)
                if end < 0:
                    buf.append(line[i:])
                    break
                buf.append(line[i:end])
                blocks.append((start, no, "\n".join(buf)))
                in_block = False
                i = end + 2
    if in_block:
        blocks.append((start, len(text.splitlines()), "\n".join(buf)))
    return blocks


def main():
    if len(sys.argv) != 2:
        print("usage: python tools/check_comments.py <file.c>")
        return 2
    path = sys.argv[1]
    bad = [b for b in comment_blocks(path) if "\ufffd" in b[2]]
    for start, end, _ in bad:
        print(f"{path}:{start}-{end}  corrupted comment block "
              f"({end - start + 1} lines)")
    print("corrupted blocks:", len(bad))
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
