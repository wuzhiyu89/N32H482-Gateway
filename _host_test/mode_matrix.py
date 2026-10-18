"""模式开关矩阵验证 —— 覆盖用户实际会用的【两条路径】:

  A) Keil → Options for Target → C/C++ → Define  (命令行 -D)
  B) 直接改 app/drivers/lora/project_config.h 源码里的 0/1

对每个场景用 gcc -E 预处理 main.c, 检查:
  1) 到底选中了哪个模式(看 MAIN_MODE_NAME 展开后出现在横幅里的那个字符串)
  2) 模式③时 rawtest 代码段是否真的被编译(看 rawtest_run 的定义在不在)
  3) 多选是否真的被 #error 拦住

用法: python _host_test/mode_matrix.py
"""
import os
import re
import shutil
import subprocess
import sys

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
ROOT_LIB = r'C:\Users\wzy886\Desktop\Nations.N32H47x_48x_Library.1.2.0\Nations.N32H47x_48x_Library.1.2.0'
CFG = os.path.join(ROOT, 'app', 'radio', 'llcc68', 'project_config.h')
MAIN = os.path.join(ROOT, 'src', 'main.c')
TMP = os.path.join(ROOT, '_host_test', '_modetmp')

INCS = [
    os.path.join(ROOT_LIB, 'firmware', 'CMSIS', 'core'),
    os.path.join(ROOT_LIB, 'firmware', 'CMSIS', 'device'),
    os.path.join(ROOT_LIB, 'firmware', 'n32h47x_48x_std_periph_driver', 'inc'),
    os.path.join(ROOT_LIB, 'firmware', 'n32h47x_48x_usbhs_driver', 'driver', 'inc'),
    os.path.join(ROOT_LIB, 'firmware', 'n32h47x_48x_usbhs_driver', 'host', 'core', 'inc'),
    os.path.join(ROOT_LIB, 'projects', 'n32h47x_48x_EVAL', 'bsp', 'inc'),
    os.path.join(ROOT, 'inc'), os.path.join(ROOT, 'app', 'bsp'),
    os.path.join(ROOT, 'app', 'business'), os.path.join(ROOT, 'app', 'common'),
    os.path.join(ROOT, 'app', 'protocol'), os.path.join(ROOT, 'app', 'ota'),
    os.path.join(ROOT, 'app', 'ring_buf'), os.path.join(ROOT, 'app', 'radio'),
    os.path.join(ROOT, 'app', 'radio', 'llcc68'), os.path.join(ROOT, 'app', 'cjson'),
    os.path.join(ROOT, 'app', 'log'), os.path.join(ROOT, 'app', 'bootloader'),
]
DEFS = ['-DN32H482', '-DUSE_STDPERIPH_DRIVER', '-DBOARD_USE_HSE=1',
        '-DHSE_VALUE=16000000U', '-DLOG_ENABLE=1']

NAMES = {
    # MAIN_MODE_NAME 展开后是 log_info 的独立参数, 所以认这三种 ASCII 后缀
    'CLOUD':     '(MAIN_MODE_CLOUD)"',
    'PAIRING':   '(MAIN_MODE_PAIRING)"',
    'LORA_TEST': '(MAIN_MODE_LORA_TEST)"',
}
LABEL = {'CLOUD': 'cloud', 'PAIRING': 'pairing', 'LORA_TEST': 'lora-test'}


def patch_cfg(text, which):
    """把 project_config.h 里某个模式宏的默认 0 改成 1(模拟用户改源码)。"""
    if which == 'PAIRING':
        text = re.sub(r'(#define MAIN_MODE_PAIRING\s+)0', r'\g<1>1', text, count=1)
    elif which == 'LORA_TEST':
        text = re.sub(r'(#define MAIN_MODE_LORA_TEST\s+)0', r'\g<1>1', text, count=1)
    else:
        raise ValueError(which)
    return text


def preprocess(extra_defs, cfg_text=None):
    """返回 (returncode, stdout)。cfg_text 非空时用改过的 project_config.h 覆盖。"""
    incs = list(INCS)
    if cfg_text is not None:
        if os.path.isdir(TMP):
            shutil.rmtree(TMP)
        os.makedirs(TMP)
        with open(os.path.join(TMP, 'project_config.h'), 'w', encoding='utf-8') as fh:
            fh.write(cfg_text)
        incs.insert(0, TMP)          # ★ 放在最前: 模拟"用户改了这个文件"
    cmd = ['gcc', '-std=c99', '-E', MAIN, '-o', '-'] + DEFS + extra_defs + \
          ['-I' + d for d in incs]
    proc = subprocess.run(cmd, capture_output=True, text=True, errors='replace')
    return proc.returncode, (proc.stdout or '') + (proc.stderr or '')


def analyze(rc, out):
    hits = [k for k, marker in NAMES.items() if marker in out]
    rawtest = 'void rawtest_run(void)' in out
    err = ('#error' in out) or ('error:' in out)
    return hits, rawtest, err


def main():
    with open(CFG, encoding='utf-8') as fh:
        cfg0 = fh.read()

    cases = [
        ('A1 no macro at all (factory default)', [], None, 'CLOUD'),
        ('A2 only -DMAIN_MODE_PAIRING=1', ['-DMAIN_MODE_PAIRING=1'], None, 'PAIRING'),
        ('A3 only -DMAIN_MODE_LORA_TEST=1', ['-DMAIN_MODE_LORA_TEST=1'], None, 'LORA_TEST'),
        ('A4 only -DMAIN_MODE_CLOUD=1 (back)', ['-DMAIN_MODE_CLOUD=1'], None, 'CLOUD'),
        ('A5 old macro -DMAIN_LORA_RAW_TEST=1', ['-DMAIN_LORA_RAW_TEST=1'], None, 'LORA_TEST'),
        ('B1 edit source: PAIRING 0->1', [], patch_cfg(cfg0, 'PAIRING'), 'PAIRING'),
        ('B2 edit source: LORA_TEST 0->1', [], patch_cfg(cfg0, 'LORA_TEST'), 'LORA_TEST'),
    ]

    ok = True
    print('%-38s %-11s %-8s %s' % ('case', 'selected', 'rawtest', 'verdict'))
    print('-' * 78)
    for name, defs, cfg, expect in cases:
        rc, out = preprocess(defs, cfg)
        hits, rawtest, err = analyze(rc, out)
        got = hits[0] if len(hits) == 1 else ('(none/multi:%s)' % hits)
        good = (len(hits) == 1) and (hits[0] == expect)
        # mode 3 must pull in the rawtest code section; the others must not
        want_raw = (expect == 'LORA_TEST')
        good = good and (rawtest == want_raw)
        if not good:
            ok = False
        print('%-38s %-11s %-8s %s' % (name, LABEL.get(got, got),
                                       'yes' if rawtest else 'no',
                                       'OK' if good else 'FAIL(want %s/rawtest=%s)'
                                       % (LABEL[expect], want_raw)))

    # multi-select must be blocked by #error
    rc, out = preprocess(['-DMAIN_MODE_PAIRING=1', '-DMAIN_MODE_LORA_TEST=1'])
    hits, rawtest, err = analyze(rc, out)
    blocked = err and (rc != 0)
    if not blocked:
        ok = False
    print('%-38s %-11s %-8s %s' % ('C1 multi-select (PAIRING+LORA_TEST)',
                                   'build-err' if blocked else '(NOT blocked)',
                                   '-', 'OK' if blocked else 'FAIL'))

    # no macro but the config file still has LORA_TEST=1 baked in? (regression guard)
    rc, out = preprocess([])
    hits, rawtest, err = analyze(rc, out)
    baked_ok = (hits == ['CLOUD'])
    if not baked_ok:
        ok = False
    print('%-38s %-11s %-8s %s' % ('D1 source default is LORA_TEST=0', 
                                   LABEL.get(hits[0] if hits else '?', '?'),
                                   'yes' if rawtest else 'no',
                                   'OK' if baked_ok else 'FAIL(default must be cloud)'))

    if os.path.isdir(TMP):
        shutil.rmtree(TMP)
    print('\n%s' % ('ALL PASS' if ok else '*** SOME CASES FAILED ***'))
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
