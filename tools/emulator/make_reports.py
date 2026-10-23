#!/usr/bin/env python3
"""Reports docs/reports/T-xx_<date>.md and the summary table from build/sitl_run/results/*.json.

The structured part (configuration, steps, checks, numbers) is generated; the analysis, defects and decision
come from tools/emulator/report_notes/T-xx.md (hand-written, versioned) and are inserted as they are.

  make_reports.py --summary     table on stdout; exit 1 if a strict scenario failed
  make_reports.py --write       also write the reports
"""

import argparse
import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
RESULTS = ROOT / "build" / "sitl_run" / "results"
NOTES = HERE / "report_notes"
REPORTS = ROOT / "docs" / "reports"


def load():
    out = {}
    for f in sorted(RESULTS.glob("*.json")):
        d = json.loads(f.read_text())
        out.setdefault(d["test"], []).append(d)
    return out


def short(v, n=90):
    s = v if isinstance(v, str) else json.dumps(v, ensure_ascii=False)
    return s if len(s) <= n else s[:n - 1] + "…"


def key_numbers(d):
    return "; ".join("%s = %s" % (k, short(d["metrics"].get(k), 60)) for k in d.get("key", []))


def summary(res):
    rows, bad = [], False
    for test in sorted(res):
        for d in res[test]:
            failed = [c["name"] for c in d["checks"] if not c["ok"]]
            if d["result"] == "FAIL":
                bad = True
            rows.append((d["test"] + (" " + d["sub"] if d["sub"] else ""), d["result"],
                         key_numbers(d) + (("; не выполнено: " + "; ".join(failed)) if failed else "")))
    w0 = max([len(r[0]) for r in rows] + [4])
    print("%-*s  %-9s  %s" % (w0, "Тест", "Результат", "Ключевые числа"))
    for r in rows:
        print("%-*s  %-9s  %s" % (w0, r[0], r[1], r[2]))
    return bad


def table(rows, head):
    out = ["| " + " | ".join(head) + " |", "| " + " | ".join("---" for _ in head) + " |"]
    for r in rows:
        out.append("| " + " | ".join(str(x).replace("|", "\\|").replace("\n", " ") for x in r) + " |")
    return "\n".join(out)


def verdict(runs):
    strict = [d for d in runs if d["strict"]]
    if any(d["result"] == "FAIL" for d in strict):
        return "**НЕ ПРОЙДЕН**"
    if all(d["result"] == "PASS" for d in strict):
        return "**ПРОЙДЕН**"
    return "**ИНФОРМАЦИОННЫЙ**"


def render(test, runs):
    date = max(d["date"] for d in runs)
    d0 = runs[0]
    lines = ["# %s — отчёт SITL, %s" % (test, date), ""]
    lines.append("Итог по критерию ТЗ: %s (подпункты: %s)." % (
        verdict(runs), ", ".join("%s — %s" % (d["sub"] or "—", d["result"]) for d in runs)))
    lines.append("")
    lines.append("## Конфигурация")
    lines.append("")
    m = d0.get("module", {})
    fc = d0.get("fc", {})
    lines.append(table([
        ("Модуль", "tof_sil %s (git %s): ядро `core/` без изменений + имитация платы" % (m.get("ver"), m.get("git"))),
        ("FC", fc.get("ver")),
        ("SITL", "ArduCopter 4.7.2, `-I 5`, модель hexa, `--speedup 1`, каталог `build/sitl_run/<сценарий>/`"),
        ("Параметры FC", "`copter.parm` + `copter-hexa.parm` + `params/sitl_tof.param` (+ параметры сценария); "
                         "CRC-32 файлов — в таблице подпунктов"),
        ("Параметры модуля", "умолчания таблицы + `params/module_sitl.json` (+ параметры сценария); CRC-32 "
                             "значений — в таблице подпунктов"),
        ("Датчики (сцена)", "3 × 4×4 @ 15 Гц, FoV 45°, азимуты −30/0/+30°, оптика 0,20 м над землёй, "
                            "до 2 целей на зону, шум 1 %"),
    ], ["Элемент", "Значение"]))
    lines.append("")
    lines.append(table([(d["sub"] or "—", d["title"], d.get("module", {}).get("params_crc"),
                         short(d.get("module", {}).get("params_set"), 80), d.get("fc", {}).get("params_crc"),
                         short(d.get("fc", {}).get("params_extra") or {}, 60), "`%s`" % d.get("run_dir"))
                        for d in runs],
                       ["Подпункт", "Сценарий", "CRC парам. модуля", "Параметры модуля (сверх умолч.)",
                        "CRC файлов FC", "Параметры FC сценария", "Логи"]))
    lines.append("")
    lines.append("Логи каждого прогона: `module_N.jsonl` (карты, события, все пакеты модуля), `scene.jsonl` (истина "
                 "сцены и поза), `gcs.tlog` (MAVLink на GCS), `logs/*.BIN` (журнал FC), `sitl_cmd.txt`.")
    lines.append("")
    lines.append("## Критерий ТЗ")
    lines.append("")
    for d in runs:
        lines.append("- %s: %s" % (d["sub"] or "—", d["criterion"]))
    for d in runs:
        lines.append("")
        lines.append("## Подпункт %s — %s: %s" % (d["sub"] or "—", d["title"], d["result"]))
        lines.append("")
        if d.get("scene", {}).get("objects"):
            lines.append("Сцена: " + "; ".join(short(o, 120) for o in d["scene"]["objects"]))
            lines.append("")
        lines.append("Шаги:")
        lines.append("")
        for s in d["steps"]:
            lines.append("1. " + s)
        lines.append("")
        lines.append(table([("да" if c["ok"] else "**НЕТ**", c["name"], short(c["value"], 80), short(c["limit"], 50))
                            for c in d["checks"]], ["OK", "Проверка", "Значение", "Норма"]))
        lines.append("")
        lines.append("<details><summary>Числа прогона</summary>")
        lines.append("")
        lines.append(table([(k, short(v, 200)) for k, v in d["metrics"].items()], ["Метрика", "Значение"]))
        lines.append("")
        lines.append("</details>")
    note = NOTES / (test + ".md")
    lines.append("")
    lines.append("## Анализ, дефекты, решение")
    lines.append("")
    lines.append(note.read_text().strip() if note.exists() else "_Нет заметок._")
    lines.append("")
    return date, "\n".join(lines)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--summary", action="store_true")
    ap.add_argument("--write", action="store_true")
    a = ap.parse_args()
    res = load()
    if not res:
        print("no results in %s" % RESULTS)
        return 1
    bad = summary(res)
    if a.write:
        REPORTS.mkdir(parents=True, exist_ok=True)
        for test, runs in res.items():
            date, text = render(test, runs)
            path = REPORTS / ("%s_%s.md" % (test, date))
            path.write_text(text)
            print("written", path.relative_to(ROOT))
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
