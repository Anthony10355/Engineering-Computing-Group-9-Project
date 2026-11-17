"""Run every scenario with the baseline and student planners, then write a score report.

Usage (from the project root):
    venv\\Scripts\\python.exe tools\\run_all.py                # run all scenarios, then write the report
    venv\\Scripts\\python.exe tools\\run_all.py --report-only  # rebuild the report from results/all_scores.json

Outputs:
    results/all_scores.json   raw parsed results (results/ is git-ignored)
    SCORES.md                 summary table
    docs/scores_chart.svg     bar chart of baseline vs. student score per scenario
"""
import argparse
import datetime as dt
import json
import re
import subprocess
import sys
from pathlib import Path
from xml.sax.saxutils import escape

ROOT = Path(__file__).resolve().parent.parent
RAW = ROOT / "results" / "all_scores.json"
REPORT = ROOT / "SCORES.md"
CHART = ROOT / "docs" / "scores_chart.svg"
PLANNERS = ("baseline", "student")
FIELDS = ["end reason", "collision", "road departure", "goal reached", "progress",
          "simulation time", "planner errors", "rule violations", "SCORE"]


def list_scenarios():
    out = subprocess.run([sys.executable, "run_simulation.py", "--list"], cwd=ROOT,
                         capture_output=True, text=True, check=True).stdout
    found = []
    for line in out.splitlines():
        m = re.match(r"^  (\S+)\s{2,}(.+)$", line)
        if m:
            found.append((m.group(1), m.group(2).strip()))
    return found


def run_one(scenario, planner):
    r = subprocess.run([sys.executable, "run_simulation.py", "--scenario", scenario,
                        "--planner", planner, "--headless"], cwd=ROOT, capture_output=True, text=True)
    d = {"exit": r.returncode}
    for f in FIELDS:
        m = re.search(r"^\s*" + re.escape(f) + r"\s*:\s*(.+)$", r.stdout, re.M)
        d[f] = m.group(1).strip() if m else None
    m = re.match(r"([\d.]+)", d["SCORE"] or "")
    d["score"] = float(m.group(1)) if m else None
    if d["score"] is None:
        d["stderr"] = r.stderr[-1500:]
    return d


def run_all():
    results = []
    for name, desc in list_scenarios():
        row = {"scenario": name, "desc": desc}
        for planner in PLANNERS:
            row[planner] = run_one(name, planner)
            print(f"{name:32s} {planner:8s} {row[planner]['score']}  {row[planner]['end reason']}", flush=True)
        results.append(row)
    RAW.parent.mkdir(exist_ok=True)
    RAW.write_text(json.dumps(results, indent=2))
    return results


def group(name):
    return "Hazard" if name.startswith("hz_") else "Situational" if name.startswith("st_") else "Core"


def mean(xs):
    xs = [x for x in xs if x is not None]
    return sum(xs) / len(xs) if xs else float("nan")


def fmt(v):
    return "—" if v is None else f"{v:.1f}"


def write_chart(results):
    # Horizontal grouped bars, one pair per scenario, grouped by scenario prefix.
    w, label_w, val_w = 760, 230, 44
    bar_h, gap, row_gap, group_h, axis_h = 10, 3, 12, 26, 26
    plot_w = w - label_w - val_w
    colors = {"baseline": "#2a78d6", "student": "#eb6834"}
    x = lambda v: label_w + plot_w * (v or 0) / 100

    parts, y, last = [], axis_h + 26, None
    for r in results:
        g = group(r["scenario"])
        if g != last:
            parts.append(f'<text x="0" y="{y + 16}" class="grp">{g.upper()}</text>')
            y += group_h
            last = g
        pair_h = bar_h * 2 + gap
        parts.append(f'<text x="{label_w - 10}" y="{y + pair_h / 2}" class="lbl" text-anchor="end" '
                     f'dominant-baseline="central">{escape(r["scenario"])}</text>')
        for i, p in enumerate(PLANNERS):
            top = y + i * (bar_h + gap)
            s = r[p]["score"]
            parts.append(f'<rect x="{label_w}" y="{top}" width="{max(x(s) - label_w, 0):.1f}" height="{bar_h}" '
                         f'rx="2" fill="{colors[p]}"><title>{escape(r["scenario"])} {p}: {fmt(s)}</title></rect>')
            parts.append(f'<text x="{x(s) + 5:.1f}" y="{top + bar_h / 2}" class="val" '
                         f'dominant-baseline="central">{fmt(s)}</text>')
        y += pair_h + row_gap
    h = y + 4

    grid = []
    for t in (0, 25, 50, 75, 100):
        grid.append(f'<line x1="{x(t)}" x2="{x(t)}" y1="{axis_h + 22}" y2="{h - row_gap}" '
                    f'stroke="{"#c3c2b7" if t == 0 else "#e1e0d9"}" stroke-width="1"/>')
        grid.append(f'<text x="{x(t)}" y="{axis_h + 14}" class="tick" text-anchor="middle">{t}</text>')

    legend = (f'<rect x="{label_w}" y="2" width="12" height="12" rx="2" fill="{colors["baseline"]}"/>'
              f'<text x="{label_w + 18}" y="12" class="lbl">Baseline</text>'
              f'<rect x="{label_w + 100}" y="2" width="12" height="12" rx="2" fill="{colors["student"]}"/>'
              f'<text x="{label_w + 118}" y="12" class="lbl">Student (ours)</text>')

    svg = f'''<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {w} {h}" width="{w}" height="{h}" role="img"
  aria-label="Score out of 100 per scenario, baseline vs. student planner">
<style>
  text {{ font-family: -apple-system, "Segoe UI", Helvetica, Arial, sans-serif; }}
  .lbl {{ font-size: 12px; fill: #3b3a37; }}
  .val {{ font-size: 10px; fill: #52514e; }}
  .tick {{ font-size: 11px; fill: #6b6a65; }}
  .grp {{ font-size: 11px; fill: #6b6a65; letter-spacing: 0.06em; font-weight: 600; }}
</style>
<rect width="100%" height="100%" fill="#ffffff"/>
{legend}
{"".join(grid)}
{"".join(parts)}
</svg>
'''
    CHART.parent.mkdir(exist_ok=True)
    CHART.write_text(svg, encoding="utf-8")


def write_report(results):
    b = [r["baseline"]["score"] for r in results]
    s = [r["student"]["score"] for r in results]
    wins = sum(1 for x, y in zip(b, s) if y is not None and x is not None and y > x)
    ties = sum(1 for x, y in zip(b, s) if y is not None and x is not None and y == x)
    losses = len(results) - wins - ties
    hz = [r for r in results if group(r["scenario"]) == "Hazard"]
    other = [r for r in results if group(r["scenario"]) != "Hazard"]
    incidents = [r["scenario"] for r in results for p in PLANNERS
                 if r[p]["collision"] == "True" or r[p]["road departure"] == "True"
                 or (r[p]["rule violations"] or "none") != "none"]

    lines = [
        "# Planner scores",
        "",
        f"Every scenario run once per planner in headless mode on {dt.date.today():%d %B %Y}. "
        "Regenerate with `venv\\Scripts\\python.exe tools\\run_all.py`.",
        "",
        f"The student planner averages **{mean(s):.1f} / 100** against the baseline's **{mean(b):.1f}**. "
        f"It scores higher on {wins} of {len(results)} scenarios, ties on {ties} and loses on {losses}.",
        "",
        "| Group | Scenarios | Baseline avg | Student avg |",
        "|---|---:|---:|---:|",
        f"| Core + situational | {len(other)} | {mean([r['baseline']['score'] for r in other]):.1f} "
        f"| {mean([r['student']['score'] for r in other]):.1f} |",
        f"| Hazard | {len(hz)} | {mean([r['baseline']['score'] for r in hz]):.1f} "
        f"| {mean([r['student']['score'] for r in hz]):.1f} |",
        f"| **All** | {len(results)} | **{mean(b):.1f}** | **{mean(s):.1f}** |",
        "",
        "Collisions, road departures or rule violations: "
        + (", ".join(sorted(set(incidents))) if incidents else "none in any run."),
        "",
        "![Baseline vs. student score per scenario](docs/scores_chart.svg)",
        "",
        "| Scenario | Group | Baseline | Student | Gain | Baseline result | Student result | Student time (s) |",
        "|---|---|---:|---:|---:|---|---|---:|",
    ]
    for r in results:
        bs, ss = r["baseline"]["score"], r["student"]["score"]
        gain = f"{ss - bs:+.1f}" if bs is not None and ss is not None else "—"
        t = re.match(r"([\d.]+)", r["student"]["simulation time"] or "")
        lines.append(f"| `{r['scenario']}` | {group(r['scenario'])} | {fmt(bs)} | {fmt(ss)} | {gain} "
                     f"| {r['baseline']['end reason']} | {r['student']['end reason']} | {t.group(1) if t else '—'} |")
    REPORT.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--report-only", action="store_true", help="rebuild the report from results/all_scores.json")
    args = ap.parse_args()
    results = json.loads(RAW.read_text()) if args.report_only else run_all()
    write_chart(results)
    write_report(results)
    print(f"Wrote {REPORT.relative_to(ROOT)} and {CHART.relative_to(ROOT)}")


if __name__ == "__main__":
    main()
