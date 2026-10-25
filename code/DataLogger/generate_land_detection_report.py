"""
着地判定（LandDetection）実証レポート生成スクリプト（Markdown版）

logs/telemetry_log_<timestamp>.csv / logs/cmd_log_<timestamp>.csv のうち、
cmd_phase == "LandDetection" の区間を対象に、LandDetection.cpp と同じ判定ロジック
（直近10秒間の高度・合成速度の変動幅が閾値以内かどうか）をログからオフラインで
再計算し、
  - 再計算した結果、いつ・どの判定（SUCCESS/FALLBACK_*/FATAL_FAILURE）が確定したか
  - 実機ログ上で実際に cmd_phase が LandDetection から次フェーズへ切り替わった
    時刻・遷移先
を突き合わせて一致しているかを機械的に確認する、Markdownレポート(.md, 図はPNG)を
自動生成する。

以前はLaTeX(.tex)を書き出してtectonic/xelatexでPDF化していたが、別ツール（tectonic等）
や日本語フォントの用意が要らず、GitHub・エディタ・チャットでそのままプレビューできる
Markdownの方が実用的なため、こちらに切り替えた。

対象ログは cmd_phase 列に "LandDetection" が現れているログのみ（Store/Eject経由の
本番シーケンスで着地判定フェーズを通過した飛行）。2026-08-14時点ではまだそのような
ログがlogs/に存在しないため、対象ログが1件も無い場合はエラーで知らせる。
着地判定を含む実機ログが増えたら、そのまま再実行すれば新しいログも自動で拾われる。

使い方:
    python3 generate_land_detection_report.py
    python3 generate_land_detection_report.py --flights 20260901_120000
    python3 generate_land_detection_report.py --flights-file selected_flights.txt

オプション:
    --logs-dir     telemetry_log_*.csv / cmd_log_*.csv が入っているディレクトリ
    --out-dir      .mdと図(fig/)の出力先ディレクトリ
    --flights      解析対象ログをタイムスタンプで明示指定（省略時はlogs-dir内を自動スキャン）
    --flights-file 解析対象タイムスタンプを1行1つ書いたテキストファイルで指定（--flightsと併用可）

必要なライブラリ: pip install pandas matplotlib
"""

from __future__ import annotations

import argparse
import datetime
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import pandas as pd

# 実行時のカレントディレクトリに依存しないよう、スクリプト自身の場所を基準にする
# （src/DataLogger/ から見てプロジェクトルートは2つ上。他のレポート生成スクリプトと同じ流儀）。
SCRIPT_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = SCRIPT_DIR.parent.parent
DEFAULT_LOGS_DIR = PROJECT_ROOT / "logs"
DEFAULT_OUT_DIR = PROJECT_ROOT / "docs" / "land_detection_report"

AUTHOR_LINE = "東京大学船瀬五十里研究室 Halcyon"

# src/phase/Land/LandDetection.hpp と同じ閾値。
# コード側の閾値を変えたら、ここも合わせて直さないとオフライン再計算が実機ロジックとずれる。
ALT_THRESHOLD_M = 0.2
VEL_THRESHOLD_MS = 0.05
MONITOR_DURATION_S = 10.0
TIMEOUT_DURATION_S = 300.0

plt.rcParams["font.size"] = 10
plt.rcParams["axes.grid"] = True
plt.rcParams["grid.alpha"] = 0.3


# ---------------------------------------------------------------------------
# データ読み込み
# ---------------------------------------------------------------------------

def discover_timestamps(logs_dir: Path) -> list[str]:
    """logs_dir内でtelemetry_log_*.csvとcmd_log_*.csvが両方揃っているタイムスタンプ一覧（昇順）"""
    out = []
    for tf in sorted(logs_dir.glob("telemetry_log_*.csv")):
        ts = tf.stem[len("telemetry_log_"):]
        if (logs_dir / f"cmd_log_{ts}.csv").exists():
            out.append(ts)
    return out


def load_merged(logs_dir: Path, ts: str) -> Optional[pd.DataFrame]:
    """telemetryとcmdをelapsed_msで突き合わせて1つのDataFrameにする（cmd側の列はcmd_プレフィックス）。
    空・壊れたCSVはNoneを返す。"""
    try:
        telem = pd.read_csv(logs_dir / f"telemetry_log_{ts}.csv")
        cmd = pd.read_csv(logs_dir / f"cmd_log_{ts}.csv")
    except pd.errors.EmptyDataError:
        return None
    if telem.empty or cmd.empty:
        return None
    cmd = cmd.rename(columns={c: c if c == "elapsed_ms" else f"cmd_{c}" for c in cmd.columns})
    df = pd.merge(telem, cmd, on="elapsed_ms", how="inner")
    if df.empty:
        return None
    df["t_s"] = df["elapsed_ms"] / 1000.0
    return df


def ts_to_dt_str(ts: str) -> str:
    """'20260901_120000' -> '2026-09-01 12:00:00'"""
    date, time = ts.split("_")
    return f"{date[0:4]}-{date[4:6]}-{date[6:8]} {time[0:2]}:{time[2:4]}:{time[4:6]}"


# ---------------------------------------------------------------------------
# LandDetectionアルゴリズムのオフライン再計算
# ---------------------------------------------------------------------------

def extract_land_detection(df: pd.DataFrame) -> Optional[pd.DataFrame]:
    """cmd_phase=="LandDetection" の行だけを抜き出し、フェーズ開始からの経過秒 t_rel_s と
    合成速度 vel_mag を付与する。1行も無ければNone。"""
    ld = df[df["cmd_phase"] == "LandDetection"].reset_index(drop=True)
    if ld.empty:
        return None
    ld = ld.copy()
    ld["t_rel_s"] = (ld["elapsed_ms"] - ld["elapsed_ms"].iloc[0]) / 1000.0
    ld["vel_mag"] = (ld["north_m_s"] ** 2 + ld["east_m_s"] ** 2 + ld["down_m_s"] ** 2) ** 0.5
    return ld


def recompute_stability(ld: pd.DataFrame) -> pd.DataFrame:
    """LandDetection::update_history()/is_stable() と同じ規則
    （直近MONITOR_DURATION_S秒の最大-最小差が閾値以内、かつ窓が十分な長さ蓄積されていること）を、
    ログの時系列に対してtime-based rollingで再計算する。
    実機のupdate()は最大20Hzで呼ばれるが、ログは10Hz記録なので、再計算の時間分解能は
    ログのサンプリング間隔（最大0.1s）に制限される（レポート本文にも明記する）。"""
    idx = pd.to_timedelta(ld["t_rel_s"], unit="s")
    win = f"{MONITOR_DURATION_S:g}s"

    t_series = pd.Series(ld["t_rel_s"].values, index=idx)
    span = t_series - t_series.rolling(win).min()
    enough_history = span >= (MONITOR_DURATION_S - 0.5)

    alt_series = pd.Series(ld["altitude_m"].values, index=idx)
    alt_diff = alt_series.rolling(win).max() - alt_series.rolling(win).min()
    ld = ld.copy()
    ld["alt_stable"] = (enough_history & (alt_diff <= ALT_THRESHOLD_M)).to_numpy()

    vel_series = pd.Series(ld["vel_mag"].values, index=idx)
    vel_diff = vel_series.rolling(win).max() - vel_series.rolling(win).min()
    ld["vel_stable"] = (enough_history & (vel_diff <= VEL_THRESHOLD_MS)).to_numpy()

    return ld


@dataclass
class Decision:
    kind: str                # SUCCESS/FALLBACK_VEL_ONLY/FALLBACK_ALT_ONLY/FATAL_FAILURE/UNDETERMINED
    t_rel_s: Optional[float]  # 判定確定時刻（フェーズ開始からの秒。UNDETERMINEDならNone）


def decide(ld: pd.DataFrame) -> Decision:
    """PhaseResultをLandDetection.cpp（execute()/update()）と同じ優先順位で決定する:
    1. 高度・速度が両方安定した最初の瞬間に即SUCCESS確定
    2. それが無いままTIMEOUT_DURATION_S(既定300s)に達したら、その時点の安定状況でフォールバック判定
    3. ログがタイムアウトにも安定成立にも達していなければUNDETERMINED（ログがそこで終わっている）
    """
    both = ld[ld["alt_stable"] & ld["vel_stable"]]
    if not both.empty:
        return Decision("SUCCESS", float(both["t_rel_s"].iloc[0]))

    timeout_rows = ld[ld["t_rel_s"] >= TIMEOUT_DURATION_S]
    if timeout_rows.empty:
        return Decision("UNDETERMINED", None)

    row0 = timeout_rows.iloc[0]
    if not row0["alt_stable"] and not row0["vel_stable"]:
        kind = "FATAL_FAILURE"
    elif not row0["alt_stable"]:
        kind = "FALLBACK_VEL_ONLY"
    elif not row0["vel_stable"]:
        kind = "FALLBACK_ALT_ONLY"
    else:
        kind = "SUCCESS"  # ちょうどこの行で両方安定になった境界ケース
    return Decision(kind, float(row0["t_rel_s"]))


def find_actual_transition(df: pd.DataFrame, ld: pd.DataFrame) -> tuple[Optional[str], Optional[float]]:
    """LandDetectionフェーズを実際に抜けた直後の行から、実機が採用した次フェーズと
    その時刻（LandDetectionフェーズ開始からの経過秒）を返す。見つからなければ(None, None)。"""
    last_ms = ld["elapsed_ms"].iloc[-1]
    after = df[df["elapsed_ms"] > last_ms]
    after = after[after["cmd_phase"] != "LandDetection"]
    if after.empty:
        return None, None
    row = after.iloc[0]
    t_rel = (row["elapsed_ms"] - ld["elapsed_ms"].iloc[0]) / 1000.0
    return str(row["cmd_phase"]), float(t_rel)


# Phase.cpp の LandDetectionPhase::run() が定義する遷移先（各PhaseResultに対応する次フェーズ）
EXPECTED_NEXT_PHASE = {
    "SUCCESS": "Deployment",
    "FALLBACK_VEL_ONLY": "Deployment",
    "FALLBACK_ALT_ONLY": "Deployment",
    "FATAL_FAILURE": "Terminate",
}

DECISION_LABEL_MD = {
    "SUCCESS": "`SUCCESS`（通常着地）",
    "FALLBACK_VEL_ONLY": "`FALLBACK_VEL_ONLY`（速度のみで着地とみなす）",
    "FALLBACK_ALT_ONLY": "`FALLBACK_ALT_ONLY`（高度のみで着地とみなす）",
    "FATAL_FAILURE": "`FATAL_FAILURE`（判定不能）",
    "UNDETERMINED": "`UNDETERMINED`（ログがここで終わっており未確定）",
}


@dataclass
class FlightResult:
    ts: str
    ld: pd.DataFrame
    n_rows: int
    duration_s: float
    decision: Decision
    actual_next_phase: Optional[str]
    actual_next_t: Optional[float]

    @property
    def matches(self) -> Optional[bool]:
        """再計算判定と実機ログの遷移先が一致するか。比較不能ならNone。"""
        if self.decision.kind == "UNDETERMINED" or self.actual_next_phase is None:
            return None
        return EXPECTED_NEXT_PHASE.get(self.decision.kind) == self.actual_next_phase


def analyze_flight(logs_dir: Path, ts: str) -> Optional[FlightResult]:
    df = load_merged(logs_dir, ts)
    if df is None:
        return None
    ld = extract_land_detection(df)
    if ld is None or len(ld) < 3:
        return None
    ld = recompute_stability(ld)
    decision = decide(ld)
    next_phase, next_t = find_actual_transition(df, ld)
    return FlightResult(
        ts=ts, ld=ld, n_rows=len(ld),
        duration_s=float(ld["t_rel_s"].iloc[-1]),
        decision=decision, actual_next_phase=next_phase, actual_next_t=next_t,
    )


# ---------------------------------------------------------------------------
# 図の生成（PNG。Markdownからは相対パスでそのまま参照する）
# ---------------------------------------------------------------------------

def fig_land_detection(fr: FlightResult, out_path: Path) -> None:
    ld = fr.ld
    t = ld["t_rel_s"]

    fig, (ax1, ax2) = plt.subplots(
        2, 1, figsize=(7.5, 5.2), sharex=True, gridspec_kw={"height_ratios": [2, 1]}
    )

    ax1.plot(t, ld["altitude_m"], color="#1f6feb", lw=1.2, label="Altitude [m]")
    ax1b = ax1.twinx()
    ax1b.plot(t, ld["vel_mag"], color="#d29922", lw=1.0, alpha=0.85, label="|velocity| [m/s]")
    ax1.set_ylabel("Altitude [m]", color="#1f6feb")
    ax1b.set_ylabel("|velocity| [m/s]", color="#d29922")
    ax1.set_title(f"LandDetection (log {fr.ts}): altitude / velocity and recomputed stability")
    lines1, labels1 = ax1.get_legend_handles_labels()
    lines2, labels2 = ax1b.get_legend_handles_labels()
    ax1.legend(lines1 + lines2, labels1 + labels2, loc="upper right", fontsize=8)

    ax2.step(t, ld["alt_stable"].astype(int), where="post", color="#1f6feb", lw=1.4, label="alt_stable")
    ax2.step(t, ld["vel_stable"].astype(int) * 0.95, where="post", color="#d29922", lw=1.4, label="vel_stable")
    ax2.set_ylim(-0.15, 1.25)
    ax2.set_yticks([0, 1])
    ax2.set_xlabel("Elapsed time since LandDetection phase start [s]")
    ax2.legend(loc="center right", fontsize=8)

    if fr.decision.t_rel_s is not None:
        for ax in (ax1, ax2):
            ax.axvline(fr.decision.t_rel_s, color="#2ea44f", ls="--", lw=1.3)
        ax2.text(fr.decision.t_rel_s, 1.05, f" recomputed: {fr.decision.kind}",
                  color="#2ea44f", fontsize=7, va="bottom")
    if fr.actual_next_t is not None:
        for ax in (ax1, ax2):
            ax.axvline(fr.actual_next_t, color="#cf222e", ls=":", lw=1.3)
        ax2.text(fr.actual_next_t, -0.13, f" actual -> {fr.actual_next_phase}",
                  color="#cf222e", fontsize=7, va="top")

    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)


AGGREGATE_COLORS = {
    "SUCCESS": "#2ea44f",
    "FALLBACK_VEL_ONLY": "#d29922",
    "FALLBACK_ALT_ONLY": "#d29922",
    "FATAL_FAILURE": "#cf222e",
    "UNDETERMINED": "#8250df",
}


def fig_aggregate(results: list[FlightResult], out_path: Path) -> None:
    labels = [r.ts[-6:] for r in results]
    durations = [r.decision.t_rel_s if r.decision.t_rel_s is not None else r.duration_s for r in results]
    colors = [AGGREGATE_COLORS.get(r.decision.kind, "#999999") for r in results]

    # タイトル2行目（色の凡例説明）が幅・高さとも図からはみ出さないよう、
    # 最小幅を広めに取りフォントも小さくする。件数が増えて図が横長になった場合はそのまま伸びる。
    fig, ax = plt.subplots(figsize=(max(7.0, 0.5 * len(results) + 2), 4.0))
    ax.bar(range(len(results)), durations, color=colors)
    ax.set_xticks(range(len(results)))
    ax.set_xticklabels(labels, rotation=90, fontsize=7)
    ax.set_ylabel("Time to decision [s]")
    ax.axhline(TIMEOUT_DURATION_S, color="gray", ls=":", lw=1, label=f"timeout = {TIMEOUT_DURATION_S:g}s")
    ax.set_title(f"Decision outcome per flight, N={len(results)}\n"
                 "(green=SUCCESS, orange=FALLBACK_*, red=FATAL_FAILURE, purple=UNDETERMINED)",
                 fontsize=9)
    ax.legend(fontsize=8)
    fig.tight_layout(rect=(0.0, 0.0, 1.0, 0.88))
    fig.savefig(out_path, dpi=150)
    plt.close(fig)


# ---------------------------------------------------------------------------
# Markdown生成
# ---------------------------------------------------------------------------

IS_STABLE_SNIPPET = """```cpp
bool LandDetection::is_stable(const std::vector<LogData>& history, float threshold,
                                      std::chrono::steady_clock::time_point now) {
    if (history.empty()) return false;
    double total_elapsed = std::chrono::duration_cast<std::chrono::seconds>(
        now - history.front().timestamp).count();
    if (total_elapsed < MONITOR_DURATION_S - 0.5) return false;
    auto [min_it, max_it] = std::minmax_element(history.begin(), history.end(),
        [](const LogData& a, const LogData& b) { return a.value < b.value; });
    return (max_it->value - min_it->value) <= threshold;
}
```
（抜粋、`LandDetection.cpp`）"""

LOGIC_SECTION_TEMPLATE = f"""## 判定ロジック（`LandDetection.cpp`）

高度・合成速度（$\\sqrt{{v_N^2+v_E^2+v_D^2}}$）それぞれについて，直近@@MONITOR_S@@秒間の
最大値と最小値の差が閾値以下なら「安定」とみなす。

- 高度の閾値：@@ALT_THRESHOLD@@ m
- 合成速度の閾値：@@VEL_THRESHOLD@@ m/s
- 安定判定に必要な監視時間：@@MONITOR_S@@ s
- タイムアウト：@@TIMEOUT_S@@ s

高度・速度が両方安定した瞬間に即 `SUCCESS` が確定する。タイムアウトまでに両方が安定
しなかった場合は，タイムアウト時点でどちらか片方だけ安定していれば `FALLBACK_*`，
どちらも安定していなければ `FATAL_FAILURE` となる。

{IS_STABLE_SNIPPET}
"""

METHOD_SECTION_TEMPLATE = """## 解析手法

上記のロジックを，記録済みログ（10Hz）に対してPythonでtime-based rolling window
（`pandas.Series.rolling("@@MONITOR_S@@s")`）として再現し，各時刻での`alt_stable`/
`vel_stable`を再計算した（`recompute_stability()`）。両方が最初にTrueになった瞬間を
`SUCCESS`，そうならないままタイムアウト秒に達した場合はその時点の状態からフォール
バック種別を決定する（`decide()`関数，実装は`LandDetection::update()`の分岐と1対1対応）。
なお実機の`update()`は最大20Hzで呼ばれるのに対し，ログは10Hz記録のため，再計算の
時間分解能はログのサンプリング間隔（最大0.1s）に制限される。
"""


def decided_t_str(d: Decision) -> str:
    if d.t_rel_s is None:
        return "未確定のまま（ログ終了時点まで安定成立・タイムアウトのいずれにも到達せず）"
    return f"{d.t_rel_s:.2f}s後"


def actual_t_str(next_phase: Optional[str], next_t: Optional[float]) -> str:
    if next_phase is None or next_t is None:
        return "実機側のフェーズ遷移がログ上に見つからなかった"
    return f"{next_t:.2f}s後"


def match_note(fr: FlightResult) -> str:
    if fr.actual_next_phase is None:
        return "実機側のフェーズ遷移がログに記録されていないため比較できない"
    m = fr.matches
    if m is True:
        return "再計算結果と実機ログ上の遷移先が一致している"
    if m is False:
        return (f"再計算結果（`{fr.decision.kind}`）の期待遷移先（"
                f"`{EXPECTED_NEXT_PHASE.get(fr.decision.kind, '?')}`）と実機ログの遷移先（"
                f"`{fr.actual_next_phase}`）が一致していない（要確認）")
    return "再計算側が未確定のため比較できない"


def summary_sentence(fr: FlightResult) -> str:
    """単一飛行レポートの「まとめ」節用の1文。"""
    if fr.actual_next_phase is None:
        return ("実機ログ上に着地判定フェーズを抜けた後の遷移が記録されていないため，"
                "再計算結果と実機側の判定の一致は確認できなかった。")
    if fr.decision.kind == "UNDETERMINED":
        return ("本ログの範囲ではオフライン再計算側の判定がタイムアウト・安定成立の"
                "いずれにも達しておらず未確定のため，実機側の判定との一致は確認できなかった。")
    if fr.matches:
        return ("再計算した判定結果と実機ログ上の実際のフェーズ遷移は一致しており，"
                "`LandDetection`の判定ロジックが設計通り実機で動作していたことが"
                "この飛行から確認できる。")
    return (f"再計算した判定結果（`{fr.decision.kind}`）と実機ログ上の実際の遷移先（"
            f"`{fr.actual_next_phase}`）が一致していない。ログのサンプリング解像度"
            "（10Hz）による再計算誤差か，実装と本レポートの再計算ロジックの乖離を疑う必要がある。")


def build_single_flight_md(fr: FlightResult) -> str:
    """--flights/--flights-fileでちょうど1件だけ指定された場合（または自動スキャンで対象が
    1件しか無かった場合）に使う、この1飛行専用のレポート。"""
    d = fr.decision
    ts = fr.ts  # マークダウンなのでLaTeXのようなエスケープは不要（コードスパンで囲むだけ）

    parts = []
    parts.append("# 着地判定（LandDetection）フェーズの実証レポート")
    parts.append(f"### {ts_to_dt_str(fr.ts)} 飛行（`{ts}`）の判定ロジック再現検証\n")
    parts.append(f"**{AUTHOR_LINE}**（本レポートは `generate_land_detection_report.py` により自動生成）\n"
                  f"作成日: {datetime.date.today().strftime('%Y年%m月%d日')}\n")

    parts.append("## 概要\n"
                  f"本レポートは，{ts_to_dt_str(fr.ts)}に記録された飛行ログ（`{ts}`）について，"
                  "Ejectフェーズ後の着地判定（`LandDetection`）フェーズの区間"
                  "（`cmd_phase == \"LandDetection\"`）を抽出し，`LandDetection.cpp`と同一の"
                  f"判定ロジック（直近{MONITOR_DURATION_S:g}秒間の高度・合成速度の変動幅が閾値以内かどうか）"
                  "をログ上でオフライン再計算した上で，その再計算結果（いつ・どの結果で判定が確定したか）と，"
                  "実機ログ上で実際に`cmd_phase`が次フェーズへ切り替わった時刻・遷移先を突き合わせ，"
                  "一致しているかを機械的に確認する。\n")

    logic = LOGIC_SECTION_TEMPLATE
    logic = logic.replace("@@MONITOR_S@@", f"{MONITOR_DURATION_S:g}")
    logic = logic.replace("@@ALT_THRESHOLD@@", f"{ALT_THRESHOLD_M:g}")
    logic = logic.replace("@@VEL_THRESHOLD@@", f"{VEL_THRESHOLD_MS:g}")
    logic = logic.replace("@@TIMEOUT_S@@", f"{TIMEOUT_DURATION_S:g}")
    parts.append(logic)

    method = METHOD_SECTION_TEMPLATE.replace("@@MONITOR_S@@", f"{MONITOR_DURATION_S:g}")
    method += ("\nこの再計算結果を，同じログ内で実際に`cmd_phase`が`LandDetection`から"
               "抜けた時刻・遷移先フェーズと突き合わせる。\n")
    parts.append(method)

    parts.append(
        "## 結果\n"
        f"LandDetectionフェーズは{fr.n_rows}行（約{fr.duration_s:.1f}s）記録されている。"
        f"オフライン再計算の結果，判定は**{DECISION_LABEL_MD[d.kind]}**として{decided_t_str(d)}に確定した。"
        f"実機のログ上では{actual_t_str(fr.actual_next_phase, fr.actual_next_t)}に`cmd_phase`が"
        f"`LandDetection`から`{fr.actual_next_phase or '(なし)'}`へ切り替わっており，{match_note(fr)}。\n"
        f"\n![{ts}](fig/land_{fr.ts}.png)\n\n"
        "*高度・合成速度（上段）と，再計算した安定フラグ（下段）。緑破線＝再計算した判定確定時刻，"
        "赤点線＝実機ログ上の実際のフェーズ遷移時刻。*\n"
    )

    parts.append(f"## まとめ\n{summary_sentence(fr)}\n")

    return "\n".join(parts)


def build_md(results: list[FlightResult]) -> str:
    """複数飛行をまとめて解析した場合のレポート。"""
    n = len(results)
    flight_sections = []
    summary_rows = []
    n_match = 0
    n_undetermined = 0

    for i, r in enumerate(results, start=1):
        d = r.decision
        sec = (
            f"### {i}. {ts_to_dt_str(r.ts)}（`{r.ts}`）\n\n"
            f"LandDetectionフェーズは{r.n_rows}行（約{r.duration_s:.1f}s）記録されている。"
            f"オフライン再計算の結果，判定は**{DECISION_LABEL_MD[d.kind]}**として{decided_t_str(d)}に確定した。"
            f"実機のログ上では{actual_t_str(r.actual_next_phase, r.actual_next_t)}に`cmd_phase`が"
            f"`LandDetection`から`{r.actual_next_phase or '(なし)'}`へ切り替わっており，{match_note(r)}。\n\n"
            f"![{r.ts}](fig/land_{r.ts}.png)\n\n"
            "*高度・合成速度（上段）と，再計算した安定フラグ（下段）。緑破線＝再計算した判定確定時刻，"
            "赤点線＝実機ログ上の実際のフェーズ遷移時刻。*\n"
        )
        flight_sections.append(sec)

        m = r.matches
        if m is True:
            n_match += 1
            match_str = "一致"
        elif m is False:
            match_str = "不一致"
        else:
            n_undetermined += 1
            match_str = "-"
        summary_rows.append(
            f"| {ts_to_dt_str(r.ts)} | `{d.kind}` | "
            f"{'-' if d.t_rel_s is None else f'{d.t_rel_s:.2f}'} | "
            f"{f'`{r.actual_next_phase}`' if r.actual_next_phase else '-'} | "
            f"{'-' if r.actual_next_t is None else f'{r.actual_next_t:.2f}'} | {match_str} |"
        )

    aggregate_section = ""
    if n > 1:
        aggregate_section = (
            "## 全飛行の集計\n"
            f"図に{n}飛行それぞれの判定確定までの時間（未確定の場合はログが途切れた時点までの"
            "経過時間）を示す。\n\n"
            "![aggregate](fig/aggregate.png)\n\n"
            "*飛行ごとの判定確定時刻。色は判定結果の種類（緑=SUCCESS，橙=FALLBACK_\\*，"
            f"赤=FATAL_FAILURE，紫=UNDETERMINED）。破線はタイムアウト({TIMEOUT_DURATION_S:g}s)。*\n"
        )

    parts = []
    parts.append("# 着地判定（LandDetection）フェーズの実証レポート")
    parts.append("### テレメトリログ・コマンドログに基づく判定ロジックの再現検証\n")
    parts.append(f"**{AUTHOR_LINE}**（本レポートは `generate_land_detection_report.py` により自動生成）\n"
                  f"作成日: {datetime.date.today().strftime('%Y年%m月%d日')}\n")

    parts.append(
        "## 概要\n"
        "本レポートは，Ejectフェーズ後の着地判定（`LandDetection`）フェーズについて，"
        "実飛行時に記録されたテレメトリログ・コマンドログから`cmd_phase == \"LandDetection\"`の"
        "区間を抽出し，`LandDetection.cpp`と同一の判定ロジック（直近"
        f"{MONITOR_DURATION_S:g}秒間の高度・合成速度の変動幅が閾値以内かどうか）をログ上で"
        "オフライン再計算した上で，その再計算結果（いつ・どの結果で判定が確定したか）と，"
        "実機ログ上で実際に`cmd_phase`が次フェーズへ切り替わった時刻・遷移先を突き合わせ，"
        f"一致しているかを機械的に確認する。対象は{n}件の飛行ログである。\n"
    )

    logic = LOGIC_SECTION_TEMPLATE
    logic = logic.replace("@@MONITOR_S@@", f"{MONITOR_DURATION_S:g}")
    logic = logic.replace("@@ALT_THRESHOLD@@", f"{ALT_THRESHOLD_M:g}")
    logic = logic.replace("@@VEL_THRESHOLD@@", f"{VEL_THRESHOLD_MS:g}")
    logic = logic.replace("@@TIMEOUT_S@@", f"{TIMEOUT_DURATION_S:g}")
    parts.append(logic)

    method = METHOD_SECTION_TEMPLATE.replace("@@MONITOR_S@@", f"{MONITOR_DURATION_S:g}")
    method += ("\nこの再計算結果を，同じログ内で実際に`cmd_phase`が`LandDetection`から"
               "抜けた時刻・遷移先フェーズと突き合わせ，下表にまとめる。\n")
    parts.append(method)

    parts.append("## 飛行ごとの結果\n" + "\n".join(flight_sections))

    if aggregate_section:
        parts.append(aggregate_section)

    parts.append(
        "## まとめ\n"
        f"{n}件の飛行のうち，{n_match}件で再計算した判定結果と実機ログ上の実際のフェーズ遷移が"
        f"一致した（{n_undetermined}件はログがタイムアウトにも安定成立にも達する前に終わっており"
        "判定不能）。一致していれば，`LandDetection`の判定ロジックが設計通り実機で動作していたことの"
        "裏付けになる。不一致がある場合は，ログのサンプリング解像度（10Hz）による再計算誤差か，"
        "実装と本レポートの再計算ロジックの乖離のいずれかを疑う必要がある。\n\n"
        "| 飛行ログ（日時） | 再計算判定 | 確定時刻[s] | 実際の遷移先 | 遷移時刻[s] | 一致 |\n"
        "|---|---|---|---|---|---|\n"
        + "\n".join(summary_rows) + "\n"
    )

    parts.append(
        "## 付録: 解析に用いたログ一覧\n"
        f"{n}件の飛行ログ（`logs/telemetry_log_<timestamp>.csv` / `logs/cmd_log_<timestamp>.csv`の"
        "ペア，`cmd_phase`に`LandDetection`が含まれるもの）："
        + ", ".join(f"`{r.ts}`" for r in results) + "。\n"
    )

    return "\n".join(parts)


# ---------------------------------------------------------------------------
# メイン
# ---------------------------------------------------------------------------

def main() -> None:
    parser = argparse.ArgumentParser(
        description="logs/内のテレメトリ・コマンドログから、着地判定(LandDetection)フェーズの"
                     "実証Markdownレポートを生成する")
    parser.add_argument("--logs-dir", type=Path, default=DEFAULT_LOGS_DIR,
                         help="telemetry_log_*.csv / cmd_log_*.csv が入っているディレクトリ")
    parser.add_argument("--out-dir", type=Path, default=DEFAULT_OUT_DIR,
                         help=".mdと図(fig/)の出力先ディレクトリ")
    parser.add_argument("--flights", type=str, nargs="+", default=None,
                         help="解析対象ログをタイムスタンプで明示指定（省略時はlogs-dir内を自動スキャン）")
    parser.add_argument("--flights-file", type=Path, default=None,
                         help="解析対象タイムスタンプを1行1つ書いたテキストファイル"
                              "（空行と#で始まる行は無視する）。--flightsと併用可能。")
    args = parser.parse_args()

    if not args.logs_dir.exists():
        parser.error(f"logs-dirが見つかりません: {args.logs_dir}")

    explicit_ts: list[str] = list(args.flights or [])
    if args.flights_file is not None:
        if not args.flights_file.exists():
            parser.error(f"--flights-fileが見つかりません: {args.flights_file}")
        for line in args.flights_file.read_text(encoding="utf-8").splitlines():
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            explicit_ts.append(line)

    if explicit_ts:
        # 明示指定モード：ユーザーが名指ししたログなので、条件を満たさなければエラーで止める
        timestamps = sorted(dict.fromkeys(explicit_ts))
        results = []
        for ts in timestamps:
            if not (args.logs_dir / f"telemetry_log_{ts}.csv").exists() or \
               not (args.logs_dir / f"cmd_log_{ts}.csv").exists():
                parser.error(f"--flightsで指定された {ts} のログペアが {args.logs_dir} に見つかりません")
            r = analyze_flight(args.logs_dir, ts)
            if r is None:
                parser.error(f"--flightsで指定された {ts} にLandDetectionフェーズの行が見つかりません"
                              "（3行以上必要）")
            results.append(r)
    else:
        # 自動スキャンモード：条件を満たすログを黙ってスキップする
        timestamps = discover_timestamps(args.logs_dir)
        results = []
        for ts in timestamps:
            r = analyze_flight(args.logs_dir, ts)
            if r is not None:
                results.append(r)
        if not results:
            parser.error(
                f"{args.logs_dir} 内に cmd_phase=='LandDetection' の行を含むログが1件も"
                "見つかりませんでした（Eject後の着地判定シーケンスを通過した飛行がまだ無い"
                "可能性があります）。特定のログを使いたい場合は --flights で明示指定してください。"
            )

    print(f"[info] 解析対象フライト: {len(results)}件")
    for r in results:
        print(f"  - {r.ts}: decision={r.decision.kind} (t={r.decision.t_rel_s}), "
              f"actual_next={r.actual_next_phase} (t={r.actual_next_t}), match={r.matches}")

    out_dir = args.out_dir
    fig_dir = out_dir / "fig"
    fig_dir.mkdir(parents=True, exist_ok=True)

    for r in results:
        fig_land_detection(r, fig_dir / f"land_{r.ts}.png")
    if len(results) > 1:
        fig_aggregate(results, fig_dir / "aggregate.png")

    # 対象が1件だけなら「N件の飛行のうちM件一致」のような複数飛行前提の言い回しが不自然になるため、
    # この1飛行専用のレポートにする。
    if len(results) == 1:
        md = build_single_flight_md(results[0])
    else:
        md = build_md(results)

    md_path = out_dir / "land_detection_report.md"
    md_path.write_text(md, encoding="utf-8")
    print(f"[info] Markdownレポートを書き出しました: {md_path}")


if __name__ == "__main__":
    main()
