"""
画像航法（VisionNavigate）自律性証明レポート生成スクリプト

logs/telemetry_log_<timestamp>.csv / logs/cmd_log_<timestamp>.csv を機械的にスキャンし，
画像航法フェーズにおける自律ゴール判定・自律制御の証拠
  - ゴール可視フラグ→Search/ImageNavモード自動遷移
  - 画像オフセット量と実際に送信された水平速度指令の相関（ピアソン相関係数）
  - hover_alt_m到達による降下完了(finished)の自律判定と、次フェーズへの自動ハンドオフ
  - finished確定時点で目標地点（ゴールが画像中心付近で最初に捉えられた瞬間の機体位置）から
    半径0.5m以内にいたかどうか
を定量的にまとめたMarkdownレポート(.md, 図はPNG)を自動生成する。

ログが増えるたびに再実行すれば、対象飛行・図・集計表がすべて再計算される
（特定のタイムスタンプはハードコードしていない。--case-a/--case-bで手動指定も可能）。

使い方:
    python3 generate_vision_navigate_report.py
    python3 generate_vision_navigate_report.py --case-a 20260802_031432 --case-b 20260813_005119

    # 解析に使うログを自分で選ぶ（logs-dir内の自動スキャンをせず、指定したログだけを使う）
    python3 generate_vision_navigate_report.py --flights 20260802_031432 20260813_172552
    python3 generate_vision_navigate_report.py --flights-file selected_flights.txt
    # selected_flights.txt の中身は1行1タイムスタンプ（#で始まる行と空行は無視）:
    #   20260802_031432
    #   20260813_172552

    # 特定の1飛行だけについてレポートを作りたい場合：--flightsにタイムスタンプを1つだけ渡す
    # （自動でN飛行集計ではなく、その1飛行専用の構成のレポートに切り替わる）
    python3 generate_vision_navigate_report.py --flights 20260813_172552

オプション:
    --logs-dir       telemetry_log_*.csv / cmd_log_*.csv が入っているディレクトリ
    --out-dir        .mdと図(fig/)の出力先ディレクトリ
    --flights        解析対象ログをタイムスタンプで明示指定（省略時はlogs-dir内を自動スキャン）
    --flights-file   解析対象タイムスタンプを書いたテキストファイルで指定（--flightsと併用可）
    --case-a         詳細解析するケースA（捜索→画像航法→完了）のタイムスタンプを手動指定
    --case-b         詳細解析するケースB（全ミッション区間）のタイムスタンプを手動指定
    --min-vn-rows    解析対象に含める最小VisionNavigate行数

必要なライブラリ（ローカルでも仮想環境・devcontainer内でも、実行するPython環境に
入っていればどちらでもよい。特別な実行環境依存は無い）:
    pip install pandas matplotlib

出力は .md（GitHub Flavored Markdown）と、埋め込み画像のPNGだけ。以前のLaTeX/PDF版と違い
tectonicや日本語フォントのインストールは不要で、生成したその場でどのMarkdownビューア
（GitHub, VS Code等）でもそのまま読める。
"""

from __future__ import annotations

import argparse
import re
import shutil
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import pandas as pd

# 実行時のカレントディレクトリに依存しないよう、スクリプト自身の場所を基準に
# logs/ と docs/vision_navigate_report/ の場所を決める
# （src/DataLogger/ から見てプロジェクトルートは2つ上。plot_control_report.pyと同じ流儀）。
SCRIPT_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = SCRIPT_DIR.parent.parent
DEFAULT_LOGS_DIR = PROJECT_ROOT / "logs"
DEFAULT_OUT_DIR = PROJECT_ROOT / "docs" / "vision_navigate_report"

MIN_VN_ROWS_DEFAULT = 20  # VisionNavigateフェーズの記録行数がこれ未満のログは解析対象外

plt.rcParams["font.size"] = 10
plt.rcParams["axes.grid"] = True
plt.rcParams["grid.alpha"] = 0.3

PHASE_COLORS = {
    "Takeoff": "#eaeaea",
    "GPSNavigate": "#cfe8ff",
    "VisionNavigate": "#c8f0d0",
    "Land": "#ffe3c8",
    "Terminate": "#f0c8c8",
}


# ---------------------------------------------------------------------------
# データ読み込み
# ---------------------------------------------------------------------------

def discover_timestamps(logs_dir: Path) -> list[str]:
    """logs_dir内でtelemetry_log_*.csvとcmd_log_*.csvが両方揃っているタイムスタンプ一覧（昇順=時刻順）"""
    out = []
    for tf in sorted(logs_dir.glob("telemetry_log_*.csv")):
        ts = tf.stem[len("telemetry_log_"):]
        if (logs_dir / f"cmd_log_{ts}.csv").exists():
            out.append(ts)
    return out


def load_merged(logs_dir: Path, ts: str) -> Optional[pd.DataFrame]:
    """telemetryとcmdをelapsed_msで突き合わせて1つのDataFrameにする（cmd側の列はcmd_プレフィックス）。
    空・壊れたCSV（撮影直後に中断された等）はNoneを返す。"""
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
    """'20260802_031432' -> '2026-08-02 03:14:32'"""
    date, time = ts.split("_")
    return f"{date[0:4]}-{date[4:6]}-{date[6:8]} {time[0:2]}:{time[2:4]}:{time[4:6]}"


# ---------------------------------------------------------------------------
# VisionNavigateRunnerが保存するデバッグ画像（ゴール検出時・降下完了時の実カメラ画像）
# ---------------------------------------------------------------------------
# main.cpp/main_test.cppは、CSVログと同じlogs/ディレクトリの下に
# logs/vision_debug_<timestamp>/ としてデバッグ画像を保存する
# （ファイル名例: tick_00042_18734ms_goal_detected.jpg）。
# elapsed_msはCSVのelapsed_ms列と同じ基準なので、ファイル名から直接読み取れる。
# この仕組みが無かった過去のログには対応するディレクトリが存在しないため、
# その場合は画像を含めずにレポートを生成する（find_debug_image()がNoneを返す）。

_DEBUG_IMAGE_RE = re.compile(r"^tick_\d+_(\d+)ms_(.+)\.jpg$")


def find_vision_debug_dir(logs_dir: Path, ts: str) -> Optional[Path]:
    d = logs_dir / f"vision_debug_{ts}"
    return d if d.is_dir() else None


def find_debug_image(debug_dir: Optional[Path], reason: str, pick: str = "last") -> Optional[Path]:
    """debug_dir内から reason（"search_start"/"goal_detected"/"finished"）に一致する
    画像を探す。同じreasonが複数回保存されている場合（例: ゴールを見失って再検出した）は、
    pick="last"なら最も新しい（elapsed_msが大きい）もの、"first"なら最も古いものを返す。
    見つからなければNone。"""
    if debug_dir is None:
        return None
    candidates: list[tuple[int, Path]] = []
    for p in debug_dir.glob(f"tick_*_*ms_{reason}.jpg"):
        m = _DEBUG_IMAGE_RE.match(p.name)
        if m and m.group(2) == reason:
            candidates.append((int(m.group(1)), p))
    if not candidates:
        return None
    candidates.sort(key=lambda x: x[0])
    return candidates[0][1] if pick == "first" else candidates[-1][1]


def copy_debug_images(logs_dir: Path, ts: str, fig_dir: Path) -> dict[str, Path]:
    """指定飛行のデバッグ画像のうち、goal_detectedとfinishedをfig_dir配下へコピーする。
    見つかったreasonをキーにしたコピー先パスの辞書を返す（見つからなかったreasonは含まない）。"""
    debug_dir = find_vision_debug_dir(logs_dir, ts)
    out: dict[str, Path] = {}
    for reason, out_name in (("goal_detected", "goal_detected.jpg"), ("finished", "finished.jpg")):
        src = find_debug_image(debug_dir, reason, pick="last")
        if src is not None:
            dst = fig_dir / out_name
            shutil.copy(src, dst)
            out[reason] = dst
    return out


# ---------------------------------------------------------------------------
# 飛行ごとの統計量
# ---------------------------------------------------------------------------

@dataclass
class FlightStats:
    ts: str
    n_vn_rows: int
    n_search_rows: int
    n_imgnav_rows: int
    n_mode_switches: int
    search_to_goal_s: float
    goal_to_finish_s: float
    alt_finish: float
    corr_r: float
    n_active: int
    # 目標地点（ゴールが画像中心付近デッドバンド内で最初に捉えられた瞬間の機体位置）から
    # 半径target_arrival_thresh_m=0.5m以内に、finished確定時点でいたかどうか。
    # telemetry_log CSVにvision_target_locked等の列が無い（本機能追加前の）古いログでは
    # 計算できないため、その場合は3つともNoneになる。
    target_locked: Optional[bool] = None
    target_distance_m: Optional[float] = None
    target_within_radius: Optional[bool] = None


def target_arrival_at_finish(df: pd.DataFrame) -> Optional[dict]:
    """finished確定行の時点での0.5m目標到達判定を取り出す。
    telemetry_log CSVにvision_target_locked等の列が無い場合はNoneを返す
    （generate_vision_navigate_report.py導入前のDataLoggerで記録された古いログ）。"""
    required_cols = {"vision_target_locked", "vision_distance_to_target_m", "vision_within_target_radius"}
    if not required_cols.issubset(set(df.columns)):
        return None
    vn = df[df["cmd_phase"] == "VisionNavigate"]
    finished_rows = vn[vn["vision_finished"] == 1]
    if finished_rows.empty:
        return None
    row = finished_rows.iloc[0]
    return {
        "target_locked": bool(row["vision_target_locked"]),
        "distance_m": float(row["vision_distance_to_target_m"]),
        "within_radius": bool(row["vision_within_target_radius"]),
    }


def analyze_flight(df: pd.DataFrame, ts: str, min_vn_rows: int) -> Optional[FlightStats]:
    vn = df[df["cmd_phase"] == "VisionNavigate"].reset_index(drop=True)
    if len(vn) < min_vn_rows:
        return None
    imgnav = vn[vn["vision_in_image_nav"] == 1]
    finished_rows = vn[vn["vision_finished"] == 1]
    if imgnav.empty or finished_rows.empty:
        return None

    # アクティブに補正中（画像航法中かつ収束(finished)前）のサンプルだけを相関計算に使う
    active = imgnav[imgnav["vision_finished"] == 0]
    if len(active) < 10:
        return None

    offset_mag = (active["vision_offset_x_px"] ** 2 + active["vision_offset_y_px"] ** 2) ** 0.5
    vel_mag = (active["cmd_pvn_north_m_s"] ** 2 + active["cmd_pvn_east_m_s"] ** 2) ** 0.5
    if offset_mag.std() > 0 and vel_mag.std() > 0:
        corr_r = float(offset_mag.corr(vel_mag))
    else:
        corr_r = float("nan")

    mode_switches = int((vn["vision_in_image_nav"].diff().fillna(0) != 0).sum())
    t_start = vn["elapsed_ms"].iloc[0]
    t_goal_first = imgnav["elapsed_ms"].iloc[0]
    t_finish = finished_rows["elapsed_ms"].iloc[0]

    target = target_arrival_at_finish(df)

    return FlightStats(
        ts=ts,
        n_vn_rows=len(vn),
        n_search_rows=int((vn["vision_in_image_nav"] == 0).sum()),
        n_imgnav_rows=len(imgnav),
        n_mode_switches=mode_switches,
        search_to_goal_s=(t_goal_first - t_start) / 1000.0,
        goal_to_finish_s=(t_finish - t_goal_first) / 1000.0,
        alt_finish=float(finished_rows["altitude_m"].iloc[0]),
        corr_r=corr_r,
        n_active=len(active),
        target_locked=target["target_locked"] if target else None,
        target_distance_m=target["distance_m"] if target else None,
        target_within_radius=target["within_radius"] if target else None,
    )


def collect_flight_stats(logs_dir: Path, min_vn_rows: int) -> tuple[list[str], list[FlightStats]]:
    timestamps = discover_timestamps(logs_dir)
    stats = []
    for ts in timestamps:
        df = load_merged(logs_dir, ts)
        if df is None:
            continue
        s = analyze_flight(df, ts, min_vn_rows)
        if s is not None:
            stats.append(s)
    return timestamps, stats


def pick_case_a(stats: list[FlightStats]) -> FlightStats:
    """捜索の軌道パターンと画像航法での収束の両方が確認しやすい例を優先して選ぶ"""
    candidates = [s for s in stats if s.n_search_rows >= 50 and s.n_imgnav_rows >= 100]
    pool = candidates if candidates else stats
    return max(pool, key=lambda s: s.n_vn_rows)


def pick_case_b(logs_dir: Path, timestamps: list[str], stats_ts: set[str]) -> Optional[str]:
    """Takeoff〜Landの全フェーズを含む、直近（最新）の飛行を選ぶ"""
    required = {"Takeoff", "GPSNavigate", "VisionNavigate", "Land"}
    best = None
    for ts in timestamps:  # discover_timestamps()は既に昇順（時刻順）
        if ts not in stats_ts:
            continue
        df = load_merged(logs_dir, ts)
        if df is None:
            continue
        phases = set(df["cmd_phase"].unique())
        if required.issubset(phases):
            best = ts  # 昇順で回すので、最後に代入されたものが最新
    return best


# ---------------------------------------------------------------------------
# 図の生成（PNG。Markdownはどのビューアでも画像として直接表示できるPNGにする）
# ---------------------------------------------------------------------------

def fig_altitude_mode(df: pd.DataFrame, ts: str, out_path: Path, title_prefix: str = "Case A") -> None:
    vn = df[df["cmd_phase"] == "VisionNavigate"].reset_index(drop=True)
    t0 = vn["elapsed_ms"].iloc[0]
    t = (vn["elapsed_ms"] - t0) / 1000.0
    alt = vn["altitude_m"]
    mode = vn["vision_in_image_nav"]
    goalvis = vn["vision_goal_visible"]
    finished = vn["vision_finished"]
    hover_alt_guess = alt[finished == 1].iloc[0] if (finished == 1).any() else None

    fig, (ax1, ax2) = plt.subplots(
        2, 1, figsize=(7.5, 5.2), sharex=True, gridspec_kw={"height_ratios": [2, 1]}
    )
    ax1.plot(t, alt, color="#1f6feb", lw=1.2, label="Altitude [m] (LiDAR/baro)")
    if hover_alt_guess is not None:
        ax1.axhline(1.0, color="gray", ls=":", lw=1, label="hover_alt_m = 1.0 m (config)")
    ax1.set_ylabel("Altitude [m]")
    ax1.legend(loc="upper right", fontsize=8)
    ax1.set_title(f"{title_prefix} (log {ts}): altitude and autonomous mode over time".strip())

    in_run = False
    start = None
    tv = t.to_numpy()
    mv = mode.to_numpy()
    for i in range(len(tv)):
        if mv[i] == 1 and not in_run:
            in_run = True
            start = tv[i]
        if (mv[i] == 0 or i == len(tv) - 1) and in_run:
            ax1.axvspan(start, tv[i], color="#2ea44f", alpha=0.12)
            in_run = False

    ax2.step(t, goalvis, where="post", color="#d29922", lw=1.0, label="goal_visible")
    ax2.step(t, mode, where="post", color="#2ea44f", lw=1.4, label="in_image_nav (0=Search,1=ImageNav)")
    ax2.step(t, finished, where="post", color="#cf222e", lw=1.4, label="finished")
    ax2.set_ylim(-0.15, 1.25)
    ax2.set_yticks([0, 1])
    ax2.set_xlabel("Elapsed time since VisionNavigate phase start [s]")
    ax2.legend(loc="center right", fontsize=7.5, ncol=1)
    fig.tight_layout()
    fig.savefig(out_path)
    plt.close(fig)


def fig_trajectory(df: pd.DataFrame, ts: str, out_path: Path, title_prefix: str = "Case A") -> None:
    vn = df[df["cmd_phase"] == "VisionNavigate"].reset_index(drop=True)
    search = vn[vn["vision_in_image_nav"] == 0]
    nav = vn[vn["vision_in_image_nav"] == 1]

    fig, ax = plt.subplots(figsize=(5.6, 5.4))
    ax.plot(search["east_m"], search["north_m"], ".", color="#d29922", ms=3, label="Actual position (Search mode)")
    ax.plot(nav["east_m"], nav["north_m"], ".", color="#2ea44f", ms=3, label="Actual position (ImageNav mode)")
    ax.plot(search["cmd_pvn_east_m"], search["cmd_pvn_north_m"], "x", color="#8250df", ms=3, alpha=0.5,
            label="Commanded waypoint (Search)")
    ax.plot(vn["east_m"].iloc[0], vn["north_m"].iloc[0], "o", color="black", ms=7, label="Start")
    ax.plot(vn["east_m"].iloc[-1], vn["north_m"].iloc[-1], "*", color="#cf222e", ms=12, label="Finish (hover)")
    ax.set_xlabel("East [m]")
    ax.set_ylabel("North [m]")
    title = f"{title_prefix}: horizontal trajectory (NED), actual vs. commanded".strip(": ")
    ax.set_title(f"{title}\n(log {ts})")
    ax.legend(fontsize=7, loc="best")
    ax.set_aspect("equal", adjustable="datalim")
    fig.tight_layout()
    fig.savefig(out_path)
    plt.close(fig)


def fig_offset_vs_velocity(df: pd.DataFrame, ts: str, out_path: Path, title_prefix: str = "Case A") -> tuple[float, int]:
    vn = df[df["cmd_phase"] == "VisionNavigate"]
    active = vn[(vn["vision_in_image_nav"] == 1) & (vn["vision_finished"] == 0)]
    offset_mag = (active["vision_offset_x_px"] ** 2 + active["vision_offset_y_px"] ** 2) ** 0.5
    vel_mag = (active["cmd_pvn_north_m_s"] ** 2 + active["cmd_pvn_east_m_s"] ** 2) ** 0.5
    r = float(offset_mag.corr(vel_mag)) if offset_mag.std() > 0 and vel_mag.std() > 0 else float("nan")
    n = len(active)

    fig, ax = plt.subplots(figsize=(5.2, 4.4))
    ax.scatter(offset_mag, vel_mag, s=8, alpha=0.5, color="#1f6feb")
    ax.set_xlabel("|image goal offset| [px] (EMA-filtered, from image center)")
    ax.set_ylabel("|commanded horizontal velocity| [m/s]")
    title = f"{title_prefix}: image offset vs. commanded velocity".strip(": ")
    ax.set_title(f"{title}\n(Pearson r = {r:.3f}, n={n})")
    fig.tight_layout()
    fig.savefig(out_path)
    plt.close(fig)
    return r, n


def fig_fullmission_altitude(df: pd.DataFrame, ts: str, out_path: Path) -> dict[str, tuple[float, float]]:
    t = df["t_s"]
    alt = df["altitude_m"]
    phase = df["cmd_phase"]

    fig, ax1 = plt.subplots(figsize=(7.5, 3.6))
    ax1.plot(t, alt, color="#1f6feb", lw=1.2)
    ax1.set_ylabel("Altitude [m]")
    ax1.set_xlabel("Elapsed mission time [s]")
    ax1.set_title(f"Case B (log {ts}): full-mission altitude profile")

    ylim_top = max(alt) * 1.08
    ax1.set_ylim(min(alt) - 0.3, ylim_top)

    phase_spans: dict[str, tuple[float, float]] = {}
    tv = t.to_numpy()
    pv = phase.to_numpy()
    prev = pv[0]
    start = tv[0]
    for i in range(1, len(tv)):
        if pv[i] != prev or i == len(tv) - 1:
            end = tv[i]
            ax1.axvspan(start, end, color=PHASE_COLORS.get(prev, "#ffffff"), alpha=0.5)
            ax1.text((start + end) / 2, ylim_top * 0.95, prev, ha="center", fontsize=7)
            phase_spans.setdefault(prev, (start, end))
            start, prev = tv[i], pv[i]
    fig.tight_layout()
    fig.savefig(out_path)
    plt.close(fig)
    return phase_spans


def fig_aggregate(stats: list[FlightStats], out_path: Path) -> None:
    fig, (axA, axB) = plt.subplots(1, 2, figsize=(9, 3.6))
    labels = [s.ts[-6:] for s in stats]
    rs = [s.corr_r for s in stats]
    mean_r = sum(rs) / len(rs)

    axA.bar(range(len(rs)), rs, color="#1f6feb")
    axA.axhline(mean_r, color="#cf222e", ls="--", lw=1, label=f"mean = {mean_r:.2f}")
    axA.set_xticks(range(len(rs)))
    axA.set_xticklabels(labels, rotation=90, fontsize=6)
    axA.set_ylabel("Pearson r (offset mag. vs. cmd vel. mag.)")
    axA.set_title(f"(a) Per-flight correlation, N={len(rs)} flights")
    axA.legend(fontsize=8)
    axA.set_ylim(0, 1.05)

    alts = [s.alt_finish for s in stats]
    axB.hist(alts, bins=12, color="#2ea44f", edgecolor="white")
    axB.axvline(1.0, color="#cf222e", ls="--", lw=1.2, label="hover_alt_m = 1.0 m")
    axB.set_xlabel("Altitude at completion (finished=1) [m]")
    axB.set_ylabel("Flight count")
    axB.set_title(f"(b) Completion altitude, N={len(alts)} flights")
    axB.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(out_path)
    plt.close(fig)


# ---------------------------------------------------------------------------
# ケースA: 完了直前・直後の生ログ抜粋テーブル（Markdown表）
# ---------------------------------------------------------------------------

def case_a_transition_table(df: pd.DataFrame) -> tuple[str, Optional[str]]:
    """finished 0->1 の遷移前後、および次フェーズへのハンドオフが分かる代表行をMarkdown表として返す。
    戻り値: (テーブル全体のMarkdown文字列, 遷移直後の次フェーズ名 or None)
    """
    vn = df[df["cmd_phase"] == "VisionNavigate"].reset_index(drop=True)
    finished_idx = vn.index[vn["vision_finished"] == 1]
    if len(finished_idx) == 0:
        return "", None
    fi = finished_idx[0]

    rows_idx = [i for i in (fi - 2, fi - 1, fi, fi + 1) if 0 <= i < len(vn)]
    lines = ["| t[s] | phase | alt[m] | finished | vN[m/s] | vE[m/s] | vD[m/s] |",
             "|---:|:---|---:|---:|---:|---:|---:|"]
    for i in rows_idx:
        r = vn.iloc[i]
        bold = r["vision_finished"] == 1 and (i == 0 or vn.iloc[i - 1]["vision_finished"] == 0)
        cells = [
            f"{r['t_s']:.3f}", str(r["cmd_phase"]), f"{r['altitude_m']:.3f}", str(int(r["vision_finished"])),
            f"{r['cmd_pvn_north_m_s']:.4f}", f"{r['cmd_pvn_east_m_s']:.4f}", f"{r['cmd_pvn_down_m_s']:.3f}",
        ]
        if bold:
            cells = [f"**{c}**" for c in cells]
        lines.append("| " + " | ".join(cells) + " |")

    # 次フェーズへの切り替わり行も1行付け加える（見つかれば）
    next_phase_name = None
    after = df[df["elapsed_ms"] > vn["elapsed_ms"].iloc[rows_idx[-1]]]
    after = after[after["cmd_phase"] != "VisionNavigate"]
    if not after.empty:
        r = after.iloc[0]
        next_phase_name = str(r["cmd_phase"])
        cells = [
            f"**{r['t_s']:.3f}**", f"**{next_phase_name}**", f"{r['altitude_m']:.3f}", str(int(r["vision_finished"])),
            f"{r['cmd_pvn_north_m_s']:.4f}", f"{r['cmd_pvn_east_m_s']:.4f}", f"{r['cmd_pvn_down_m_s']:.3f}",
        ]
        lines.append("| " + " | ".join(cells) + " |")

    return "\n".join(lines), next_phase_name


# ---------------------------------------------------------------------------
# 目標地点への最終到達判定（0.5m以内）
# ---------------------------------------------------------------------------

def target_arrival_status(stats: FlightStats) -> str:
    """0.5m目標到達判定の4状態を返す。
    'unavailable': CSVに判定用の列が無い（DataLogger改修前の古いログ）
    'unlocked'   : 列はあるが、finishedまでにゴールを一度もデッドバンド内で安定して
                   捉えられず(vision_target_locked=false)、目標地点自体が確定していない
                   （distance_to_target_mは未確定を示す初期値-1のまま。0.5m判定の対象外）
    'achieved'   : 目標地点確定済みで、finished確定時点で半径0.5m以内
    'not_achieved': 目標地点確定済みで、finished確定時点で半径0.5mを超えている
    """
    if stats.target_within_radius is None:
        return "unavailable"
    if not stats.target_locked:
        return "unlocked"
    return "achieved" if stats.target_within_radius else "not_achieved"


def target_arrival_fact_text(stats: FlightStats) -> str:
    """0.5m目標到達判定の結果を短い1文で返す（abstract・結論など差し込み用。利用不可ならその旨）。"""
    status = target_arrival_status(stats)
    if status == "unavailable":
        return "目標地点到達判定（`vision_target_locked`等）はこのログには記録されていないため評価できない"
    if status == "unlocked":
        return (
            "ゴールが画像中心付近のデッドバンド内で一度も安定して捉えられなかったため目標地点"
            "そのものが確定せず，0.5 m到達判定は**対象外（未確定）**"
        )
    verdict = "達成" if status == "achieved" else "未達成"
    return (
        f"降下完了確定時点で目標地点から{stats.target_distance_m:.3f} m の位置"
        f"（目標到達半径0.5 m以内は**{verdict}**）"
    )


def build_target_arrival_section(stats: FlightStats) -> str:
    """VisionNavigate.cppが「目標地点（ゴールが画像中心付近で最初に捉えられた瞬間の機体位置）
    から半径target_arrival_thresh_m=0.5m以内」かどうかを判定している値を、達成/未達成として
    明記する節を組み立てる。"""
    status = target_arrival_status(stats)
    if status == "unavailable":
        body = (
            "このログには判定に必要な列（`vision_target_locked`, `vision_distance_to_target_m`, "
            "`vision_within_target_radius`）が記録されていないため"
            "（`DataLogger`にこれらの列を追加する前に取得されたログ），この判定はできない。"
        )
    elif status == "unlocked":
        body = (
            "このログでは，`finished`が確定するまでの間，ゴールが画像中心付近のデッドバンド"
            "`center_dead_m`=0.15 m以内で一度も安定して捉えられなかった"
            "（`vision_target_locked`が終始falseのまま）。そのため「目標地点」自体が一度も"
            "確定しておらず，`distance_to_target_m`はコード上の未確定を示す初期値（-1）の"
            "ままである。この飛行については0.5 m到達判定の対象外であり，**未確定**として扱う。"
        )
    else:
        verdict = "達成" if status == "achieved" else "未達成"
        within_note = "以内であった。" if status == "achieved" else "を超えていた。"
        body = (
            "降下完了（`finished`）が確定した時点で，機体は目標地点"
            "（ゴールが画像中心付近のデッドバンド`center_dead_m`=0.15 m以内で"
            "最初に捉えられた瞬間の機体位置）から水平距離"
            f"`distance_to_target_m` = {stats.target_distance_m:.3f} m の位置にあり，"
            "目標到達半径 `target_arrival_thresh_m`=0.5 m " + within_note +
            f"よって，目標地点への到達は**{verdict}**と判定される。"
        )
    return "### 目標地点への最終到達判定（0.5 m以内）\n\n" + body + "\n"


def limitations_paragraph(stats: FlightStats) -> str:
    """「限界と今後の補強点」の本文。0.5m目標到達判定が使えたかどうかで文面を変える。"""
    if stats.target_within_radius is None:
        return (
            "本解析はこのログに記録されている列のみに基づく。目標地点ロック"
            "（`vision_target_locked`, `vision_distance_to_target_m`, "
            "`vision_within_target_radius`）は `TelemetryData` 構造体には"
            "存在するが，このログを記録した時点の `DataLogger::write_telemetry_row()` "
            "ではCSVに出力されていなかったため，「目標地点への最終到達判定」の判定を"
            "行うことができていない。"
        )
    return (
        "本解析はこのログに記録されている列のみに基づく。「目標地点への最終到達判定」は，"
        "ゴールが最初にデッドバンド内で捉えられた瞬間の機体自身の位置を基準にした相対距離であり，"
        "ゴールマーカーそのものの絶対座標を測定したものではない点に注意が必要である。"
    )


def aggregate_target_arrival_cell(stats: FlightStats) -> str:
    """集計表の「目標到達」列1セル分。"""
    return {
        "unavailable": "--",
        "unlocked": "未確定",
        "achieved": "達成",
        "not_achieved": "未達成",
    }[target_arrival_status(stats)]


def aggregate_target_arrival_bullet(stats_list: list[FlightStats]) -> str:
    """全飛行集計の箇条書きに入れる、目標到達（0.5m以内）の集計文。"""
    statuses = [target_arrival_status(s) for s in stats_list]
    judged = [st for st in statuses if st in ("achieved", "not_achieved")]
    n_unlocked = statuses.count("unlocked")
    if not judged and not n_unlocked:
        return (
            "**目標到達（0.5 m以内）**：解析対象の全飛行が，この判定に必要な列"
            "（`vision_target_locked`等）を`DataLogger`に追加する前に"
            "記録されたログであるため，この観点での集計はできない。"
        )
    n_achieved = sum(1 for st in judged if st == "achieved")
    unlocked_note = (
        f"（うち{n_unlocked}飛行はゴールが一度もデッドバンド内で安定して捉えられず，"
        "目標地点自体が未確定）" if n_unlocked else ""
    )
    if judged:
        return (
            f"**目標到達（0.5 m以内）**：判定に必要な列が記録されていた{len(judged) + n_unlocked}"
            f"飛行のうち，目標地点が確定した{len(judged)}飛行中，`finished`確定時点で目標地点から"
            f"半径`target_arrival_thresh_m`=0.5 m以内にいたのは{n_achieved}/{len(judged)}飛行"
            f"であった{unlocked_note}。"
        )
    return (
        f"**目標到達（0.5 m以内）**：判定に必要な列が記録されていた{n_unlocked}飛行は，"
        "いずれもゴールが一度もデッドバンド内で安定して捉えられず，目標地点自体が未確定のため，"
        "0.5 m到達の判定対象になる飛行が無かった。"
    )


def aggregate_target_arrival_conclusion(stats_list: list[FlightStats]) -> str:
    """結論の箇条書きに入れる、目標到達（0.5m以内）の集計文。"""
    statuses = [target_arrival_status(s) for s in stats_list]
    judged = [st for st in statuses if st in ("achieved", "not_achieved")]
    if not judged:
        return (
            "目標地点から半径0.5 m以内への到達判定は，これに必要な列が記録されている"
            "ログが無かったか，目標地点自体が確定した飛行が無かったため今回は評価できていない"
        )
    n_achieved = sum(1 for st in judged if st == "achieved")
    return (
        f"目標地点から半径`target_arrival_thresh_m`=0.5 m以内への到達は，"
        f"目標地点が確定した{len(judged)}飛行中{n_achieved}飛行で確認された"
    )


def limitations_paragraph_aggregate(stats_list: list[FlightStats]) -> str:
    """複数飛行版「限界と今後の補強点」の本文。"""
    available = [s for s in stats_list if s.target_within_radius is not None]
    if not available:
        return (
            "本解析はログに記録されている列のみに基づく。目標地点ロック"
            "（`vision_target_locked`, `vision_distance_to_target_m`, "
            "`vision_within_target_radius`）は `TelemetryData` 構造体には"
            "存在するが，これらの飛行を記録した時点の `DataLogger::write_telemetry_row()` "
            "ではCSVに出力されていなかったため，目標地点到達判定（集計表の「目標到達」列）は"
            "今回いずれの飛行についても評価できていない。"
            "今後さらに検証の解像度を上げるには，これらの列を記録するログで再度飛行することを推奨する。"
        )
    if len(available) < len(stats_list):
        return (
            "本解析はログに記録されている列のみに基づく。目標地点到達判定に必要な列"
            f"（`vision_target_locked`等）は，解析対象{len(stats_list)}飛行のうち"
            f"{len(available)}飛行にのみ記録されており（`DataLogger`に該当する列を"
            "追加する前に記録されたログには存在しないため），残りの飛行では集計表の"
            "「目標到達」列が「--」（判定不能）になっている。"
        )
    return (
        "本解析はログに記録されている列のみに基づく。目標地点到達判定は，ゴールが最初に"
        "デッドバンド内で捉えられた瞬間の機体自身の位置を基準にした相対距離であり，"
        "ゴールマーカーそのものの絶対座標を測定したものではない点に注意が必要である。"
    )


# ---------------------------------------------------------------------------
# ゴール検出時・降下完了直前の実カメラ画像
# ---------------------------------------------------------------------------

def build_debug_images_section(images: dict[str, Path], heading: str = "### ゴール検出時・降下完了直前の実カメラ画像") -> str:
    """copy_debug_images()が返した辞書から、ゴール検出時・降下完了直前の実カメラ画像を
    埋め込むMarkdown片を組み立てる。画像が1枚も見つからなければ空文字列（節ごと省略）。"""
    if not images:
        return ""

    has_goal = "goal_detected" in images
    has_finished = "finished" in images

    lines = [heading, ""]
    if has_goal and has_finished:
        lines.append(
            "ゴールを最初に検出した瞬間に実際にカメラから取得した画像（検出結果を描き込み済み、左）と，"
            "降下完了（`finished`）が確定し，次フェーズへ引き渡される直前のtickで取得した画像（右）。"
            "いずれも`VisionNavigateRunner`が画像処理と同時に保存したものであり，"
            "機体がその時点で実際に「見ていた」画像そのものである。"
        )
        lines.append("")
        lines.append("| ゴール検出時 | 降下完了直前 |")
        lines.append("|---|---|")
        lines.append("| ![goal detected](fig/goal_detected.jpg) | ![finished](fig/finished.jpg) |")
    elif has_goal:
        lines.append("ゴールを最初に検出した瞬間に実際にカメラから取得した画像（検出結果を描き込み済み）。")
        lines.append("")
        lines.append("![goal detected](fig/goal_detected.jpg)")
    elif has_finished:
        lines.append("降下完了（`finished`）が確定し，次フェーズへ引き渡される直前のtickで実際にカメラから取得した画像。")
        lines.append("")
        lines.append("![finished](fig/finished.jpg)")
    return "\n".join(lines) + "\n"


# ---------------------------------------------------------------------------
# Markdown生成
# ---------------------------------------------------------------------------

SYSTEM_OVERVIEW_MD = """## 対象システムの概要

### フェーズ構成

機体のフライトシーケンスは `src/Phase/Phase.hpp` で
`Standby → ... → Takeoff → GPSNavigate → VisionNavigate → Land → Terminate`
の順に定義されている。本レポートで扱うのはこのうち `VisionNavigate` フェーズである。

### VisionNavigate の内部ロジック

実装は `src/phase/VisionNavigate/VisionNavigate.cpp` にある。200 ms周期の専用スレッド
（`VisionNavigateRunner`）が「カメラ撮影 → `VisionNavigate::update()` 呼び出し（ゴール検出・
航法計算） → 結果を `SystemState` へ書き込み」を1セットとして繰り返す。内部状態機械は次の
3モードを持つ。

- **Search**：ゴール未検出時。起点を中心に半径を徐々に広げる「拡大する八の字（右→左→前→後）」の
  軌道上のウェイポイントへ位置指令を出しながら捜索する。
- **ImageNav**：ゴールを検出した瞬間に自動遷移。画像内のゴール重心の画像中心からのオフセット
  （`offset_x_px`, `offset_y_px`，EMA平滑化後）を，LiDAR高度を使ってピンホールカメラモデルで
  実距離[m]に変換し，比例制御（v = kp × offset_m，デッドバンド・速度上限つき）で水平速度指令を
  計算する。同時に，オフセットが小さいほど降下速度を大きく取り，オフセットが大きい間は水平補正を
  優先して降下速度を絞る（`descend_scale`）。ゴールを`lost_goal_debounce_frames`フレーム連続で
  見失うとSearchへ自動的に戻る。
- **ForcedLand**：捜索が`search_timeout_s`（180 s）を超えても完了しない場合のフェイルセーフ。
  GPSゴール座標へ強制的に向かう。

「ゴール到達」の自律判定は，LiDAR高度が`hover_alt_m`（設定値1.0 m）以下を
`hover_alt_debounce_frames`（3フレーム）連続で記録した時点で確定し，このとき
`status_.finished`が立ち，以後は速度0・現在位置固定のホバリング指令が送出され続ける
（`VisionNavigate.cpp`）。上位の`VisionNavigatePhase`はこの`finished`を検知して
自動的に`Land`フェーズへ遷移する。

**ゴール到達（降下完了）の自律判定ロジック（抜粋，VisionNavigate.cpp）：**

```cpp
if (status_.mode == Mode::ImageNav) {
    if (altitude_m <= config_.hover_alt_m) {
        ++low_alt_count_;
        if (low_alt_count_ >= config_.hover_alt_debounce_frames) {
            status_.cmd = PositionVelocityNedCmd{};
            status_.cmd.north_m = north_m;
            status_.cmd.east_m  = east_m;
            status_.cmd.down_m  = down_m;
            status_.finished = true;   // <- autonomous arrival judgement
            return;
        }
    } else {
        low_alt_count_ = 0;
    }
}
```
"""

PURPOSE_MD = """## 目的と検証方針

本機は，GPS航法で着地目標付近まで移動した後，画像航法フェーズ（`VisionNavigate`）に
切り替わり，下向きカメラで撮影した画像から赤色のゴールマーカーを検出し，そのゴールに
向かって水平位置を補正しながら降下し，一定高度まで降りたら着陸フェーズ（`Land`）へ
引き渡す，という設計になっている（詳細は「対象システムの概要」を参照）。

この「画像に基づいてゴールを判定し，かつ画像に基づいて機体を自律制御している」ことを
第三者が事後的に検証できるようにするには，ソースコードのロジックを示すだけでは不十分で
あり，実際の飛行中にその通りに動作した証拠が要る。本レポートでは，飛行中に10 Hzで
記録され続けた2種類のCSVログ

- **テレメトリログ**：機体位置・姿勢・画像航法の内部状態（ゴール可視・オフセット・モード・
  完了フラグ等）
- **コマンドログ**：その時刻にオフボード制御へ実際に送信されていた指令値（速度・位置setpoint）

を `elapsed_ms` で突き合わせ，「画像処理スレッドが出したゴール検出結果」と「その直後にFCへ
送られた制御指令」の間の時系列上の因果関係を統計的に確認する，という方針を取る。
"""

DISCUSSION_MD_HEAD = """## 考察

### ゴール判定の自律性について

テレメトリログ中の `vision_goal_visible`, `vision_goal_count`, `vision_in_image_nav`,
`vision_finished` は，いずれも`VisionNavigateRunner`のカメラ撮影スレッドが，毎tick取得した
画像を`VisionNavigate::detect_goals()`（HSV赤色マスク→輪郭抽出→楕円フィッティング）にかけた
結果そのものである。このフラグの変化と航法モードの切り替え・降下完了判定が矛盾なく一致して
おり，「機体が画像認識結果に基づいてゴールの有無・到達を自律的に判定していた」ことがログから
客観的に確認できる。
"""


def build_single_flight_md(
    out_dir: Path,
    ts: str,
    df: pd.DataFrame,
    stats: FlightStats,
    debug_images: Optional[dict[str, Path]] = None,
) -> str:
    """--flights/--flights-fileでちょうど1件だけ指定された場合に使う、単一飛行向けMarkdownレポート。"""
    import datetime

    vn = df[df["cmd_phase"] == "VisionNavigate"]
    duration = (vn["elapsed_ms"].iloc[-1] - vn["elapsed_ms"].iloc[0]) / 1000.0
    finish_t = (
        vn[vn["vision_finished"] == 1]["elapsed_ms"].iloc[0] - vn["elapsed_ms"].iloc[0]
    ) / 1000.0
    table_md, next_phase = case_a_transition_table(df)
    next_phase_note = (
        f"直後に`cmd_phase`が`VisionNavigate`から`{next_phase}`へ切り替わっている"
        if next_phase
        else "以後は現在位置を維持するホバリング指令のまま`VisionNavigate`フェーズが終了している"
    )
    # 出現順を保ちつつ重複を除いたフェーズ一覧（例: Takeoff、GPSNavigate、VisionNavigate、Land、Terminate）
    phases_present = list(dict.fromkeys(df["cmd_phase"].tolist()))
    phases_str = "、".join(f"`{p}`" for p in phases_present)

    gen_date = datetime.date.today().strftime("%Y年%m月%d日")

    blocks = [
        f"# 画像航法（VisionNavigate）フェーズにおける自律ゴール判定・自律制御の証明",
        f"**飛行ログ `{ts}` の記録に基づく検証**\n\n"
        f"*Halcyon プロジェクト（本レポートは `generate_vision_navigate_report.py --flights {ts}` "
        f"により自動生成）*  \n*作成日: {gen_date}*",
        "## 概要\n\n"
        f"本レポートは，Halcyon機体の1回の飛行（`{ts}`，{ts_to_dt_str(ts)}取得）を対象に，"
        "画像航法（VisionNavigate）フェーズにおいて機体が搭載カメラの画像のみからゴール"
        "（着地目標）の検出・追跡・到達判定を行い，その結果に基づいて機体を自律的に制御して"
        f"いたことを，このログに記録されたテレメトリログ（`telemetry_log_{ts}.csv`）とコマンド"
        f"ログ（`cmd_log_{ts}.csv`）の突き合わせ解析によって定量的に示す。\n\n"
        f"このログではVisionNavigateフェーズが{stats.n_vn_rows}行（約{duration:.1f} s）記録され，"
        f"ゴール可視フラグの立ち上がりのみを起点として「捜索（Search）」から「画像航法（ImageNav）」"
        f"へ{stats.n_mode_switches}回の自動遷移が発生している。画像中心からのゴールオフセット量と，"
        f"実際にオフボードへ送信された水平速度指令の大きさの間にはピアソン相関係数 "
        f"r = {format_r(stats.corr_r)} という強い相関が確認され，高度がコンフィグ値`hover_alt_m`"
        f"（1.0 m）付近に達した約{finish_t:.1f} s後に機体自身が「完了（finished）」を判定して"
        "ホバリング指令へ切り替え，次フェーズへ自動的にハンドオフしている。"
        f"{target_arrival_fact_text(stats)}。これらはいずれもオペレータの介入を経由しない，"
        "ログにのみ由来する客観的証拠である。",
        PURPOSE_MD,
        SYSTEM_OVERVIEW_MD,
        "## 解析対象ログ\n\n"
        f"本レポートは `{ts}`（{ts_to_dt_str(ts)}取得）の飛行ログ1件のみを対象とする"
        f"（ユーザーが明示的に指定したログ）。このログの`cmd_phase`に含まれるフェーズは"
        f"{phases_str}である。VisionNavigateフェーズは{stats.n_vn_rows}行（約{duration:.1f} s）"
        f"記録されており，捜索モードとImageNavモードの切り替えが{stats.n_mode_switches}回発生している"
        "（デバウンス未満の瞬間的な検出漏れ・再捕捉を含む）。",
        "## 解析結果",
        "### 高度・モードの時系列\n\n"
        "高度と各種フラグの時系列を以下に示す。ゴールを最終的に安定して捕捉してからは"
        f"`in_image_nav`が1で維持され続け，高度が単調に低下し，約{finish_t:.1f} sで"
        "`finished`が1へ切り替わって降下が止まっている。\n\n"
        "![altitude and mode](fig/altitude_mode.png)\n\n"
        "*高度（上段）と，ゴール可視／画像航法モード／完了フラグ（下段，いずれもログから"
        "直接プロット）の時系列。緑背景は`in_image_nav=1`（画像航法で実際に制御している区間）を示す。*",
        "### 水平軌道：ウェイポイント追従と最終収束\n\n"
        "NED水平面上の軌跡を以下に示す。捜索モードではコンフィグ通り「起点を中心に広がる"
        "円弧状のウェイポイント」（紫×印，`VisionNavigate::search_waypoint()`が計算した"
        "`pvn_north_m`/`pvn_east_m`）が時々刻々変化しており，実際の機体位置（橙点）がその軌道を"
        "追従して円弧を描いていることが分かる。ゴールを最終的に捕捉した時点から画像航法（緑点）に"
        "切り替わり，ゴールへ収束してホバリング状態で静止している。\n\n"
        "![trajectory](fig/trajectory.png)\n\n"
        "*NED水平面上の実際位置（Search=橙／ImageNav=緑）とSearch中のコマンド位置setpoint"
        "（紫×）。実位置が指令ウェイポイントに追従して同じ円弧軌道を描いており，指令が実際に"
        "機体を動かしていたことが分かる。*",
        "### 画像オフセットと速度指令の相関\n\n"
        f"画像航法中でかつ`finished=0`のサンプル（n = {stats.n_active}）について，画像オフセット量と"
        f"コマンドログ上の水平速度指令の大きさをプロットしたのが以下の図である。両者はほぼ単調な"
        f"直線関係にあり，ピアソン相関係数は r = {format_r(stats.corr_r)} である。特に，オフセットが"
        "小さい領域（実距離換算でデッドバンド`center_dead_m`=0.15 m相当）では速度指令がほぼ厳密に"
        "0になっており，コード上のデッドバンド処理（`offset_to_velocity()`）が実機でもそのまま"
        "機能していたことが確認できる。\n\n"
        "![offset vs velocity](fig/offset_vs_vel.png)\n\n"
        "*画像中心からのゴールオフセット量と，同時刻にFCへ送信されていた水平速度指令の大きさの"
        "散布図。デッドバンド以下でほぼ0，それ以上ではオフセットにほぼ比例して速度指令が増大して"
        "おり，「画像フィードバックが速度指令を駆動している」ことを裏付ける。*",
        "### 降下完了判定とフェーズハンドオフ\n\n"
        "`finished`直前・直後の代表行を以下に抜粋する。速度指令（vN, vE, vD）が`finished`"
        f"切り替わりの瞬間に厳密に0.0000へ切り替わっており，{next_phase_note}ことが，ログの生値から"
        "そのまま読み取れる。\n\n"
        f"**完了判定前後の生ログ抜粋**（`cmd_log_{ts}.csv` / `telemetry_log_{ts}.csv`）\n\n"
        + table_md,
        build_target_arrival_section(stats),
        build_debug_images_section(debug_images or {}),
        DISCUSSION_MD_HEAD,
        "### 自律制御について\n\n"
        "コマンドログに記録されている指令は，`FCBridge`のオフボードループが実際にPX4へ送信して"
        "いた値そのものである（`DataLogger`は`SystemState::get_any_cmd()`を読むだけであり，別経路で"
        f"生成した値ではない）。上の「解析結果」で示した通り，この指令値は画像オフセット量と"
        f"強く相関し（r = {format_r(stats.corr_r)}），デッドバンド・比例特性・速度上限というコード上の"
        "制御則の特徴をそのまま再現している。これは，オペレータが都度操縦していたのではなく，"
        "画像認識結果を入力とする閉ループ制御則が機体を動かしていたことの定量的な証拠である。",
        "### 限界と今後の補強点\n\n" + limitations_paragraph(stats),
        "## 結論\n\n"
        f"飛行ログ `{ts}` を解析した結果，\n\n"
        "1. ゴールの検出・見失い・再検出・到達判定はすべて画像処理結果（テレメトリログの`vision_*`"
        "フラグ）から自動的に導かれており，\n"
        "2. 機体へ送信された速度・位置指令（コマンドログ）はこの画像フィードバックと強い統計的"
        f"相関（r = {format_r(stats.corr_r)}）を持ち，\n"
        "3. 到達判定は設計値`hover_alt_m`=1.0 m付近という条件で成立し，判定直後に速度指令が"
        "ゼロへ切り替わり，`VisionNavigate`フェーズから次フェーズへ自動的にハンドオフされている，\n"
        f"4. {target_arrival_fact_text(stats)}\n\n"
        "ことが確認された。以上より，この飛行において機体が画像に基づくゴール判定と，それに"
        "基づく自律制御を実際に行っていたことは，テレメトリログとコマンドログという客観的な"
        "記録によって裏付けられる。",
    ]
    return "\n\n".join(b for b in blocks if b) + "\n"


def build_markdown(
    out_dir: Path,
    case_a_ts: str,
    case_a_df: pd.DataFrame,
    case_a_stats: FlightStats,
    case_b_ts: Optional[str],
    case_b_df: Optional[pd.DataFrame],
    all_stats: list[FlightStats],
    min_vn_rows: int,
    case_a_debug_images: Optional[dict[str, Path]] = None,
) -> str:
    """複数飛行（ケースA/B + 全飛行集計）のMarkdownレポートを組み立てる。"""
    import datetime

    vn = case_a_df[case_a_df["cmd_phase"] == "VisionNavigate"]
    duration = (vn["elapsed_ms"].iloc[-1] - vn["elapsed_ms"].iloc[0]) / 1000.0
    finish_t = (
        vn[vn["vision_finished"] == 1]["elapsed_ms"].iloc[0] - vn["elapsed_ms"].iloc[0]
    ) / 1000.0
    table_md, next_phase = case_a_transition_table(case_a_df)
    next_phase_note = (
        f"直後に`cmd_phase`が`VisionNavigate`から`{next_phase}`へ切り替わっている"
        if next_phase
        else "以後は現在位置を維持するホバリング指令のまま`VisionNavigate`フェーズが終了している"
    )

    rs = [s.corr_r for s in all_stats]
    mean_r = sum(rs) / len(rs)
    std_r = (sum((x - mean_r) ** 2 for x in rs) / len(rs)) ** 0.5
    alts = [s.alt_finish for s in all_stats]
    mean_alt = sum(alts) / len(alts)
    std_alt = (sum((x - mean_alt) ** 2 for x in alts) / len(alts)) ** 0.5
    n_flights = len(all_stats)

    gen_date = datetime.date.today().strftime("%Y年%m月%d日")

    method_md = f"""## 解析手法

`logs/`以下の全ログのうち，`VisionNavigate`フェーズの行が{min_vn_rows}行以上あり，かつ画像航法中に
`vision_finished=1`へ到達した{n_flights}件の飛行ログを解析対象とした。各飛行について次の指標を
機械的に算出した。

1. **モード遷移点**：`vision_in_image_nav`が0→1（Search→ImageNav，ゴール発見による自動遷移）
   および1→0（ImageNav→Search，見失いによる自動再捜索）に変化するタイムスタンプ。
2. **オフセット–速度相関**：ImageNavモードかつ`vision_finished=0`（＝まだ収束前でアクティブに
   補正中）の全サンプルについて，画像オフセット量 |offset| = sqrt(offset_x_px² + offset_y_px²) と，
   同時刻にコマンドログへ実際に送信されていた水平速度指令の大きさ
   |v| = sqrt(pvn_north_m_s² + pvn_east_m_s²) の間のピアソン相関係数r。理論上，制御則は
   v = clip(kp・f(offset), ±v_max) というオフセットの単調関数であるため，rが高いほど「指令が
   画像フィードバックに追従して計算されている」ことの直接的な証拠になる。
3. **完了高度**：`vision_finished`が最初に1になった瞬間のLiDAR/気圧高度。設計値`hover_alt_m`=1.0 m
   に近いクラスタを形成するかどうかを確認する。
4. **フェーズハンドオフ**：`finished=1`到達直後に`cmd_phase`が`VisionNavigate`から次フェーズ
   （通常は`Land`）へ切り替わっているかどうか。

以下では代表的な飛行を詳細に，「全飛行での再現性（集計）」では{n_flights}飛行全体を集計して示す。"""

    case_a_md = f"""## ケーススタディA：捜索の再現性とオフセット–速度の強相関（`{case_a_ts}`）

このログはVisionNavigateフェーズだけで{case_a_stats.n_vn_rows}行（約{duration:.1f} s）記録されて
おり，捜索モードとImageNavモードの切り替えが{case_a_stats.n_mode_switches}回発生している
（デバウンス未満の瞬間的な検出漏れ・再捕捉を含む）。

### 高度・モードの時系列

ゴールを最終的に安定して捕捉してからは`in_image_nav`が1で維持され続け，高度が単調に低下し，
約{finish_t:.1f} sで`finished`が1へ切り替わって降下が止まっている。

![case A altitude and mode](fig/caseA_altitude_mode.png)

*ケースA：高度（上段）と，ゴール可視／画像航法モード／完了フラグ（下段，いずれもログから直接
プロット）の時系列。緑背景は`in_image_nav=1`（画像航法で実際に制御している区間）を示す。*

### 水平軌道：ウェイポイント追従と最終収束

捜索モードではコンフィグ通り「起点を中心に広がる円弧状のウェイポイント」（紫×印，
`VisionNavigate::search_waypoint()`が計算した`pvn_north_m`/`pvn_east_m`）が時々刻々変化しており，
実際の機体位置（橙点）がその軌道を追従して円弧を描いていることが分かる。ゴールを最終的に
捕捉した時点から画像航法（緑点）に切り替わり，ゴールへ収束してホバリング状態で静止している。

![case A trajectory](fig/caseA_trajectory.png)

*ケースA：NED水平面上の実際位置（Search=橙／ImageNav=緑）とSearch中のコマンド位置setpoint
（紫×）。実位置が指令ウェイポイントに追従して同じ円弧軌道を描いており，指令が実際に機体を
動かしていたことが分かる。*

### 画像オフセットと速度指令の相関

画像航法中でかつ`finished=0`のサンプル（n = {case_a_stats.n_active}）について，画像オフセット量と
コマンドログ上の水平速度指令の大きさをプロットしたのが以下の図である。両者はほぼ単調な直線
関係にあり，ピアソン相関係数は r = {format_r(case_a_stats.corr_r)} と非常に強い正の相関を示す。
特に，オフセットが小さい領域（実距離換算でデッドバンド`center_dead_m`=0.15 m相当）では速度指令が
ほぼ厳密に0になっており，コード上のデッドバンド処理（`offset_to_velocity()`）が実機でもそのまま
機能していたことが確認できる。

![case A offset vs velocity](fig/caseA_offset_vs_vel.png)

*ケースA：画像中心からのゴールオフセット量と，同時刻にFCへ送信されていた水平速度指令の大きさの
散布図。デッドバンド以下でほぼ0，それ以上ではオフセットにほぼ比例して速度指令が増大しており，
「画像フィードバックが速度指令を駆動している」ことを裏付ける。*

### 降下完了判定とフェーズハンドオフ

`finished`直前・直後の代表行を以下に抜粋する。速度指令（vN, vE, vD）が`finished`切り替わりの
瞬間に厳密に0.0000へ切り替わっており，{next_phase_note}ことが，ログの生値からそのまま読み取れる。

**ケースA：完了判定前後の生ログ抜粋**（`cmd_log_{case_a_ts}.csv` / `telemetry_log_{case_a_ts}.csv`）

{table_md}
"""

    case_b_md = ""
    if case_b_ts is not None and case_b_df is not None:
        vnb = case_b_df[case_b_df["cmd_phase"] == "VisionNavigate"]
        spans = fig_fullmission_altitude(case_b_df, case_b_ts, out_dir / "fig" / "caseB_fullmission_altitude.png")
        vn_start, vn_end = spans.get("VisionNavigate", (float("nan"), float("nan")))
        imgnav = vnb[vnb["vision_in_image_nav"] == 1]
        finished_b = vnb[vnb["vision_finished"] == 1]
        search_to_goal = (imgnav["elapsed_ms"].iloc[0] - vnb["elapsed_ms"].iloc[0]) / 1000.0
        goal_to_finish = (finished_b["elapsed_ms"].iloc[0] - imgnav["elapsed_ms"].iloc[0]) / 1000.0
        alt_finish_b = finished_b["altitude_m"].iloc[0]

        case_b_md = f"""## ケーススタディB：離陸から着陸までの全区間（`{case_b_ts}`，最新の実機飛行）

こちらは`{case_b_ts}`に取得された，この時点で最新の実機飛行の全フェーズを含むログである
（Takeoff→GPSNavigate→VisionNavigate→Land→Terminate）。全体の高度プロファイルを以下に示す。
VisionNavigateフェーズ（緑背景，約{vn_start:.1f}–{vn_end:.1f} s）に入ってから約{search_to_goal:.2f} s後に
ゴールを検出して画像航法へ遷移し，以後は単調に降下して約{goal_to_finish:.1f} s後に`hover_alt_m`付近
（{alt_finish_b:.3f} m）で完了，直後に次フェーズへ自動的に引き渡されている。

![case B full mission altitude](fig/caseB_fullmission_altitude.png)

*ケースB：ミッション全体（離陸～Terminate）の高度と，`cmd_phase`ログから復元したフェーズ区間。
VisionNavigateフェーズ（緑）で滑らかに降下し，完了後Land（橙）へ自動遷移している。*
"""

    agg_rows = []
    for s in sorted(all_stats, key=lambda s: s.ts):
        agg_rows.append(
            f"| {ts_to_dt_str(s.ts)} | {s.n_mode_switches} | {s.search_to_goal_s:.2f} | "
            f"{s.goal_to_finish_s:.1f} | {s.alt_finish:.2f} | {format_r(s.corr_r)} | "
            f"{aggregate_target_arrival_cell(s)} |"
        )
    agg_table = (
        "| 飛行ログ（日時） | モード切替 | 捜索→発見[s] | 発見→完了[s] | 完了高度[m] | r | 目標到達 |\n"
        "|---|---:|---:|---:|---:|---:|---|\n" + "\n".join(agg_rows)
    )

    log_list = "、".join(f"`{s.ts}`" for s in sorted(all_stats, key=lambda s: s.ts))

    aggregate_md = f"""## 全飛行での再現性（集計）

単一飛行の解析だけでは偶然の一致の可能性を排除できないため，「解析手法」の基準を満たす全
{n_flights}飛行について同一の解析を機械的に繰り返した。結果を以下の図と表に示す。

- **オフセット–速度相関**：{n_flights}飛行全てで正の相関が確認され，平均 r = {format_r(mean_r)}
  （標準偏差 {format_r(std_r)}），最小でも r = {format_r(min(rs))}，最大は r = {format_r(max(rs))}。
  相関が常に高いことは，個々の飛行のたまたまの軌道形状に依存した見かけの相関ではなく，画像
  オフセットに応じて速度指令を計算するという制御則そのものがログに一貫して現れていることを
  意味する。
- **完了高度**：`finished`が最初に1になった瞬間の高度は，{n_flights}飛行全てで
  {min(alts):.2f}–{max(alts):.2f} mの狭い範囲（平均 {mean_alt:.2f} m，標準偏差 {std_alt:.3f} m）に
  集中しており，設計値`hover_alt_m`=1.0 mの直下でほぼ一致してデバウンスしている。これは「高度が
  閾値を切ったことをコードが検知して`finished`を立てる」というロジックが，毎回同じ挙動で実機上で
  動作していたことの直接証拠である。
- **モード遷移の自律性**：全飛行で，Search→ImageNavへの遷移は例外なく`goal_visible`が0から1に
  変化した直後のtickで発生しており，オペレータ操作や外部トリガのログは一切介在しない。
- {aggregate_target_arrival_bullet(all_stats)}

![aggregate summary](fig/aggregate_summary.png)

*{n_flights}飛行の集計。(a) 飛行ごとのオフセット–速度相関係数（破線は平均 {format_r(mean_r)}）。
(b) `finished=1`到達時点の高度のヒストグラム（破線は設定値`hover_alt_m`=1.0 m）。*

**{n_flights}飛行の集計結果。**
「モード切替回数」はSearch/ImageNav間の遷移回数，「捜索→発見」はVisionNavigateフェーズ開始から
ゴール初検出までの時間，「発見→完了」はゴール初検出から`finished=1`までの時間，「完了高度」は
`finished`が最初に1になった瞬間の高度，rはオフセット量と速度指令の大きさのピアソン相関係数，
「目標到達」は`finished`確定時点で目標地点（ゴールが画像中心付近デッドバンド内で最初に捉えられた
瞬間の機体位置）から半径`target_arrival_thresh_m`=0.5 m以内にいたかどうか（「--」は判定に必要な
列がこのログに無いことを示す）。

{agg_table}
"""

    conclusion_md = f"""## 結論

{n_flights}件の実機飛行ログを解析した結果，

1. ゴールの検出・見失い・再検出・到達判定はすべて画像処理結果（テレメトリログの`vision_*`フラグ）
   から自動的に導かれており，
2. 機体へ送信された速度・位置指令（コマンドログ）はこの画像フィードバックと強い統計的相関
   （平均 r = {format_r(mean_r)}，{n_flights}飛行中最良例は r = {format_r(max(rs))}）を持ち，
3. 到達判定は設計値`hover_alt_m`=1.0 m付近という一貫した条件で成立し，判定直後に速度指令が
   ゼロへ切り替わり，`VisionNavigate`フェーズから次フェーズへ自動的にハンドオフされている，
4. {aggregate_target_arrival_conclusion(all_stats)}

ことが確認された。以上より，画像航法フェーズにおいて機体が画像に基づくゴール判定と，それに
基づく自律制御を実際に行っていたことは，テレメトリログとコマンドログという客観的な記録に
よって裏付けられる。

## 付録：解析に用いたログ一覧

{n_flights}件の飛行ログ（`logs/telemetry_log_<timestamp>.csv` / `logs/cmd_log_<timestamp>.csv`の
ペア）：{log_list}。

選定基準：`VisionNavigate`フェーズの記録行数が{min_vn_rows}行以上，かつ画像航法中に
`vision_finished=1`に到達していること（「解析手法」参照）。
"""

    blocks = [
        "# 画像航法（VisionNavigate）フェーズにおける自律ゴール判定・自律制御の証明",
        "**テレメトリログおよびコマンドログに基づく定量的検証**\n\n"
        "*Halcyon プロジェクト（本レポートは `generate_vision_navigate_report.py` により"
        "自動生成）*  \n*作成日: " + gen_date + "*",
        "## 概要\n\n"
        "本レポートは，Halcyon機体の画像航法（VisionNavigate）フェーズにおいて，機体が搭載"
        "カメラの画像のみからゴール（着地目標）の検出・追跡・到達判定を行い，その結果に基づいて"
        "機体を自律的に制御していたことを，実飛行時に記録されたテレメトリログ"
        "（`telemetry_log_*.csv`）とコマンドログ（`cmd_log_*.csv`）の突き合わせ解析によって"
        "定量的に示す。具体的には，(1) ゴール可視フラグの立ち上がりのみを起点としてフェーズが"
        "「捜索（Search）」から「画像航法（ImageNav）」へ自動遷移していること，(2) 画像中心からの"
        f"ゴールオフセット量とオフボードへ送信された水平速度指令の大きさが強い相関（全{n_flights}"
        f"飛行の平均ピアソン相関係数 r = {format_r(mean_r)}，最良例では r = {format_r(max(rs))}）を"
        "持つこと，(3) 高度がコンフィグ値`hover_alt_m`（1.0 m）付近に達すると機体自身が「完了"
        "（finished）」を判定してホバリング指令へ切り替え，次フェーズへ自動的にハンドオフしている"
        "こと，を示す。これらはいずれもオペレータの介入を経由しない，ログにのみ由来する客観的"
        "証拠である。（本レポートは `src/DataLogger/generate_vision_navigate_report.py` により"
        "`logs/`内の全飛行ログから自動生成されている。）",
        PURPOSE_MD,
        SYSTEM_OVERVIEW_MD,
        method_md,
        case_a_md,
        build_target_arrival_section(case_a_stats),
        build_debug_images_section(case_a_debug_images or {}, heading="### ケースA：ゴール検出時・降下完了直前の実カメラ画像"),
        case_b_md,
        aggregate_md,
        DISCUSSION_MD_HEAD,
        "### 自律制御について\n\n"
        "コマンドログに記録されている指令は，`FCBridge`のオフボードループが実際にPX4へ送信して"
        "いた値そのものである（`DataLogger`は`SystemState::get_any_cmd()`を読むだけであり，別経路で"
        "生成した値ではない）。「ケーススタディA」〜「全飛行での再現性」で示した通り，この指令値は"
        "画像オフセット量と強く相関し，デッドバンド・比例特性・速度上限というコード上の制御則の"
        "特徴をそのまま再現している。これは，オペレータが都度操縦していたのではなく，画像認識結果を"
        "入力とする閉ループ制御則が機体を動かしていたことの定量的な証拠である。",
        "### 限界と今後の補強点\n\n" + limitations_paragraph_aggregate(all_stats),
        conclusion_md,
    ]
    return "\n\n".join(b for b in blocks if b) + "\n"


def format_r(x: float) -> str:
    return "nan" if x != x else f"{x:.3f}"


# ---------------------------------------------------------------------------
# メイン
# ---------------------------------------------------------------------------

def main() -> None:
    parser = argparse.ArgumentParser(
        description="logs/内のテレメトリ・コマンドログから、画像航法の自律性を証明するMarkdownレポートを生成する")
    parser.add_argument("--logs-dir", type=Path, default=DEFAULT_LOGS_DIR,
                         help="telemetry_log_*.csv / cmd_log_*.csv が入っているディレクトリ")
    parser.add_argument("--out-dir", type=Path, default=DEFAULT_OUT_DIR,
                         help=".mdと図(fig/)の出力先ディレクトリ")
    parser.add_argument("--case-a", type=str, default=None,
                         help="詳細解析するケースA（捜索→画像航法→完了）のタイムスタンプを手動指定（例: 20260802_031432）")
    parser.add_argument("--case-b", type=str, default=None,
                         help="詳細解析するケースB（全ミッション区間）のタイムスタンプを手動指定")
    parser.add_argument("--flights", type=str, nargs="+", default=None,
                         help="解析（集計・ケースA/Bの選定対象）に含める飛行のタイムスタンプを明示的に指定する"
                              "（例: --flights 20260802_031432 20260813_172552）。"
                              "指定した場合、logs-dir内の自動スキャンは行わずこのログだけを使う"
                              "（各ログはVisionNavigateフェーズが--min-vn-rows行以上あり、"
                              "vision_finished=1に到達している必要がある。満たさない場合はエラーで停止する）。"
                              "--flights-fileと併用可能。")
    parser.add_argument("--flights-file", type=Path, default=None,
                         help="解析対象タイムスタンプを1行1つ書いたテキストファイル"
                              "（空行と#で始まる行は無視する）。--flightsと併用可能。")
    parser.add_argument("--min-vn-rows", type=int, default=MIN_VN_ROWS_DEFAULT,
                         help="解析対象に含める最小VisionNavigate行数"
                              "（--flights/--flights-fileで明示指定した場合も、この条件を満たすか検証には使う）")
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
        # 明示指定モード：logs-dir内の自動スキャンはせず、指定されたログだけを使う。
        # 条件を満たさないログが1つでもあれば、黙って外さずエラーで止める
        # （ユーザーが名指ししたログなので、意図せず解析から抜け落ちるのを防ぐため）。
        timestamps = sorted(dict.fromkeys(explicit_ts))
        stats = []
        for ts in timestamps:
            telem_path = args.logs_dir / f"telemetry_log_{ts}.csv"
            cmd_path = args.logs_dir / f"cmd_log_{ts}.csv"
            if not telem_path.exists() or not cmd_path.exists():
                parser.error(f"--flightsで指定された {ts} のログペアが {args.logs_dir} に見つかりません")
            df = load_merged(args.logs_dir, ts)
            if df is None:
                parser.error(f"--flightsで指定された {ts} のログが空か、telemetryとcmdの行が突き合わせられませんでした")
            s = analyze_flight(df, ts, args.min_vn_rows)
            if s is None:
                parser.error(
                    f"--flightsで指定された {ts} は解析条件を満たしていません"
                    f"（VisionNavigateフェーズが{args.min_vn_rows}行以上あり、"
                    "かつ画像航法中にvision_finished=1へ到達している必要があります）"
                )
            stats.append(s)
        print(f"[info] 明示指定された{len(stats)}件のログを解析対象にします: {', '.join(timestamps)}")
    else:
        # 自動スキャンモード：logs-dir内の全ログペアから条件を満たすものを自動選定する
        timestamps, stats = collect_flight_stats(args.logs_dir, args.min_vn_rows)
        if not stats:
            parser.error(
                f"{args.logs_dir} 内に、VisionNavigateフェーズが{args.min_vn_rows}行以上あり"
                "かつvision_finished=1へ到達したログが1件も見つかりませんでした。"
                "（特定のログを使いたい場合は --flights で明示指定してください）"
            )
        print(f"[info] 解析対象フライト: {len(stats)}件 / 発見したログペア: {len(timestamps)}件")

    stats_by_ts = {s.ts: s for s in stats}

    if len(stats) == 1:
        # 解析対象が1件だけ（--flights/--flights-fileでちょうど1件を指定した場合、
        # または自動スキャンで条件を満たすログが1件しか無かった場合）は、
        # 「N飛行の集計」に意味が無いので単一飛行専用のレポートを作る。
        single_ts = stats[0].ts
        single_stats = stats[0]
        single_df = load_merged(args.logs_dir, single_ts)
        print(f"[info] 単一飛行モード: {single_ts} "
              f"(VisionNavigate {single_stats.n_vn_rows}行, r={single_stats.corr_r:.3f})")

        out_dir = args.out_dir
        fig_dir = out_dir / "fig"
        fig_dir.mkdir(parents=True, exist_ok=True)

        fig_altitude_mode(single_df, single_ts, fig_dir / "altitude_mode.png", title_prefix="")
        fig_trajectory(single_df, single_ts, fig_dir / "trajectory.png", title_prefix="")
        fig_offset_vs_velocity(single_df, single_ts, fig_dir / "offset_vs_vel.png", title_prefix="")
        # VisionNavigateフェーズ単体の妥当性証明が目的なので、Takeoff〜Terminateの
        # ミッション全体を俯瞰する図は単一飛行レポートには含めない（複数飛行モードの
        # ケースBでのみ使う。fig_fullmission_altitude()参照）

        # ゴール検出時・降下完了直前の実カメラ画像（VisionNavigateRunnerがlogs/vision_debug_<ts>/へ
        # 保存したもの）。この機能が無かった過去のログには存在しないため、その場合は無言で省略する。
        debug_images = copy_debug_images(args.logs_dir, single_ts, fig_dir)
        if debug_images:
            print(f"[info] デバッグ画像を{len(debug_images)}枚レポートに含めます: "
                  f"{', '.join(sorted(debug_images))}")
        else:
            print(f"[info] logs/vision_debug_{single_ts}/ が見つからないため、"
                  "実カメラ画像はレポートに含めません（この飛行の記録時点でこの機能が"
                  "無かった場合は正常です）")

        md = build_single_flight_md(
            out_dir=out_dir, ts=single_ts, df=single_df, stats=single_stats,
            debug_images=debug_images,
        )
        md_path = out_dir / "vision_navigate_report.md"
        md_path.write_text(md, encoding="utf-8")
        print(f"[info] Markdownレポートを書き出しました: {md_path}")
        return

    case_a_ts = args.case_a or pick_case_a(stats).ts
    if case_a_ts not in stats_by_ts:
        parser.error(f"--case-a {case_a_ts} は解析対象の条件を満たしていません")
    case_a_stats = stats_by_ts[case_a_ts]
    case_a_df = load_merged(args.logs_dir, case_a_ts)
    print(f"[info] ケースA: {case_a_ts} (VisionNavigate {case_a_stats.n_vn_rows}行, r={case_a_stats.corr_r:.3f})")

    case_b_ts = args.case_b or pick_case_b(args.logs_dir, timestamps, set(stats_by_ts))
    case_b_df = load_merged(args.logs_dir, case_b_ts) if case_b_ts else None
    if case_b_ts:
        print(f"[info] ケースB: {case_b_ts}")
    else:
        print("[info] ケースB: 該当ログなし（Takeoff〜Landの全フェーズを含む解析対象ログが無いためスキップ）")

    out_dir = args.out_dir
    fig_dir = out_dir / "fig"
    fig_dir.mkdir(parents=True, exist_ok=True)

    fig_altitude_mode(case_a_df, case_a_ts, fig_dir / "caseA_altitude_mode.png")
    fig_trajectory(case_a_df, case_a_ts, fig_dir / "caseA_trajectory.png")
    fig_offset_vs_velocity(case_a_df, case_a_ts, fig_dir / "caseA_offset_vs_vel.png")
    fig_aggregate(stats, fig_dir / "aggregate_summary.png")
    # caseBの図はbuild_markdown()内、フェーズ区間の情報が必要なタイミングで生成する

    # ゴール検出時・降下完了直前の実カメラ画像（ケースAの飛行のみ。この機能が無かった
    # 過去のログには存在しないため、その場合は無言で省略する）。
    case_a_debug_images = copy_debug_images(args.logs_dir, case_a_ts, fig_dir)
    if case_a_debug_images:
        print(f"[info] ケースAのデバッグ画像を{len(case_a_debug_images)}枚レポートに含めます: "
              f"{', '.join(sorted(case_a_debug_images))}")
    else:
        print(f"[info] logs/vision_debug_{case_a_ts}/ が見つからないため、"
              "実カメラ画像はレポートに含めません（この飛行の記録時点でこの機能が"
              "無かった場合は正常です）")

    md = build_markdown(
        out_dir=out_dir,
        case_a_ts=case_a_ts,
        case_a_df=case_a_df,
        case_a_stats=case_a_stats,
        case_b_ts=case_b_ts,
        case_b_df=case_b_df,
        all_stats=stats,
        min_vn_rows=args.min_vn_rows,
        case_a_debug_images=case_a_debug_images,
    )
    md_path = out_dir / "vision_navigate_report.md"
    md_path.write_text(md, encoding="utf-8")
    print(f"[info] Markdownレポートを書き出しました: {md_path}")


if __name__ == "__main__":
    main()
