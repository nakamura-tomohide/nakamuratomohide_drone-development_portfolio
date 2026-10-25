#!/usr/bin/env python3
# Hakoniwaシミュレータの3D LiDAR点群から「機体直下のAGL距離」を計算し、
# TFMiniBridge(C++)に橋渡しするスクリプト。hakoniwa_camera_bridge.pyと同じ
# 「ポーリングしてファイルに書き出すだけ」の構成。
#
# --- 出力方式（カメラブリッジに合わせてある） ---
#   有効なAGLが計算できたtickだけ、一時ファイル→os.replace で AGL_PATH を更新する。
#   計算できなかったtickは「ファイルを一切触らない」（-1等の無効マーカーも書かない）。
#   C++側(TFMiniBridge::read_distance_from_hakoniwa)がファイルのmtimeを見て
#   kMaxStaleMs(=500ms)以上更新が無ければ自動的に無効扱いにするので、これで足りる。
#   hakoniwa_camera_bridge.py が撮影失敗時に最後のフレームをそのまま残し、
#   C++側にフォールバックを任せるのと同じ考え方。
#   （旧版は毎tick必ず書き込み、無効時は -1.0 を書いていた。点群が疎で1tickおきに
#     -1 が書かれ、C++側が no_reading とAGL値を交互に受け取っていた。）
#
# --- AGLの求め方と、その頑健化について ---
#   実機のTFmini-Sは下向き固定の単一ビームで最も近い1点の距離を返す。
#   hakosim.getLidarData() は3D点群を返すので、機体直下を軸とした細い円錐
#   (半頂角 CONE_HALF_ANGLE_DEG) の中に入る点を抽出して使う。
#
#   ただし「円錐内の最小z(=最も近い点)」をそのまま採用すると、地面より手前にある
#   ものを1点でも拾った瞬間にAGLが大きく飛ぶ。実測(2026-09-01 SITL): 対地約4mで
#   飛行中に値が ~1m ⇔ ~4m を数秒周期で往復し、機体が上下に ±1.5〜2m スイングした。
#   手前点の発生源になりうるもの:
#     - 吊り下げ荷物・テザー・着陸脚など、機体直下1m前後にある構造物
#     - ゴールのボール(3Dオブジェクト)の上端を細い円錐がかすめる
#     - 点群が疎で、たまたま外れ値だけが円錐に入るtickがある
#   対策:
#     1) 円錐内の点は min ではなく「中央値」を採る。地面が円錐内点群の多数派である
#        限り、少数の手前点に引っ張られない（min だと1点で決まる）。
#     2) さらに直近 MEDIAN_WINDOW 個の“採用値”の移動中央値でならす。単発の外れ
#        フレームを時系列の多数決で潰す。
#   中央値ベースにしたことで、円錐を少し広げても（=センサFOV内の点を取りこぼさない
#   ように）安全になった。旧版が円錐を 2.5° まで絞っていたのは min 採用時に斜めの
#   障害物を拾わないための措置で、中央値なら広めでも地面が多数派なら問題ない。
#
# --- 座標系（実測で確認済み） ---
#   hakosim.getLidarData() が返す点群は data_frame='VehicleInertialFrame'
#   （機体位置を原点とした、姿勢に依存しない慣性座標系）。Z軸「上向き」が正だった
#   （DOWN_AXIS_SIGN=-1 で対地3m飛行時に 3.12〜3.13m と実測一致）。
#
#   Hakoniwa/Unity側のLiDARセンサー設定(Pitch:90=真下, FOV±3°)は従来通り
#   （drone_config.json / Default3DLiDARController.cs の変更履歴を参照）。
#
# --- 使い方 ---
#   python3 hakoniwa_lidar_bridge.py <config_path> [vehicle_name]
#
#   <config_path>: hakoniwa_camera_bridge.py に渡しているのと同じ pdudef JSON
#                  （lidar_points/lidar_pos チャンネルが定義されている必要がある）
#   [vehicle_name]: 既定は "Drone"
#
#   環境変数 HAKONIWA_LIDAR_DEBUG=1 を付けると、毎tick次の統計を標準出力に出す:
#     総点数 / z>0(下向き)の点数 / 円錐内の点数 / 円錐内zの[min, median, max] /
#     このtickのフレーム値 / 移動中央値フィルタ後の出力値
#   スイング再現時にこのログを見れば、円錐内に手前点(1m前後)が混ざっているか、
#   単に点群が疎で外れているのか、を切り分けられる。
#
# 事前準備は hakoniwa_camera_bridge.py と同じ（HAKONIWA_DRONE_API_DIR 等）。

import collections
import math
import os
import statistics
import sys
import time

HAKONIWA_DRONE_API_DIR = os.environ.get(
    "HAKONIWA_DRONE_API_DIR",
    os.path.expanduser("~/project/hakoniwa-drone-core/drone_api"),
)
sys.path.insert(0, os.path.join(HAKONIWA_DRONE_API_DIR, "libs"))
sys.path.insert(0, HAKONIWA_DRONE_API_DIR)

import hakosim  # noqa: E402  (sys.path調整後にimportする必要があるため)
import libs.hakosim_lidar as hakosim_lidar  # noqa: E402

# --- hakosim.getLidarData()内部バグへの回避パッチ ---
# 手元のhakoniwa-drone-coreでは、lidar_pdu_data.data（PointCloud2のdata）が
# bytesではなくtuple(intの並び)として渡ってくるため、struct.unpack_from()が
# 「a bytes-like object is required, not 'tuple'」で例外を吐く。
# hakosim.py自体は変更せず、実際に使われるextract_xyz_from_point_cloud()側で
# tuple→bytes変換を挟むことで吸収する。
_original_extract_xyz_from_point_cloud = hakosim_lidar.LidarData.extract_xyz_from_point_cloud


def _extract_xyz_from_point_cloud_patched(point_cloud_bytes, total_data_bytes):
    if not isinstance(point_cloud_bytes, (bytes, bytearray)):
        point_cloud_bytes = bytes(point_cloud_bytes)
    return _original_extract_xyz_from_point_cloud(point_cloud_bytes, total_data_bytes)


hakosim_lidar.LidarData.extract_xyz_from_point_cloud = staticmethod(
    _extract_xyz_from_point_cloud_patched
)

AGL_PATH = "/tmp/hakoniwa_lidar_agl.txt"
POLL_INTERVAL_SEC = 0.05  # 20Hz（実機TFMiniBridgeのkPollingPeriodMsと合わせる）

# 機体直下を軸にした円錐の半頂角。センサFOV(±3°)内の点を取りこぼさない程度に取る。
# 中央値ベースなので広めでも地面が多数派なら問題ない（詳細は上部コメント参照）。
CONE_HALF_ANGLE_DEG = 3.5

# NEDのZ軸(下向き正)基準に直すための符号。実測で -1（点群のZは上向き正）。
DOWN_AXIS_SIGN = -1.0

# 直近この個数の“採用値”の移動中央値を出力する（5 * 0.05s = 0.25s）。
# 単発の外れフレームを時系列の多数決で潰すため。大きくするほど頑健だが遅延が増える。
MEDIAN_WINDOW = 5

# これ未満の値は測距不能とみなして採用しない（手前のごく近い点・ゼロ距離対策）。
MIN_PLAUSIBLE_AGL_M = 0.05

DEBUG = os.environ.get("HAKONIWA_LIDAR_DEBUG", "").strip().lower() not in ("", "0", "false", "no")

_stats_logged = False


def _fmt(v):
    return "  none" if v is None else f"{v:6.2f}"


def cone_agl_and_stats(points_flat):
    """points_flat: [x0,y0,z0,x1,y1,z1,...] のフラットなリスト。
    (agl_m or None, stats dict) を返す。agl_m は円錐内zの中央値。"""
    global _stats_logged

    tan_half_angle = math.tan(math.radians(CONE_HALF_ANGLE_DEG))

    n = len(points_flat) // 3
    downward = 0
    cone_zs = []
    z_all_lo = z_all_hi = None

    for i in range(n):
        x = points_flat[i * 3]
        y = points_flat[i * 3 + 1]
        z = points_flat[i * 3 + 2] * DOWN_AXIS_SIGN

        if z_all_lo is None or z < z_all_lo:
            z_all_lo = z
        if z_all_hi is None or z > z_all_hi:
            z_all_hi = z

        if z <= 0.0:
            continue  # 下向きでない点（真上・水平含む）は対象外
        downward += 1

        if math.hypot(x, y) <= z * tan_half_angle:
            cone_zs.append(z)

    agl = None
    if cone_zs:
        agl = statistics.median(cone_zs)
        if agl < MIN_PLAUSIBLE_AGL_M:
            agl = None

    if not _stats_logged and n > 0:
        _stats_logged = True
        print(f"[DEBUG] 初回点群: 点数={n}, z(down)全体範囲=[{z_all_lo:.2f}, {z_all_hi:.2f}], "
              f"円錐内={len(cone_zs)}点 → z_all_hi が対地高度とおおよそ一致するか確認"
              f"（違えば DOWN_AXIS_SIGN を反転）", flush=True)

    stats = {
        "total": n,
        "downward": downward,
        "cone": len(cone_zs),
        "cone_lo": min(cone_zs) if cone_zs else None,
        "cone_med": agl,
        "cone_hi": max(cone_zs) if cone_zs else None,
    }
    return agl, stats


def main():
    if len(sys.argv) < 2:
        print(f"Usage: {sys.argv[0]} <config_path> [vehicle_name]")
        return 1

    config_path = sys.argv[1]
    vehicle_name = sys.argv[2] if len(sys.argv) > 2 else "Drone"

    client = hakosim.MultirotorClient(config_path, vehicle_name)
    if not client.confirmConnection():
        print("ERROR: Hakoniwaへの接続に失敗しました。")
        return 1
    client.run_nowait()

    print(f"LiDAR AGL距離を {AGL_PATH} に書き出します（{POLL_INTERVAL_SEC}s間隔, "
          f"円錐半頂角={CONE_HALF_ANGLE_DEG}deg, 円錐内=中央値, "
          f"移動中央値窓={MEDIAN_WINDOW}）。Ctrl+Cで終了。"
          + ("  [DEBUG有効]" if DEBUG else ""))

    tmp_path = AGL_PATH + ".tmp"
    recent = collections.deque(maxlen=MEDIAN_WINDOW)
    was_valid = None  # None=初回, True/False=直近の出力有効性

    while True:
        start = time.time()

        lidar = client.getLidarData(vehicle_name=vehicle_name)
        frame_agl, stats = (None, {"total": 0})
        if lidar is not None and lidar.point_cloud:
            frame_agl, stats = cone_agl_and_stats(lidar.point_cloud)

        out_agl = None
        if frame_agl is not None:
            recent.append(frame_agl)
            out_agl = statistics.median(recent)
        else:
            # このtickは値が採れなかった。移動中央値の窓は敢えて減らさず、直前の
            # 採用値のまま維持する（数tickの取りこぼしで窓が空になり出力が途切れる
            # のを防ぐ）。長時間採れなければ下の「更新しない」でC++がstale無効化する。
            pass

        if out_agl is not None:
            with open(tmp_path, "w") as f:
                f.write(f"{out_agl:.4f}\n")
            os.replace(tmp_path, AGL_PATH)
        # out_agl is None: ファイルを更新しない（C++が mtime stale で無効化）

        now_valid = out_agl is not None
        if was_valid is not None and now_valid != was_valid:
            if now_valid:
                print(f"[LIDAR] 有効値を再取得: {out_agl:.2f}m", flush=True)
            else:
                print("[LIDAR] 有効値なし（AGLファイルの更新を停止。C++側はstaleで無効化）", flush=True)
        was_valid = now_valid

        if DEBUG:
            print(
                f"[LIDAR] pts={stats.get('total', 0):5d} down={stats.get('downward', 0):5d} "
                f"cone={stats.get('cone', 0):3d} "
                f"coneZ[min/med/max]=[{_fmt(stats.get('cone_lo'))}/"
                f"{_fmt(stats.get('cone_med'))}/{_fmt(stats.get('cone_hi'))}] "
                f"frame={_fmt(frame_agl)} out={_fmt(out_agl)}",
                flush=True,
            )

        elapsed = time.time() - start
        time.sleep(max(0.0, POLL_INTERVAL_SEC - elapsed))

    return 0


if __name__ == "__main__":
    sys.exit(main())
