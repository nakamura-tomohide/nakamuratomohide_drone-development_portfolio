#!/usr/bin/env python3
# 箱庭シミュレータに「時間変化する風」(定常風＋周期的な突風＋乱流)を吹かせるスクリプト。
#
# カメラ/LiDARブリッジと同じ「SHMクライアントとしてPDUを読み書きするだけ」の構成。
# hako_env_event.py と違い hakopy でのアセット登録はしない（= venv も PYTHONPATH も不要、
# 起動順の制約もなし）。毎ループ Disturbance PDU (Drone/disturb ch, hako_msgs/Disturbance)
# を共有メモリへ書き、物理側 (drone_config_*.json の enable_disturbance:true) がそれを読む。
# hako_env_event.py がエリア(area.json)方式で「機体位置に応じた一定風」なのに対し、
# こちらは全域一律で「時間的にゆらぐ風」を与える。突風・乱流の耐性テスト用。
#
# --- 使い方（カメラブリッジと同じノリ。シミュ本体が起動したあとに別プロセスで並走）---
#   cd ~/github/Halcyon
#   python3 src/621_testcode/hakoniwa_wind_bridge.py \
#       ~/project/hakoniwa-drone-core/config/pdudef/drone-pdudef-1.json \
#       20 \
#       src/621_testcode/hakoniwa_wind_profile.json \
#       [robot_name]
#
#   arg1 : pdudef JSON（カメラ/LiDARブリッジに渡しているのと同じもの。
#          Drone/disturb チャンネルが drone-pdutypes.json 由来で含まれること）
#   arg2 : 送信間隔[ms]（20 で 50Hz。風は緩やかに変化するのでこれで十分）
#   arg3 : 風プロファイル JSON（hakoniwa_wind_profile.json 参照）
#   arg4 : 機体名（省略時 "Drone"。マルチ機体構成なら "Drone-1" 等）
#
# --- wind_profile.json（キーは全て任意。無ければ DEFAULTS） ---
#   {
#     "frame": "ros",                 # "ros"(x=前/北, y=左, z=上) / "ned"(x=北, y=東, z=下)。
#                                     #   ned は内部で箱庭ROS系へ変換して送る (x,y,z)->(x,-y,-z)
#     "base_wind":  [0.0, 3.0, 0.0],  # 定常風ベクトル [m/s]
#     "gust": {
#       "amplitude":  [0.0, 4.0, 0.0],# 突風のピーク付加ベクトル [m/s]（base_wind に加算）
#       "period_sec": 6.0,            # 突風の周期 [s]
#       "duty":       0.4             # 1周期のうち突風が立ち上がっている割合 (0 < duty <= 1)
#     },
#     "turbulence": {
#       "sigma":   [0.3, 0.3, 0.1],   # 乱流(OU過程)の定常標準偏差 [m/s]（軸別）
#       "tau_sec": 1.0                # 乱流の相関時間 [s]。大きいほどゆっくり揺れる
#     },
#     "ramp_up_sec":   2.0,           # 起動直後、風を 0 からこの秒数かけて線形に立ち上げる
#     "seed": 0,                      # 乱流の乱数シード
#     "print_every_sec": 1.0,         # 現在風ベクトルのログ出力間隔 [s]。0 で無効
#
#     # ↓ 通常は指定しないこと。指定すると箱庭の大気モデル(atmospheric_pressure)を
#     #   上書きし、シミュ既定(地上高度121.3m / ISA)とズレて気圧計・EKF垂直推定が
#     #   壊れる（2026-09-02: sea_level_atm=1.0 を送って baro が -9m ずれ、降下暴走）。
#     #   キーを書かなければ 0.0(=上書きしない) のまま。気温・気圧の外乱を本当に
#     #   試したいときだけ、値の意味を理解した上で設定する。
#     "temperature_c": null,          # 気温 [℃]（null / 未指定 で上書きしない）
#     "sea_level_atm": null           # 海面気圧 [atm]（null / 未指定 で上書きしない）
#   }
#
# --- 風モデル（毎ループ、経過時刻 t） ---
#   w(t) = ramp(t) * ( base_wind + gust_env(t) * gust.amplitude + turb(t) )
#     ramp(t)     : min(1, t / ramp_up_sec)
#     gust_env(t) : 周期 period_sec のうち先頭 duty 区間だけ 0->1->0 のレイズドコサイン、他は 0
#     turb(t)     : 軸別 Ornstein-Uhlenbeck 過程（定常std=sigma, 相関時間=tau_sec）

import json
import math
import random
import sys
import time

# hakosim.confirmConnection() と同じ手順で箱庭共有メモリに外部プロセスとして接続する。
# hakopy は環境変数 (DYLD_LIBRARY_PATH / PYTHONPATH。~/.zshrc で設定済み) 経由で import
# される。カメラ/LiDARブリッジが動く対話シェルならそのまま通る。
import hakopy
from hakoniwa_pdu.pdu_manager import PduManager
from hakoniwa_pdu.impl.shm_communication_service import ShmCommunicationService
from hakoniwa_pdu.pdu_msgs.hako_msgs.pdu_pytype_Disturbance import Disturbance
from hakoniwa_pdu.pdu_msgs.hako_msgs.pdu_conv_Disturbance import py_to_pdu_Disturbance


DEFAULTS = {
    "frame": "ros",
    "base_wind": [0.0, 0.0, 0.0],
    "gust": {"amplitude": [0.0, 0.0, 0.0], "period_sec": 6.0, "duty": 0.4},
    "turbulence": {"sigma": [0.0, 0.0, 0.0], "tau_sec": 1.0},
    "ramp_up_sec": 2.0,
    "seed": 0,
    "print_every_sec": 1.0,
    # 温度・気圧はデフォルトで送らない(0.0=上書きしない)。非ゼロを送ると箱庭の
    # 大気モデルを上書きして気圧計・EKF垂直推定が壊れる。profile で明示指定した
    # ときだけ送る。
    "temperature_c": None,
    "sea_level_atm": None,
}


def _merge(base, override):
    out = dict(base)
    for k, v in (override or {}).items():
        if isinstance(v, dict) and isinstance(out.get(k), dict):
            out[k] = _merge(out[k], v)
        else:
            out[k] = v
    return out


class WindModel:
    def __init__(self, profile, dt_s):
        self.p = profile
        self.dt = dt_s
        self.t = 0.0
        self.rng = random.Random(profile.get("seed", 0))
        self._turb = [0.0, 0.0, 0.0]
        tau = max(1e-3, float(profile["turbulence"]["tau_sec"]))
        # OU過程の離散係数: turb[k] = a*turb[k-1] + b*sigma*N(0,1)  (定常std=sigma)
        self._a = math.exp(-dt_s / tau)
        self._b = math.sqrt(max(0.0, 1.0 - self._a * self._a))

    def _gust_envelope(self):
        g = self.p["gust"]
        period = max(1e-3, float(g["period_sec"]))
        duty = min(1.0, max(1e-3, float(g["duty"])))
        u = (self.t % period) / period
        if u >= duty:
            return 0.0
        # 立ち上がり -> ピーク -> 立ち下がり のレイズドコサイン (u=0 と u=duty で 0)
        return 0.5 * (1.0 - math.cos(2.0 * math.pi * (u / duty)))

    def step(self):
        p = self.p
        env = self._gust_envelope()
        sigma = p["turbulence"]["sigma"]
        base = p["base_wind"]
        amp = p["gust"]["amplitude"]
        w = [0.0, 0.0, 0.0]
        for i in range(3):
            self._turb[i] = self._a * self._turb[i] + self._b * float(sigma[i]) * self.rng.gauss(0.0, 1.0)
            w[i] = float(base[i]) + env * float(amp[i]) + self._turb[i]

        r = float(p.get("ramp_up_sec", 0.0))
        ramp = min(1.0, self.t / r) if r > 0.0 else 1.0
        w = [c * ramp for c in w]

        self.t += self.dt

        if str(p.get("frame", "ros")).lower() == "ned":
            # NED(x=北, y=東, z=下) -> 箱庭ROS(x=北/前, y=左=西, z=上)
            w = [w[0], -w[1], -w[2]]
        return w


def main():
    if len(sys.argv) < 4:
        print(f"Usage: {sys.argv[0]} <pdudef.json> <delta_time_msec> <wind_profile.json> [robot_name]")
        return 1

    config_path = sys.argv[1]
    dt_s = int(sys.argv[2]) / 1000.0
    with open(sys.argv[3], "r") as f:
        profile = _merge(DEFAULTS, json.load(f))
    robot_name = sys.argv[4] if len(sys.argv) >= 5 else "Drone"

    pdu_manager = PduManager()
    pdu_manager.initialize(config_path=config_path, comm_service=ShmCommunicationService())
    pdu_manager.start_service_nowait()
    # hakosim.confirmConnection() と同じ: これを呼ばないと SHM read/write が
    # "not initialized" (error 22) になる。
    if hakopy.init_for_external() is False:
        print("ERROR: hakopy.init_for_external() failed. シミュ本体は起動済みか？ hakopy の環境変数は？")
        return 1

    # 接続確立を数回まわして待つ（start_service_nowait は非同期のため）
    for _ in range(50):
        pdu_manager.run_nowait()
        time.sleep(0.02)

    model = WindModel(profile, dt_s)

    print(
        "INFO: START gust wind  "
        f"robot={robot_name}  dt={dt_s * 1000:.0f}ms  frame={profile['frame']}  "
        f"base={profile['base_wind']}  gust_amp={profile['gust']['amplitude']}  "
        f"period={profile['gust']['period_sec']}s duty={profile['gust']['duty']}  "
        f"turb_sigma={profile['turbulence']['sigma']}",
        flush=True,
    )

    print_every = float(profile.get("print_every_sec", 0.0))
    last_print = -1e9
    # None のときは送らない（d_temp/d_atm は 0.0 のまま = 箱庭の大気モデルを上書きしない）。
    temp_c = profile.get("temperature_c", None)
    atm = profile.get("sea_level_atm", None)
    if temp_c is not None or atm is not None:
        print(f"WARN: 大気外乱を上書きします (temperature_c={temp_c}, sea_level_atm={atm})。"
              f"気圧計・EKF垂直推定がシミュ既定とズレることがあります。", flush=True)

    try:
        while True:
            pdu_manager.run_nowait()

            wx, wy, wz = model.step()

            d = Disturbance()
            d.d_wind.value.x = wx
            d.d_wind.value.y = wy
            d.d_wind.value.z = wz
            if temp_c is not None:
                d.d_temp.value = float(temp_c)
            if atm is not None:
                d.d_atm.sea_level_atm = float(atm)

            raw = py_to_pdu_Disturbance(d)
            if not pdu_manager.flush_pdu_raw_data_nowait(robot_name, "disturb", raw):
                print("ERROR: disturb PDU の書き込みに失敗 (robot_name / pdudef の disturb チャンネルを確認)", flush=True)

            if print_every > 0.0 and (model.t - last_print) >= print_every:
                last_print = model.t
                print(f"[WIND] t={model.t:6.1f}s  w=({wx:+.2f}, {wy:+.2f}, {wz:+.2f}) m/s (箱庭ROS系)", flush=True)

            time.sleep(dt_s)
    except KeyboardInterrupt:
        print("\nINFO: stop (Ctrl+C)")
        return 0


if __name__ == "__main__":
    sys.exit(main())
