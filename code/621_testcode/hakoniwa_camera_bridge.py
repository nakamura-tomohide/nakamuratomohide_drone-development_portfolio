#!/usr/bin/env python3
# Hakoniwaシミュレータのカメラ画像をimage_navigation_sim(C++)に橋渡しするスクリプト。
# hakoniwa-drone-core の hakosim.simGetImage() で画像を取得し続け、
# FRAME_PATH に保存する。C++側はこのファイルをcv::imreadで読むだけ。
#
# 使い方:
#   python3 hakoniwa_camera_bridge.py <config_path> [vehicle_name]
#
#   <config_path>: hakoniwa-drone-core の config/pdudef/*.json （実行中のシミュレーションと同じもの）
#   [vehicle_name]: 既定は "Drone"
#
# 事前準備（実行環境側。詳細はチームのセットアップ手順を参照）:
#   - hakopy(.so)がPython実行系と噛み合うビルドであること。
#     ズレていると「import hakopy」がsegfaultするか、シンボル不足でImportErrorになる。
#   - DYLD_LIBRARY_PATH / PYTHONPATH に、上記hakopyと対になるlibassets/libconductorの
#     ビルドディレクトリが（stale版より優先されるよう）通っていること。
#
# 注意: enableApiControl/armDisarmは呼ばない。
#       機体の制御はC++側(MAVSDK Offboard)が行うため、ここでは画像取得のみに専念する。

import sys
import os
import time

HAKONIWA_DRONE_API_DIR = os.environ.get(
    "HAKONIWA_DRONE_API_DIR",
    os.path.expanduser("~/project/hakoniwa-drone-core/drone_api"),
)
# hakosim.py 自体は <drone_api>/libs/hakosim.py にあるので import hakosim できるよう
# libs/ を、かつ hakosim.py 内部が `import libs.hakosim_types` のように drone_api を
# 起点にした絶対importをしているため drone_api 自体も、両方sys.pathに入れる必要がある。
sys.path.insert(0, os.path.join(HAKONIWA_DRONE_API_DIR, "libs"))
sys.path.insert(0, HAKONIWA_DRONE_API_DIR)

import hakosim  # noqa: E402  (sys.path調整後にimportする必要があるため)

FRAME_PATH = "/tmp/hakoniwa_frame.png"
CAPTURE_INTERVAL_SEC = 0.1  # 10Hz（C++側の撮影間隔200msより十分速く更新する）
CAMERA_ID = "0"


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
    # confirmConnection()直後に一度PduManagerを回しておかないと、
    # 以降のPDU書き込み(カメラコマンド等)がFailed to write PDU dataで失敗する
    client.run_nowait()

    # 注意: simSetCameraOrientation()はカメラのピッチ(上下の傾き)ではなく
    # ヨー(水平方向の向き)しか変更できない(Unity側 CameraMoveRequest の実装がangle.yを
    # SetCameraYawAngle()にしか渡していないため)。ピッチはUnity側の
    # DefaultCameraController.cs の manual_rotation_deg(private, デフォルト90度=ほぼ真下)
    # で固定されており、Python側からは制御できない。
    #
    # ここで設定しているのは、実機カメラの取り付けズレを再現するヨー角。
    # Config.hpp の vision_camera_mount_rotation_deg (時計回り135度) と一致させるため、
    # -135を送るとUnity側のrequest_yaw_degが135度になる
    # (SetCameraYawAngle()内で符号反転されるため: request_yaw_deg = -(送った値))。
    # ※ Config側の値を変えたら、この値も必ず同じ角度に合わせること（片方だけ変えると
    #   箱庭とC++側のpx→NED変換が食い違い、SITLだけ動く方向がズレる）。
    client.simSetCameraOrientation(CAMERA_ID, -135)

    print(f"カメラ画像を {FRAME_PATH} に書き出します（{CAPTURE_INTERVAL_SEC}s間隔）。Ctrl+Cで終了。")

    tmp_path = FRAME_PATH + ".tmp"
    while True:
        start = time.time()

        png_bytes = client.simGetImage(CAMERA_ID, hakosim.ImageType.Scene)
        if png_bytes:
            # 途中状態のファイルをC++側が読まないよう、一時ファイルに書いてからrenameする
            with open(tmp_path, "wb") as f:
                f.write(png_bytes)
            os.replace(tmp_path, FRAME_PATH)
        else:
            print("警告: カメラ画像の取得に失敗しました。")

        elapsed = time.time() - start
        time.sleep(max(0.0, CAPTURE_INTERVAL_SEC - elapsed))

    return 0


if __name__ == "__main__":
    sys.exit(main())
