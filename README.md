# nakamuratomohide_drone-development_portfolio
模擬人工衛星開発の概要をリポジトリにまとめています。

- GitHub ID: [nakamura-tomohide](https://github.com/nakamura-tomohide)
- 開発リポジトリ: [ARLISS-UTAT/Halcyon](https://github.com/ARLISS-UTAT/Halcyon)（チームのプライベートリポジトリのため、本リポジトリに概要をまとめています。）

---

## 1. 経験

### プロジェクト概要：ARLISS 2026 チーム Halcyon（東京大学 船瀬・五十里研究室）

| 項目 | 内容 |
|---|---|
| 期間 | 2026年4月 〜 2026年9月 |
| 内容 | ロケットで上空まで運ばれ放出されるドローン型CanSat（模擬人工衛星）が、着地後にGPSをもとに自律飛行し、最後にゴールを画像認識によって認識し、ゴールする。 |
| チーム | 全9名 うち制御担当は4名 |
| 自分の役割 | 機体の飛行ソフトウェア（C++）担当 |

**ミッションの流れ（フェーズの状態遷移）**

```
Standby → Store → Eject（放出判定） → LandDetection（着地判定） → Deployment（機体展開）
        → Takeoff → GPSNavigate → VisionNavigate（画像航法） → Land → Terminate
```

**構成**

| 区分 | 使ったもの |
|---|---|
| 搭載計算機 | Raspberry Pi Zero 2 W |
| フライトコントローラ | PX4（Kakute H7）。MAVSDK を使い、オフボード制御で速度・位置を指令 |
| センサ | GPS、気圧計、IMU、LiDAR（TFmini-S）、下向きカメラ（RaspberryPi CameraModule3Wide） |
| 通信 | LoRaでダウンリンク、補助マイコン XIAO RP2040 |
| 言語・ツール | C++、OpenCV、MAVSDK、pigpio、CMake（aarch64 向けクロスコンパイル）、Docker（devcontainer）、Python、TypeScript / Three.js（地上局） |

### 主な経験

#### (1) 画像認識によるゴール検出と画像航法（VisionNavigate）

下向きカメラの画像だけで赤いゴールマーカーを見つけ、そのゴールに向かって自律で降下する機能を実装しました。

- **画像認識**：OpenCV で HSV 色空間の赤色マスク → モルフォロジー処理でノイズ除去 → 輪郭抽出 → 面積・形状フィルタ → 楕円フィッティング、の順でゴール候補を検出。複数のマーカーが見えた場合は、まとめて中心を求める処理も入れています。
- **画像から制御へのつなぎ**：画像中心からゴールまでのずれ（ピクセル）を、LiDAR の対地高度とカメラの画角（102°）からピンホールカメラモデルで実距離（m）に換算。カメラの取り付け角（135°）の回転補正もかけています。
- **制御則**：ずれに比例した水平速度指令（不感帯・速度上限つき）。ずれが大きい間は水平補正を優先し、ずれが小さくなるほど降下を速める。ずれの値は指数移動平均（EMA）でならしています。
- **状態機械**：`Search`（半径を少しずつ広げるウェイポイントでゴールを探す）→ `ImageNav`（見つけたら画像で誘導）→ 一定時間見失うと `Search` に戻る。探索が時間切れのときは GPS 座標へ強制的に向かう `ForcedLand` に切り替えるフェイルセーフ付き。
- **並列処理**：200 ms 周期の専用スレッドで「撮影 → 認識 → 航法計算 → 共有状態へ書き込み」を回し、50 Hz の制御ループとは分けて動かしています。

#### (2) 実機ログによる検証レポートの自動生成

「機体が画像認識の結果をもとに自分で動いていた」ことを、ログから客観的に示すための解析スクリプト（Python、約1,300行）を作りました。

- 10 Hz で記録したテレメトリログ（機体の状態と画像認識の内部状態）とコマンドログ（実際にフライトコントローラへ送った指令）を、時刻で突き合わせて解析。
- グラフ・カメラ画像・生ログの抜粋を含む Markdown / PDF のレポートをコマンド一つで作れるようにしました。着地判定（LandDetection）についても同じ形のレポートを作成しています（[docs/](docs/)）。

#### (3) センサの外れ値処理と高度推定

ノイズや外れ値の多いセンサの値から、制御に使える高度を作る処理を担当しました。

- LiDAR の値は直近5回の中央値を採用し、1秒あたりの変化量が大きすぎる値は異常値として捨てる（ただし一定回数続いたら受け入れる）ゲート処理を実装。
- LiDAR と気圧計の高度がずれすぎたときの切り替え（フォールバック）を設計。
- 離陸時の振動対策として、PX4 のフィルタパラメータを調整（パラメータを JSON で管理し、ツールで一括書き込み）。

#### (4) シミュレータ連携（箱庭 / PX4 SITL）

実機を飛ばす前に試せるよう、シミュレータと飛行ソフトをつなぐブリッジを Python で書きました。

- **カメラ**：シミュレータ内のカメラ画像を、実機と同じ画像航法のコードに流す。
- **LiDAR**：3D点群から機体の真下にある細い円錐内の点だけを取り出し、実機の単一ビームLiDARに近い値を計算。点群が疎で値が飛ぶ問題（高度が約1 m ⇔ 約4 m を行き来して機体が上下に揺れる）を、ログから原因を追って対策しました。
- **風**：風のプロファイルを JSON で与え、風がある中での航法を試験。

#### (5) 飛行ソフト全体の設計・統合

- フェーズの状態遷移（`Phase`）と、調整用パラメータを一か所にまとめた設定（`Config`）の設計・保守。
- 電源が一瞬切れても続きから再開できるよう、今のフェーズを保存して復帰する仕組みを実装。
- LiDAR ドライバ（`TFMiniBridge`）、GPS 航法（加減速・到着判定・ヨー制御）、気圧計による高度制御、バッテリーが減るまで飛び続ける耐久試験のコード。
- 各メンバーのブランチを `integrate` ブランチに統合する役割。

### AIツールを使った開発

- Claude Code（AIコーディングエージェント）を日常的に使って開発しました。PX4 や MAVSDK のソースを読んで仕様を確かめる作業、ログ解析・レポート生成スクリプトの作成などに活用しています。

### 講義・独学での学習経験

<!-- TODO: 受講した講義（機械学習・画像処理・制御工学など）、独学した書籍・オンライン講座、Kaggle 等があれば追記 -->

### 開発環境とビルドの流れ

搭載計算機の Raspberry Pi Zero 2 W は非力なので、PC 上でクロスコンパイルしてからバイナリを Raspberry Pi に送る流れで開発しました。
この環境はチームメンバーが作ったもので、自分はそれを使って開発しています（自分が作ったのは下の「シミュレータでの試験」のつなぎ込み部分です）。

```
 開発PC（Docker / VS Code devcontainer, Ubuntu 22.04）
   ├ aarch64-linux-gnu-g++（クロスコンパイラ）
   ├ OpenCV 4.10.0（aarch64 向けにソースからビルド）
   ├ MAVSDK 3.17.1（arm64 版）/ pigpio（ソースからビルド）
   └ scripts/build-raspi.sh
       └ cmake -DCMAKE_TOOLCHAIN_FILE=cmake/aarch64-linux-gnu.cmake
           → bin/raspi/main（aarch64 バイナリ）
                │
                │ rsync / scp
                ▼
 Raspberry Pi Zero 2 W（Raspberry Pi OS Lite 64-bit）
   ~/Halcyon/bin/raspi/main を実行
```

| 項目 | 内容 |
|---|---|
| ビルド | CMake（C++20）。`CMAKE_CROSSCOMPILING` で出力先を `bin/raspi/`（実機用）と `bin/local/`（PC用）に分ける |
| ツールチェーン | `cmake/aarch64-linux-gnu.cmake`。sysroot は `/usr/aarch64-linux-gnu`、ライブラリとヘッダはターゲット側だけを探す設定 |
| Docker | OpenCV のクロスビルドに時間がかかるため、OpenCV 入りのベースイメージ（`Dockerfile.base`）を一度だけ作り、その上に開発用イメージを重ねる二段構成 |
| PC でのビルド | `scripts/build-local.sh`。シミュレータ試験用に、同じコードを PC 向けにもビルドできる |
| Raspberry Pi の準備 | `Raspi/setup.sh` で UART・I2C・カメラの有効化、Wi-Fi の省電力無効化などを設定。飛ばす前に `preflight_check.sh` でセンサや接続を確認 |
| 補助マイコン | XIAO RP2040 は PlatformIO でビルド |

**シミュレータでの試験**

```
 箱庭シミュレータ + PX4 SITL
   ├ カメラ画像 ─▶ hakoniwa_camera_bridge.py ─▶ 画像ファイル ─▶ VisionNavigateRunner が読み込み
   ├ 3D LiDAR 点群 ─▶ hakoniwa_lidar_bridge.py ─▶ 高度ファイル ─▶ TFMiniBridge が読み込み
   └ 風 ◀─ hakoniwa_wind_bridge.py（共有メモリに外乱を書き込み）◀─ 風プロファイル（JSON）

 飛行プログラム（実機と同じコード）─ MAVSDK（オフボード制御）─▶ PX4 SITL
```

実機のセンサを読むクラス（`TFMiniBridge` など）に、シミュレータが書き出したファイルを読む経路を足しました。これで、飛行プログラムを書き換えずに、実機と同じコードのまま PC 上で試験できます。

### ディレクトリ構成（Halcyonリポジトリ）

★ は自分が主に実装した部分、☆ は一部を担当した部分です（詳しくは [code/README.md](code/README.md)）。

```text
Halcyon/
├── CMakeLists.txt          # 親 CMake。各モジュールを add_subdirectory で読み込む
├── cmake/
│   └── aarch64-linux-gnu.cmake   # Raspberry Pi 向けクロスコンパイル設定
├── scripts/
│   ├── build-local.sh      # PC 向けビルド
│   └── build-raspi.sh      # Raspberry Pi 向けクロスビルド
├── .devcontainer/          # Docker 開発環境（Dockerfile.base: OpenCV 入りベースイメージ）
├── src/                    # 機体の飛行プログラム（C++）
│   ├── main/               # ☆ メインプログラム（main.cpp / main_test.cpp）
│   ├── phase/              # フェーズごとの処理と状態遷移
│   │   ├── Phase.cpp       # ☆ 各フェーズの処理（Takeoff・GPSNavigate の大部分）
│   │   ├── PhaseStateStore.*     # ★ フェーズ保存と電源断からの再開
│   │   ├── AltitudeAglGate.hpp   # ★ 対地高度の外れ値ゲート
│   │   ├── VisionNavigate/ # ★ 画像航法（Runner は☆）
│   │   ├── GPSNavigate/    # ☆ GPS航法
│   │   ├── Eject/  Land/  Store/ # 放出判定・着地判定・格納
│   ├── image_recognition/  # ★ 画像認識の試作
│   ├── Lidar/              # ★ LiDAR（TFmini-S）ドライバ
│   ├── FCBridge/           # PX4 との通信（MAVSDK）
│   ├── Config/             # ☆ 調整用パラメータ
│   ├── SystemState/        # スレッド間で共有する機体状態
│   ├── DataLogger/         # ログ記録（★ 検証レポート生成スクリプト）
│   ├── Downlink/           # LoRa ダウンリンク
│   ├── RP2040Bridge/       # 補助マイコンとの通信
│   ├── EmergencyStop/      # 緊急停止
│   └── 621_testcode/       # 単体試験コード（★ 箱庭ブリッジ、visnavi.cpp）
├── FC/                     # PX4 パラメータ（JSON）と一括書き込みツール（☆ vibration.json）
├── XIAO/                   # 補助マイコン XIAO RP2040 のファームウェア（PlatformIO）
├── Raspi/                  # Raspberry Pi のセットアップ・飛行前チェック
├── groundstation/          # 地上局（Python + TypeScript / Three.js）
├── docs/                   # 技術資料・検証レポート（★ 画像航法・着地判定レポート）
├── Reports/                # 飛行試験ごとの制御レポート
└── logs/                   # 飛行ログ（テレメトリ・コマンド）
```

---

## 2. ポートフォリオ / GitHub

- GitHub: [nakamura-tomohide](https://github.com/nakamura-tomohide)
- 本リポジトリ: ARLISS 2026 チーム Halcyon での開発内容のまとめ
- 開発リポジトリ: [ARLISS-UTAT/Halcyon](https://github.com/ARLISS-UTAT/Halcyon)（非公開リポジトリです。）

### 自分が実装したコード

Halcyonリポジトリは非公開のため、自分が実装したコードを [`code/`](code/) に移しています。
**ファイルごとの自分の担当範囲（`git blame` による行数と実装内容）は [code/README.md](code/README.md) にまとめています。**

| パス | 内容 | 自分の行 / 全体 |
|---|---|---|
| [`code/phase/VisionNavigate/`](code/phase/VisionNavigate/) | 画像認識によるゴール検出と画像航法（`VisionNavigate.cpp` が本体） | 1,001 / 1,002 |
| [`code/image_recognition/`](code/image_recognition/) | 画像認識の試作・テストコード | 940 / 940 |
| [`code/Lidar/`](code/Lidar/) | LiDAR（TFmini-S）ドライバと、シミュレータからの読み込み | 472 / 480 |
| [`code/phase/`](code/phase/)（`PhaseStateStore`） | 電源断から復帰するためのフェーズ保存 | 185 / 185 |
| [`code/DataLogger/`](code/DataLogger/) | 画像航法・着地判定の検証レポート自動生成スクリプト | 1,990 / 1,990 |
| [`code/621_testcode/`](code/621_testcode/) | 箱庭シミュレータとのブリッジ（カメラ・LiDAR・風）と風プロファイル | 579 / 579 |
| [`docs/`](docs/) | 画像航法・着地判定の検証レポート（自動生成） | — |

チームメンバーと共同で書いたファイル（`Phase.cpp`、`GPSNavigate.cpp`、`VisionNavigateRunner.cpp` など）はコピーしていません。自分の担当分だけを [code/README.md](code/README.md) に書いています。
チーム共通のヘッダ（`Config`、`SystemState` など）を含めていないため、このリポジトリ単体ではビルドできません。読むためのコードとして置いています。
