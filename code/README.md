# 収録コードと自分の担当範囲

チームの開発リポジトリ [ARLISS-UTAT/Halcyon](https://github.com/ARLISS-UTAT/Halcyon) は非公開のため、そのなかで**自分（GitHub: nakamura-tomohide）が実装したコード**をここに移しています。
ディレクトリ構成は Halcyon の `src/` に合わせています。

- 「自分の行」は、Halcyon リポジトリで `git blame` をとり、最終的に残っている行のうち自分のコミットによる行を数えたものです（2026年9月時点）。
- チーム共通のヘッダ（`Config`、`SystemState` など）は含めていないため、このディレクトリ単体ではビルドできません。読むためのコードとして置いています。

## このディレクトリに収録したもの

| ファイル | 自分の行 / 全体 | 自分が実装した内容 |
|---|---|---|
| [`phase/VisionNavigate/VisionNavigate.cpp`](phase/VisionNavigate/VisionNavigate.cpp) | 683 / 683（100%） | 画像航法の本体。ゴール検出（HSV赤色マスク → モルフォロジー → 輪郭 → 楕円フィッティング）、ピクセルのずれを実距離に換算、比例制御、`Search` / `ImageNav` / `ForcedLand` の状態機械、到達判定 |
| [`phase/VisionNavigate/VisionNavigate.hpp`](phase/VisionNavigate/VisionNavigate.hpp) | 318 / 319 | 上記のインターフェースと設定項目 |
| [`image_recognition/`](image_recognition/)（8ファイル） | 940 / 940（100%） | 画像認識の試作。色・形状によるゴール認識、カメラでのゴール検出、画像航法のプロトタイプ、各種テスト |
| [`Lidar/TFMiniBridge.cpp`](Lidar/TFMiniBridge.cpp) | 332 / 338 | LiDAR（TFmini-S）のシリアル読み取り、無効値の除外、シミュレータからの読み込み。※198〜200行目と236〜238行目（近距離の無効値判定）はチームメンバーの追加 |
| [`Lidar/TFMiniBridge.hpp`](Lidar/TFMiniBridge.hpp) | 140 / 142 | 上記のインターフェース |
| [`phase/PhaseStateStore.cpp`](phase/PhaseStateStore.cpp), [`.hpp`](phase/PhaseStateStore.hpp) | 185 / 185（100%） | 今のフェーズをファイルに保存し、電源が一瞬切れても続きから再開する仕組み |
| [`DataLogger/generate_vision_navigate_report.py`](DataLogger/generate_vision_navigate_report.py) | 1,336 / 1,336（100%） | テレメトリログとコマンドログを突き合わせ、画像航法の検証レポート（図・相関・生ログ抜粋）を自動生成 |
| [`DataLogger/generate_land_detection_report.py`](DataLogger/generate_land_detection_report.py) | 654 / 654（100%） | 着地判定の検証レポートを自動生成 |
| [`621_testcode/hakoniwa_camera_bridge.py`](621_testcode/hakoniwa_camera_bridge.py) | 94 / 94（100%） | 箱庭シミュレータのカメラ画像を、実機と同じ画像航法コードに渡す |
| [`621_testcode/hakoniwa_lidar_bridge.py`](621_testcode/hakoniwa_lidar_bridge.py) | 246 / 246（100%） | 3D点群から機体直下の対地高度を計算し、実機の単一ビームLiDARを再現 |
| [`621_testcode/hakoniwa_wind_bridge.py`](621_testcode/hakoniwa_wind_bridge.py) と風プロファイル（JSON） | 239 / 239（100%） | シミュレータ内で風を吹かせる |

## 共同で書いたため収録していないもの

チームメンバーのコードと混ざっているファイルは、ここにはコピーしていません。自分の担当分だけ記載します。

| ファイル（Halcyon 内） | 自分の行 / 全体 | 自分が実装した内容 |
|---|---|---|
| `src/phase/VisionNavigate/VisionNavigateRunner.cpp`, `.hpp` | 461 / 613（約75%） | 画像処理スレッド（200 ms 周期で撮影 → 認識 → 航法計算）。最初の骨組みはチームメンバー、その後の改修の大部分を担当 |
| `src/phase/AltitudeAglGate.hpp` | 78 / 78（100%） | 対地高度の外れ値ゲート（変化が速すぎる値を捨て、続いた場合だけ受け入れる） |
| `src/phase/Phase.cpp` | 410 / 728（約56%） | フェーズごとの処理。離陸（Takeoff、194 / 238行）と GPS航法（GPSNavigate、155 / 204行）の大部分 |
| `src/phase/GPSNavigate/GPSNavigate.cpp` | 141 / 243（約58%） | GPS航法の加減速・到着判定・ヨー制御 |
| `src/Config/Config.hpp` | 311 / 401（約78%） | ミッション全体の調整用パラメータ（ゴール座標などを含むため非公開） |
| `src/main/main.cpp`, `main_test.cpp` | 247 / 859（約29%） | メインプログラム。フェーズ保存・電源断からの再開の組み込みなど |
| `src/621_testcode/visnavi.cpp` | 392 / 392（100%） | 画像航法の単体試験プログラム |
| `src/621_testcode/endurance.cpp` | 132 / 807（約16%） | バッテリーが減るまで飛び続ける耐久試験の改良 |
| `FC/params/vibration.json` | 211 / 211（100%） | 離陸時の振動対策用の PX4 パラメータ |

## ドキュメント

[`../docs/`](../docs/) に、自分が作成した検証レポートを置いています。

- [`vision_navigate_report/`](../docs/vision_navigate_report/)：画像航法の検証レポート。対象ログ `20260814_160631` は箱庭 + PX4 SITL によるシミュレーション飛行のもので、図中のカメラ画像もシミュレータの描画です（レポート本文中の「飛行」「実カメラ画像」はシミュレーション上のものを指します）。
- [`land_detection_report/`](../docs/land_detection_report/)：着地判定の検証レポート
