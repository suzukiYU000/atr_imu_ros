# atr_imu_ros

ATR-Promotions **AMWS020**をLinuxからBluetooth SPPで読み出す、複数IMU対応のROSセンサドライバーです。ペアリング済み機器へ自動接続し、加速度・角速度を `sensor_msgs/Imu` で配信します。メーカー公式ドライバーではありません。

| ブランチ | 対象 | ビルド |
|---|---|---|
| `ros2-jazzy`（このブランチ） | ROS 2 Jazzy / Python 3.12 | colcon / ament_python |
| `ros1-noetic` | ROS 1 Noetic / Python 3.8 | catkin |

AMWS020Bの通信仕様を対象とします。TSND121/TSND151への対応は主張しません。

## 機能

- OSで一度ペアリングしたAMWS020を自動検出。2台以上でも機器ごとに独立した接続・再接続処理
- 明示的な機器一覧、役割名、`frame_id` の指定も可能
- 加速度m/s²・角速度rad/sで配信。推定していない姿勢は `orientation_covariance[0] = -1`
- 通信のBCC検査、24 bit符号付き値の復元、センサ時計の日付またぎ処理
- `/diagnostics` に機器別の状態・受信数・再接続回数・時刻間隔異常・電池情報を配信
- 正常終了時に計測停止コマンドを送信し、停止状態を確認

## セットアップ

LinuxのBluetoothアダプターとBlueZが必要です。Bluetooth設定画面でセンサを一度ペアリングし、十分に充電して電源を入れてください。**ペアリング操作や電源投入自体は自動化しません。** 自動検出はBlueZに登録済み・ペアリング済みの `AMWS020_*` が対象です。未登録機器の探索はOS側で行います。

```sh
mkdir -p ~/imu_ws/src
cd ~/imu_ws/src
git clone -b ros2-jazzy https://github.com/suzukiYU000/atr_imu_ros.git
cd ~/imu_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select atr_imu_ros
source install/setup.bash
ros2 launch atr_imu_ros imu.launch.py
```

追加Python依存はディストリビューションの `python3-dbus` と `python3-yaml` です。Bluetooth用にBlueZもインストールしてください。rootでの常用は不要です。

## トピック

自動検出時の機器IDは `imu_<シリアル番号の小文字>` です。以下の `base` は設定で割り当てた役割名の例です。

| トピック | 型 | 内容 |
|---|---|---|
| `/imu/base/data_raw` | `sensor_msgs/msg/Imu` | 加速度・角速度 |
| `/imu/arm/data_raw` | `sensor_msgs/msg/Imu` | 別の機器の加速度・角速度 |
| `/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | 1 Hzの機器別診断 |

ROS 2はSensor Data QoS（best effort、volatile、depth 5）です。名前空間を付ける場合、上記の相対トピックはその配下になります。

```sh
ros2 topic list -t
ros2 topic echo /imu/base/data_raw --qos-reliability best_effort
ros2 topic hz /imu/base/data_raw
ros2 topic echo /diagnostics
ros2 bag record /imu/base/data_raw /imu/arm/data_raw /diagnostics
```

TFは配信しません。取り付け位置・軸の向きに合うTFは利用側で設定してください。値はセンサ本体の座標軸のままです。加速度には重力を含みます。姿勢推定、重力除去、軸の補正を行いません。加速度・角速度の共分散は不明を表す全0です。

## 複数台の設定

`config/sensors.yaml` をコピーし、自分のアドレスを指定します。

```yaml
auto_discover: false
sample_period_ms: 10
sensors:
  - address: "AA:BB:CC:DD:EE:01"
    name: base
    frame_id: base_imu_link
  - address: "AA:BB:CC:DD:EE:02"
    name: arm
    frame_id: arm_imu_link
```

```sh
ros2 launch atr_imu_ros imu.launch.py config_file:=/absolute/path/to/sensors.yaml
```

`auto_discover: true` と明示設定を併用すると、明示機器の名前を優先し、その他のペアリング済みAMWS020も追加します。特定機器だけを使う場合は `false` にしてください。センサ名とframe_idは重複させないでください。

| 設定 | 初期値 | 説明 |
|---|---|---|
| `auto_discover` | `true` | ペアリング済みAMWS020の自動追加 |
| `adapter` | `hci0` | BlueZ登録機器の検索対象。RFCOMM送信経路はOSが選択 |
| `sample_period_ms` | `10` | 1〜255 ms。10 ms = 100 Hz |
| `rfcomm_channel` | `2` | AMWS020Bで確認したSPPチャネル |
| `read_timeout` | `5.0` | 通信・データ待機タイムアウト秒 |
| `reconnect_delay` | `3.0` | 接続失敗後の再試行間隔秒 |
| `discovery_interval` | `5.0` | 登録機器一覧を見直す間隔秒 |
| `topic_prefix` | `imu` | データトピックの接頭辞 |
| `sensors` | `[]` | 明示接続する機器リスト |

設定は起動時に読み込みます。変更後はノードを再起動してください。実行中の動的パラメーター変更には対応していません。

## 時刻・機器の設定

各接続の最初のサンプルをホストROS時刻に対応付け、それ以降はセンサのミリ秒カウンターから時刻を算出します。Bluetoothの到着間隔のばらつきをサンプル間隔へ反映させません。ただし、初回転送遅延と各センサの時計ドリフトは補償しないため、**複数IMUの厳密な同期は保証しません**。再接続すると対応付けをやり直します。実機では `use_sim_time=false` を使用してください。実行中のホスト時計の大きな変更は想定していません。

センサのRTC・レンジ・校正値は変更しません。加速度角速度の計測設定は指定周期、平均1回、内部記録なしへ変更し、その設定は本体に残ります。開始モード100により内部メモリ記録を抑止し、既存記録は消去しません。既に計測中の機器は奪わずエラー表示します。元の計測設定は診断の `original_settings` に残ります。

電池情報は接続時の値です。電池低下で機器が停止した場合は診断エラーとなり、再試行間隔を最低30秒にします。未対応イベントのバイトは読み飛ばして `discarded_bytes` に計上します。停止応答が確認できない場合は `stop_error` を診断に残します。SIGKILLや電源断では停止処理を保証できません。

## 接続できないとき

1. 十分に充電し、本体をBluetooth待機状態にします。赤点滅・終了コード3は電池残量低下です。
2. 同じセンサを使うGUIや別ドライバーを停止します。
3. OS側で「接続済み」でもコマンド応答がない場合は、その機器だけ一度切断してから本ノードで接続します。ノードは他の接続を強制切断しません。
4. USB通信とBluetooth通信は排他です。BlueZ、ペアリング状態、DBusアクセスを確認してください。

同一ユーザーの複数ノードが同じ機器を開かないようローカルロックを使用します。別ユーザーや別コンテナーとの排他が必要な場合は `ATR_IMU_LOCK_DIR` に共有ディレクトリーを指定してください。別のアプリとの排他は保証しません。

DockerではBluetoothソケット用のホストネットワークと、ホストBlueZのシステムDBusへのアクセスが必要です。既存のROS環境に合わせて接続してください。`--privileged` を前提にはしていません。

## テスト

```sh
python3 -m unittest discover -s test -v
python3 test/ros2_smoke.py
```

後者は合成データで2つの実ROSトピックを購読する検証です。機器を操作しません。ビルド・テスト結果と実機確認の範囲は [VALIDATION.md](VALIDATION.md) を参照してください。

## 仕様・権利

- [AMWS020コマンド仕様 v1.03](https://www.atr-p.com/products/AMWS020/manuals/amws020_command_manual_v103.pdf)
- [AMWS020ユーザーマニュアル v1.04](https://www.atr-p.com/products/AMWS020/manuals/amws020_users_manual_v104.pdf)
- ROS `sensor_msgs/Imu` の単位・共分散規約に従います。

実測データ、個別の機器アドレス、メーカーPDFは含めません。再利用ライセンスは未設定です（package.xmlでは `Proprietary` と表記）。
