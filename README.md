# autoware_trajectory_controller_py

Autoware の経路追従制御 (`autoware_trajectory_follower_node` 相当) を **ROS 2 なし** で
Python から実行するためのラッパーです。

- 横方向: **MPC** (`autoware_mpc_lateral_controller`) / **Pure Pursuit** (`autoware_pure_pursuit`)
- 縦方向: **PID** (`autoware_pid_longitudinal_controller`)

Autoware の C++ ソースは **一切改変せずに** そのままビルドしています。ROS 2 に依存する部分
(`rclcpp` の Node / Clock / Parameter / Logger / Publisher、メッセージ型、`tf2`、
`diagnostic_updater` など) だけを `shim/` の軽量な再実装で置き換え、pybind11 で Python に公開しています。

依存関係は `uv` だけで完結します (Eigen と OSQP はビルド時に CMake が自動取得します)。

## 必要なもの

- [uv](https://docs.astral.sh/uv/)
- C++17 コンパイラ (GCC 9+ / Clang 10+) と `git`
- 初回ビルド時のネットワーク接続 (PyPI と、Eigen / OSQP の取得のため)

ROS 2・Autoware のワークスペース・Boost・Eigen などのシステムパッケージは不要です。

## インストール

```bash
git clone https://github.com/hakuturu583/autoware_trajectory_controller_py
cd autoware_trajectory_controller_py
uv sync              # C++ 拡張のビルドを含め、全依存がインストールされます (初回 1〜2 分)
uv run pytest        # 動作確認
uv run python examples/closed_loop_simulation.py
```

別プロジェクトから使う場合:

```bash
uv add git+https://github.com/hakuturu583/autoware_trajectory_controller_py
```

## 使い方

```python
import numpy as np
import autoware_trajectory_controller as atc

# 参照軌道 (x, y, 速度)。yaw と time_from_start は省略すると自動計算されます。
x = np.arange(0.0, 200.0, 1.0)
y = 5.0 * np.sin(x / 20.0)
v = np.minimum(8.0, np.sqrt(np.maximum(0.0, x[-5] - x)))
trajectory = atc.make_trajectory(x, y, v)

# パラメータは Autoware のデフォルト (*.param.yaml) が使われ、必要なものだけ上書きできます
follower = atc.TrajectoryFollower(lateral="mpc", longitudinal="pid",
                                  parameters={"mpc_prediction_horizon": 40})

# 閉ループを回すための簡易車両モデル (自前のシミュレータに置き換えて構いません)
sim = atc.SimpleVehicleSimulator(dt=follower.ctrl_period)

for _ in range(1000):
    result = follower.control(
        trajectory=trajectory,
        odometry=sim.odometry(),           # nav_msgs/Odometry 相当
        steering=sim.steering_report(),    # SteeringReport もしくは float [rad]
        accel=sim.accel(),                 # AccelWithCovarianceStamped もしくは float [m/s^2]
        time=sim.time,                     # 制御器の時計 [s] (シミュレーション時刻)
    )
    if result:                             # result.is_published
        steer = result.control_cmd.lateral.steering_tire_angle
        acc = result.control_cmd.longitudinal.acceleration
    sim.step(result.control_cmd if result else None)
```

### 主な API

| API | 内容 |
| --- | --- |
| `TrajectoryFollower(lateral, longitudinal, parameters)` | `trajectory_follower_node` 相当。`control(...)` / `step(InputData)` で 1 制御周期を実行し `ControlResult` を返す |
| `LateralController(mode, parameters)` | 横方向制御器単体 (`"mpc"` / `"pure_pursuit"`)。`control(...)` → `LateralOutput` |
| `LongitudinalController(mode, parameters)` | 縦方向制御器単体 (`"pid"`)。`control(...)` → `LongitudinalOutput` |
| `atc.msg.*` | ROS 2 と同名・同フィールドのメッセージ型 (`Trajectory`, `Odometry`, `Control`, ...) |
| `default_parameters(lateral, longitudinal, overrides)` | Autoware のデフォルトパラメータ一式 (dict) |
| `load_param_file(path)` | ROS 2 形式のパラメータ YAML を読み込み (`ros__parameters` 以下をドット区切りでフラット化) |
| `make_trajectory / make_odometry / make_steering_report / make_accel / make_operation_mode` | 入力メッセージの生成ヘルパー |
| `SimpleVehicleSimulator` | 運動学的自転車モデル + 1 次遅れ・むだ時間の簡易シミュレータ |

制御器オブジェクト共通の機能:

- `time` / `time_ns`: 制御器の時計。**自動では進みません**。`control(time=...)` か `follower.time = t` で毎周期設定します (ROS の `use_sim_time` と同じ扱い)
- `get_parameters()` / `get_parameter(name)` / `set_parameters({...})`: 実行中のパラメータ変更 (`ros2 param set` と同様に各制御器のパラメータコールバックを経由します)
- `get_debug(topic)` / `get_debug_all()` / `debug_topics()`: 制御器が publish したデバッグトピックの最新値
  (例: `"~/output/predicted_trajectory"`, `"~/output/lateral_diagnostic"`)
- `update_diagnostics()`: `diagnostic_updater` のステータス一覧

ログ (`RCLCPP_*`) は標準エラー出力に出ます。`atc.set_log_level("warn")` でレベル変更、
`atc.route_logs_to_python_logging()` または `atc.set_log_handler(fn)` で Python 側に転送できます。

### 入力について

`control()` に渡す値は `trajectory_follower_node` の購読トピックに対応します。

| 引数 | 元のトピック | 省略時 |
| --- | --- | --- |
| `trajectory` | `~/input/reference_trajectory` | (必須) |
| `odometry` | `~/input/current_odometry` | (必須) |
| `steering` | `~/input/current_steering` | 0 rad |
| `accel` | `~/input/current_accel` | 0 m/s² |
| `operation_mode` | `~/input/current_operation_mode` | `True` (AUTONOMOUS + 制御有効) |

`operation_mode=False` のときは Autoware 本体と同様に、制御器は停止指令・初期値を出し続けます。

## ROS 2 版との違い・制限

- 時刻は手動駆動です (タイマーはありません)。1 周期 = `control()` 1 回の呼び出しです
- publisher は送信せず、トピックごとに最新メッセージを保持するだけです
- `tf` はありません。Pure Pursuit を含め、自己位置は `odometry` から与えます
- `trajectory_follower_node` の購読・タイムアウト監視 (`cyclic_message_timeout`) は Python 側の責務です。
  `timeout_thr_sec` による古い指令の破棄は `TrajectoryFollower.step()` に実装しています
- `fmt::format` / `tl::expected` / `boost::optional` は最小限の互換実装です (エラーメッセージの数値書式が若干異なる場合があります)

## 検証

上流の gtest をシム上でそのままビルド・実行できます (MPC / PID の 101 テストがパス)。

```bash
cmake -S . -B build/cpp -DAUTOWARE_TC_BUILD_PYTHON=OFF -DAUTOWARE_TC_BUILD_TESTS=ON
cmake --build build/cpp -j
ctest --test-dir build/cpp
```

Python 側のテストは `uv run pytest` です (閉ループでの追従・停止、パラメータ、デバッグ出力など)。

## ディレクトリ構成

```
third_party/          上流ソース (無改変)。取得元のコミットは third_party/UPSTREAM.md
shim/include, src/    ROS 2 / tf2 / boost 等の代替実装 (メッセージ型は scripts/gen_msgs.py で生成)
src/                  TrajectoryFollower (trajectory_follower_node 相当) と pybind11 バインディング
python/               Python パッケージ (パラメータ、ヘルパー、簡易シミュレータ)
scripts/              上流ソースの同期・メッセージ型生成スクリプト
examples/, tests/     サンプルとテスト
```

### 上流の更新

```bash
python scripts/sync_upstream.py --universe ../autoware_universe --core ../autoware_core --utils ../autoware_utils
python scripts/gen_msgs.py   # メッセージ型を追加・変更した場合
uv sync && uv run pytest
```

## ライセンス

同梱の Autoware ソースは Apache License 2.0 です (`third_party/*/LICENSE`)。
