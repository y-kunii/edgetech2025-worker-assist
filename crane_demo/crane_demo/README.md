# crane_plus_demo

demo向けに実装したコードです

## 準備（実機を使う場合）

![crane_plus](https://rt-net.github.io/images/crane-plus/CRANEV2-500x500.png)

### 1. CRANE+ V2本体をPCに接続する

CRANE+ V2本体をPCに接続します。
接続方法は製品マニュアルを参照してください。

**※CRANE+ V2本体が接触しないように、十分なスペースを確保してください。**

### 2. USB通信ポートの接続を確認する

USB通信ポートの設定については`crane_plus_control`の
[README](../../crane_plus_control/README.md)
を参照してください。

**正しく設定できていない場合、CRANE+ V2が動作しない、振動する、などの不安定な動きになるので注意してください**

### 3. move_groupとcontrollerを起動する

#### 標準のCRANE+ V2を使用する場合

次のコマンドでmove_group (`crane_plus_moveit_config`)と
controller (`crane_plus_control`)を起動します。

```sh
$ ros2 launch crane_plus_examples demo.launch.py port_name:=/dev/ttyUSB0
```



## 準備（Gazeboを使う場合）
=======
![crane_plus_ignition](https://rt-net.github.io/images/crane-plus/crane_plus_ignition.png)

### 1. move_groupとGazeboを起動する

次のコマンドでmove_group (`crane_plus_moveit_config`)とGazeboを起動します。

```sh
$ ros2 launch crane_plus_gazebo crane_plus_with_table.launch.py
```

## デモプログラムを実行する

準備ができたらサンプルプログラムを実行します。
例えばグリッパを開閉するサンプルは次のコマンドで実行できます。

```sh
$ ros2 launch crane_plus_demo demo.launch.py demo:='pick_and_place_with_pos'
```

終了するときは`Ctrl+c`を入力します。

## Gazeboでサンプルプログラムを実行する場合

Gazeboでサンプルプログラムを実行する場合は`use_sim_time`オプションを付けます。

```sh
$ ros2 launch crane_plus_demo demo.launch.py demo:='pick_and_place_with_pos' use_sim_time:=true
```

## Examples

`demo.launch.py`を実行している状態で各デモを実行できます。

- [pick_and_place](#pick_and_place)

実行できるサンプルの一覧は、`demo.launch.py`にオプション`-s`を付けて実行することで表示できます。

```sh
$ ros2 launch crane_plus_demo demo.launch.py -s
Arguments (pass arguments as '<name>:=<value>'):

    'example':
        Set an example executable name: [pick_and_place]
        (default: 'pick_and_place')
```

---

### pick_and_place_with_pos

モノを掴む・持ち上げる・運ぶ・置くコード例です。

次のコマンドを実行します。

```sh
$ ros2 launch crane_plus_demo demo.launch.py demo:='pick_and_place_with_pos' use_sim_time:=true
```

[back to example list](#examples)

---

### pick_and_place_with_posがsubscribeする値の確認
```sh
$ ros2 topic pub /pick_and_place_topic std_msgs/msg/String "{data: "motion1"}" --once
$ ros2 topic pub /pick_and_place_topic std_msgs/msg/String "{data: "motion2"}" --once
```
|    値   |                        動作                        |
| ------- | -------------------------------------------------- |
| motion1 | アームから見て右前方の物体を掴み、固定位置に置く   |
| motion2 | アームから見て中央前方の物体を掴み、固定位置に置く |
| motion3 | アームから見て左前方の物体を掴み、固定位置に置く   |
| motion4 | 固定位置の物体を掴み、アームから見て右前方に置く   |
| motion5 | 固定位置の物体を掴み、アームから見て中央前方に置く |
| motion6 | 固定位置の物体を掴み、アームから見て左前方に置く   |


### pick_and_place_with_posがpublishする値の確認
```sh
$ ros2 topic echo /operating_status_topic
$ ros2 topic echo /gripper_status_topic
```