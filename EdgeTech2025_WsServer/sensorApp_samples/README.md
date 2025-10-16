# センサーデータ連携アプリ サンプル実装

このディレクトリには、Raspberry Pi WebSocketサーバーと連携するセンサーデータ取得アプリケーションのサンプル実装が含まれています。

## ファイル構成

```
samples/
├── README.md                           # このファイル
├── SENSOR_APP_SPECIFICATION.md         # センサーアプリ開発仕様書
├── javascript/                         # JavaScript実装例
│   ├── basic-sensor-app.js             # 基本的なセンサーアプリ
│   ├── mock-sensor-app.js              # モックデータ送信アプリ
│   └── package.json                    # 依存関係
├── python/                             # Python実装例
│   ├── basic_sensor_app.py             # 基本的なセンサーアプリ
│   ├── integrated_sensor_app.py        # 統合センサーアプリ
│   ├── sensors/                        # センサー個別実装
│   │   ├── camera_sensor.py            # カメラセンサー
│   │   ├── vibration_sensor.py         # 振動センサー
│   │   └── torque_sensor.py            # トルクセンサー
│   └── requirements.txt                # Python依存関係
└── cpp/                                # C++実装例
    ├── sensor_app.cpp                  # C++センサーアプリ
    ├── CMakeLists.txt                  # CMakeビルド設定
    └── README.md                       # C++実装の説明
```

## クイックスタート

### JavaScript版

```bash
cd samples/javascript
npm install
node basic-sensor-app.js
```

### Python版

```bash
cd samples/python
pip install -r requirements.txt
python basic_sensor_app.py
```

### C++版

```bash
cd samples/cpp
mkdir build && cd build
cmake ..
make
./sensor_app
```

## 開発者向けドキュメント

詳細な仕様と実装ガイドは [SENSOR_APP_SPECIFICATION.md](SENSOR_APP_SPECIFICATION.md) を参照してください。

## サポート

- WebSocketサーバーの仕様: [../README.md](../README.md)
- 型定義: [../src/types/index.ts](../src/types/index.ts)
- テスト例: [../tests/](../tests/)