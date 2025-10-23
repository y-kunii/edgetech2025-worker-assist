# Requirements Document

## Introduction

工場現場向けデジタルツインデモプログラムのクライアントアプリケーション。エッジ端末からWSサーバー経由で受信する現場状態データをリアルタイムで表示し、履歴を蓄積・可視化するダッシュボードを提供する。また、ロボットへの動作指示をWSサーバー経由で送信する機能を持つ。

## Glossary

- **Digital_Twin_Dashboard**: 工場現場のデジタルツイン状態を表示するElectronベースのクライアントアプリケーション
- **WS_Server**: WebSocketサーバー（twin-gateway-rosbridgeベース）
- **Edge_Terminal**: センサーデータやロボット状態を取得し、画像ファイルを生成する現場端末
- **Status_Data**: 作業者、作業スペース、ロボットの現在状態を含むJSON形式のデータ
- **Command_Data**: ロボットへの動作指示を含むJSON形式のデータ
- **History_Storage**: クライアント側で蓄積する過去の状態履歴データ
- **Demo_Tasks**: ネジ締め、積み木、アンケート回答の3つの作業タスク
- **CRANE_V2**: デモで使用するロボットアーム
- **Live_Image**: エッジ端末から指定フォルダに保存される現場のライブ映像画像

## Requirements

### Requirement 1

**User Story:** 工場現場の管理者として、現在のデモ状況をリアルタイムで確認したいので、ダッシュボードで各種状態を一目で把握できるようにしたい

#### Acceptance Criteria

1. WHEN Status_Dataを受信した時、THE Digital_Twin_Dashboard SHALL デモ状況、作業者状態、ロボット状態を即座に画面に表示する
2. THE Digital_Twin_Dashboard SHALL 作業者状態に応じて適切なイラストを表示する
3. THE Digital_Twin_Dashboard SHALL CRANE_V2ロボットアームの状態に応じて適切なイラストを表示する
4. THE Digital_Twin_Dashboard SHALL WSサーバーとのWebSocket接続を自動的に確立し維持する
5. WHILE WSサーバーとの接続が確立されている間、THE Digital_Twin_Dashboard SHALL 受信したStatus_Dataを1秒以内に画面に反映する

### Requirement 2

**User Story:** 工場現場の管理者として、ライブ映像を確認したいので、エッジ端末から送信される画像をリアルタイムで表示したい

#### Acceptance Criteria

1. THE Digital_Twin_Dashboard SHALL 指定されたフォルダから最新の画像ファイルを自動的に読み込む
2. THE Digital_Twin_Dashboard SHALL ライブ映像エリアに現場の画像を表示する
3. WHEN 新しい画像ファイルが保存された時、THE Digital_Twin_Dashboard SHALL 表示を自動更新する
4. IF 画像ファイルが見つからない場合、THEN THE Digital_Twin_Dashboard SHALL 適切なプレースホルダーを表示する

### Requirement 3

**User Story:** 工場現場の管理者として、過去の作業履歴を分析したいので、時系列でデータを蓄積し可視化できるようにしたい

#### Acceptance Criteria

1. WHEN Status_Dataを受信した時、THE Digital_Twin_Dashboard SHALL データをローカルストレージに永続化する
2. THE Digital_Twin_Dashboard SHALL Demo_Tasksの実行履歴をタイムチャート形式で表示する
3. THE Digital_Twin_Dashboard SHALL ネジ締め、積み木、アンケート回答の各作業実行回数を表示する
4. THE Digital_Twin_Dashboard SHALL 直近の作業評価として標準時間との比較結果を表示する
5. THE Digital_Twin_Dashboard SHALL 作業者状態の変遷を時系列で可視化する

### Requirement 4

**User Story:** 工場現場の管理者として、ロボットの動作を制御したいので、ダッシュボードから指示を送信できるようにしたい

#### Acceptance Criteria

1. THE Digital_Twin_Dashboard SHALL マテハン指示（tool_handover、tool_collection、wait）を選択できるUIを提供する
2. WHEN ユーザーがマテハン指示を選択した時、THE Digital_Twin_Dashboard SHALL Command_DataをWSサーバーに送信する
3. THE Digital_Twin_Dashboard SHALL 送信したコマンドの履歴を表示する
4. THE Digital_Twin_Dashboard SHALL コマンド送信の成功・失敗状態を視覚的にフィードバックする

### Requirement 5

**User Story:** 工場現場の作業者として、直感的で見やすいインターフェースを使いたいので、シンプルかつモダンなデザインのダッシュボードを提供してほしい

#### Acceptance Criteria

1. THE Digital_Twin_Dashboard SHALL 工場現場に適したダークテーマベースのUIを提供する
2. THE Digital_Twin_Dashboard SHALL 重要な状態変化を色分けとアイコンで直感的に表示する
3. THE Digital_Twin_Dashboard SHALL レスポンシブデザインで異なる画面サイズに対応する
4. THE Digital_Twin_Dashboard SHALL 日本語UIを提供する
5. IF WSサーバーとの接続が切断された場合、THEN THE Digital_Twin_Dashboard SHALL 接続状態を視覚的に表示し自動再接続を試行する

### Requirement 6

**User Story:** システム管理者として、マルチOS環境で動作するアプリケーションが必要なので、Electronベースで構築してほしい

#### Acceptance Criteria

1. THE Digital_Twin_Dashboard SHALL Windows、macOS、Linuxで動作する
2. THE Digital_Twin_Dashboard SHALL Electronフレームワークを使用して構築される
3. THE Digital_Twin_Dashboard SHALL アプリケーション起動時にWSサーバーへの接続設定を行える
4. THE Digital_Twin_Dashboard SHALL アプリケーション終了時にデータを安全に保存する