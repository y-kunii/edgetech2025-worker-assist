# 統合テスト

このディレクトリには、Raspberry Pi WebSocketサーバーの統合テストが含まれています。

## テストファイル

### 1. BasicConnection.test.ts
**要件: 1.2, 3.1, 3.2, 3.3, 3.4, 3.5**

基本的な接続機能をテストします：
- 各タイプのクライアント（Electron、センサー、ロボット）の接続
- クライアントタイプの識別と登録
- 複数クライアントの同時接続
- 接続確認メッセージの送信
- クライアント切断とリソース解放

**テスト数: 14**

### 2. DataRouting.test.ts
**要件: 2.1, 2.2, 2.3, 4.1, 4.2, 4.3, 4.4**

データ中継機能をテストします：
- センサーデータのElectronクライアントへの転送
- 画像データを含むセンサーデータの転送
- ロボット指示のロボット制御プログラムへの転送
- 各種ロボットコマンドタイプのサポート
- ロボット応答のElectronクライアントへの転送
- エンドツーエンドデータフロー

**テスト数: 11**

### 3. ErrorHandling.test.ts
**要件: 2.4, 2.5, 4.5, 4.6, 9.1, 9.2, 9.3, 9.6**

エラーハンドリング機能をテストします：
- センサープログラム未接続時のエラー処理
- ロボット制御プログラム未接続時のエラー処理
- 外部プログラムの予期せぬ切断の処理
- 不正なデータ形式の処理
- WebSocket送信エラーの処理
- 接続タイムアウトの検出
- 複数の外部プログラム切断の処理

**テスト数: 11**

### 4. Heartbeat.test.ts
**要件: 1.3, 9.6**

ハートビート機能をテストします：
- サーバーからの定期的なping送信
- クライアントからのpong応答の処理
- pong応答がない場合のタイムアウト検出
- pong応答を送信し続ける限り接続が維持されること
- 一部のクライアントがタイムアウトしても他のクライアントは影響を受けないこと
- クライアント再接続時のハートビート再開

**テスト数: 8**

### 5. HealthCheck.test.ts
**要件: 5.1, 5.2, 5.3, 5.4**

ヘルスチェックAPI機能をテストします：
- /healthエンドポイントの応答
- HTTPステータスコード200の返却
- JSON形式の応答
- status、timestamp、connectionsの含有
- タイプ別接続数の報告
- sensor_connected、robot_connectedの状態報告
- 接続状態の動的更新
- 複数回のヘルスチェック実行
- 同時リクエストの処理

**テスト数: 18**

## テストの実行

### 全ての統合テストを実行
```bash
npm test -- tests/integration --runInBand
```

### 個別のテストファイルを実行
```bash
npm test -- tests/integration/BasicConnection.test.ts --runInBand
npm test -- tests/integration/DataRouting.test.ts --runInBand
npm test -- tests/integration/ErrorHandling.test.ts --runInBand
npm test -- tests/integration/Heartbeat.test.ts --runInBand --testTimeout=30000
npm test -- tests/integration/HealthCheck.test.ts --runInBand
```

## 注意事項

- 統合テストは実際のサーバーインスタンスを起動するため、`--runInBand`オプションを使用して順次実行することを推奨します
- Heartbeat.test.tsは長時間実行されるテストを含むため、`--testTimeout=30000`オプションを指定してください
- 各テストファイルは異なるポート番号を使用しています：
  - BasicConnection: 3002
  - DataRouting: 3003
  - ErrorHandling: 3004
  - Heartbeat: 3005
  - HealthCheck: 3006

## テスト結果サマリー

- **合計テスト数**: 62
- **カバーされる要件**: 1.2, 1.3, 2.1, 2.2, 2.3, 2.4, 2.5, 3.1, 3.2, 3.3, 3.4, 3.5, 4.1, 4.2, 4.3, 4.4, 4.5, 4.6, 5.1, 5.2, 5.3, 5.4, 9.1, 9.2, 9.3, 9.6
- **全テスト成功**: ✓

## トラブルシューティング

### Jest did not exit one second after the test run has completed

これは非同期操作が完全にクリーンアップされていないことを示す警告です。テスト自体は成功していますが、以下の対処法があります：

1. `--forceExit`オプションを使用（推奨されません）
2. テスト後に適切なクリーンアップを実装
3. この警告を無視（テストは正常に完了しています）

### タイムアウトエラー

Heartbeat.test.tsでタイムアウトエラーが発生する場合：
- `--testTimeout=30000`オプションを使用してください
- ハートビート間隔とタイムアウト設定を確認してください
