# systemdサービステスト ドキュメント

## 概要

このドキュメントは、Raspberry Pi WebSocketサーバーのsystemdサービステストの実施方法と結果を記録します。

## テスト対象

- **サービス名**: `raspberry-pi-websocket-server`
- **サービスファイル**: `/etc/systemd/system/raspberry-pi-websocket-server.service`
- **要件**: 要件10.5（起動・停止管理）

## テスト項目

### 1. サービス起動テスト

**目的**: サービスが正常に起動し、WebSocketサーバーが動作することを確認

**テスト手順**:
1. サービスを起動: `sudo systemctl start raspberry-pi-websocket-server`
2. サービスステータスを確認: `sudo systemctl status raspberry-pi-websocket-server`
3. ヘルスチェックAPIを確認: `curl http://localhost:3001/health`
4. プロセスIDを確認
5. ログを確認

**期待結果**:
- サービスが `active (running)` 状態になる
- ヘルスチェックAPIが正常に応答する（HTTP 200）
- プロセスが起動している
- ログに起動メッセージが記録される

### 2. サービス停止テスト

**目的**: サービスが正常に停止し、リソースが適切に解放されることを確認

**テスト手順**:
1. サービスを停止: `sudo systemctl stop raspberry-pi-websocket-server`
2. サービスステータスを確認
3. ヘルスチェックAPIが応答しないことを確認
4. プロセスが終了していることを確認

**期待結果**:
- サービスが `inactive (dead)` 状態になる
- ヘルスチェックAPIが応答しない
- プロセスが終了している
- 優雅なシャットダウンが実行される

### 3. サービス再起動テスト

**目的**: サービスが正常に再起動し、新しいプロセスで動作することを確認

**テスト手順**:
1. 再起動前のプロセスIDを記録
2. サービスを再起動: `sudo systemctl restart raspberry-pi-websocket-server`
3. サービスステータスを確認
4. 新しいプロセスIDを確認
5. ヘルスチェックAPIを確認

**期待結果**:
- サービスが正常に再起動する
- プロセスIDが変更される
- ヘルスチェックAPIが正常に応答する
- 既存の接続が適切に処理される

### 4. 自動起動設定テスト

**目的**: システム起動時にサービスが自動的に起動する設定を確認

**テスト手順**:
1. 自動起動設定を確認: `sudo systemctl is-enabled raspberry-pi-websocket-server`
2. 自動起動を有効化: `sudo systemctl enable raspberry-pi-websocket-server`
3. 設定を確認: `systemctl show raspberry-pi-websocket-server -p WantedBy`

**期待結果**:
- 自動起動が有効になっている（`enabled`）
- `WantedBy=multi-user.target` が設定されている
- システム再起動後にサービスが自動起動する

### 5. クラッシュ後の自動再起動テスト

**目的**: プロセスがクラッシュした場合に自動的に再起動することを確認

**テスト手順**:
1. サービスを起動
2. プロセスIDを取得
3. プロセスを強制終了: `sudo kill -9 <PID>`
4. 10秒待機（RestartSec設定）
5. サービスステータスを確認
6. ヘルスチェックAPIを確認

**期待結果**:
- プロセス終了後、自動的に再起動する
- 新しいプロセスIDで動作する
- ヘルスチェックAPIが正常に応答する
- ログに再起動の記録が残る

### 6. リソース制限テスト

**目的**: サービスに設定されたリソース制限が適用されることを確認

**テスト手順**:
1. サービスを起動
2. リソース制限を確認: `systemctl show raspberry-pi-websocket-server -p MemoryLimit -p CPUQuota`
3. 実際のリソース使用量を確認: `ps aux | grep node`

**期待結果**:
- メモリ制限: 512MB
- CPU制限: 50%
- 実際の使用量が制限内に収まっている

### 7. ログ出力テスト

**目的**: サービスのログが適切に記録されることを確認

**テスト手順**:
1. サービスを起動
2. journalログを確認: `sudo journalctl -u raspberry-pi-websocket-server -n 50`
3. ログファイルを確認: `tail -f logs/server-*.log`

**期待結果**:
- journalにログが記録される
- ログファイルにログが記録される
- ログローテーションが機能する

## テスト実行方法

### 自動テストスクリプトの使用

```bash
# ラズパイ上で実行
cd /home/pi/raspberry-pi-websocket-server
sudo ./scripts/test-systemd-service.sh
```

### 手動テスト

```bash
# 1. サービス起動テスト
sudo systemctl start raspberry-pi-websocket-server
sudo systemctl status raspberry-pi-websocket-server
curl http://localhost:3001/health | jq

# 2. サービス停止テスト
sudo systemctl stop raspberry-pi-websocket-server
sudo systemctl status raspberry-pi-websocket-server

# 3. サービス再起動テスト
sudo systemctl restart raspberry-pi-websocket-server
sudo systemctl status raspberry-pi-websocket-server

# 4. 自動起動設定テスト
sudo systemctl is-enabled raspberry-pi-websocket-server
sudo systemctl enable raspberry-pi-websocket-server

# 5. ログ確認
sudo journalctl -u raspberry-pi-websocket-server -f
tail -f logs/server-*.log
```

## 前提条件

1. **サービスファイルのインストール**:
   ```bash
   sudo cp raspberry-pi-websocket-server.service /etc/systemd/system/
   sudo systemctl daemon-reload
   ```

2. **ビルド済みファイルの存在**:
   ```bash
   npm run build
   ```

3. **依存パッケージのインストール**:
   ```bash
   npm install --production
   ```

4. **設定ファイルの配置**:
   ```bash
   cp config/config.template.json config/config.json
   ```

## トラブルシューティング

### サービスが起動しない

```bash
# ステータス確認
sudo systemctl status raspberry-pi-websocket-server

# 詳細ログ確認
sudo journalctl -u raspberry-pi-websocket-server -n 100 --no-pager

# 設定ファイルの検証
sudo systemd-analyze verify raspberry-pi-websocket-server.service
```

### ヘルスチェックが失敗する

```bash
# ポートが使用中か確認
sudo netstat -tlnp | grep 3001

# ファイアウォール確認
sudo ufw status

# 手動起動でエラー確認
cd /home/pi/raspberry-pi-websocket-server
node dist/index.js
```

### 自動再起動が機能しない

```bash
# サービス設定を確認
systemctl show raspberry-pi-websocket-server -p Restart -p RestartSec

# Restart=always が設定されているか確認
grep Restart /etc/systemd/system/raspberry-pi-websocket-server.service
```

### メモリ制限に達する

```bash
# メモリ使用量を確認
systemctl status raspberry-pi-websocket-server

# メモリ制限を調整（必要に応じて）
sudo systemctl edit raspberry-pi-websocket-server
# [Service]
# MemoryLimit=1G
```

## テスト結果の記録

### テスト実行日時
- 日時: [記入してください]
- 実行者: [記入してください]
- 環境: Raspberry Pi [モデル]

### テスト結果

| テスト項目 | 結果 | 備考 |
|-----------|------|------|
| サービス起動 | ☐ 成功 / ☐ 失敗 | |
| サービス停止 | ☐ 成功 / ☐ 失敗 | |
| サービス再起動 | ☐ 成功 / ☐ 失敗 | |
| 自動起動設定 | ☐ 成功 / ☐ 失敗 | |
| クラッシュ後の自動再起動 | ☐ 成功 / ☐ 失敗 | |
| リソース制限 | ☐ 成功 / ☐ 失敗 | |
| ログ出力 | ☐ 成功 / ☐ 失敗 | |

### パフォーマンス測定

- **起動時間**: [記入してください] 秒
- **メモリ使用量**: [記入してください] MB
- **CPU使用率**: [記入してください] %

### 問題点と対応

[テスト中に発見された問題点と対応方法を記入してください]

## 参考資料

- [systemd.service マニュアル](https://www.freedesktop.org/software/systemd/man/systemd.service.html)
- [systemd.unit マニュアル](https://www.freedesktop.org/software/systemd/man/systemd.unit.html)
- [Raspberry Pi systemd 設定ガイド](https://www.raspberrypi.org/documentation/linux/usage/systemd.md)

## 関連ドキュメント

- [PROJECT_SETUP.md](../PROJECT_SETUP.md) - プロジェクトセットアップ手順
- [DEPLOYMENT.md](../DEPLOYMENT.md) - デプロイメント手順
- [raspberry-pi-websocket-server-README.md](../raspberry-pi-websocket-server-README.md) - サーバー概要

## 更新履歴

| 日付 | 変更内容 | 担当者 |
|------|---------|--------|
| 2025-10-14 | 初版作成 | - |
