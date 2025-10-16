# systemdサービステスト クイックスタート

## 概要

Raspberry Pi WebSocketサーバーのsystemdサービステストを実施するためのクイックスタートガイドです。

## 前提条件

1. Raspberry Pi上でテストを実行
2. サーバーがビルド済み（`npm run build`実行済み）
3. サービスファイルがインストール済み
4. sudo権限を持つユーザーでログイン

## クイックテスト（推奨）

### 自動テストスクリプトの実行

```bash
# ラズパイにSSH接続
ssh pi@raspberrypi

# プロジェクトディレクトリに移動
cd raspberry-pi-websocket-server

# テストスクリプトを実行（sudo権限が必要）
sudo ./scripts/test-systemd-service.sh
```

このスクリプトは以下のテストを自動的に実行します：
- ✅ サービス起動テスト
- ✅ サービス停止テスト
- ✅ サービス再起動テスト
- ✅ 自動起動設定テスト
- ✅ クラッシュ後の自動再起動テスト
- ✅ リソース制限テスト
- ✅ ログ出力テスト

## 手動テスト

### 1. サービスのインストール（初回のみ）

```bash
# サービスファイルをコピー
sudo cp raspberry-pi-websocket-server.service /etc/systemd/system/

# systemdをリロード
sudo systemctl daemon-reload

# 自動起動を有効化
sudo systemctl enable raspberry-pi-websocket-server
```

### 2. 基本的なサービス操作

```bash
# サービス起動
sudo systemctl start raspberry-pi-websocket-server

# サービス停止
sudo systemctl stop raspberry-pi-websocket-server

# サービス再起動
sudo systemctl restart raspberry-pi-websocket-server

# サービスステータス確認
sudo systemctl status raspberry-pi-websocket-server

# サービスログ確認
sudo journalctl -u raspberry-pi-websocket-server -f
```

### 3. ヘルスチェック

```bash
# ヘルスチェックAPI呼び出し
curl http://localhost:3001/health

# JSON整形して表示
curl -s http://localhost:3001/health | jq
```

### 4. 自動起動の確認

```bash
# 自動起動設定を確認
sudo systemctl is-enabled raspberry-pi-websocket-server

# 出力が "enabled" であればOK
```

### 5. クラッシュ後の自動再起動テスト

```bash
# 現在のプロセスIDを確認
PID=$(systemctl show -p MainPID --value raspberry-pi-websocket-server)
echo "Current PID: $PID"

# プロセスを強制終了（クラッシュをシミュレート）
sudo kill -9 $PID

# 10秒待機（RestartSec設定）
sleep 12

# サービスが自動再起動したか確認
sudo systemctl status raspberry-pi-websocket-server

# ヘルスチェック
curl http://localhost:3001/health
```

## テスト結果の確認

### 成功の判定基準

✅ **サービス起動テスト**
- `systemctl status` で `active (running)` と表示される
- ヘルスチェックAPIが HTTP 200 を返す
- ログに起動メッセージが記録される

✅ **サービス停止テスト**
- `systemctl status` で `inactive (dead)` と表示される
- ヘルスチェックAPIが応答しない
- プロセスが終了している

✅ **サービス再起動テスト**
- サービスが正常に再起動する
- プロセスIDが変更される
- ヘルスチェックAPIが正常に応答する

✅ **自動起動設定テスト**
- `systemctl is-enabled` が `enabled` を返す
- システム再起動後にサービスが自動起動する

✅ **クラッシュ後の自動再起動テスト**
- プロセス強制終了後、10秒以内に自動再起動する
- 新しいプロセスIDで動作する
- ヘルスチェックAPIが正常に応答する

## トラブルシューティング

### サービスが起動しない

```bash
# 詳細なエラーログを確認
sudo journalctl -u raspberry-pi-websocket-server -n 100 --no-pager

# サービスファイルの検証
sudo systemd-analyze verify raspberry-pi-websocket-server.service

# 手動起動でエラーを確認
cd /home/pi/raspberry-pi-websocket-server
node dist/index.js
```

### ポートが既に使用されている

```bash
# ポート3001を使用しているプロセスを確認
sudo netstat -tlnp | grep 3001

# または
sudo lsof -i :3001

# 必要に応じてプロセスを終了
sudo kill <PID>
```

### ビルドファイルが見つからない

```bash
# ビルドを実行
npm run build

# ビルドファイルの確認
ls -la dist/
```

### 権限エラー

```bash
# ログディレクトリの権限を確認
ls -la logs/

# 必要に応じて権限を修正
sudo chown -R pi:pi logs/
chmod 755 logs/
```

## リソース監視

### メモリ使用量の確認

```bash
# サービスのメモリ使用量
systemctl status raspberry-pi-websocket-server | grep Memory

# プロセスの詳細情報
PID=$(systemctl show -p MainPID --value raspberry-pi-websocket-server)
ps -p $PID -o pid,vsz,rss,%mem,%cpu,cmd
```

### CPU使用率の確認

```bash
# リアルタイム監視
top -p $(systemctl show -p MainPID --value raspberry-pi-websocket-server)
```

### ログファイルサイズの確認

```bash
# ログディレクトリのサイズ
du -sh logs/

# ログファイル一覧
ls -lh logs/
```

## 継続的な監視

### サービスの監視スクリプト

```bash
# 5秒ごとにヘルスチェック
watch -n 5 'curl -s http://localhost:3001/health | jq'

# リアルタイムログ監視
sudo journalctl -u raspberry-pi-websocket-server -f
```

### システム起動時の自動起動確認

```bash
# システムを再起動
sudo reboot

# 再起動後、サービスが自動起動したか確認
sudo systemctl status raspberry-pi-websocket-server
```

## テスト完了チェックリスト

- [ ] サービスが正常に起動する
- [ ] サービスが正常に停止する
- [ ] サービスが正常に再起動する
- [ ] 自動起動が有効になっている
- [ ] クラッシュ後に自動再起動する
- [ ] メモリ使用量が512MB以下
- [ ] CPU使用率が50%以下
- [ ] ヘルスチェックAPIが正常に応答する
- [ ] ログが正常に記録される
- [ ] システム再起動後に自動起動する

## 次のステップ

テストが完了したら：

1. **テスト結果を記録**: `docs/SYSTEMD_SERVICE_TEST.md` にテスト結果を記入
2. **長時間運用テスト**: タスク14.2の24時間運用テストを実施
3. **本番環境デプロイ**: `DEPLOYMENT.md` を参照してデプロイ

## 関連ドキュメント

- [docs/SYSTEMD_SERVICE_TEST.md](docs/SYSTEMD_SERVICE_TEST.md) - 詳細なテストドキュメント
- [DEPLOYMENT.md](DEPLOYMENT.md) - デプロイメント手順
- [PROJECT_SETUP.md](PROJECT_SETUP.md) - プロジェクトセットアップ
- [raspberry-pi-websocket-server-README.md](raspberry-pi-websocket-server-README.md) - サーバー概要

## サポート

問題が発生した場合は、以下を確認してください：

1. サービスログ: `sudo journalctl -u raspberry-pi-websocket-server -n 100`
2. アプリケーションログ: `tail -f logs/server-*.log`
3. システムリソース: `htop` または `top`
4. ネットワーク接続: `netstat -tlnp | grep 3001`
