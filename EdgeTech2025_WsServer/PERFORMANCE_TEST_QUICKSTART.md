# パフォーマンステスト クイックスタート

## 🚀 すぐに始める

### 1. ビルド
```bash
npm run build
```

### 2. テスト実行
```bash
npm run test:performance
```

## 📊 何が測定されるか

- ✅ **起動時間**: サーバーが起動するまでの時間
- ✅ **メモリ使用量**: 60秒間の最小/最大/平均/中央値（閾値: 512MB）
- ✅ **CPU使用率**: 60秒間の最小/最大/平均/中央値（閾値: 50%）

## ✅ 合格基準

- メモリ最大値 ≤ 512MB
- CPU平均値 ≤ 50%

## 📖 詳細ドキュメント

詳しい情報は [docs/RASPI_PERFORMANCE_TEST.md](docs/RASPI_PERFORMANCE_TEST.md) を参照してください。

## 🔧 トラブルシューティング

### エラー: dist/index.js not found
```bash
npm run build
```

### ポートが使用中
```bash
pkill -f "node dist/index.js"
```

### テストを中断したい
`Ctrl+C` を押してください（サーバーは自動的に停止します）
