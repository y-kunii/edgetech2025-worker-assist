# デプロイメントガイド (Deployment Guide)

Digital Twin Dashboard のビルド、パッケージング、デプロイメントに関するガイドです。

## 前提条件 (Prerequisites)

### 開発環境
- Node.js 18.x 以降
- npm 9.x 以降
- Git

### プラットフォーム固有の要件

#### Windows
- Windows 10 以降
- Visual Studio Build Tools または Visual Studio Community
- Windows SDK

#### macOS
- macOS 10.15 以降
- Xcode Command Line Tools
- Apple Developer アカウント（コード署名用、オプション）

#### Linux
- Ubuntu 18.04 以降 / CentOS 7 以降
- build-essential パッケージ
- libnss3-dev, libatk-bridge2.0-dev, libdrm2, libxcomposite1, libxdamage1, libxrandr2, libgbm1, libxss1, libasound2

## ビルドプロセス (Build Process)

### 1. 依存関係のインストール
```bash
npm install
```

### 2. 開発ビルド
```bash
npm run build
```

### 3. 本番ビルド
```bash
npm run build:script
```

## パッケージング (Packaging)

### 現在のプラットフォーム用
```bash
npm run package
```

### 特定のプラットフォーム用
```bash
# Windows用
npm run package:win

# macOS用
npm run package:mac

# Linux用
npm run package:linux
```

### 全プラットフォーム用
```bash
npm run package:all
```

## リリース準備 (Release Preparation)

### 完全なリリースビルド
```bash
npm run release
```

このコマンドは以下を実行します：
1. テストの実行
2. 全プラットフォーム用のパッケージ作成
3. チェックサムの生成
4. リリースノートテンプレートの作成

## 生成されるファイル (Generated Files)

### Windows
- `Digital-Twin-Dashboard-Setup-{version}.exe` - NSISインストーラー
- `Digital-Twin-Dashboard-{version}-win.zip` - ポータブル版
- `Digital-Twin-Dashboard-{version}-ia32-win.zip` - 32bit版

### macOS
- `Digital-Twin-Dashboard-{version}-arm64.dmg` - Apple Silicon用
- `Digital-Twin-Dashboard-{version}-x64.dmg` - Intel用
- `Digital-Twin-Dashboard-{version}-mac.zip` - ZIP版

### Linux
- `Digital-Twin-Dashboard-{version}-x64.AppImage` - AppImage
- `Digital-Twin-Dashboard-{version}-x64.deb` - Debian/Ubuntu用
- `Digital-Twin-Dashboard-{version}-x64.rpm` - RedHat/CentOS用
- `Digital-Twin-Dashboard-{version}-linux.tar.gz` - アーカイブ版

## 設定とカスタマイズ (Configuration and Customization)

### アプリケーションアイコン
- Windows: `build/icon.ico` (256x256 ICO形式)
- macOS: `build/icon.icns` (512x512 ICNS形式)
- Linux: `build/icon.png` (512x512 PNG形式)

### インストーラーカスタマイズ
- Windows NSIS: `build/installer.nsh`
- macOS DMG背景: `build/dmg-background.png`
- macOS権限設定: `build/entitlements.mac.plist`

### ビルド設定
`package.json` の `build` セクションで以下を設定可能：
- アプリケーションID
- 製品名
- 著作権情報
- ターゲットアーキテクチャ
- 配布形式

## コード署名 (Code Signing)

### Windows
```bash
# 環境変数を設定
export CSC_LINK="path/to/certificate.p12"
export CSC_KEY_PASSWORD="certificate_password"

npm run package:win
```

### macOS
```bash
# 環境変数を設定
export CSC_LINK="path/to/certificate.p12"
export CSC_KEY_PASSWORD="certificate_password"
export APPLE_ID="your@apple.id"
export APPLE_ID_PASSWORD="app_specific_password"

npm run package:mac
```

## 自動更新 (Auto Updates)

### 設定
`package.json` の `build.publish` セクションを編集：

```json
"publish": {
  "provider": "github",
  "owner": "your-username",
  "repo": "digital-twin-dashboard"
}
```

### サポートされるプロバイダー
- GitHub Releases
- Amazon S3
- Generic HTTP server
- Electron Builder Server

## CI/CD パイプライン (CI/CD Pipeline)

### GitHub Actions 例
```yaml
name: Build and Release

on:
  push:
    tags:
      - 'v*'

jobs:
  build:
    runs-on: ${{ matrix.os }}
    strategy:
      matrix:
        os: [windows-latest, macos-latest, ubuntu-latest]
    
    steps:
    - uses: actions/checkout@v3
    - uses: actions/setup-node@v3
      with:
        node-version: '18'
    
    - run: npm install
    - run: npm run test
    - run: npm run package
    
    - uses: actions/upload-artifact@v3
      with:
        name: packages-${{ matrix.os }}
        path: release/
```

## トラブルシューティング (Troubleshooting)

### よくある問題

#### ビルドエラー
```bash
# node_modulesを削除して再インストール
rm -rf node_modules package-lock.json
npm install
```

#### 権限エラー (Linux/macOS)
```bash
# スクリプトに実行権限を付与
chmod +x scripts/*.js
```

#### メモリ不足エラー
```bash
# Node.jsのメモリ制限を増加
export NODE_OPTIONS="--max-old-space-size=4096"
npm run package
```

### ログとデバッグ
```bash
# 詳細ログを有効化
export DEBUG=electron-builder
npm run package
```

## セキュリティ考慮事項 (Security Considerations)

1. **コード署名証明書の管理**
   - 証明書ファイルをバージョン管理に含めない
   - 環境変数やシークレット管理システムを使用

2. **依存関係の監査**
   ```bash
   npm audit
   npm audit fix
   ```

3. **セキュリティスキャン**
   - パッケージ化前にセキュリティスキャンを実行
   - 既知の脆弱性をチェック

## パフォーマンス最適化 (Performance Optimization)

1. **バンドルサイズの最適化**
   - 不要な依存関係の除去
   - Tree shakingの活用
   - コード分割

2. **起動時間の最適化**
   - 遅延読み込みの実装
   - 初期化処理の最適化

3. **メモリ使用量の最適化**
   - メモリリークの検出と修正
   - 適切なガベージコレクション

## サポートとメンテナンス (Support and Maintenance)

### バージョン管理
- セマンティックバージョニング (x.y.z) を使用
- 破壊的変更はメジャーバージョンアップ
- 新機能はマイナーバージョンアップ
- バグ修正はパッチバージョンアップ

### リリースサイクル
1. 開発とテスト
2. ベータリリース（オプション）
3. リリース候補
4. 本番リリース
5. ホットフィックス（必要に応じて）