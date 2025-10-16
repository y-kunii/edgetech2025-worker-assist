#!/bin/bash

# systemdサービスセットアップ検証スクリプト
# テスト実行前の前提条件をチェックします

set -e

# 色付き出力
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

SERVICE_NAME="raspberry-pi-websocket-server"
SERVICE_FILE="${SERVICE_NAME}.service"

# チェック結果カウンター
CHECKS_PASSED=0
CHECKS_FAILED=0
CHECKS_WARNING=0

log_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

log_success() {
    echo -e "${GREEN}[✓]${NC} $1"
    ((CHECKS_PASSED++))
}

log_error() {
    echo -e "${RED}[✗]${NC} $1"
    ((CHECKS_FAILED++))
}

log_warning() {
    echo -e "${YELLOW}[!]${NC} $1"
    ((CHECKS_WARNING++))
}

echo ""
log_info "========================================="
log_info "systemdサービスセットアップ検証"
log_info "========================================="
echo ""

# 1. 実行環境チェック
log_info "1. 実行環境をチェック中..."

if [ -f /etc/os-release ]; then
    . /etc/os-release
    log_success "OS: $PRETTY_NAME"
else
    log_warning "OS情報を取得できませんでした"
fi

if command -v systemctl &> /dev/null; then
    log_success "systemctl コマンドが利用可能です"
else
    log_error "systemctl コマンドが見つかりません"
fi

echo ""

# 2. プロジェクトファイルチェック
log_info "2. プロジェクトファイルをチェック中..."

if [ -f "package.json" ]; then
    log_success "package.json が存在します"
else
    log_error "package.json が見つかりません"
fi

if [ -f "tsconfig.json" ]; then
    log_success "tsconfig.json が存在します"
else
    log_error "tsconfig.json が見つかりません"
fi

if [ -f "$SERVICE_FILE" ]; then
    log_success "サービスファイル ($SERVICE_FILE) が存在します"
else
    log_error "サービスファイル ($SERVICE_FILE) が見つかりません"
fi

echo ""

# 3. ビルドファイルチェック
log_info "3. ビルドファイルをチェック中..."

if [ -d "dist" ]; then
    log_success "dist ディレクトリが存在します"
    
    if [ -f "dist/index.js" ]; then
        log_success "dist/index.js が存在します"
    else
        log_error "dist/index.js が見つかりません（npm run build を実行してください）"
    fi
else
    log_error "dist ディレクトリが見つかりません（npm run build を実行してください）"
fi

echo ""

# 4. 依存パッケージチェック
log_info "4. 依存パッケージをチェック中..."

if [ -d "node_modules" ]; then
    log_success "node_modules ディレクトリが存在します"
    
    # 主要パッケージの確認
    if [ -d "node_modules/socket.io" ]; then
        log_success "socket.io がインストールされています"
    else
        log_error "socket.io が見つかりません（npm install を実行してください）"
    fi
    
    if [ -d "node_modules/express" ]; then
        log_success "express がインストールされています"
    else
        log_error "express が見つかりません（npm install を実行してください）"
    fi
else
    log_error "node_modules ディレクトリが見つかりません（npm install を実行してください）"
fi

echo ""

# 5. 設定ファイルチェック
log_info "5. 設定ファイルをチェック中..."

if [ -d "config" ]; then
    log_success "config ディレクトリが存在します"
    
    if [ -f "config/config.json" ]; then
        log_success "config/config.json が存在します"
    else
        log_warning "config/config.json が見つかりません（デフォルト設定が使用されます）"
    fi
else
    log_warning "config ディレクトリが見つかりません"
fi

echo ""

# 6. ログディレクトリチェック
log_info "6. ログディレクトリをチェック中..."

if [ -d "logs" ]; then
    log_success "logs ディレクトリが存在します"
    
    # 書き込み権限チェック
    if [ -w "logs" ]; then
        log_success "logs ディレクトリに書き込み権限があります"
    else
        log_error "logs ディレクトリに書き込み権限がありません"
    fi
else
    log_warning "logs ディレクトリが見つかりません（自動作成されます）"
fi

echo ""

# 7. systemdサービスチェック（sudo権限が必要）
log_info "7. systemdサービスをチェック中..."

if [ "$EUID" -eq 0 ]; then
    # root権限で実行されている場合
    
    if [ -f "/etc/systemd/system/$SERVICE_FILE" ]; then
        log_success "サービスファイルがインストールされています"
        
        # サービスの状態確認
        if systemctl list-unit-files | grep -q "$SERVICE_NAME"; then
            log_success "サービスが systemd に認識されています"
            
            # 自動起動設定確認
            if systemctl is-enabled --quiet "$SERVICE_NAME" 2>/dev/null; then
                log_success "自動起動が有効になっています"
            else
                log_warning "自動起動が無効です（sudo systemctl enable $SERVICE_NAME で有効化できます）"
            fi
            
            # サービスの状態確認
            if systemctl is-active --quiet "$SERVICE_NAME" 2>/dev/null; then
                log_info "サービスは現在実行中です"
            else
                log_info "サービスは現在停止中です"
            fi
        else
            log_error "サービスが systemd に認識されていません（sudo systemctl daemon-reload を実行してください）"
        fi
    else
        log_error "サービスファイルがインストールされていません"
        log_info "インストールコマンド: sudo cp $SERVICE_FILE /etc/systemd/system/"
    fi
else
    log_warning "sudo権限がないため、systemdサービスの詳細チェックをスキップします"
    log_info "完全なチェックを実行するには: sudo $0"
fi

echo ""

# 8. Node.jsチェック
log_info "8. Node.js環境をチェック中..."

if command -v node &> /dev/null; then
    NODE_VERSION=$(node --version)
    log_success "Node.js がインストールされています: $NODE_VERSION"
else
    log_error "Node.js が見つかりません"
fi

if command -v npm &> /dev/null; then
    NPM_VERSION=$(npm --version)
    log_success "npm がインストールされています: $NPM_VERSION"
else
    log_error "npm が見つかりません"
fi

echo ""

# 9. ポートチェック
log_info "9. ポート使用状況をチェック中..."

if command -v netstat &> /dev/null; then
    if netstat -tlnp 2>/dev/null | grep -q ":3001"; then
        log_warning "ポート3001は既に使用されています"
        netstat -tlnp 2>/dev/null | grep ":3001" || true
    else
        log_success "ポート3001は使用可能です"
    fi
elif command -v ss &> /dev/null; then
    if ss -tlnp 2>/dev/null | grep -q ":3001"; then
        log_warning "ポート3001は既に使用されています"
        ss -tlnp 2>/dev/null | grep ":3001" || true
    else
        log_success "ポート3001は使用可能です"
    fi
else
    log_warning "ポートチェックツール（netstat/ss）が見つかりません"
fi

echo ""

# 結果サマリー
log_info "========================================="
log_info "検証結果サマリー"
log_info "========================================="
log_success "成功: $CHECKS_PASSED"
if [ $CHECKS_WARNING -gt 0 ]; then
    log_warning "警告: $CHECKS_WARNING"
fi
if [ $CHECKS_FAILED -gt 0 ]; then
    log_error "失敗: $CHECKS_FAILED"
fi
echo ""

# 推奨アクション
if [ $CHECKS_FAILED -gt 0 ]; then
    log_error "セットアップが完了していません。以下のアクションを実行してください："
    echo ""
    
    if [ ! -d "dist" ] || [ ! -f "dist/index.js" ]; then
        echo "  1. ビルドを実行: npm run build"
    fi
    
    if [ ! -d "node_modules" ]; then
        echo "  2. 依存パッケージをインストール: npm install"
    fi
    
    if [ ! -f "/etc/systemd/system/$SERVICE_FILE" ]; then
        echo "  3. サービスファイルをインストール:"
        echo "     sudo cp $SERVICE_FILE /etc/systemd/system/"
        echo "     sudo systemctl daemon-reload"
    fi
    
    echo ""
    exit 1
elif [ $CHECKS_WARNING -gt 0 ]; then
    log_warning "いくつかの警告がありますが、テストを実行できます"
    echo ""
    log_info "systemdサービステストを実行するには:"
    echo "  sudo ./scripts/test-systemd-service.sh"
    echo ""
    exit 0
else
    log_success "すべてのチェックが成功しました！"
    echo ""
    log_info "systemdサービステストを実行するには:"
    echo "  sudo ./scripts/test-systemd-service.sh"
    echo ""
    exit 0
fi
