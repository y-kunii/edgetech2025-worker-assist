#!/bin/bash

# systemdサービステストスクリプト
# このスクリプトはラズパイ上で実行し、systemdサービスの動作を検証します

set -e

# 色付き出力
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# サービス名
SERVICE_NAME="raspberry-pi-websocket-server"
SERVICE_FILE="${SERVICE_NAME}.service"
HEALTH_URL="http://localhost:3001/health"

# テスト結果カウンター
TESTS_PASSED=0
TESTS_FAILED=0

# ログ関数
log_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

log_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
    ((TESTS_PASSED++))
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
    ((TESTS_FAILED++))
}

log_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

# サービスの状態を確認
check_service_status() {
    systemctl is-active --quiet "$SERVICE_NAME"
}

# ヘルスチェック
check_health() {
    local max_attempts=10
    local attempt=1
    
    while [ $attempt -le $max_attempts ]; do
        if curl -s -f "$HEALTH_URL" > /dev/null 2>&1; then
            return 0
        fi
        log_info "ヘルスチェック待機中... (試行 $attempt/$max_attempts)"
        sleep 2
        ((attempt++))
    done
    
    return 1
}

# サービスが停止するまで待機
wait_for_stop() {
    local max_attempts=15
    local attempt=1
    
    while [ $attempt -le $max_attempts ]; do
        if ! check_service_status; then
            return 0
        fi
        log_info "サービス停止待機中... (試行 $attempt/$max_attempts)"
        sleep 2
        ((attempt++))
    done
    
    return 1
}

# 前提条件チェック
check_prerequisites() {
    log_info "前提条件をチェック中..."
    
    # rootまたはsudo権限チェック
    if [ "$EUID" -ne 0 ]; then
        log_error "このスクリプトはroot権限で実行する必要があります (sudo を使用してください)"
        exit 1
    fi
    
    # サービスファイルの存在確認
    if [ ! -f "/etc/systemd/system/$SERVICE_FILE" ]; then
        log_error "サービスファイルが見つかりません: /etc/systemd/system/$SERVICE_FILE"
        log_info "サービスをインストールしてください: sudo cp $SERVICE_FILE /etc/systemd/system/"
        exit 1
    fi
    
    # ビルド済みファイルの確認
    if [ ! -f "dist/index.js" ]; then
        log_error "ビルド済みファイルが見つかりません: dist/index.js"
        log_info "ビルドを実行してください: npm run build"
        exit 1
    fi
    
    log_success "前提条件チェック完了"
}

# テスト1: サービス起動テスト
test_service_start() {
    log_info "========================================="
    log_info "テスト1: サービス起動テスト"
    log_info "========================================="
    
    # サービスが既に起動している場合は停止
    if check_service_status; then
        log_info "既存のサービスを停止中..."
        systemctl stop "$SERVICE_NAME"
        wait_for_stop
    fi
    
    # サービス起動
    log_info "サービスを起動中..."
    systemctl start "$SERVICE_NAME"
    sleep 3
    
    # 起動確認
    if check_service_status; then
        log_success "サービスが正常に起動しました"
    else
        log_error "サービスの起動に失敗しました"
        systemctl status "$SERVICE_NAME" --no-pager
        return 1
    fi
    
    # ヘルスチェック
    log_info "ヘルスチェックを実行中..."
    if check_health; then
        log_success "ヘルスチェックが成功しました"
        
        # ヘルスチェックの詳細を表示
        local health_response=$(curl -s "$HEALTH_URL")
        echo "$health_response" | jq '.' 2>/dev/null || echo "$health_response"
    else
        log_error "ヘルスチェックに失敗しました"
        return 1
    fi
    
    # プロセス確認
    log_info "プロセス情報を確認中..."
    local pid=$(systemctl show -p MainPID --value "$SERVICE_NAME")
    if [ -n "$pid" ] && [ "$pid" != "0" ]; then
        log_success "プロセスID: $pid"
        ps aux | grep "$pid" | grep -v grep || true
    else
        log_error "プロセスIDが取得できませんでした"
        return 1
    fi
    
    # ログ確認
    log_info "最新のログを確認中..."
    journalctl -u "$SERVICE_NAME" -n 10 --no-pager
    
    return 0
}

# テスト2: サービス停止テスト
test_service_stop() {
    log_info "========================================="
    log_info "テスト2: サービス停止テスト"
    log_info "========================================="
    
    # サービスが起動していることを確認
    if ! check_service_status; then
        log_info "サービスを起動中..."
        systemctl start "$SERVICE_NAME"
        sleep 3
    fi
    
    # サービス停止
    log_info "サービスを停止中..."
    systemctl stop "$SERVICE_NAME"
    
    # 停止確認
    if wait_for_stop; then
        log_success "サービスが正常に停止しました"
    else
        log_error "サービスの停止に失敗しました"
        systemctl status "$SERVICE_NAME" --no-pager
        return 1
    fi
    
    # ヘルスチェックが失敗することを確認
    log_info "サービス停止後のヘルスチェック..."
    if curl -s -f "$HEALTH_URL" > /dev/null 2>&1; then
        log_error "サービス停止後もヘルスチェックが成功しました（異常）"
        return 1
    else
        log_success "サービス停止後、ヘルスチェックが正常に失敗しました"
    fi
    
    return 0
}

# テスト3: サービス再起動テスト
test_service_restart() {
    log_info "========================================="
    log_info "テスト3: サービス再起動テスト"
    log_info "========================================="
    
    # サービスを起動
    if ! check_service_status; then
        log_info "サービスを起動中..."
        systemctl start "$SERVICE_NAME"
        sleep 3
    fi
    
    # 再起動前のPIDを記録
    local old_pid=$(systemctl show -p MainPID --value "$SERVICE_NAME")
    log_info "再起動前のPID: $old_pid"
    
    # サービス再起動
    log_info "サービスを再起動中..."
    systemctl restart "$SERVICE_NAME"
    sleep 3
    
    # 再起動確認
    if check_service_status; then
        log_success "サービスが正常に再起動しました"
    else
        log_error "サービスの再起動に失敗しました"
        systemctl status "$SERVICE_NAME" --no-pager
        return 1
    fi
    
    # 新しいPIDを確認
    local new_pid=$(systemctl show -p MainPID --value "$SERVICE_NAME")
    log_info "再起動後のPID: $new_pid"
    
    if [ "$old_pid" != "$new_pid" ]; then
        log_success "プロセスが新しく生成されました"
    else
        log_warning "PIDが変更されていません"
    fi
    
    # ヘルスチェック
    log_info "再起動後のヘルスチェック..."
    if check_health; then
        log_success "再起動後のヘルスチェックが成功しました"
    else
        log_error "再起動後のヘルスチェックに失敗しました"
        return 1
    fi
    
    return 0
}

# テスト4: 自動起動設定テスト
test_auto_start() {
    log_info "========================================="
    log_info "テスト4: 自動起動設定テスト"
    log_info "========================================="
    
    # 自動起動が有効か確認
    if systemctl is-enabled --quiet "$SERVICE_NAME"; then
        log_success "自動起動が有効になっています"
    else
        log_warning "自動起動が無効です。有効化を推奨します。"
        log_info "有効化コマンド: sudo systemctl enable $SERVICE_NAME"
    fi
    
    # 自動起動設定の詳細を表示
    log_info "自動起動設定の詳細:"
    systemctl show "$SERVICE_NAME" -p WantedBy --no-pager
    
    return 0
}

# テスト5: クラッシュ後の自動再起動テスト
test_crash_recovery() {
    log_info "========================================="
    log_info "テスト5: クラッシュ後の自動再起動テスト"
    log_info "========================================="
    
    # サービスを起動
    if ! check_service_status; then
        log_info "サービスを起動中..."
        systemctl start "$SERVICE_NAME"
        sleep 3
    fi
    
    # プロセスIDを取得
    local pid=$(systemctl show -p MainPID --value "$SERVICE_NAME")
    log_info "現在のPID: $pid"
    
    # プロセスを強制終了（クラッシュをシミュレート）
    log_info "プロセスを強制終了してクラッシュをシミュレート..."
    kill -9 "$pid"
    
    # 自動再起動を待機
    log_info "自動再起動を待機中（RestartSec=10秒）..."
    sleep 12
    
    # 再起動確認
    if check_service_status; then
        log_success "クラッシュ後、サービスが自動的に再起動しました"
        
        local new_pid=$(systemctl show -p MainPID --value "$SERVICE_NAME")
        log_info "新しいPID: $new_pid"
        
        # ヘルスチェック
        if check_health; then
            log_success "再起動後のヘルスチェックが成功しました"
        else
            log_error "再起動後のヘルスチェックに失敗しました"
            return 1
        fi
    else
        log_error "クラッシュ後、サービスが自動再起動しませんでした"
        systemctl status "$SERVICE_NAME" --no-pager
        return 1
    fi
    
    return 0
}

# テスト6: リソース制限テスト
test_resource_limits() {
    log_info "========================================="
    log_info "テスト6: リソース制限テスト"
    log_info "========================================="
    
    # サービスを起動
    if ! check_service_status; then
        log_info "サービスを起動中..."
        systemctl start "$SERVICE_NAME"
        sleep 3
    fi
    
    # リソース制限の確認
    log_info "設定されているリソース制限:"
    systemctl show "$SERVICE_NAME" -p MemoryLimit -p CPUQuota -p LimitNOFILE --no-pager
    
    # 実際のメモリ使用量を確認
    local pid=$(systemctl show -p MainPID --value "$SERVICE_NAME")
    if [ -n "$pid" ] && [ "$pid" != "0" ]; then
        log_info "実際のリソース使用状況:"
        ps -p "$pid" -o pid,vsz,rss,%mem,%cpu,cmd --no-headers
        
        # メモリ使用量をチェック（KB単位）
        local rss=$(ps -p "$pid" -o rss --no-headers | tr -d ' ')
        local rss_mb=$((rss / 1024))
        log_info "メモリ使用量: ${rss_mb}MB"
        
        if [ "$rss_mb" -lt 512 ]; then
            log_success "メモリ使用量が制限内です (${rss_mb}MB < 512MB)"
        else
            log_warning "メモリ使用量が制限に近づいています (${rss_mb}MB)"
        fi
    fi
    
    return 0
}

# テスト7: ログ出力テスト
test_logging() {
    log_info "========================================="
    log_info "テスト7: ログ出力テスト"
    log_info "========================================="
    
    # サービスを起動
    if ! check_service_status; then
        log_info "サービスを起動中..."
        systemctl start "$SERVICE_NAME"
        sleep 3
    fi
    
    # journalログの確認
    log_info "journalログの最新エントリ:"
    journalctl -u "$SERVICE_NAME" -n 20 --no-pager
    
    # ログファイルの確認
    if [ -d "logs" ]; then
        log_info "ログファイルの確認:"
        ls -lh logs/
        
        if [ -f "logs/server-$(date +%Y-%m-%d).log" ]; then
            log_success "本日のログファイルが存在します"
            log_info "最新のログエントリ:"
            tail -n 10 "logs/server-$(date +%Y-%m-%d).log"
        else
            log_warning "本日のログファイルが見つかりません"
        fi
    fi
    
    return 0
}

# メイン実行
main() {
    echo ""
    log_info "========================================="
    log_info "systemdサービステスト開始"
    log_info "サービス名: $SERVICE_NAME"
    log_info "========================================="
    echo ""
    
    # 前提条件チェック
    check_prerequisites
    echo ""
    
    # テスト実行
    test_service_start
    echo ""
    
    test_service_stop
    echo ""
    
    test_service_restart
    echo ""
    
    test_auto_start
    echo ""
    
    test_crash_recovery
    echo ""
    
    test_resource_limits
    echo ""
    
    test_logging
    echo ""
    
    # 結果サマリー
    log_info "========================================="
    log_info "テスト結果サマリー"
    log_info "========================================="
    log_success "成功: $TESTS_PASSED"
    if [ $TESTS_FAILED -gt 0 ]; then
        log_error "失敗: $TESTS_FAILED"
    else
        log_info "失敗: $TESTS_FAILED"
    fi
    echo ""
    
    if [ $TESTS_FAILED -eq 0 ]; then
        log_success "すべてのテストが成功しました！"
        exit 0
    else
        log_error "一部のテストが失敗しました"
        exit 1
    fi
}

# スクリプト実行
main
