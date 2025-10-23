# Design Document

## Overview

デジタルツインダッシュボードは、工場現場のデモ用Electronアプリケーションです。WSサーバー（twin-gateway-rosbridge）からリアルタイムでデータを受信し、現場状況を可視化します。また、ロボット制御コマンドの送信、履歴データの蓄積・表示、ライブ映像の表示機能を提供します。

## Architecture

### システム構成
```
[エッジ端末] --WebSocket--> [WSサーバー] --WebSocket--> [Electronクライアント]
     |                                                           |
     +-- 画像ファイル保存 --> [共有フォルダ] --> ファイル監視 ----+
```

### 技術スタック
- **フロントエンド**: Electron + React + TypeScript
- **UI フレームワーク**: Material-UI (MUI) with ダークテーマ
- **チャートライブラリ**: Chart.js / Recharts
- **WebSocket通信**: ws ライブラリ
- **データ永続化**: SQLite (better-sqlite3)
- **ファイル監視**: chokidar
- **状態管理**: React Context + useReducer

## Components and Interfaces

### 1. メインアプリケーション構造
```
src/
├── main/                    # Electronメインプロセス
│   ├── main.ts             # アプリケーションエントリーポイント
│   ├── websocket-client.ts # WebSocket通信管理
│   └── file-watcher.ts     # 画像ファイル監視
├── renderer/               # Electronレンダラープロセス
│   ├── App.tsx            # メインアプリケーション
│   ├── components/        # UIコンポーネント
│   ├── hooks/            # カスタムフック
│   ├── services/         # データサービス
│   └── types/            # TypeScript型定義
└── shared/               # 共有型定義・ユーティリティ
```

### 2. WebSocket通信インターフェース

#### 受信データ（Status_Data）
```typescript
interface StatusData {
  worker_status: 'Absent' | 'Waiting' | 'Working' | 'Work Completed';
  space_status: 'Nothing' | 'Screw_tightening' | 'Building_blocks' | 'Survey_responses';
  robot_status: {
    state: string;
    grip: 'open' | 'closed';
  };
  timestamp: string; // YYYYMMDDhhmmss
  tool_delivery: number;
  status: 'Absent' | 'Waiting' | 'Working' | 'Work Completed';
}
```

#### 送信データ（Command_Data）
```typescript
interface CommandData {
  command: 'tool_handover' | 'tool_collection' | 'wait';
  timestamp: string; // YYYYMMDDhhmmss
}
```

### 3. ダッシュボードレイアウト

```
┌─────────────────────────────────────────────────────────────┐
│ ヘッダー: 接続状態 | 現在時刻 | デモ状況                      │
├─────────────────────────────────────────────────────────────┤
│ ┌─────────────┐ ┌─────────────┐ ┌─────────────┐ ┌─────────┐ │
│ │ 作業者状況  │ │ ロボット状況│ │ ライブ映像  │ │ 制御    │ │
│ │ (イラスト)  │ │ (イラスト)  │ │             │ │ パネル  │ │
│ └─────────────┘ └─────────────┘ └─────────────┘ └─────────┘ │
├─────────────────────────────────────────────────────────────┤
│ ┌─────────────────────────────┐ ┌─────────────────────────┐ │
│ │ タイムチャート              │ │ 作業統計                │ │
│ │ (作業履歴の時系列表示)      │ │ - 各作業実行回数        │ │
│ │                             │ │ - 直近作業評価          │ │
│ └─────────────────────────────┘ └─────────────────────────┘ │
└─────────────────────────────────────────────────────────────┘
```

### 4. コンポーネント設計

#### StatusPanel Component
```typescript
interface StatusPanelProps {
  title: string;
  status: string;
  illustration: string;
  color: 'success' | 'warning' | 'error' | 'info';
}
```

#### TimeChart Component
```typescript
interface TimeChartProps {
  data: WorkHistoryEntry[];
  timeRange: '1h' | '4h' | '24h';
}

interface WorkHistoryEntry {
  timestamp: Date;
  task: 'Screw_tightening' | 'Building_blocks' | 'Survey_responses' | 'Waiting';
  duration: number;
}
```

#### LiveVideo Component
```typescript
interface LiveVideoProps {
  imagePath: string;
  refreshInterval: number;
}
```

#### ControlPanel Component
```typescript
interface ControlPanelProps {
  onCommandSend: (command: CommandData) => void;
  isConnected: boolean;
}
```

## Data Models

### 1. データベーススキーマ（SQLite）

```sql
-- 状態履歴テーブル
CREATE TABLE status_history (
  id INTEGER PRIMARY KEY AUTOINCREMENT,
  timestamp TEXT NOT NULL,
  worker_status TEXT NOT NULL,
  space_status TEXT NOT NULL,
  robot_state TEXT NOT NULL,
  robot_grip TEXT NOT NULL,
  tool_delivery INTEGER NOT NULL,
  demo_status TEXT NOT NULL,
  created_at DATETIME DEFAULT CURRENT_TIMESTAMP
);

-- コマンド履歴テーブル
CREATE TABLE command_history (
  id INTEGER PRIMARY KEY AUTOINCREMENT,
  timestamp TEXT NOT NULL,
  command TEXT NOT NULL,
  success BOOLEAN NOT NULL,
  created_at DATETIME DEFAULT CURRENT_TIMESTAMP
);

-- 作業統計テーブル
CREATE TABLE work_statistics (
  id INTEGER PRIMARY KEY AUTOINCREMENT,
  date TEXT NOT NULL,
  screw_count INTEGER DEFAULT 0,
  blocks_count INTEGER DEFAULT 0,
  survey_count INTEGER DEFAULT 0,
  total_work_time INTEGER DEFAULT 0,
  average_work_time REAL DEFAULT 0
);
```

### 2. アプリケーション状態管理

```typescript
interface AppState {
  connection: {
    isConnected: boolean;
    lastHeartbeat: Date | null;
    reconnectAttempts: number;
  };
  currentStatus: StatusData | null;
  liveImage: {
    path: string | null;
    lastUpdated: Date | null;
  };
  workHistory: WorkHistoryEntry[];
  workStatistics: {
    screwCount: number;
    blocksCount: number;
    surveyCount: number;
    recentPerformance: 'fast' | 'normal' | 'slow' | null;
  };
  settings: {
    wsServerUrl: string;
    imageWatchPath: string;
    refreshInterval: number;
  };
}
```

## Error Handling

### 1. WebSocket接続エラー
- 自動再接続機能（指数バックオフ）
- 接続状態の視覚的表示
- オフライン時のデータ蓄積継続

### 2. ファイル監視エラー
- 画像ファイルアクセス失敗時のフォールバック
- ファイルパス設定の検証
- プレースホルダー画像の表示

### 3. データベースエラー
- SQLiteファイルの自動作成
- データ整合性チェック
- バックアップ機能

### 4. UI エラーハンドリング
- エラーバウンダリーの実装
- ユーザーフレンドリーなエラーメッセージ
- 復旧操作の提供

## Testing Strategy

### 1. 単体テスト
- WebSocket通信ロジック
- データ変換・検証ロジック
- データベース操作
- ファイル監視機能

### 2. 統合テスト
- WebSocketサーバーとの通信
- データベースとの連携
- ファイルシステムとの連携

### 3. E2Eテスト
- ダッシュボード表示の確認
- リアルタイムデータ更新
- コマンド送信機能
- 履歴データ表示

### 4. パフォーマンステスト
- 大量データ処理
- メモリ使用量監視
- レンダリング性能

## セキュリティ考慮事項

### 1. WebSocket通信
- 接続先URLの検証
- 不正データの検証・サニタイズ
- 通信タイムアウト設定

### 2. ファイルアクセス
- 画像ファイルパスの検証
- ディレクトリトラバーサル対策
- ファイルサイズ制限

### 3. データ保護
- SQLiteファイルの適切な権限設定
- 機密データの暗号化（必要に応じて）
- ログファイルの適切な管理

## パフォーマンス最適化

### 1. リアルタイム更新
- データ更新の最適化（差分更新）
- 不要な再レンダリングの防止
- メモ化の活用

### 2. チャート描画
- 仮想化による大量データ処理
- 適切なデータサンプリング
- Canvas最適化

### 3. メモリ管理
- 古いデータの自動削除
- 画像キャッシュの管理
- WebSocketメッセージの適切な処理