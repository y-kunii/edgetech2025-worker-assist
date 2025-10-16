/**
 * ObjectPool
 * オブジェクトプールの実装
 * 頻繁に生成されるオブジェクトを再利用してメモリ使用量を削減
 */

export class ObjectPool<T> {
  private pool: T[];
  private factory: () => T;
  private reset: (obj: T) => void;
  private maxSize: number;

  constructor(factory: () => T, reset: (obj: T) => void, initialSize: number = 10, maxSize: number = 100) {
    this.factory = factory;
    this.reset = reset;
    this.maxSize = maxSize;
    this.pool = [];

    // 初期オブジェクトを作成
    for (let i = 0; i < initialSize; i++) {
      this.pool.push(this.factory());
    }
  }

  /**
   * プールからオブジェクトを取得
   */
  acquire(): T {
    if (this.pool.length > 0) {
      return this.pool.pop()!;
    }
    // プールが空の場合は新規作成
    return this.factory();
  }

  /**
   * オブジェクトをプールに返却
   */
  release(obj: T): void {
    // プールサイズが最大値を超えない場合のみ返却
    if (this.pool.length < this.maxSize) {
      this.reset(obj);
      this.pool.push(obj);
    }
    // 最大値を超える場合はGCに任せる
  }

  /**
   * プールの現在のサイズを取得
   */
  size(): number {
    return this.pool.length;
  }

  /**
   * プールをクリア
   */
  clear(): void {
    this.pool = [];
  }
}

/**
 * ErrorInfoオブジェクトプール
 */
export interface PooledErrorInfo {
  code: string;
  message: string;
  timestamp: string;
}

export const errorInfoPool = new ObjectPool<PooledErrorInfo>(
  () => ({ code: '', message: '', timestamp: '' }),
  (obj) => {
    obj.code = '';
    obj.message = '';
    obj.timestamp = '';
  },
  5,
  20
);

/**
 * タイムスタンプ付きデータオブジェクトプール
 */
export interface PooledTimestampData {
  timestamp: string;
  [key: string]: any;
}

export const timestampDataPool = new ObjectPool<PooledTimestampData>(
  () => ({ timestamp: '' }),
  (obj) => {
    // すべてのプロパティをクリア
    for (const key in obj) {
      if (key !== 'timestamp') {
        delete obj[key];
      }
    }
    obj.timestamp = '';
  },
  10,
  50
);
