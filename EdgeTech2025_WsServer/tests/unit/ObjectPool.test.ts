/**
 * ObjectPool単体テスト
 */

import { ObjectPool, errorInfoPool, timestampDataPool } from '../../src/utils/ObjectPool';

describe('ObjectPool', () => {
  describe('基本機能', () => {
    test('オブジェクトを取得できる', () => {
      const pool = new ObjectPool<{ value: number }>(
        () => ({ value: 0 }),
        (obj) => { obj.value = 0; },
        5,
        10
      );

      const obj = pool.acquire();
      expect(obj).toBeDefined();
      expect(obj.value).toBe(0);
    });

    test('オブジェクトを返却できる', () => {
      const pool = new ObjectPool<{ value: number }>(
        () => ({ value: 0 }),
        (obj) => { obj.value = 0; },
        5,
        10
      );

      const initialSize = pool.size();
      const obj = pool.acquire();
      expect(pool.size()).toBe(initialSize - 1);
      
      pool.release(obj);
      expect(pool.size()).toBe(initialSize);
    });

    test('返却時にオブジェクトがリセットされる', () => {
      const pool = new ObjectPool<{ value: number }>(
        () => ({ value: 0 }),
        (obj) => { obj.value = 0; },
        5,
        10
      );

      const obj = pool.acquire();
      obj.value = 100;
      pool.release(obj);

      const obj2 = pool.acquire();
      expect(obj2.value).toBe(0);
    });

    test('プールが空の場合は新規オブジェクトを作成', () => {
      const pool = new ObjectPool<{ value: number }>(
        () => ({ value: 42 }),
        (obj) => { obj.value = 0; },
        0, // 初期サイズ0
        10
      );

      expect(pool.size()).toBe(0);
      const obj = pool.acquire();
      expect(obj).toBeDefined();
      expect(obj.value).toBe(42);
    });

    test('最大サイズを超えるオブジェクトは返却されない', () => {
      const pool = new ObjectPool<{ value: number }>(
        () => ({ value: 0 }),
        (obj) => { obj.value = 0; },
        2,
        2 // 最大サイズ2
      );

      const obj1 = pool.acquire();
      const obj2 = pool.acquire();
      const obj3 = pool.acquire(); // プールが空なので新規作成

      expect(pool.size()).toBe(0);

      pool.release(obj1);
      pool.release(obj2);
      expect(pool.size()).toBe(2);

      pool.release(obj3); // 最大サイズを超えるので返却されない
      expect(pool.size()).toBe(2);
    });

    test('プールをクリアできる', () => {
      const pool = new ObjectPool<{ value: number }>(
        () => ({ value: 0 }),
        (obj) => { obj.value = 0; },
        5,
        10
      );

      expect(pool.size()).toBe(5);
      pool.clear();
      expect(pool.size()).toBe(0);
    });
  });

  describe('errorInfoPool', () => {
    beforeEach(() => {
      errorInfoPool.clear();
    });

    test('ErrorInfoオブジェクトを取得できる', () => {
      const error = errorInfoPool.acquire();
      expect(error).toBeDefined();
      expect(error).toHaveProperty('code');
      expect(error).toHaveProperty('message');
      expect(error).toHaveProperty('timestamp');
    });

    test('ErrorInfoオブジェクトを設定して返却できる', () => {
      const error = errorInfoPool.acquire();
      error.code = 'TEST_ERROR';
      error.message = 'Test error message';
      error.timestamp = '2025-10-14T00:00:00.000Z';

      errorInfoPool.release(error);

      const error2 = errorInfoPool.acquire();
      expect(error2.code).toBe('');
      expect(error2.message).toBe('');
      expect(error2.timestamp).toBe('');
    });

    test('複数のErrorInfoオブジェクトを管理できる', () => {
      const error1 = errorInfoPool.acquire();
      const error2 = errorInfoPool.acquire();
      const error3 = errorInfoPool.acquire();

      error1.code = 'ERROR_1';
      error2.code = 'ERROR_2';
      error3.code = 'ERROR_3';

      expect(error1.code).toBe('ERROR_1');
      expect(error2.code).toBe('ERROR_2');
      expect(error3.code).toBe('ERROR_3');

      errorInfoPool.release(error1);
      errorInfoPool.release(error2);
      errorInfoPool.release(error3);
    });
  });

  describe('timestampDataPool', () => {
    beforeEach(() => {
      timestampDataPool.clear();
    });

    test('タイムスタンプ付きデータオブジェクトを取得できる', () => {
      const data = timestampDataPool.acquire();
      expect(data).toBeDefined();
      expect(data).toHaveProperty('timestamp');
    });

    test('追加プロパティを設定して返却できる', () => {
      const data = timestampDataPool.acquire();
      data.timestamp = '2025-10-14T00:00:00.000Z';
      data.customField = 'test value';
      data.count = 42;

      timestampDataPool.release(data);

      const data2 = timestampDataPool.acquire();
      expect(data2.timestamp).toBe('');
      expect(data2.customField).toBeUndefined();
      expect(data2.count).toBeUndefined();
    });
  });

  describe('メモリ効率', () => {
    test('オブジェクトの再利用により新規作成を削減', () => {
      let createCount = 0;
      const pool = new ObjectPool<{ value: number }>(
        () => {
          createCount++;
          return { value: 0 };
        },
        (obj) => { obj.value = 0; },
        3,
        10
      );

      // 初期作成
      expect(createCount).toBe(3);

      // 既存オブジェクトを使用
      const obj1 = pool.acquire();
      const obj2 = pool.acquire();
      pool.acquire(); // obj3
      expect(createCount).toBe(3); // 新規作成なし

      // プールが空になったら新規作成
      pool.acquire(); // obj4
      expect(createCount).toBe(4);

      // 返却して再利用
      pool.release(obj1);
      pool.release(obj2);
      pool.acquire(); // obj5
      expect(createCount).toBe(4); // 新規作成なし
    });
  });
});
