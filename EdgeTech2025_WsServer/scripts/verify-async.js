#!/usr/bin/env node

/**
 * 非同期処理の検証スクリプト
 * イベントループのブロッキングを検出
 */

const fs = require('fs');
const path = require('path');

console.log('='.repeat(60));
console.log('非同期処理の検証');
console.log('='.repeat(60));
console.log('');

// 検証項目
const checks = {
  syncFileOps: {
    name: '同期ファイル操作',
    patterns: [
      /readFileSync/g,
      /writeFileSync/g,
      /existsSync/g,
      /mkdirSync/g,
    ],
    allowedFiles: [
      'src/utils/ConfigManager.ts',  // 起動時のみ
      'src/utils/Logger.ts',          // 起動時のみ
    ],
  },
  blockingLoops: {
    name: 'ブロッキングループ',
    patterns: [
      /while\s*\(/g,
      /for\s*\([^)]*;\s*[^)]*;\s*[^)]*\)\s*{[^}]{100,}}/g,  // 長いforループ
    ],
    allowedFiles: [],
  },
  syncCrypto: {
    name: '同期暗号化処理',
    patterns: [
      /crypto\.[a-zA-Z]*Sync/g,
    ],
    allowedFiles: [],
  },
};

// ソースファイルを再帰的に取得
function getSourceFiles(dir, files = []) {
  const entries = fs.readdirSync(dir, { withFileTypes: true });
  
  for (const entry of entries) {
    const fullPath = path.join(dir, entry.name);
    
    if (entry.isDirectory()) {
      if (entry.name !== 'node_modules' && entry.name !== 'dist' && entry.name !== '.git') {
        getSourceFiles(fullPath, files);
      }
    } else if (entry.isFile() && (entry.name.endsWith('.ts') || entry.name.endsWith('.js'))) {
      files.push(fullPath);
    }
  }
  
  return files;
}

// ファイルをチェック
function checkFile(filePath, check) {
  const content = fs.readFileSync(filePath, 'utf-8');
  const relativePath = path.relative(process.cwd(), filePath);
  const issues = [];
  
  for (const pattern of check.patterns) {
    const matches = content.match(pattern);
    if (matches) {
      // 許可されたファイルかチェック
      const isAllowed = check.allowedFiles.some(allowed => 
        relativePath.includes(allowed.replace(/\//g, path.sep))
      );
      
      if (!isAllowed) {
        issues.push({
          file: relativePath,
          pattern: pattern.source,
          count: matches.length,
        });
      }
    }
  }
  
  return issues;
}

// メイン処理
function main() {
  const srcDir = path.join(process.cwd(), 'src');
  const files = getSourceFiles(srcDir);
  
  console.log(`検証対象ファイル数: ${files.length}`);
  console.log('');
  
  let totalIssues = 0;
  
  for (const [checkName, check] of Object.entries(checks)) {
    console.log(`[${check.name}]`);
    
    let checkIssues = 0;
    
    for (const file of files) {
      const issues = checkFile(file, check);
      
      if (issues.length > 0) {
        for (const issue of issues) {
          console.log(`  ⚠️  ${issue.file}`);
          console.log(`     パターン: ${issue.pattern}`);
          console.log(`     検出数: ${issue.count}`);
          checkIssues += issue.count;
          totalIssues += issue.count;
        }
      }
    }
    
    if (checkIssues === 0) {
      console.log('  ✅ 問題なし');
    } else {
      console.log(`  ❌ ${checkIssues}件の問題を検出`);
    }
    
    console.log('');
  }
  
  // 許可されたファイルの同期処理を表示
  console.log('[許可された同期処理]');
  console.log('  ℹ️  src/utils/ConfigManager.ts - 起動時の設定読み込み');
  console.log('  ℹ️  src/utils/Logger.ts - 起動時のディレクトリ作成');
  console.log('');
  
  // 結果サマリー
  console.log('='.repeat(60));
  if (totalIssues === 0) {
    console.log('✅ 検証完了: 問題は検出されませんでした');
  } else {
    console.log(`⚠️  検証完了: ${totalIssues}件の潜在的な問題を検出`);
    console.log('');
    console.log('注意: 検出された項目が必ずしも問題とは限りません。');
    console.log('各項目を確認し、イベントループをブロックしないか検討してください。');
  }
  console.log('='.repeat(60));
  
  process.exit(totalIssues > 0 ? 1 : 0);
}

main();
