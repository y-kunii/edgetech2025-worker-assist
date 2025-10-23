#!/usr/bin/env node

/**
 * Release script for Digital Twin Dashboard
 * Handles version management and release preparation
 */

const { execSync } = require('child_process');
const fs = require('fs');
const path = require('path');

const packageJsonPath = path.join(__dirname, '..', 'package.json');
const packageJson = JSON.parse(fs.readFileSync(packageJsonPath, 'utf8'));

console.log(`🚀 Preparing release for Digital Twin Dashboard v${packageJson.version}`);

// Validate version format
const versionRegex = /^\d+\.\d+\.\d+$/;
if (!versionRegex.test(packageJson.version)) {
  console.error('❌ Invalid version format. Use semantic versioning (x.y.z)');
  process.exit(1);
}

// Check if git is clean
try {
  const gitStatus = execSync('git status --porcelain', { encoding: 'utf8' });
  if (gitStatus.trim()) {
    console.warn('⚠️  Git working directory is not clean:');
    console.warn(gitStatus);
    console.warn('Consider committing changes before release');
  }
} catch (error) {
  console.warn('⚠️  Could not check git status');
}

// Run tests
console.log('🧪 Running tests...');
try {
  execSync('npm test', { stdio: 'inherit' });
  console.log('✅ All tests passed');
} catch (error) {
  console.error('❌ Tests failed. Fix tests before release.');
  process.exit(1);
}

// Build and package for all platforms
console.log('📦 Building and packaging for all platforms...');
try {
  execSync('node scripts/package.js --all', { stdio: 'inherit' });
  console.log('✅ All packages created successfully');
} catch (error) {
  console.error('❌ Packaging failed:', error.message);
  process.exit(1);
}

// Generate checksums
console.log('🔐 Generating checksums...');
const releaseDir = path.join(__dirname, '..', 'release');
if (fs.existsSync(releaseDir)) {
  const files = fs.readdirSync(releaseDir).filter(file => {
    const filePath = path.join(releaseDir, file);
    return fs.statSync(filePath).isFile() && !file.endsWith('.blockmap');
  });
  
  const crypto = require('crypto');
  const checksums = [];
  
  files.forEach(file => {
    const filePath = path.join(releaseDir, file);
    const fileBuffer = fs.readFileSync(filePath);
    const hashSum = crypto.createHash('sha256');
    hashSum.update(fileBuffer);
    const hex = hashSum.digest('hex');
    checksums.push(`${hex}  ${file}`);
    console.log(`   ✓ ${file}: ${hex.substring(0, 16)}...`);
  });
  
  // Write checksums file
  const checksumsPath = path.join(releaseDir, 'checksums.txt');
  fs.writeFileSync(checksumsPath, checksums.join('\n') + '\n');
  console.log(`✅ Checksums written to ${checksumsPath}`);
}

// Create release notes template
const releaseNotesPath = path.join(releaseDir, 'RELEASE_NOTES.md');
const releaseNotes = `# Digital Twin Dashboard v${packageJson.version}

## 新機能 (New Features)
- 

## 改善 (Improvements)
- 

## バグ修正 (Bug Fixes)
- 

## 技術的変更 (Technical Changes)
- 

## インストール方法 (Installation)

### Windows
- \`Digital-Twin-Dashboard-Setup-${packageJson.version}.exe\` をダウンロードして実行

### macOS
- \`Digital-Twin-Dashboard-${packageJson.version}-arm64.dmg\` (Apple Silicon) または
- \`Digital-Twin-Dashboard-${packageJson.version}-x64.dmg\` (Intel) をダウンロードして実行

### Linux
- \`Digital-Twin-Dashboard-${packageJson.version}-x64.AppImage\` をダウンロードして実行権限を付与後実行
- または \`Digital-Twin-Dashboard-${packageJson.version}-x64.deb\` / \`Digital-Twin-Dashboard-${packageJson.version}-x64.rpm\` をインストール

## システム要件 (System Requirements)
- Windows 10 以降
- macOS 10.15 以降
- Linux (Ubuntu 18.04 以降推奨)

## 検証済みチェックサム (Verified Checksums)
チェックサムファイル: \`checksums.txt\`
`;

fs.writeFileSync(releaseNotesPath, releaseNotes);
console.log(`📝 Release notes template created: ${releaseNotesPath}`);

console.log('\n🎉 Release preparation completed!');
console.log('📋 Next steps:');
console.log('   1. Review and edit the release notes');
console.log('   2. Test the generated packages on target platforms');
console.log('   3. Upload packages to your distribution platform');
console.log('   4. Tag the release in git: git tag v' + packageJson.version);
console.log('   5. Push the tag: git push origin v' + packageJson.version);