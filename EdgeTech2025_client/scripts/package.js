#!/usr/bin/env node

/**
 * Packaging script for Digital Twin Dashboard
 * Handles cross-platform packaging with electron-builder
 */

const { execSync } = require('child_process');
const fs = require('fs');
const path = require('path');
const os = require('os');

const platform = process.platform;
const args = process.argv.slice(2);

console.log(`📦 Packaging Digital Twin Dashboard for ${platform}`);

// Determine target platforms
let targets = [];
if (args.includes('--all')) {
  targets = ['--win', '--mac', '--linux'];
} else if (args.includes('--win')) {
  targets = ['--win'];
} else if (args.includes('--mac')) {
  targets = ['--mac'];
} else if (args.includes('--linux')) {
  targets = ['--linux'];
} else {
  // Default to current platform
  switch (platform) {
    case 'darwin':
      targets = ['--mac'];
      break;
    case 'win32':
      targets = ['--win'];
      break;
    case 'linux':
      targets = ['--linux'];
      break;
    default:
      console.error(`❌ Unsupported platform: ${platform}`);
      process.exit(1);
  }
}

console.log(`🎯 Target platforms: ${targets.join(', ')}`);

// Ensure build directory exists
const buildDir = path.join(__dirname, '..', 'build');
if (!fs.existsSync(buildDir)) {
  console.log('📁 Creating build directory...');
  fs.mkdirSync(buildDir, { recursive: true });
}

// Check if icons exist (create placeholders if needed)
const iconFiles = {
  'icon.ico': 'Windows icon placeholder',
  'icon.icns': 'macOS icon placeholder', 
  'icon.png': 'Linux icon placeholder'
};

Object.entries(iconFiles).forEach(([filename, placeholder]) => {
  const iconPath = path.join(buildDir, filename);
  if (!fs.existsSync(iconPath)) {
    console.log(`⚠️  Creating placeholder for ${filename}`);
    fs.writeFileSync(iconPath, placeholder);
  }
});

// Run build first
console.log('🔨 Building application...');
try {
  execSync('node scripts/build.js', { stdio: 'inherit' });
} catch (error) {
  console.error('❌ Build failed:', error.message);
  process.exit(1);
}

// Package for each target
for (const target of targets) {
  console.log(`📦 Packaging for ${target}...`);
  try {
    const command = `electron-builder ${target}`;
    execSync(command, { stdio: 'inherit' });
    console.log(`✅ Successfully packaged for ${target}`);
  } catch (error) {
    console.error(`❌ Packaging failed for ${target}:`, error.message);
    // Continue with other targets
  }
}

// Display results
const releaseDir = path.join(__dirname, '..', 'release');
if (fs.existsSync(releaseDir)) {
  console.log('\n🎉 Packaging completed!');
  console.log('📦 Generated packages:');
  
  const files = fs.readdirSync(releaseDir);
  files.forEach(file => {
    const filePath = path.join(releaseDir, file);
    const stats = fs.statSync(filePath);
    if (stats.isFile()) {
      const sizeInMB = (stats.size / (1024 * 1024)).toFixed(2);
      console.log(`   📄 ${file} (${sizeInMB} MB)`);
    }
  });
  
  console.log(`\n📍 Packages location: ${releaseDir}`);
} else {
  console.log('⚠️  No packages were generated');
}