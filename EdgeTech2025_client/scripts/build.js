#!/usr/bin/env node

/**
 * Build script for Digital Twin Dashboard
 * Handles cross-platform building and packaging
 */

const { execSync } = require('child_process');
const fs = require('fs');
const path = require('path');
const os = require('os');

const platform = process.platform;
const arch = process.arch;

console.log(`🚀 Building Digital Twin Dashboard for ${platform}-${arch}`);

// Clean previous builds
console.log('🧹 Cleaning previous builds...');
try {
  execSync('npm run clean', { stdio: 'inherit' });
} catch (error) {
  console.warn('⚠️  Clean command failed, continuing...');
}

// Build the application
console.log('🔨 Building application...');
try {
  execSync('npm run build', { stdio: 'inherit' });
  console.log('✅ Application built successfully');
} catch (error) {
  console.error('❌ Build failed:', error.message);
  process.exit(1);
}

// Verify build output
const distPath = path.join(__dirname, '..', 'dist');
if (!fs.existsSync(distPath)) {
  console.error('❌ Build output not found');
  process.exit(1);
}

console.log('📦 Build verification passed');

// Platform-specific post-build tasks
switch (platform) {
  case 'darwin':
    console.log('🍎 macOS specific build tasks...');
    // Ensure proper permissions for macOS
    try {
      execSync('chmod +x dist/main/main.js', { stdio: 'inherit' });
    } catch (error) {
      console.warn('⚠️  Could not set executable permissions');
    }
    break;
    
  case 'win32':
    console.log('🪟 Windows specific build tasks...');
    // Windows-specific build tasks can be added here
    break;
    
  case 'linux':
    console.log('🐧 Linux specific build tasks...');
    // Ensure proper permissions for Linux
    try {
      execSync('chmod +x dist/main/main.js', { stdio: 'inherit' });
    } catch (error) {
      console.warn('⚠️  Could not set executable permissions');
    }
    break;
    
  default:
    console.log(`ℹ️  No specific build tasks for ${platform}`);
}

console.log('🎉 Build completed successfully!');
console.log(`📍 Build output: ${distPath}`);
console.log('💡 Run "npm run package" to create distributable packages');