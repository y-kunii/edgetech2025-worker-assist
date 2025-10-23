#!/usr/bin/env node

/**
 * Build validation script for Digital Twin Dashboard
 * Validates that all build artifacts are correctly generated
 */

const fs = require('fs');
const path = require('path');

console.log('🔍 Validating build artifacts...');

// Check required directories
const requiredDirs = [
  'dist/main/main',
  'dist/renderer',
  'dist/shared'
];

// Check required files
const requiredFiles = [
  'dist/main/main/main.js',
  'dist/renderer/bundle.js',
  'dist/renderer/index.html',
  'package.json'
];

let isValid = true;

// Validate directories
requiredDirs.forEach(dir => {
  if (fs.existsSync(dir)) {
    console.log(`✅ Directory exists: ${dir}`);
  } else {
    console.log(`❌ Missing directory: ${dir}`);
    isValid = false;
  }
});

// Validate files
requiredFiles.forEach(file => {
  if (fs.existsSync(file)) {
    const stats = fs.statSync(file);
    console.log(`✅ File exists: ${file} (${stats.size} bytes)`);
  } else {
    console.log(`❌ Missing file: ${file}`);
    isValid = false;
  }
});

if (isValid) {
  console.log('🎉 Build validation passed!');
  process.exit(0);
} else {
  console.log('❌ Build validation failed!');
  process.exit(1);
}