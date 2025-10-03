const fs = require('fs');
const path = require('path');

console.log('🔍 Validating project setup...\n');

// Check required files
const requiredFiles = [
  'package.json',
  'tsconfig.json', 
  'webpack.config.js',
  '.eslintrc.js',
  '.prettierrc',
  'jest.config.js',
  'README.md',
  '.gitignore'
];

console.log('📁 Checking configuration files:');
requiredFiles.forEach(file => {
  const exists = fs.existsSync(file);
  console.log(`  ${exists ? '✅' : '❌'} ${file}`);
});

// Check source directories
const requiredDirs = [
  'src/main',
  'src/renderer',
  'src/mock-server', 
  'src/types',
  'src/__tests__'
];

console.log('\n📂 Checking source directories:');
requiredDirs.forEach(dir => {
  const exists = fs.existsSync(dir);
  console.log(`  ${exists ? '✅' : '❌'} ${dir}`);
});

// Check entry files
const entryFiles = [
  'src/main/main.ts',
  'src/renderer/index.tsx',
  'src/renderer/App.tsx',
  'src/renderer/index.html',
  'src/mock-server/server.ts',
  'src/mock-server/mockDataGenerator.ts',
  'src/types/index.ts'
];

console.log('\n📄 Checking entry files:');
entryFiles.forEach(file => {
  const exists = fs.existsSync(file);
  console.log(`  ${exists ? '✅' : '❌'} ${file}`);
});

// Validate package.json
console.log('\n📦 Validating package.json:');
try {
  const packageJson = JSON.parse(fs.readFileSync('package.json', 'utf8'));
  
  console.log(`  ✅ Name: ${packageJson.name}`);
  console.log(`  ✅ Main entry: ${packageJson.main}`);
  
  const requiredScripts = ['build', 'build:dev', 'start', 'dev', 'test', 'mock-server'];
  const missingScripts = requiredScripts.filter(script => !packageJson.scripts[script]);
  
  if (missingScripts.length === 0) {
    console.log('  ✅ All required scripts present');
  } else {
    console.log(`  ❌ Missing scripts: ${missingScripts.join(', ')}`);
  }
  
  // Check dependencies
  const requiredDeps = ['react', 'react-dom', 'socket.io-client', 'chart.js'];
  const requiredDevDeps = ['electron', 'typescript', 'webpack', 'jest'];
  
  const missingDeps = requiredDeps.filter(dep => !packageJson.dependencies[dep]);
  const missingDevDeps = requiredDevDeps.filter(dep => !packageJson.devDependencies[dep]);
  
  if (missingDeps.length === 0) {
    console.log('  ✅ All required dependencies present');
  } else {
    console.log(`  ❌ Missing dependencies: ${missingDeps.join(', ')}`);
  }
  
  if (missingDevDeps.length === 0) {
    console.log('  ✅ All required dev dependencies present');
  } else {
    console.log(`  ❌ Missing dev dependencies: ${missingDevDeps.join(', ')}`);
  }
  
} catch (error) {
  console.log('  ❌ Error reading package.json:', error.message);
}

console.log('\n🎉 Project structure validation complete!');
console.log('\n📋 Next steps:');
console.log('  1. Install dependencies: npm install');
console.log('  2. Build project: npm run build:dev');
console.log('  3. Start mock server: npm run mock-server');
console.log('  4. Start application: npm start');
console.log('  5. Or run in dev mode: npm run dev');