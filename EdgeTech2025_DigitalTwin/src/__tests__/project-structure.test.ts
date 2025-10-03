import * as fs from 'fs';
import * as path from 'path';

describe('Project Structure', () => {
  test('should have all required configuration files', () => {
    const requiredFiles = [
      'package.json',
      'tsconfig.json',
      'webpack.config.js',
      '.eslintrc.js',
      '.prettierrc',
      'jest.config.js',
      'README.md'
    ];

    requiredFiles.forEach(file => {
      expect(fs.existsSync(path.join(process.cwd(), file))).toBe(true);
    });
  });

  test('should have correct source directory structure', () => {
    const requiredDirs = [
      'src/main',
      'src/renderer', 
      'src/mock-server',
      'src/types'
    ];

    requiredDirs.forEach(dir => {
      expect(fs.existsSync(path.join(process.cwd(), dir))).toBe(true);
    });
  });

  test('should have main entry files', () => {
    const entryFiles = [
      'src/main/main.ts',
      'src/renderer/index.tsx',
      'src/renderer/App.tsx',
      'src/mock-server/server.ts',
      'src/types/index.ts'
    ];

    entryFiles.forEach(file => {
      expect(fs.existsSync(path.join(process.cwd(), file))).toBe(true);
    });
  });

  test('package.json should have correct main entry', () => {
    const packageJson = JSON.parse(fs.readFileSync('package.json', 'utf8'));
    expect(packageJson.main).toBe('dist/main.js');
    expect(packageJson.name).toBe('manufacturing-digital-twin-demo');
  });

  test('should have required npm scripts', () => {
    const packageJson = JSON.parse(fs.readFileSync('package.json', 'utf8'));
    const requiredScripts = [
      'build',
      'build:dev', 
      'start',
      'dev',
      'test',
      'mock-server'
    ];

    requiredScripts.forEach(script => {
      expect(packageJson.scripts[script]).toBeDefined();
    });
  });
});