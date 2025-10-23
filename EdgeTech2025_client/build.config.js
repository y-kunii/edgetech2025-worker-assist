/**
 * Build configuration for Digital Twin Dashboard
 * Centralizes build settings for different environments and platforms
 */

const path = require('path');
const os = require('os');

const isDevelopment = process.env.NODE_ENV === 'development';
const isProduction = process.env.NODE_ENV === 'production';
const platform = process.platform;

// Base configuration
const baseConfig = {
  appId: 'com.digitaltwin.dashboard',
  productName: 'Digital Twin Dashboard',
  copyright: 'Copyright © 2024 Digital Twin Team',
  
  // Directories
  directories: {
    output: 'release',
    buildResources: 'build'
  },
  
  // Files to include
  files: [
    'dist/**/*',
    'package.json',
    'node_modules/**/*',
    '!node_modules/**/*.{md,txt}',
    '!node_modules/**/test/**',
    '!node_modules/**/tests/**',
    '!node_modules/**/*.test.*',
    '!node_modules/**/*.spec.*'
  ],
  
  // Extra resources
  extraResources: [
    {
      from: 'sample_data',
      to: 'sample_data'
    }
  ]
};

// Platform-specific configurations
const platformConfigs = {
  darwin: {
    mac: {
      category: 'public.app-category.productivity',
      icon: 'build/icon.icns',
      target: [
        { target: 'dmg', arch: ['x64', 'arm64'] },
        { target: 'zip', arch: ['x64', 'arm64'] }
      ],
      hardenedRuntime: true,
      gatekeeperAssess: false,
      entitlements: 'build/entitlements.mac.plist',
      entitlementsInherit: 'build/entitlements.mac.plist',
      // Code signing configuration
      identity: process.env.CSC_NAME || null,
      provisioningProfile: process.env.PROVISIONING_PROFILE || null
    },
    dmg: {
      title: '${productName} ${version}',
      artifactName: '${productName}-${version}-${arch}.${ext}',
      background: 'build/dmg-background.png',
      iconSize: 100,
      contents: [
        { x: 380, y: 280, type: 'link', path: '/Applications' },
        { x: 110, y: 280, type: 'file' }
      ],
      window: { width: 540, height: 380 }
    }
  },
  
  win32: {
    win: {
      icon: 'build/icon.ico',
      target: [
        { target: 'nsis', arch: ['x64', 'ia32'] },
        { target: 'portable', arch: ['x64', 'ia32'] },
        { target: 'zip', arch: ['x64', 'ia32'] }
      ],
      publisherName: 'Digital Twin Team',
      verifyUpdateCodeSignature: false,
      // Code signing configuration
      certificateFile: process.env.CSC_LINK || null,
      certificatePassword: process.env.CSC_KEY_PASSWORD || null
    },
    nsis: {
      oneClick: false,
      allowToChangeInstallationDirectory: true,
      createDesktopShortcut: true,
      createStartMenuShortcut: true,
      shortcutName: 'Digital Twin Dashboard',
      include: 'build/installer.nsh',
      artifactName: '${productName}-Setup-${version}.${ext}',
      deleteAppDataOnUninstall: false,
      // Installer customization
      installerIcon: 'build/icon.ico',
      uninstallerIcon: 'build/icon.ico',
      installerHeaderIcon: 'build/icon.ico'
    }
  },
  
  linux: {
    linux: {
      icon: 'build/icon.png',
      target: [
        { target: 'AppImage', arch: ['x64'] },
        { target: 'deb', arch: ['x64'] },
        { target: 'rpm', arch: ['x64'] },
        { target: 'tar.gz', arch: ['x64'] }
      ],
      category: 'Development',
      desktop: {
        Name: 'Digital Twin Dashboard',
        Comment: '工場現場向けデジタルツインデモプログラム',
        Keywords: 'digital-twin;dashboard;factory;electron;'
      }
    },
    appImage: {
      artifactName: '${productName}-${version}-${arch}.${ext}'
    },
    deb: {
      artifactName: '${productName}-${version}-${arch}.${ext}',
      depends: [
        'gconf2',
        'gconf-service', 
        'libnotify4',
        'libappindicator1',
        'libxtst6',
        'libnss3'
      ]
    },
    rpm: {
      artifactName: '${productName}-${version}-${arch}.${ext}',
      depends: ['libXScrnSaver']
    }
  }
};

// Environment-specific overrides
const environmentConfigs = {
  development: {
    compression: 'store', // Faster builds
    removePackageScripts: false,
    // Include source maps and debug info
    files: [
      ...baseConfig.files,
      'src/**/*',
      '*.map'
    ]
  },
  
  production: {
    compression: 'maximum',
    removePackageScripts: true,
    // Exclude development files
    files: [
      ...baseConfig.files,
      '!**/*.map',
      '!src/**/*'
    ]
  },
  
  test: {
    compression: 'store',
    removePackageScripts: false,
    // Include test files for validation
    files: [
      ...baseConfig.files,
      'tests/**/*'
    ]
  }
};

// Auto-update configuration
const autoUpdateConfig = {
  publish: {
    provider: process.env.PUBLISH_PROVIDER || 'generic',
    url: process.env.PUBLISH_URL || 'https://releases.example.com/',
    channel: process.env.PUBLISH_CHANNEL || 'latest'
  }
};

// Build the final configuration
function createConfig() {
  const config = { ...baseConfig };
  
  // Add platform-specific config
  if (platformConfigs[platform]) {
    Object.assign(config, platformConfigs[platform]);
  }
  
  // Add environment-specific config
  const envConfig = environmentConfigs[process.env.NODE_ENV] || environmentConfigs.production;
  Object.assign(config, envConfig);
  
  // Add auto-update config if enabled
  if (process.env.ENABLE_AUTO_UPDATE === 'true') {
    config.publish = autoUpdateConfig.publish;
  }
  
  return config;
}

module.exports = {
  baseConfig,
  platformConfigs,
  environmentConfigs,
  autoUpdateConfig,
  createConfig,
  
  // Utility functions
  isDevelopment,
  isProduction,
  platform,
  
  // Build helpers
  getOutputPath: () => path.resolve(__dirname, 'release'),
  getBuildResourcesPath: () => path.resolve(__dirname, 'build'),
  getDistPath: () => path.resolve(__dirname, 'dist')
};