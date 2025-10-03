const path = require('path');
const HtmlWebpackPlugin = require('html-webpack-plugin');

const mainConfig = {
  mode: 'development',
  entry: './src/main/main.ts',
  target: 'electron-main',
  module: {
    rules: [
      {
        test: /\.ts$/,
        use: 'ts-loader',
        exclude: /node_modules/,
      },
    ],
  },
  resolve: {
    extensions: ['.ts', '.js'],
    fallback: {
      "bufferutil": false,
      "utf-8-validate": false
    }
  },
  output: {
    filename: 'main.js',
    path: path.resolve(__dirname, 'dist'),
  },
  node: {
    __dirname: false,
    __filename: false,
  },
  ignoreWarnings: [
    /Module not found: Error: Can't resolve 'bufferutil'/,
    /Module not found: Error: Can't resolve 'utf-8-validate'/,
    /Critical dependency: the request of a dependency is an expression/
  ],
};

const rendererConfig = {
  mode: 'development',
  entry: './src/renderer/index.tsx',
  target: 'electron-renderer',
  module: {
    rules: [
      {
        test: /\.tsx?$/,
        use: 'ts-loader',
        exclude: /node_modules/,
      },
      {
        test: /\.css$/,
        use: ['style-loader', 'css-loader'],
      },
    ],
  },
  resolve: {
    extensions: ['.tsx', '.ts', '.js'],
  },
  output: {
    filename: 'renderer.js',
    path: path.resolve(__dirname, 'dist'),
  },
  plugins: [
    new HtmlWebpackPlugin({
      template: './src/renderer/index.html',
      filename: 'index.html',
    }),
  ],
};

const mockServerConfig = {
  mode: 'development',
  entry: './src/mock-server/server.ts',
  target: 'node',
  module: {
    rules: [
      {
        test: /\.ts$/,
        use: 'ts-loader',
        exclude: /node_modules/,
      },
    ],
  },
  resolve: {
    extensions: ['.ts', '.js'],
  },
  output: {
    filename: 'mock-server.js',
    path: path.resolve(__dirname, 'dist'),
  },
  node: {
    __dirname: false,
    __filename: false,
  },
  ignoreWarnings: [
    /Module not found: Error: Can't resolve 'bufferutil'/,
    /Module not found: Error: Can't resolve 'utf-8-validate'/,
    /Critical dependency: the request of a dependency is an expression/
  ],
};

module.exports = [mainConfig, rendererConfig, mockServerConfig];