// Mock electron module for testing
export const app = {
  getPath: jest.fn((name: string) => {
    if (name === 'userData') {
      return '/tmp/test-user-data';
    }
    return '/tmp/test-path';
  }),
  on: jest.fn(),
  quit: jest.fn(),
  isReady: jest.fn(() => true),
};

export const BrowserWindow = jest.fn(() => ({
  loadFile: jest.fn(),
  on: jest.fn(),
  webContents: {
    send: jest.fn(),
    on: jest.fn(),
  },
}));

export const ipcMain = {
  handle: jest.fn(),
  on: jest.fn(),
  removeAllListeners: jest.fn(),
};