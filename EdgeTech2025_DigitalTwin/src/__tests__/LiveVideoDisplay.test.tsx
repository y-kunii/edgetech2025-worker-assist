import React from 'react';
import { render, screen, fireEvent, waitFor } from '@testing-library/react';
import '@testing-library/jest-dom';
import LiveVideoDisplay from '../renderer/components/LiveVideoDisplay';

// Mock the CSS imports
jest.mock('../renderer/styles/layout.css', () => ({}));

describe('LiveVideoDisplay', () => {
  const mockBase64Image = 'iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAYAAAAfFcSJAAAADUlEQVR42mNkYPhfDwAChAI9jU77zgAAAABJRU5ErkJggg==';

  beforeEach(() => {
    jest.clearAllMocks();
  });

  describe('接続状態の表示', () => {
    it('接続中の場合、接続インジケーターを表示する', () => {
      render(<LiveVideoDisplay isConnected={true} />);
      
      expect(screen.getByText('接続中')).toBeInTheDocument();
      expect(screen.getByText('📹 ライブ映像')).toBeInTheDocument();
    });

    it('切断中の場合、切断メッセージを表示する', () => {
      render(<LiveVideoDisplay isConnected={false} />);
      
      expect(screen.getByText('切断')).toBeInTheDocument();
      expect(screen.getByText('接続が切断されています')).toBeInTheDocument();
      expect(screen.getByText('WebSocket接続を確認してください')).toBeInTheDocument();
    });
  });

  describe('画像表示機能', () => {
    it('画像データがない場合、待機メッセージを表示する', () => {
      render(<LiveVideoDisplay isConnected={true} />);
      
      expect(screen.getByText('映像待機中...')).toBeInTheDocument();
      expect(screen.getByText('カメラからの映像を待機しています')).toBeInTheDocument();
    });

    it('画像データがある場合、画像を表示する', async () => {
      render(<LiveVideoDisplay isConnected={true} image={mockBase64Image} />);
      
      const image = screen.getByAltText('Live video with skeleton overlay');
      expect(image).toBeInTheDocument();
      expect(image).toHaveAttribute('src', `data:image/jpeg;base64,${mockBase64Image}`);
    });

    it('画像読み込み中はローディング表示する', () => {
      render(<LiveVideoDisplay isConnected={true} image={mockBase64Image} />);
      
      expect(screen.getByText('読み込み中...')).toBeInTheDocument();
    });
  });

  describe('エラーハンドリング', () => {
    it('画像読み込みエラー時の基本動作を確認', () => {
      render(<LiveVideoDisplay isConnected={true} image="invalid-base64" />);
      
      const image = screen.getByAltText('Live video with skeleton overlay');
      expect(image).toBeInTheDocument();
      expect(image).toHaveAttribute('src', 'data:image/jpeg;base64,invalid-base64');
    });

    it('接続切断時のエラー表示', () => {
      render(<LiveVideoDisplay isConnected={false} image="some-image" />);
      
      expect(screen.getByText('接続が切断されています')).toBeInTheDocument();
      expect(screen.getByText('WebSocket接続を確認してください')).toBeInTheDocument();
    });

    it('画像データなしの場合の表示', () => {
      render(<LiveVideoDisplay isConnected={true} />);
      
      expect(screen.getByText('映像待機中...')).toBeInTheDocument();
      expect(screen.getByText('カメラからの映像を待機しています')).toBeInTheDocument();
    });
  });

  describe('レスポンシブ機能', () => {
    it('画像が正しいスタイルで表示される', async () => {
      render(<LiveVideoDisplay isConnected={true} image={mockBase64Image} />);
      
      const image = screen.getByAltText('Live video with skeleton overlay');
      
      // 画像読み込み成功をシミュレート
      fireEvent.load(image);
      
      await waitFor(() => {
        expect(image).toHaveStyle({
          maxWidth: '100%',
          maxHeight: '100%',
          objectFit: 'contain',
          borderRadius: '8px'
        });
      });
    });
  });

  describe('統計情報表示', () => {
    it('画像読み込み成功時に画像サイズを表示する', async () => {
      render(<LiveVideoDisplay isConnected={true} image={mockBase64Image} />);
      
      const image = screen.getByAltText('Live video with skeleton overlay');
      
      // naturalWidth/Heightをモック
      Object.defineProperty(image, 'naturalWidth', {
        get: () => 640,
        configurable: true
      });
      Object.defineProperty(image, 'naturalHeight', {
        get: () => 480,
        configurable: true
      });
      
      fireEvent.load(image);
      
      await waitFor(() => {
        expect(screen.getByText('640 × 480')).toBeInTheDocument();
      });
    });
  });

  describe('アニメーション効果', () => {
    it('待機中アニメーションが表示される', () => {
      render(<LiveVideoDisplay isConnected={true} />);
      
      const waitingAnimation = document.querySelector('.waiting-animation');
      expect(waitingAnimation).toBeInTheDocument();
      
      const dots = waitingAnimation?.querySelectorAll('.dot');
      expect(dots).toHaveLength(3);
    });
  });
});