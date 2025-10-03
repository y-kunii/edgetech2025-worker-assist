import React, { Component, ErrorInfo, ReactNode } from 'react';

interface Props {
  children: ReactNode;
}

interface State {
  hasError: boolean;
  error?: Error;
}

class ErrorBoundary extends Component<Props, State> {
  public state: State = {
    hasError: false
  };

  public static getDerivedStateFromError(error: Error): State {
    return { hasError: true, error };
  }

  public componentDidCatch(error: Error, errorInfo: ErrorInfo) {
    console.error('Uncaught error:', error, errorInfo);
  }

  public render() {
    if (this.state.hasError) {
      return (
        <div style={{
          padding: '20px',
          textAlign: 'center',
          backgroundColor: '#f5f5f5',
          height: '100vh',
          display: 'flex',
          flexDirection: 'column',
          justifyContent: 'center',
          alignItems: 'center'
        }}>
          <h2>🚨 アプリケーションエラー</h2>
          <p>申し訳ございません。アプリケーションでエラーが発生しました。</p>
          <details style={{ marginTop: '20px', textAlign: 'left' }}>
            <summary>エラー詳細</summary>
            <pre style={{ 
              backgroundColor: '#fff', 
              padding: '10px', 
              border: '1px solid #ccc',
              borderRadius: '4px',
              marginTop: '10px',
              fontSize: '12px'
            }}>
              {this.state.error?.stack}
            </pre>
          </details>
          <button 
            onClick={() => window.location.reload()}
            style={{
              marginTop: '20px',
              padding: '10px 20px',
              backgroundColor: '#2196F3',
              color: 'white',
              border: 'none',
              borderRadius: '4px',
              cursor: 'pointer'
            }}
          >
            🔄 アプリケーションを再読み込み
          </button>
        </div>
      );
    }

    return this.props.children;
  }
}

export default ErrorBoundary;