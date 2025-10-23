import { FileWatcher, FileWatcherConfig, ImageUpdateEvent } from '../../src/main/file-watcher';
import fs from 'fs';
import path from 'path';
import { EventEmitter } from 'events';

describe('File Watcher Integration Tests', () => {
  let fileWatcher: FileWatcher;
  let testDir: string;
  let config: FileWatcherConfig;

  beforeEach(async () => {
    // Create temporary test directory
    testDir = path.join('/tmp', `test-images-${Date.now()}`);
    fs.mkdirSync(testDir, { recursive: true });

    config = {
      watchPath: testDir,
      supportedExtensions: ['.jpg', '.jpeg', '.png', '.bmp'],
      debounceDelay: 100,
      maxFileSize: 10 * 1024 * 1024, // 10MB
    };

    fileWatcher = new FileWatcher(config);
  });

  afterEach(async () => {
    if (fileWatcher) {
      await fileWatcher.stop();
    }

    // Clean up test directory
    if (fs.existsSync(testDir)) {
      const files = fs.readdirSync(testDir);
      files.forEach(file => {
        fs.unlinkSync(path.join(testDir, file));
      });
      fs.rmdirSync(testDir);
    }
  });

  describe('File Detection and Monitoring', () => {
    test('should detect existing image files on startup', async () => {
      // Create test image files before starting watcher
      const testFiles = ['image1.jpg', 'image2.png', 'image3.jpeg'];
      testFiles.forEach((filename, index) => {
        const filePath = path.join(testDir, filename);
        const content = `test image content ${index}`;
        fs.writeFileSync(filePath, content);
        
        // Set different modification times
        const time = new Date(Date.now() - (testFiles.length - index) * 1000);
        fs.utimesSync(filePath, time, time);
      });

      await fileWatcher.start();

      // The latest file should be detected
      const latestPath = fileWatcher.getLatestImagePath();
      expect(latestPath).toBeTruthy();
      expect(latestPath).toContain('image1.jpg'); // Most recent file
    });

    test('should detect new image files', (done) => {
      fileWatcher.on('imageUpdated', (event: ImageUpdateEvent) => {
        expect(event.fileName).toBe('new-image.jpg');
        expect(event.filePath).toContain('new-image.jpg');
        expect(event.size).toBeGreaterThan(0);
        expect(event.timestamp).toBeInstanceOf(Date);
        done();
      });

      fileWatcher.start().then(() => {
        // Create new image file after watcher is started
        setTimeout(() => {
          const filePath = path.join(testDir, 'new-image.jpg');
          fs.writeFileSync(filePath, 'new image content');
        }, 200);
      });
    }, 10000);

    test('should update latest image when newer file is added', (done) => {
      let updateCount = 0;
      const expectedFiles = ['first.jpg', 'second.png'];

      fileWatcher.on('imageUpdated', (event: ImageUpdateEvent) => {
        updateCount++;
        
        if (updateCount === 1) {
          expect(event.fileName).toBe('first.jpg');
          
          // Add second file after first is detected
          setTimeout(() => {
            const secondPath = path.join(testDir, 'second.png');
            fs.writeFileSync(secondPath, 'second image content');
          }, 100);
        } else if (updateCount === 2) {
          expect(event.fileName).toBe('second.png');
          
          // Verify latest path is updated
          const latestPath = fileWatcher.getLatestImagePath();
          expect(latestPath).toContain('second.png');
          done();
        }
      });

      fileWatcher.start().then(() => {
        setTimeout(() => {
          const firstPath = path.join(testDir, 'first.jpg');
          fs.writeFileSync(firstPath, 'first image content');
        }, 200);
      });
    }, 10000);

    test('should ignore non-image files', (done) => {
      let imageUpdateCount = 0;

      fileWatcher.on('imageUpdated', (event: ImageUpdateEvent) => {
        imageUpdateCount++;
        expect(event.fileName).toBe('valid-image.jpg');
        
        // Should only receive one update for the image file
        setTimeout(() => {
          expect(imageUpdateCount).toBe(1);
          done();
        }, 500);
      });

      fileWatcher.start().then(() => {
        setTimeout(() => {
          // Create non-image files
          fs.writeFileSync(path.join(testDir, 'document.txt'), 'text content');
          fs.writeFileSync(path.join(testDir, 'data.json'), '{"key": "value"}');
          
          // Create valid image file
          fs.writeFileSync(path.join(testDir, 'valid-image.jpg'), 'image content');
        }, 200);
      });
    }, 10000);

    test('should handle file modifications', (done) => {
      let updateCount = 0;

      fileWatcher.on('imageUpdated', (event: ImageUpdateEvent) => {
        updateCount++;
        
        if (updateCount === 1) {
          expect(event.fileName).toBe('modified-image.jpg');
          
          // Modify the file
          setTimeout(() => {
            const filePath = path.join(testDir, 'modified-image.jpg');
            fs.writeFileSync(filePath, 'modified image content');
          }, 200);
        } else if (updateCount === 2) {
          expect(event.fileName).toBe('modified-image.jpg');
          expect(event.size).toBeGreaterThan(0);
          done();
        }
      });

      fileWatcher.start().then(() => {
        setTimeout(() => {
          const filePath = path.join(testDir, 'modified-image.jpg');
          fs.writeFileSync(filePath, 'original content');
        }, 200);
      });
    }, 10000);
  });

  describe('File Deletion Handling', () => {
    test('should handle file deletion and find new latest image', (done) => {
      let updateCount = 0;
      const files = ['old-image.jpg', 'latest-image.png'];

      fileWatcher.on('imageUpdated', (event: ImageUpdateEvent) => {
        updateCount++;
        
        if (updateCount === 2) {
          // Both files created, now delete the latest
          setTimeout(() => {
            const latestPath = path.join(testDir, 'latest-image.png');
            fs.unlinkSync(latestPath);
            
            // Check that the watcher falls back to the older file
            setTimeout(() => {
              const currentLatest = fileWatcher.getLatestImagePath();
              expect(currentLatest).toContain('old-image.jpg');
              done();
            }, 300);
          }, 200);
        }
      });

      fileWatcher.start().then(() => {
        // Create files with different timestamps
        setTimeout(() => {
          const oldPath = path.join(testDir, 'old-image.jpg');
          fs.writeFileSync(oldPath, 'old content');
          
          setTimeout(() => {
            const latestPath = path.join(testDir, 'latest-image.png');
            fs.writeFileSync(latestPath, 'latest content');
          }, 100);
        }, 200);
      });
    }, 10000);

    test('should handle deletion of all files', (done) => {
      fileWatcher.on('imageUpdated', (event: ImageUpdateEvent) => {
        // Delete the file after it's detected
        setTimeout(() => {
          const filePath = path.join(testDir, 'only-image.jpg');
          fs.unlinkSync(filePath);
          
          // Check that no latest image is available
          setTimeout(() => {
            const latestPath = fileWatcher.getLatestImagePath();
            expect(latestPath).toBeNull();
            done();
          }, 300);
        }, 200);
      });

      fileWatcher.start().then(() => {
        setTimeout(() => {
          const filePath = path.join(testDir, 'only-image.jpg');
          fs.writeFileSync(filePath, 'only image content');
        }, 200);
      });
    }, 10000);
  });

  describe('Configuration and Error Handling', () => {
    test('should handle non-existent watch directory', async () => {
      const invalidConfig = {
        ...config,
        watchPath: '/non/existent/path'
      };

      const invalidWatcher = new FileWatcher(invalidConfig);
      
      // Should not throw error, but should handle gracefully
      await expect(invalidWatcher.start()).resolves.not.toThrow();
      
      const latestPath = invalidWatcher.getLatestImagePath();
      expect(latestPath).toBeNull();
      
      await invalidWatcher.stop();
    });

    test('should respect file size limits', (done) => {
      const smallSizeConfig = {
        ...config,
        maxFileSize: 10 // Very small limit
      };

      const smallSizeWatcher = new FileWatcher(smallSizeConfig);
      let updateReceived = false;

      smallSizeWatcher.on('imageUpdated', () => {
        updateReceived = true;
      });

      smallSizeWatcher.start().then(() => {
        setTimeout(() => {
          // Create file larger than limit
          const largePath = path.join(testDir, 'large-image.jpg');
          const largeContent = 'x'.repeat(100); // Larger than 10 bytes
          fs.writeFileSync(largePath, largeContent);
          
          // Wait and check that no update was received
          setTimeout(() => {
            expect(updateReceived).toBe(false);
            smallSizeWatcher.stop().then(() => done());
          }, 500);
        }, 200);
      });
    }, 10000);

    test('should handle file extension filtering', (done) => {
      const restrictiveConfig = {
        ...config,
        supportedExtensions: ['.jpg'] // Only JPG files
      };

      const restrictiveWatcher = new FileWatcher(restrictiveConfig);
      let updateCount = 0;

      restrictiveWatcher.on('imageUpdated', (event: ImageUpdateEvent) => {
        updateCount++;
        expect(event.fileName).toBe('valid.jpg');
      });

      restrictiveWatcher.start().then(() => {
        setTimeout(() => {
          // Create files with different extensions
          fs.writeFileSync(path.join(testDir, 'invalid.png'), 'png content');
          fs.writeFileSync(path.join(testDir, 'invalid.gif'), 'gif content');
          fs.writeFileSync(path.join(testDir, 'valid.jpg'), 'jpg content');
          
          setTimeout(() => {
            expect(updateCount).toBe(1); // Only JPG file should be detected
            restrictiveWatcher.stop().then(() => done());
          }, 500);
        }, 200);
      });
    }, 10000);
  });

  describe('Performance and Debouncing', () => {
    test('should debounce rapid file changes', (done) => {
      let updateCount = 0;
      const debounceConfig = {
        ...config,
        debounceDelay: 200
      };

      const debounceWatcher = new FileWatcher(debounceConfig);

      debounceWatcher.on('imageUpdated', () => {
        updateCount++;
      });

      debounceWatcher.start().then(() => {
        const filePath = path.join(testDir, 'rapid-changes.jpg');
        
        // Make rapid changes
        setTimeout(() => {
          for (let i = 0; i < 5; i++) {
            setTimeout(() => {
              fs.writeFileSync(filePath, `content ${i}`);
            }, i * 20); // 20ms intervals
          }
          
          // Check after debounce period
          setTimeout(() => {
            expect(updateCount).toBeLessThan(5); // Should be debounced
            debounceWatcher.stop().then(() => done());
          }, 1000);
        }, 200);
      });
    }, 10000);

    test('should handle concurrent file operations', (done) => {
      let updateCount = 0;
      const expectedFiles = 5;

      fileWatcher.on('imageUpdated', () => {
        updateCount++;
        if (updateCount === expectedFiles) {
          done();
        }
      });

      fileWatcher.start().then(() => {
        // Create multiple files concurrently
        setTimeout(() => {
          for (let i = 0; i < expectedFiles; i++) {
            setTimeout(() => {
              const filePath = path.join(testDir, `concurrent-${i}.jpg`);
              fs.writeFileSync(filePath, `content ${i}`);
            }, i * 50); // Staggered creation
          }
        }, 200);
      });
    }, 10000);
  });

  describe('State Management', () => {
    test('should maintain correct state after start/stop cycles', async () => {
      // Start watcher
      await fileWatcher.start();
      expect(fileWatcher.isWatching()).toBe(true);

      // Create a file
      const filePath = path.join(testDir, 'state-test.jpg');
      fs.writeFileSync(filePath, 'test content');

      // Wait for detection
      await new Promise(resolve => setTimeout(resolve, 300));

      // Stop watcher
      await fileWatcher.stop();
      expect(fileWatcher.isWatching()).toBe(false);

      // Latest path should still be available
      const latestPath = fileWatcher.getLatestImagePath();
      expect(latestPath).toContain('state-test.jpg');

      // Restart watcher
      await fileWatcher.start();
      expect(fileWatcher.isWatching()).toBe(true);

      // Should still have the same latest path
      const latestPathAfterRestart = fileWatcher.getLatestImagePath();
      expect(latestPathAfterRestart).toBe(latestPath);
    });

    test('should update configuration dynamically', async () => {
      await fileWatcher.start();

      // Update configuration
      const newConfig = {
        ...config,
        supportedExtensions: ['.png'] // Change to only PNG
      };

      fileWatcher.updateConfig(newConfig);

      // Create files with different extensions
      let pngDetected = false;
      let jpgDetected = false;

      fileWatcher.on('imageUpdated', (event: ImageUpdateEvent) => {
        if (event.fileName.endsWith('.png')) {
          pngDetected = true;
        } else if (event.fileName.endsWith('.jpg')) {
          jpgDetected = true;
        }
      });

      setTimeout(() => {
        fs.writeFileSync(path.join(testDir, 'test.jpg'), 'jpg content');
        fs.writeFileSync(path.join(testDir, 'test.png'), 'png content');
      }, 200);

      await new Promise(resolve => setTimeout(resolve, 500));

      expect(pngDetected).toBe(true);
      expect(jpgDetected).toBe(false); // Should be ignored due to config change
    });
  });
});