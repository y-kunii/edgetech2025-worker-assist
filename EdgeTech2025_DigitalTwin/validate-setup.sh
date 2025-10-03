#!/bin/bash

echo "🔍 Validating project setup..."
echo ""

# Check required files
echo "📁 Checking configuration files:"
files=("package.json" "tsconfig.json" "webpack.config.js" ".eslintrc.js" ".prettierrc" "jest.config.js" "README.md" ".gitignore")

for file in "${files[@]}"; do
  if [ -f "$file" ]; then
    echo "  ✅ $file"
  else
    echo "  ❌ $file"
  fi
done

# Check source directories
echo ""
echo "📂 Checking source directories:"
dirs=("src/main" "src/renderer" "src/mock-server" "src/types" "src/__tests__")

for dir in "${dirs[@]}"; do
  if [ -d "$dir" ]; then
    echo "  ✅ $dir"
  else
    echo "  ❌ $dir"
  fi
done

# Check entry files
echo ""
echo "📄 Checking entry files:"
entry_files=("src/main/main.ts" "src/renderer/index.tsx" "src/renderer/App.tsx" "src/renderer/index.html" "src/mock-server/server.ts" "src/mock-server/mockDataGenerator.ts" "src/types/index.ts")

for file in "${entry_files[@]}"; do
  if [ -f "$file" ]; then
    echo "  ✅ $file"
  else
    echo "  ❌ $file"
  fi
done

echo ""
echo "🎉 Project structure validation complete!"
echo ""
echo "📋 Next steps:"
echo "  1. Install Node.js if not already installed"
echo "  2. Install dependencies: npm install"
echo "  3. Build project: npm run build:dev"
echo "  4. Start mock server: npm run mock-server"
echo "  5. Start application: npm start"
echo "  6. Or run in dev mode: npm run dev"