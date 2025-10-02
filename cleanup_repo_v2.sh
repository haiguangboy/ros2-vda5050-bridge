#!/bin/bash
set -e

echo "Starting repository cleanup (v2)..."

# Navigate to repository root
cd /home/yhg/Documents/docs/zhongli

echo "Current files:"
ls -A

# Step 1: Use cpp_version_temp (which has the latest content)
echo ""
echo "Step 1: Using cpp_version_temp as source..."

# Step 2: Move cpp_version_temp contents to root
echo "Step 2: Moving cpp_version_temp contents to root..."
mv cpp_version_temp/* .
mv cpp_version_temp/.* . 2>/dev/null || true
rmdir cpp_version_temp

# Step 3: Remove backup and old files
echo "Step 3: Cleaning up..."
rm -rf cpp_version_backup cleanup_repo.sh README.md

echo ""
echo "Cleanup complete! Current directory structure:"
ls -A

# Step 4: Git add all changes
echo ""
echo "Step 4: Adding changes to git..."
git add -A

# Step 5: Commit changes
echo "Step 5: Committing changes..."
git commit -m "$(cat <<'EOF'
refactor: move cpp_version to root, remove legacy Python code

- Move all cpp_version contents to repository root
- Remove legacy Python VDA5050 bridge code
- Remove temporary test files and documentation
- Clean repository structure for C++ ROS2 bridge only
- Keep Beta-3 protocol implementation and improvements

🤖 Generated with [Claude Code](https://claude.com/claude-code)

Co-Authored-By: Claude <noreply@anthropic.com>
EOF
)"

# Step 6: Show status
echo ""
echo "Step 6: Git status..."
git status
git log --oneline -3

echo ""
echo "✅ Cleanup complete!"
echo "📋 Repository structure:"
echo ""
ls -la
echo ""
echo "🚀 To push to remote run:"
echo "   git push origin main --force-with-lease"
