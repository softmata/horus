#!/bin/bash

# HORUS Documentation Reorganization Script
# Implements the full documentation improvement plan

set -e
cd "$(dirname "$0")/../content/docs"

echo "========================================="
echo "HORUS Documentation Reorganization"
echo "========================================="
echo ""

# Colors
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
NC='\033[0m'

# Helper function to update order in frontmatter
update_order() {
    local file=$1
    local new_order=$2

    if [ -f "$file" ]; then
        # Use sed to replace the order field
        sed -i "s/^order: .*/order: $new_order/" "$file"
        echo -e "${GREEN}${NC} Updated $file  order: $new_order"
    else
        echo -e "${YELLOW}[WARNING]${NC} File not found: $file"
    fi
}

echo "=== PHASE 1: Fix Order Numbers & Reorganize ==="
echo ""

# Getting Started Section (1-4)
echo "Getting Started Section..."
update_order "getting-started.mdx" 1
# Note: will create what-is-horus.mdx separately
# update_order "installation.mdx" 2  # Doesn't exist yet
# update_order "quick-start.mdx" 3   # Doesn't exist yet

# Core Concepts (5-8)
echo "Core Concepts Section..."
update_order "core-concepts-nodes.mdx" 5
update_order "core-concepts-hub.mdx" 6
update_order "core-concepts-scheduler.mdx" 7
update_order "message-types.mdx" 8
# Note: will create second-application.mdx as order 9

# Developer Tools (10-13)
echo "Developer Tools Section..."
update_order "node-macro.mdx" 10
update_order "monitor.mdx" 11
update_order "troubleshooting.mdx" 12  # Will enhance later
# Note: will create testing.mdx as order 13

# Building Applications (14-18)
echo "Building Applications Section..."
update_order "using-prebuilt-nodes.mdx" 14
# Note: will create basic-examples.mdx as order 15
# Note: will create design-patterns.mdx as order 16
# Note: will create using-packages.mdx as order 17
update_order "multi-language.mdx" 18

# Advanced Topics (20-25)
echo "Advanced Topics Section..."
# Note: will create advanced-examples.mdx as order 20
update_order "performance.mdx" 21
update_order "core-concepts-shared-memory.mdx" 22
update_order "core-concepts-link.mdx" 23
update_order "architecture.mdx" 24
update_order "benchmarks.mdx" 25

# Production & Publishing (30-33)
echo "Production & Publishing Section..."
update_order "package-management.mdx" 30  # Will split later
update_order "environment-management.mdx" 31
update_order "remote-deployment.mdx" 32
update_order "authentication.mdx" 33

# Reference (40-45)
echo "Reference Section..."
update_order "cli-reference.mdx" 40
update_order "api.mdx" 41
update_order "api-node.mdx" 41  # Will consolidate
update_order "api-hub.mdx" 41   # Will consolidate
update_order "api-scheduler.mdx" 41  # Will consolidate
update_order "api-link.mdx" 41  # Will consolidate
update_order "library-reference.mdx" 42
update_order "goals.mdx" 43
update_order "roadmap.mdx" 44
update_order "examples.mdx" 45  # Will split later

# Integrations (50-52)
echo "Integrations Section..."
update_order "ai-integration.mdx" 50
update_order "python-bindings.mdx" 51
update_order "c-bindings.mdx" 52

# Special/unlisted docs
update_order "core.mdx" 99  # Internal/meta doc
update_order "message-macro.mdx" 10  # Should be near node-macro
update_order "parameters.mdx" 16  # Should be in Building Applications

echo ""
echo "========================================="
echo "Order numbers updated successfully!"
echo "========================================="
echo ""
echo "Next steps (manual):"
echo "1. Create new documentation files:"
echo "   - what-is-horus.mdx"
echo "   - second-application.mdx"
echo "   - troubleshooting-runtime.mdx"
echo "   - testing.mdx"
echo "   - design-patterns.mdx"
echo "   - basic-examples.mdx"
echo "   - advanced-examples.mdx"
echo "   - using-packages.mdx"
echo "   - publishing-packages.mdx"
echo "   - api-reference.mdx"
echo ""
echo "2. Add 'Key Takeaways' sections to core-concepts-*.mdx"
echo ""
echo "3. Split existing files:"
echo "   - examples.mdx  basic + advanced"
echo "   - package-management.mdx  using + publishing"
echo ""
echo "4. Consolidate API docs into single api-reference.mdx"
echo ""
echo "Run 'npm run dev' to see changes"
echo "========================================="
