#!/bin/bash
#
# HORUS Comprehensive Release Script
#
# Updates ALL version references across the entire HORUS ecosystem:
#   - Cargo.toml package versions (17+ files)
#   - pyproject.toml Python packages (3 files)
#   - Python __version__ in __init__.py (3 files)
#   - Rust source hardcoded versions (8 files)
#   - YAML config files (11+ files)
#   - Documentation (.md, .mdx files)
#   - package.json (docs-site)
#   - Test files
#   - GitHub issue templates
#   - Install scripts
#
# Usage: ./scripts/release.sh <version>
# Example: ./scripts/release.sh 0.1.6
#

set -e  # Exit on error

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# Check arguments
if [ $# -eq 0 ]; then
    echo -e "${RED}Error: Version number required${NC}"
    echo ""
    echo "Usage: $0 <version>"
    echo "Example: $0 0.1.6"
    echo ""
    exit 1
fi

NEW_VERSION=$1

# Validate version format (semver)
if ! [[ $NEW_VERSION =~ ^[0-9]+\.[0-9]+\.[0-9]+(-[a-zA-Z0-9.]+)?$ ]]; then
    echo -e "${RED}Error: Invalid version format${NC}"
    echo "Expected format: X.Y.Z or X.Y.Z-suffix"
    echo "Examples: 0.1.6, 1.0.0, 0.2.0-beta1"
    exit 1
fi

# Check we're in the right directory
if [ ! -f "Cargo.toml" ] || [ ! -d "horus_py" ]; then
    echo -e "${RED}Error: Must be run from HORUS root directory${NC}"
    exit 1
fi

# Get current version
CURRENT_VERSION=$(grep -m1 '^version = ' horus/Cargo.toml | sed 's/version = "\(.*\)"/\1/')

echo -e "${BLUE}============================================${NC}"
echo -e "${BLUE}   HORUS Comprehensive Release Script${NC}"
echo -e "${BLUE}============================================${NC}"
echo ""
echo -e "${CYAN}Current version:${NC} $CURRENT_VERSION"
echo -e "${CYAN}New version:${NC}     $NEW_VERSION"
echo ""

# Check for uncommitted changes
if [ -n "$(git status --porcelain)" ]; then
    echo -e "${YELLOW}Warning: You have uncommitted changes${NC}"
    git status --short
    echo ""
    read -p "Continue anyway? [y/N]: " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        echo "Aborted"
        exit 1
    fi
fi

# Check if tag already exists
if git rev-parse "v$NEW_VERSION" >/dev/null 2>&1; then
    echo -e "${RED}Error: Tag v$NEW_VERSION already exists${NC}"
    exit 1
fi

# Confirm
read -p "$(echo -e ${YELLOW}Update version and create release? [y/N]:${NC}) " -n 1 -r
echo
if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    echo "Aborted"
    exit 0
fi

echo ""
echo -e "${CYAN}[1/10] Updating Cargo.toml package versions...${NC}"

# All Cargo.toml files with version
CARGO_FILES=(
    "horus/Cargo.toml"
    "horus_core/Cargo.toml"
    "horus_macros/Cargo.toml"
    "horus_library/Cargo.toml"
    "horus_py/Cargo.toml"
    "horus_manager/Cargo.toml"
    "horus_router/Cargo.toml"
    "benchmarks/Cargo.toml"
    "horus_library/python/Cargo.toml"
    "horus_library/tools/Cargo.toml"
    "horus_library/tools/sim2d/Cargo.toml"
    "horus_library/tools/sim3d/Cargo.toml"
    "horus_library/apps/tanksim/Cargo.toml"
    "tests/horus_run/Cargo.toml"
    "tests/horus_run/cargo_test/Cargo.toml"
)

for file in "${CARGO_FILES[@]}"; do
    if [ -f "$file" ]; then
        sed -i "s/^version = \"$CURRENT_VERSION\"/version = \"$NEW_VERSION\"/" "$file"
        echo -e "  ${GREEN}+${NC} $file"
    else
        echo -e "  ${YELLOW}-${NC} $file (not found)"
    fi
done

echo ""
echo -e "${CYAN}[2/10] Updating pyproject.toml files...${NC}"

PYPROJECT_FILES=(
    "horus_py/pyproject.toml"
    "horus_library/python/pyproject.toml"
    "horus_library/tools/sim3d/pyproject.toml"
)

for file in "${PYPROJECT_FILES[@]}"; do
    if [ -f "$file" ]; then
        sed -i "s/^version = \"$CURRENT_VERSION\"/version = \"$NEW_VERSION\"/" "$file"
        echo -e "  ${GREEN}+${NC} $file"
    fi
done

echo ""
echo -e "${CYAN}[3/10] Updating Python __version__...${NC}"

PYTHON_INIT_FILES=(
    "horus_py/horus/__init__.py"
    "horus_library/python/horus/library/__init__.py"
    "horus_library/tools/sim3d/python/sim3d_rl/__init__.py"
)

for file in "${PYTHON_INIT_FILES[@]}"; do
    if [ -f "$file" ]; then
        sed -i "s/__version__ = \"$CURRENT_VERSION\"/__version__ = \"$NEW_VERSION\"/" "$file"
        echo -e "  ${GREEN}+${NC} $file"
    fi
done

echo ""
echo -e "${CYAN}[4/10] Updating Rust source code versions...${NC}"

# main.rs - CLI version
if [ -f "horus_manager/src/main.rs" ]; then
    sed -i "s/#\[command(version = \"$CURRENT_VERSION\")]/#[command(version = \"$NEW_VERSION\")]/" horus_manager/src/main.rs
    echo -e "  ${GREEN}+${NC} horus_manager/src/main.rs (CLI version)"
fi

# run.rs - multiple version references
if [ -f "horus_manager/src/commands/run.rs" ]; then
    sed -i "s/version = \"$CURRENT_VERSION\"/version = \"$NEW_VERSION\"/g" horus_manager/src/commands/run.rs
    sed -i "s/horus = \"$CURRENT_VERSION\"/horus = \"$NEW_VERSION\"/g" horus_manager/src/commands/run.rs
    sed -i "s/version: $CURRENT_VERSION/version: $NEW_VERSION/g" horus_manager/src/commands/run.rs
    echo -e "  ${GREEN}+${NC} horus_manager/src/commands/run.rs"
fi

# new.rs - template version
if [ -f "horus_manager/src/commands/new.rs" ]; then
    sed -i "s/version: $CURRENT_VERSION/version: $NEW_VERSION/g" horus_manager/src/commands/new.rs
    echo -e "  ${GREEN}+${NC} horus_manager/src/commands/new.rs"
fi

# registry.rs - default version
if [ -f "horus_manager/src/registry.rs" ]; then
    sed -i "s/String::from(\"$CURRENT_VERSION\")/String::from(\"$NEW_VERSION\")/g" horus_manager/src/registry.rs
    echo -e "  ${GREEN}+${NC} horus_manager/src/registry.rs"
fi

# monitor.rs - JSON version
if [ -f "horus_manager/src/monitor.rs" ]; then
    sed -i "s/\"version\": \"$CURRENT_VERSION\"/\"version\": \"$NEW_VERSION\"/g" horus_manager/src/monitor.rs
    echo -e "  ${GREEN}+${NC} horus_manager/src/monitor.rs"
fi

# monitor_tui.rs - TUI version display
if [ -f "horus_manager/src/monitor_tui.rs" ]; then
    sed -i "s/v$CURRENT_VERSION/v$NEW_VERSION/g" horus_manager/src/monitor_tui.rs
    echo -e "  ${GREEN}+${NC} horus_manager/src/monitor_tui.rs"
fi

# workspace.rs - workspace template
if [ -f "horus_manager/src/workspace.rs" ]; then
    sed -i "s/version: $CURRENT_VERSION/version: $NEW_VERSION/g" horus_manager/src/workspace.rs
    echo -e "  ${GREEN}+${NC} horus_manager/src/workspace.rs"
fi

# cargo_config.rs - cargo config defaults
if [ -f "horus_manager/src/config/cargo_config.rs" ]; then
    sed -i "s/\"$CURRENT_VERSION\"/\"$NEW_VERSION\"/g" horus_manager/src/config/cargo_config.rs
    echo -e "  ${GREEN}+${NC} horus_manager/src/config/cargo_config.rs"
fi

echo ""
echo -e "${CYAN}[5/10] Updating YAML config files...${NC}"

YAML_FILES=(
    "horus.yaml"
    "tests/env/user_a/horus.yaml"
    "tests/env/user_a/horus-freeze.yaml"
    "tests/env/user_b/horus.yaml"
    "tests/monitor/robot_fleet_rust/horus.yaml"
    "tests/monitor/robot_fleet_python/horus.yaml"
    "tests/monitor/1pub/horus.yaml"
    "tests/multi_language_example/horus.yaml"
    "horus_core/tests/horus.yaml"
    "horus_library/apps/snakesim/horus.yaml"
    "horus_library/apps/snakesim/snakesim_gui/horus.yaml"
    "horus_library/apps/wallesim/horus.yaml"
    "tests/sim2d/sim2d_driver/horus.yaml"
    "tests/sim3d/figure8_racer/horus.yaml"
)

for file in "${YAML_FILES[@]}"; do
    if [ -f "$file" ]; then
        sed -i "s/$CURRENT_VERSION/$NEW_VERSION/g" "$file"
        echo -e "  ${GREEN}+${NC} $file"
    fi
done

echo ""
echo -e "${CYAN}[6/10] Updating package.json files...${NC}"

if [ -f "docs-site/package.json" ]; then
    sed -i "s/\"version\": \"$CURRENT_VERSION\"/\"version\": \"$NEW_VERSION\"/" docs-site/package.json
    echo -e "  ${GREEN}+${NC} docs-site/package.json"
fi

echo ""
echo -e "${CYAN}[7/10] Updating documentation files...${NC}"

# Main README
if [ -f "README.md" ]; then
    sed -i "s/$CURRENT_VERSION/$NEW_VERSION/g" README.md
    echo -e "  ${GREEN}+${NC} README.md"
fi

# Component READMEs
for readme in horus_py/README.md horus_manager/README.md horus_library/tools/sim3d/docs/README.md horus_library/nodes/README.md; do
    if [ -f "$readme" ]; then
        sed -i "s/$CURRENT_VERSION/$NEW_VERSION/g" "$readme"
        echo -e "  ${GREEN}+${NC} $readme"
    fi
done

# Docs site content (.mdx files)
if [ -d "docs-site/content" ]; then
    find docs-site/content -type f \( -name "*.mdx" -o -name "*.md" \) | while read -r file; do
        if grep -q "$CURRENT_VERSION" "$file" 2>/dev/null; then
            sed -i "s/$CURRENT_VERSION/$NEW_VERSION/g" "$file"
            echo -e "  ${GREEN}+${NC} $file"
        fi
    done
fi

# Sim3d spec
if [ -f "horus_library/tools/sim3d/SIM3D_SPEC.md" ]; then
    sed -i "s/$CURRENT_VERSION/$NEW_VERSION/g" horus_library/tools/sim3d/SIM3D_SPEC.md
    echo -e "  ${GREEN}+${NC} horus_library/tools/sim3d/SIM3D_SPEC.md"
fi

echo ""
echo -e "${CYAN}[8/10] Updating GitHub templates and CI...${NC}"

if [ -f ".github/ISSUE_TEMPLATE/bug_report.yml" ]; then
    sed -i "s/placeholder: \"$CURRENT_VERSION\"/placeholder: \"$NEW_VERSION\"/" .github/ISSUE_TEMPLATE/bug_report.yml
    echo -e "  ${GREEN}+${NC} .github/ISSUE_TEMPLATE/bug_report.yml"
fi

# Update PYPI_SETUP.md semver examples
if [ -f ".github/PYPI_SETUP.md" ]; then
    sed -i "s/\`$CURRENT_VERSION\`/\`$NEW_VERSION\`/g" .github/PYPI_SETUP.md
    echo -e "  ${GREEN}+${NC} .github/PYPI_SETUP.md"
fi

echo ""
echo -e "${CYAN}[9/10] Updating test files...${NC}"

# Python test files
find . -path "*/tests/*.py" -not -path "*/target/*" -not -path "*/.horus/*" -type f | while read -r file; do
    if grep -q "version = \"$CURRENT_VERSION\"" "$file" 2>/dev/null; then
        sed -i "s/version = \"$CURRENT_VERSION\"/version = \"$NEW_VERSION\"/g" "$file"
        echo -e "  ${GREEN}+${NC} $file"
    fi
done

echo ""
echo -e "${CYAN}[10/10] Git operations...${NC}"

# Stage the changes
git add -A
echo -e "  ${GREEN}+${NC} Changes staged"

# Show what will be committed
echo ""
git diff --cached --stat
echo ""

# Create commit
git commit -m "Release v$NEW_VERSION

- Bump version from $CURRENT_VERSION to $NEW_VERSION
- Update all Cargo.toml files (17 files)
- Update pyproject.toml files (3 files)
- Update Python __version__ (3 files)
- Update Rust source hardcoded versions (8 files)
- Update YAML config files
- Update documentation and README files
- Update GitHub templates

Generated with HORUS release script"

echo -e "  ${GREEN}+${NC} Created commit"

# Create tag
git tag "v$NEW_VERSION" -m "Release v$NEW_VERSION"
echo -e "  ${GREEN}+${NC} Created tag v$NEW_VERSION"

echo ""
echo -e "${GREEN}============================================${NC}"
echo -e "${GREEN}   Release v$NEW_VERSION prepared!${NC}"
echo -e "${GREEN}============================================${NC}"
echo ""
echo "Next steps:"
echo ""
echo "1. Review the changes:"
echo -e "   ${CYAN}git show HEAD${NC}"
echo ""
echo "2. Push to GitHub to trigger release:"
echo -e "   ${CYAN}git push origin main --tags${NC}"
echo ""
echo "3. Monitor GitHub Actions:"
echo "   https://github.com/softmata/horus/actions"
echo ""
echo "4. Verify on PyPI (after ~15 mins):"
echo "   https://pypi.org/project/horus-robotics/"
echo ""
echo "5. Test installation:"
echo -e "   ${CYAN}pip install horus-robotics==$NEW_VERSION${NC}"
echo ""
echo -e "${YELLOW}To cancel, run:${NC}"
echo -e "   ${CYAN}git reset --hard HEAD~1 && git tag -d v$NEW_VERSION${NC}"
echo ""
