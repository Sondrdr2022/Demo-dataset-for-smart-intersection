# Move SUMO configs to data directory
cp *.sumocfg data/sumo_configs/ 2>/dev/null || true
cp *.xml data/sumo_configs/ 2>/dev/null || true
cp *.add data/sumo_configs/ 2>/dev/null || true

# Create __init__.py files for proper Python packaging
touch src/__init__.py
touch src/traffic_controller/__init__.py
touch src/rl_agent/__init__.py
touch src/simulation/__init__.py
touch src/models/__init__.py
touch src/utils/__init__.py

# Create .gitignore for build artifacts and dependencies
cat > .gitignore << 'EOF'
# Python
__pycache__/
*.py[cod]
*$py.class
*.so
.Python
build/
develop-eggs/
dist/
downloads/
eggs/
.eggs/
lib/
lib64/
parts/
sdist/
var/
wheels/
*.egg-info/
.installed.cfg
*.egg

# Virtual environments
venv/
env/
ENV/

# Data and logs
data/training_logs/*.csv
data/training_logs/*.json
data/models/*.npy
data/models/*.pkl

# SUMO temporary files
*.log
*.out

# IDE
.vscode/
.idea/
*.swp
*.swo

# OS
.DS_Store
Thumbs.db
EOF

echo "✅ Setup completed"