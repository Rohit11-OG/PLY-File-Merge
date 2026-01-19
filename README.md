# 🔀 PLY File Merge

A collection of Python tools for merging and transforming PLY (Polygon File Format) point cloud files. ☁️

## ✨ Features

- 🔄 **Manual Rotate** - Interactively rotate point clouds
- 🎯 **Align and Merge** - Automatically align and merge multiple PLY files
- 📐 **Register and Merge** - Point cloud registration and merging
- 🔃 **Rotate Transform** - Apply rotation transformations to point clouds

## 📋 Requirements

- 🐍 Python 3.x
- 📦 Open3D
- 🔢 NumPy

## 🚀 Installation

```bash
pip install open3d numpy
```

## 💻 Usage

Place your PLY files in the project directory and run the appropriate script:

```bash
python manual_rotate.py
python align_and_merge.py
python register_and_merge.py
```

## 📁 Output

Merged point clouds are saved to the `merged_output/` directory.
