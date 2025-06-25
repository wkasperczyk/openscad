# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build Commands

Build the project:
```bash
# Configure with CMake (enable experimental features for development)
cmake -B build -DEXPERIMENTAL=1

# Build
cmake --build build

# Build with parallel jobs
cmake --build build -j8
```

Install (Linux):
```bash
cmake --install build
```

Code beautification:
```bash
# Format changed files
scripts/beautify.sh

# Format all files (use sparingly)
scripts/beautify.sh --all
```

## Testing

Run tests:
```bash
# From build directory - run default tests with 8 parallel processes
cd build && ctest -j8

# Run specific test pattern
ctest -R <regex>

# Run extended test suites
ctest -C Heavy    # More time-consuming tests
ctest -C Examples # Test all examples
ctest -C All      # Test everything
```

Generate new test expectations:
```bash
TEST_GENERATE=1 ctest -R mytest
```

Test output locations:
- Expected results: `tests/regression/*/`
- Actual results: `build/tests/output/*/`
- Test logs: `build/Testing/Temporary/`

## Development Best Practices

- Before implementing any changes, analyse how to test their correctness and prevent any regression bugs
- If there are no unit tests for changed code, start by implementing them
- If there are unit tests for code you are about to change, ensure that you modify the test first in a way that won't break the rest of the program

## Architecture Overview

OpenSCAD is a 3D CAD modeling application using a script-based approach with two main modeling techniques: Constructive Solid Geometry (CSG) and 2D outline extrusion.

### Core Components

**Parser & Language (`src/core/`)**:
- `lexer.l`, `parser.y` - OpenSCAD language lexer/parser
- `AST.cc/.h` - Abstract Syntax Tree representation
- `Context.cc/.h` - Variable scoping and evaluation context
- `Value.cc/.h` - OpenSCAD value types and operations
- `Expression.cc/.h` - Expression evaluation
- `Module.cc/.h`, `UserModule.cc/.h` - Module system
- `Builtins.cc/.h` - Built-in functions and modules

**Geometry Engine (`src/geometry/`)**:
- `Geometry.cc/.h` - Base geometry classes
- `PolySet.cc/.h` - Polygon mesh representation
- `Polygon2d.cc/.h` - 2D polygon operations
- `GeometryEvaluator.cc/.h` - Converts AST to geometry
- `cgal/` - CGAL backend for solid geometry operations
- `manifold/` - Manifold backend (alternative to CGAL)
- CSG operations: union, difference, intersection

**Rendering (`src/glview/`)**:
- `GLView.cc/.h` - OpenGL 3D viewport
- `Renderer.cc/.h` - Base rendering interface
- `PolySetRenderer.cc/.h` - Mesh rendering
- `Camera.cc/.h` - 3D camera controls
- `preview/` - Real-time preview renderers (OpenCSG, ThrownTogether)
- `cgal/CGALRenderer.cc/.h` - CGAL geometry rendering

**GUI Application (`src/gui/`)**:
- `MainWindow.cc/.h` - Main application window
- `Editor.cc/.h` - Code editor with OpenSCAD syntax highlighting
- `Preferences.cc/.h` - Application settings
- `parameter/` - Customizer UI for parameterized models
- Built with Qt framework

**Import/Export (`src/io/`)**:
- File format support: STL, OFF, AMF, 3MF, DXF, SVG, OBJ
- `export_*.cc` - Various export format implementations
- `import_*.cc` - Import format parsers

**Node System (`src/core/`)**:
- Scene graph using visitor pattern
- `Node.cc/.h` - Base node class
- `*Node.cc/.h` - Specific node types (CSG, Transform, Color, etc.)
- `Tree.cc/.h` - Scene tree management
- `NodeVisitor.cc/.h` - Tree traversal

### Key Data Flow

1. **Parse**: OpenSCAD script → AST (Abstract Syntax Tree)
2. **Evaluate**: AST → Node tree (scene graph)
3. **Geometry**: Node tree → Geometry objects (PolySet, Polygon2d)
4. **Render**: Geometry → OpenGL visualization

### Development Notes

- Uses C++17 features
- Code style: 2-space indentation, defined in `.uncrustify.cfg`
- Supports both CGAL and Manifold geometry backends
- Dual geometry engines: CGAL (mature) and Manifold (newer, faster)
- Cross-platform: Linux, macOS, Windows (cross-compiled via MXE)
- WebAssembly build available for browser use

### Platform-Specific Build

**Linux/BSD**: Use system package manager for dependencies or `scripts/uni-get-dependencies.sh`
**macOS**: Use Homebrew via `scripts/macosx-build-homebrew.sh`
**Windows**: Cross-compile from Linux using MXE toolchain
**WebAssembly**: Use Docker-based build via `scripts/wasm-base-docker-run.sh`