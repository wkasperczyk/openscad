# OpenSCAD Preview Rendering Performance Optimization Plan

## Investigation Phase - Understanding Current Architecture ✅ COMPLETED

### 1. Map Preview Rendering Pipeline ✅ COMPLETED
- [x] **Examined main preview rendering entry points** 
  - GLView.cc/.h: Main 3D viewport with OpenGL rendering
  - Renderer.cc/.h: Base rendering interface with color management
  - preview/ directory: OpenCSGRenderer and ThrownTogetherRenderer implementations
  - Data flow: Script → AST → Node Tree → CSGNode Tree → CSGProducts → VBO → OpenGL

### 2. Found Existing Caching Mechanisms ✅ COMPLETED
- [x] **Discovered sophisticated caching infrastructure**
  - GeometryCache & CGALCache: Dual-cache system with LRU eviction (100MB each)
  - SourceFileCache: Hash-based caching for file includes (recently added)
  - NodeCache: Caches string representations of AST nodes
  - Smart Cache Strategy: AST subtree-based cache keys with automatic invalidation

### 3. Identified Critical Bottlenecks ✅ COMPLETED
- [x] **Found geometry generation is primary bottleneck, not OpenGL rendering**
  - Minkowski operations: O(n⁴) complexity - most expensive
  - 3D Boolean operations: O(n²) to O(n³) complexity
  - Linear extrusion: O(n × slices) with expensive transform calculations
  - Hull operations: O(n⁴) for 3D with point cloud extraction

## Key Findings from Investigation

**Current State**: OpenSCAD already has sophisticated geometry-level caching
**Primary Opportunity**: Cache expensive intermediate computational results
**Bottleneck Location**: Geometry generation, not OpenGL rendering
**Performance Target**: 50-80% improvement for parameter changes, 20-40% for structural changes

## Implementation Phase - High-Impact Caching Opportunities

**Status Update**: ✅ **Priority 1 caching COMPLETED** - All three highest-impact cache types implemented and integrated:
- Convex Decomposition Cache (Minkowski operations)
- Transformation Matrix Cache (Linear extrusion)
- Slice Parameter Cache (Extrusion calculations)
- IntermediateCache Framework with comprehensive testing
- All existing tests pass (76/76 Minkowski, 34/34 linear_extrude)

### Priority 1: Intermediate Result Caching (Highest Impact - 50-90% improvement) ✅ COMPLETED

#### 1. Convex Decomposition Cache ✅ COMPLETED
- [x] **Target**: `cgalutils-applyops-minkowski.cc:50-77`
- [x] **Implementation**: Cache decomposed convex parts for Minkowski operations
- [x] **Cache key**: hash(geometry) + "convex_decomposition"
- [x] **Cache value**: vector<CGAL_Polyhedron> (decomposed convex parts)
- [x] **Expected improvement**: Up to 90% for repeated Minkowski operations
- [x] **Integration point**: Before `CGAL::convex_decomposition_3()` call

#### 2. Transformation Matrix Cache ✅ COMPLETED
- [x] **Target**: `linear_extrude.cc` transform calculations
- [x] **Implementation**: Cache computed transformation matrices for extrusion slices
- [x] **Cache key**: hash(scale_params + rotation_params + slice_index)
- [x] **Cache value**: Eigen::Affine2d transformation matrix
- [x] **Expected improvement**: 40-60% for extrusion operations
- [x] **Integration point**: In slice generation loops

#### 3. Slice Parameter Cache ✅ COMPLETED
- [x] **Target**: Slice count and parameter calculations
- [x] **Implementation**: Cache computed slice counts and segment parameters
- [x] **Cache key**: hash(extrusion_parameters + polygon_complexity)
- [x] **Cache value**: SliceParameters struct with counts and divisions
- [x] **Expected improvement**: 30-50% for complex extrusions
- [x] **Integration point**: `calc_num_slices()` function

### Priority 2: Geometric Operation Caching (Medium Impact - 20-40% improvement)

#### 4. Point Cloud Cache
- [ ] **Target**: Hull operations point extraction
- [ ] **Implementation**: Cache extracted point clouds for hull operations
- [ ] **Cache key**: hash(geometry) + "point_cloud"
- [ ] **Cache value**: vector<Point_3> extracted points
- [ ] **Integration point**: Before `CGAL::convex_hull_3()` call

#### 5. Tessellation Cache
- [ ] **Target**: Polygon tessellation results in GeometryEvaluator.cc:92-93
- [ ] **Implementation**: Cache tessellated polygon results
- [ ] **Cache key**: hash(polygon) + "tessellation"
- [ ] **Cache value**: tessellated PolySet
- [ ] **Integration point**: `PolySetUtils::tessellate_faces()` call

#### 6. Union Tree Optimization
- [ ] **Target**: Partial union results in large CSG trees
- [ ] **Implementation**: Cache intermediate union results for subtrees
- [ ] **Cache key**: hash(geometry_set) + "partial_union"
- [ ] **Cache value**: intermediate CGALNefGeometry
- [ ] **Integration point**: Priority queue processing in `applyUnion3D()`

### Core Infrastructure Extensions

#### 7. Intermediate Cache Framework ✅ COMPLETED
- [x] **Create IntermediateCache class**
  - Specialized cache for computational intermediates
  - Integration with existing Cache<Key,T> template
  - Memory management compatible with GeometryCache/CGALCache

#### 8. Enhanced Cache Key Generation ✅ COMPLETED
- [x] **Implement content-based hashing**
  - Replace expensive geometry comparisons with fast hash lookups
  - Geometry fingerprinting for better cache hit rates
  - Parameter canonicalization for equivalent inputs

#### 9. Cache Integration Points
- [ ] **GeometryEvaluator integration**
  - Check intermediate caches before expensive operations
  - Insert results into appropriate caches after computation
  - Coordinate between geometry and intermediate caches

- [x] **CGAL/Manifold utilities integration** ✅ COMPLETED
  - Add cache lookup/insert around expensive functions
  - Maintain existing cache semantics and invalidation
  - Preserve thread safety for preview updates

## Testing and Validation

### 10. Performance Testing
- [ ] **Create benchmark suite for high-impact operations**
  - Minkowski operations with various geometry complexities
  - Linear extrusion with different slice counts and parameters
  - CSG operations with deep nesting
  - Hull operations with different point cloud sizes
  - Before/after performance measurements for each cache type

- [ ] **Measure cache effectiveness**
  - Cache hit rates for different workflow patterns
  - Memory usage analysis for intermediate caches
  - Performance regression testing
  - Cache invalidation frequency analysis

### 11. Implementation Validation
- [ ] **Cache correctness verification**
  - Verify cached intermediate results match non-cached computation
  - Test cache invalidation triggers work correctly
  - Validate memory management and LRU eviction
  - Test thread safety for concurrent preview updates

- [ ] **Real-world workflow testing**
  - Parameter tweaking workflows (should show 50-80% improvement)
  - Structural code changes (should show 20-40% improvement)
  - Complex Minkowski operations (should show up to 90% improvement)
  - Large extrusion operations (should show 40-60% improvement)

## Implementation Details and Technical Notes

### Key Files to Modify:
- `src/geometry/GeometryEvaluator.cc/.h` - Main evaluation logic
- `src/glview/preview/` - Preview renderers
- `src/core/Context.cc/.h` - Variable evaluation context
- `src/core/Value.cc/.h` - Value computation and caching
- `src/io/` - File include handling for cache invalidation

### Cache Architecture Considerations:
- **Memory management**: Use smart pointers for cached geometry objects
- **Serialization**: Consider using existing OpenSCAD serialization or binary formats
- **Concurrency**: Preview updates happen on different thread than UI
- **Platform compatibility**: Cache should work on Linux/macOS/Windows
- **Version compatibility**: Cache format should handle OpenSCAD version changes

### Performance Targets (Based on Investigation Results):
- **Parameter changes**: 50-80% reduction in preview update time
- **Structural changes**: 20-40% reduction (depending on change locality)
- **Minkowski operations**: Up to 90% improvement with convex decomposition caching
- **Extrusion operations**: 40-60% improvement with transform/slice caching
- **Memory overhead**: < 15% increase for typical models (intermediate caches)
- **Cache lookup performance**: < 1ms for cache hits

### Risk Mitigation:
- **Fallback mechanism**: Always allow bypassing cache if issues occur
- **Cache corruption handling**: Detect and recover from invalid cache entries
- **Memory limits**: Prevent cache from consuming excessive memory
- **Debugging support**: Comprehensive logging for cache behavior analysis