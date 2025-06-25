#pragma once

#include <cstddef>
#include <memory>
#include <string>
#include <vector>
#include <list>
#include <functional>

#include "Cache.h"
#include "geometry/Geometry.h"

// Include necessary headers instead of forward declarations
#include "geometry/linalg.h"

// Forward declarations for CGAL types to avoid heavy includes
#ifdef ENABLE_CGAL
#include "geometry/cgal/cgal.h"
#else
// Stub for when CGAL is not available
class CGAL_Polyhedron {};
#endif

// Intermediate computation result types for caching
namespace IntermediateResults {

  // Convex decomposition result for Minkowski operations
  struct ConvexDecomposition {
    std::list<CGAL_Polyhedron> convex_parts;
    size_t num_parts;
    
    ConvexDecomposition() : num_parts(0) {}
    explicit ConvexDecomposition(std::list<CGAL_Polyhedron>&& parts) 
      : convex_parts(std::move(parts)), num_parts(convex_parts.size()) {}
    
    size_t memoryUsage() const;
  };

  // Transformation matrix cache for linear extrusion
  struct TransformationMatrix {
    Eigen::Affine2d transform;
    
    explicit TransformationMatrix(const Eigen::Affine2d& t) : transform(t) {}
    
    size_t memoryUsage() const { return sizeof(Eigen::Affine2d); }
  };

  // Slice parameters for extrusion operations
  struct SliceParameters {
    unsigned int slice_count;
    std::vector<double> slice_heights;
    std::vector<double> scale_factors_x;
    std::vector<double> scale_factors_y;
    std::vector<double> rotation_angles;
    
    size_t memoryUsage() const {
      return sizeof(unsigned int) + 
             slice_heights.size() * sizeof(double) +
             scale_factors_x.size() * sizeof(double) +
             scale_factors_y.size() * sizeof(double) +
             rotation_angles.size() * sizeof(double);
    }
  };

  // Point cloud for hull operations
  struct PointCloud {
    std::vector<Vector3d> points;
    
    explicit PointCloud(std::vector<Vector3d>&& pts) : points(std::move(pts)) {}
    
    size_t memoryUsage() const { return points.size() * sizeof(Vector3d); }
  };
}

// Generic intermediate cache for computational results
template<typename T>
class IntermediateCache {
public:
  explicit IntermediateCache(size_t memory_limit_mb = 50) 
    : cache_(memory_limit_mb * 1024 * 1024) {}

  // Check if result exists in cache
  bool contains(const std::string& key) const { 
    return cache_.contains(key); 
  }

  // Get cached result
  std::shared_ptr<const T> get(const std::string& key) const {
    auto* entry = cache_.object(key);
    return entry ? entry->result : nullptr;
  }

  // Insert result into cache
  bool insert(const std::string& key, std::shared_ptr<const T> result) {
    if (!result) return false;
    
    auto entry = std::make_unique<CacheEntry>(result);
    size_t cost = entry->memoryUsage();
    
    return cache_.insert(key, entry.release(), cost);
  }

  // Cache statistics
  size_t size() const { return cache_.size(); }
  size_t totalCost() const { return cache_.totalCost(); }
  size_t maxCostMB() const { return cache_.maxCost() / (1024 * 1024); }
  void setMaxCostMB(size_t limit_mb) { cache_.setMaxCost(limit_mb * 1024 * 1024); }
  void clear() { cache_.clear(); }

private:
  struct CacheEntry {
    std::shared_ptr<const T> result;
    
    explicit CacheEntry(std::shared_ptr<const T> r) : result(std::move(r)) {}
    
    size_t memoryUsage() const {
      // All our intermediate result types have memoryUsage() method
      return result->memoryUsage() + sizeof(CacheEntry);
    }
  };

  Cache<std::string, CacheEntry> cache_;
};

// Specialized caches for different intermediate results
using ConvexDecompositionCache = IntermediateCache<IntermediateResults::ConvexDecomposition>;
using TransformationMatrixCache = IntermediateCache<IntermediateResults::TransformationMatrix>;
using SliceParametersCache = IntermediateCache<IntermediateResults::SliceParameters>;
using PointCloudCache = IntermediateCache<IntermediateResults::PointCloud>;

// Global cache manager for intermediate computational results
class IntermediateCacheManager {
public:
  static IntermediateCacheManager& instance();

  // Cache accessors
  ConvexDecompositionCache& convexDecompositionCache() { return convex_decomposition_cache_; }
  TransformationMatrixCache& transformationMatrixCache() { return transformation_matrix_cache_; }
  SliceParametersCache& sliceParametersCache() { return slice_parameters_cache_; }
  PointCloudCache& pointCloudCache() { return point_cloud_cache_; }

  // Global operations
  void clearAll();
  void setMemoryLimits(size_t convex_mb, size_t transform_mb, size_t slice_mb, size_t point_mb);
  
  // Statistics and debugging
  void printStatistics() const;
  size_t totalMemoryUsage() const;

private:
  IntermediateCacheManager() = default;
  
  ConvexDecompositionCache convex_decomposition_cache_{25}; // 25MB default
  TransformationMatrixCache transformation_matrix_cache_{10}; // 10MB default  
  SliceParametersCache slice_parameters_cache_{10}; // 10MB default
  PointCloudCache point_cloud_cache_{15}; // 15MB default
};

// Utility functions for cache key generation
namespace CacheKeyUtils {
  
  // Generate hash-based cache key from geometry
  std::string geometryKey(const std::shared_ptr<const Geometry>& geom, const std::string& operation);
  
  // Generate cache key for transformation parameters
  std::string transformKey(double scale_x, double scale_y, double rotation, unsigned int slice_index);
  
  // Generate cache key for slice parameters
  std::string sliceKey(double height, double twist, double scale_x, double scale_y, 
                      unsigned int fn, unsigned int fs, double fa);
  
  // Generate cache key for point extraction
  std::string pointCloudKey(const std::shared_ptr<const Geometry>& geom);

  // Fast hash function for floating point parameters
  std::string hashFloats(const std::vector<double>& values);
}