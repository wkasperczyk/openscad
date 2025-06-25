#include "geometry/IntermediateCache.h"

#include <sstream>
#include <iomanip>
#include <functional>

#include "geometry/Geometry.h"
#include "geometry/linalg.h"
#include "utils/printutils.h"

// Convex decomposition memory usage calculation
namespace IntermediateResults {
  size_t ConvexDecomposition::memoryUsage() const {
    size_t total = sizeof(ConvexDecomposition);
    for (const auto& poly : convex_parts) {
      // Rough estimate: vertices + faces + edges
      total += poly.size_of_vertices() * sizeof(void*) * 3; // Point + connectivity
      total += poly.size_of_facets() * sizeof(void*) * 4;   // Triangle/quad + connectivity  
      total += poly.size_of_halfedges() * sizeof(void*) * 2; // Edge connectivity
    }
    return total;
  }
}

// Global cache manager implementation
IntermediateCacheManager& IntermediateCacheManager::instance() {
  static IntermediateCacheManager instance;
  return instance;
}

void IntermediateCacheManager::clearAll() {
  convex_decomposition_cache_.clear();
  transformation_matrix_cache_.clear();
  slice_parameters_cache_.clear();
  point_cloud_cache_.clear();
}

void IntermediateCacheManager::setMemoryLimits(size_t convex_mb, size_t transform_mb, 
                                               size_t slice_mb, size_t point_mb) {
  convex_decomposition_cache_.setMaxCostMB(convex_mb);
  transformation_matrix_cache_.setMaxCostMB(transform_mb);
  slice_parameters_cache_.setMaxCostMB(slice_mb);
  point_cloud_cache_.setMaxCostMB(point_mb);
}

void IntermediateCacheManager::printStatistics() const {
  PRINTDB("Intermediate Cache Statistics: %1%", "");
  PRINTDB("  Convex Decomposition: %1% entries, %2% MB / %3% MB", 
          convex_decomposition_cache_.size() %
          (convex_decomposition_cache_.totalCost() / (1024*1024)) %
          convex_decomposition_cache_.maxCostMB());
  PRINTDB("  Transformation Matrix: %1% entries, %2% MB / %3% MB",
          transformation_matrix_cache_.size() % 
          (transformation_matrix_cache_.totalCost() / (1024*1024)) %
          transformation_matrix_cache_.maxCostMB());
  PRINTDB("  Slice Parameters: %1% entries, %2% MB / %3% MB",
          slice_parameters_cache_.size() %
          (slice_parameters_cache_.totalCost() / (1024*1024)) % 
          slice_parameters_cache_.maxCostMB());
  PRINTDB("  Point Cloud: %1% entries, %2% MB / %3% MB",
          point_cloud_cache_.size() %
          (point_cloud_cache_.totalCost() / (1024*1024)) %
          point_cloud_cache_.maxCostMB());
}

size_t IntermediateCacheManager::totalMemoryUsage() const {
  return convex_decomposition_cache_.totalCost() +
         transformation_matrix_cache_.totalCost() +
         slice_parameters_cache_.totalCost() +
         point_cloud_cache_.totalCost();
}

// Cache key utility implementations
namespace CacheKeyUtils {

  std::string geometryKey(const std::shared_ptr<const Geometry>& geom, const std::string& operation) {
    if (!geom) return operation + "_null";
    
    // Use a combination of geometry properties to create a cache key
    std::ostringstream key;
    key << operation << "_";
    
    // Use geometry dump as content identifier (may be expensive but accurate)
    std::string dump = geom->dump();
    std::hash<std::string> hasher;
    key << std::hex << hasher(dump);
    
    return key.str();
  }

  std::string transformKey(double scale_x, double scale_y, double rotation, unsigned int slice_index) {
    std::ostringstream key;
    key << "transform_" << std::fixed << std::setprecision(6)
        << scale_x << "_" << scale_y << "_" << rotation << "_" << slice_index;
    return key.str();
  }

  std::string sliceKey(double height, double twist, double scale_x, double scale_y,
                      unsigned int fn, unsigned int fs, double fa) {
    std::ostringstream key;
    key << "slice_" << std::fixed << std::setprecision(6)
        << height << "_" << twist << "_" << scale_x << "_" << scale_y << "_"
        << fn << "_" << fs << "_" << fa;
    return key.str();
  }

  std::string pointCloudKey(const std::shared_ptr<const Geometry>& geom) {
    return geometryKey(geom, "pointcloud");
  }

  std::string hashFloats(const std::vector<double>& values) {
    std::hash<double> hasher;
    size_t seed = values.size();
    
    for (const auto& val : values) {
      seed ^= hasher(val) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    }
    
    std::ostringstream result;
    result << std::hex << seed;
    return result.str();
  }
}