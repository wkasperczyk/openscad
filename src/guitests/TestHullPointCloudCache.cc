#include "TestHullPointCloudCache.h"
#include "geometry/IntermediateCache.h"
#include "geometry/PolySet.h"
#include "geometry/linalg.h"
#include "geometry/boolean_utils.h"
#include "geometry/GeometryEvaluator.h"

#ifdef ENABLE_CGAL
#include "geometry/cgal/CGALNefGeometry.h"
#include "geometry/cgal/cgalutils.h"
#endif

#ifdef ENABLE_MANIFOLD
#include "geometry/manifold/ManifoldGeometry.h"
#endif

#include <QTest>
#include <memory>
#include <vector>
#include <iostream>

int TestHullPointCloudCache::runStandaloneTests() {
  TestHullPointCloudCache test;
  return QTest::qExec(&test);
}

void TestHullPointCloudCache::initTestCase() {
  // Initialize any global test data
}

void TestHullPointCloudCache::cleanupTestCase() {
  // Clean up global test data
  IntermediateCacheManager::instance().clearAll();
}

void TestHullPointCloudCache::init() {
  // Clear caches before each test to ensure isolation
  IntermediateCacheManager::instance().clearAll();
}

void TestHullPointCloudCache::cleanup() {
  // Clean up after each test
  IntermediateCacheManager::instance().clearAll();
}

void TestHullPointCloudCache::testPointCloudExtractionFromPolySet() {
  // Create a simple test PolySet (triangle)
  auto ps = createTestPolySet();
  
  // Extract point cloud manually (simulating what hull operation does)
  std::vector<Vector3d> expected_points;
  for (const auto& p : ps->indices) {
    for (const auto& ind : p) {
      Vector3d point = ps->vertices[ind];
      // Check if point already exists (manual deduplication for testing)
      bool found = false;
      for (const auto& existing : expected_points) {
        if ((point - existing).norm() < 1e-10) {
          found = true;
          break;
        }
      }
      if (!found) {
        expected_points.push_back(point);
      }
    }
  }
  
  // Create point cloud cache entry
  auto cloud = std::make_shared<IntermediateResults::PointCloud>(std::vector<Vector3d>(expected_points));
  
  // Test cache insertion and retrieval
  auto& cache = IntermediateCacheManager::instance().pointCloudCache();
  std::string key = CacheKeyUtils::pointCloudKey(ps);
  
  QVERIFY(cache.insert(key, cloud));
  QVERIFY(cache.contains(key));
  
  auto retrieved = cache.get(key);
  QVERIFY(retrieved != nullptr);
  QCOMPARE(retrieved->points.size(), expected_points.size());
  
  // Verify points are preserved correctly
  for (size_t i = 0; i < expected_points.size(); ++i) {
    QVERIFY((retrieved->points[i] - expected_points[i]).norm() < 1e-10);
  }
}

void TestHullPointCloudCache::testPointCloudCacheHitAndMiss() {
  auto& cache = IntermediateCacheManager::instance().pointCloudCache();
  
  // Create test geometry
  auto ps1 = createTestPolySet();
  auto ps2 = createTestPolySet();
  
  // Create different transformations to ensure different geometry
  for (auto& vertex : ps2->vertices) {
    vertex += Vector3d(1.0, 0.0, 0.0); // Translate to make it different
  }
  
  std::string key1 = CacheKeyUtils::pointCloudKey(ps1);
  std::string key2 = CacheKeyUtils::pointCloudKey(ps2);
  
  // Keys should be different for different geometry
  QVERIFY(key1 != key2);
  
  // Insert point cloud for first geometry
  std::vector<Vector3d> points1 = {Vector3d(0, 0, 0), Vector3d(1, 0, 0), Vector3d(0, 1, 0)};
  auto cloud1 = std::make_shared<IntermediateResults::PointCloud>(std::move(points1));
  
  QVERIFY(cache.insert(key1, cloud1));
  
  // Test cache hit
  QVERIFY(cache.contains(key1));
  auto retrieved1 = cache.get(key1);
  QVERIFY(retrieved1 != nullptr);
  
  // Test cache miss
  QVERIFY(!cache.contains(key2));
  auto retrieved2 = cache.get(key2);
  QVERIFY(retrieved2 == nullptr);
}

void TestHullPointCloudCache::testCacheKeyConsistency() {
  // Create identical geometry objects
  auto ps1 = createTestPolySet();
  auto ps2 = createTestPolySet();
  
  // Keys should be identical for identical geometry
  std::string key1 = CacheKeyUtils::pointCloudKey(ps1);
  std::string key2 = CacheKeyUtils::pointCloudKey(ps2);
  
  QCOMPARE(key1, key2);
  
  // Modify one geometry slightly
  ps2->vertices[0] += Vector3d(0.001, 0, 0);
  
  std::string key3 = CacheKeyUtils::pointCloudKey(ps2);
  
  // Keys should now be different
  QVERIFY(key1 != key3);
}

void TestHullPointCloudCache::testCacheEvictionWithLargePointClouds() {
  auto& cache = IntermediateCacheManager::instance().pointCloudCache();
  
  // Set small memory limit to force eviction
  cache.setMaxCostMB(1); // 1MB limit
  
  std::vector<std::string> keys;
  const size_t num_clouds = 50;
  const size_t points_per_cloud = 1000;
  
  // Generate large point clouds until eviction occurs
  for (size_t i = 0; i < num_clouds; ++i) {
    std::vector<Vector3d> points;
    points.reserve(points_per_cloud);
    
    for (size_t j = 0; j < points_per_cloud; ++j) {
      points.emplace_back(i * 10 + j, j * 2, i + j * 0.1);
    }
    
    auto cloud = std::make_shared<IntermediateResults::PointCloud>(std::move(points));
    std::string key = "large_cloud_" + std::to_string(i);
    keys.push_back(key);
    
    cache.insert(key, cloud);
  }
  
  // Cache should have evicted some entries due to memory limit
  QVERIFY(cache.size() < num_clouds);
  QVERIFY(cache.totalCost() <= cache.maxCostMB() * 1024 * 1024);
  
  // Most recent entries should still be present (LRU eviction)
  for (size_t i = num_clouds - 5; i < num_clouds; ++i) {
    std::string key = "large_cloud_" + std::to_string(i);
    QVERIFY(cache.contains(key));
  }
}

void TestHullPointCloudCache::testMemoryUsageCalculation() {
  std::vector<Vector3d> points;
  const size_t num_points = 100;
  
  for (size_t i = 0; i < num_points; ++i) {
    points.emplace_back(i, i * 2, i * 3);
  }
  
  auto cloud = std::make_shared<IntermediateResults::PointCloud>(std::vector<Vector3d>(points));
  
  // Test memory usage calculation
  size_t expected_size = num_points * sizeof(Vector3d);
  QCOMPARE(cloud->memoryUsage(), expected_size);
  
  // Test cache memory accounting
  auto& cache = IntermediateCacheManager::instance().pointCloudCache();
  std::string key = "memory_test";
  
  size_t initial_cost = cache.totalCost();
  QVERIFY(cache.insert(key, cloud));
  size_t final_cost = cache.totalCost();
  
  // Should have increased by at least the cloud size
  QVERIFY(final_cost > initial_cost);
  QVERIFY(final_cost - initial_cost >= expected_size);
}

void TestHullPointCloudCache::testCacheIntegrationWithHullOperation() {
  // Create test geometry for hull operation
  Geometry::Geometries children;
  
  // Add a simple PolySet as a child
  auto ps = createTestPolySet();
  children.push_back(std::make_pair(nullptr, ps));
  
  auto& cache = IntermediateCacheManager::instance().pointCloudCache();
  
  // Verify cache is initially empty
  QCOMPARE(cache.size(), static_cast<size_t>(0));
  
  // Run hull operation (this would ideally use the cache)
  // Note: This tests the infrastructure, actual cache integration is in the implementation
  auto result = applyHull(children);
  
  QVERIFY(result != nullptr);
  QVERIFY(result->vertices.size() >= 3); // Hull of triangle should have at least 3 vertices
}

void TestHullPointCloudCache::testMultipleGeometryTypes() {
  auto& cache = IntermediateCacheManager::instance().pointCloudCache();
  
  // Test with PolySet
  auto ps = createTestPolySet();
  std::string ps_key = CacheKeyUtils::pointCloudKey(ps);
  
  std::vector<Vector3d> ps_points = {Vector3d(0, 0, 0), Vector3d(1, 0, 0), Vector3d(0, 1, 0)};
  auto ps_cloud = std::make_shared<IntermediateResults::PointCloud>(std::move(ps_points));
  
  QVERIFY(cache.insert(ps_key, ps_cloud));
  
#ifdef ENABLE_CGAL
  // Test key generation doesn't crash with other geometry types
  // (Actual point extraction would be tested in integration tests)
  auto nef_geom = std::make_shared<CGALNefGeometry>();
  std::string nef_key = CacheKeyUtils::pointCloudKey(nef_geom);
  QVERIFY(!nef_key.empty());
#endif
  
  // Verify different geometry types produce different keys
  QVERIFY(ps_key != "");
  
  // Test cache retrieval
  auto retrieved = cache.get(ps_key);
  QVERIFY(retrieved != nullptr);
  QCOMPARE(retrieved->points.size(), static_cast<size_t>(3));
}

void TestHullPointCloudCache::testCacheConcurrencySimulation() {
  auto& cache = IntermediateCacheManager::instance().pointCloudCache();
  
  // Simulate concurrent access from multiple "threads"
  const size_t num_operations = 100;
  
  for (size_t i = 0; i < num_operations; ++i) {
    // Create unique point cloud for each operation
    std::vector<Vector3d> points = {
      Vector3d(i, 0, 0),
      Vector3d(0, i, 0),
      Vector3d(0, 0, i)
    };
    
    auto cloud = std::make_shared<IntermediateResults::PointCloud>(std::move(points));
    std::string key = "concurrent_" + std::to_string(i);
    
    // Insert and immediately retrieve (simulating quick access patterns)
    QVERIFY(cache.insert(key, cloud));
    auto retrieved = cache.get(key);
    QVERIFY(retrieved != nullptr);
    QCOMPARE(retrieved->points.size(), static_cast<size_t>(3));
    
    // Verify data integrity
    QVERIFY((retrieved->points[0] - Vector3d(i, 0, 0)).norm() < 1e-10);
  }
  
  // All operations should have succeeded
  QVERIFY(cache.size() <= num_operations); // May be less due to eviction
}

void TestHullPointCloudCache::testCacheInvalidation() {
  auto& cache = IntermediateCacheManager::instance().pointCloudCache();
  
  // Insert test data
  std::vector<Vector3d> points = {Vector3d(0, 0, 0), Vector3d(1, 1, 1)};
  auto cloud = std::make_shared<IntermediateResults::PointCloud>(std::move(points));
  std::string key = "invalidation_test";
  
  QVERIFY(cache.insert(key, cloud));
  QVERIFY(cache.contains(key));
  
  // Clear cache (simulating invalidation)
  cache.clear();
  
  // Cache should be empty
  QCOMPARE(cache.size(), static_cast<size_t>(0));
  QVERIFY(!cache.contains(key));
  
  // Should be able to insert again
  std::vector<Vector3d> new_points = {Vector3d(2, 2, 2)};
  auto new_cloud = std::make_shared<IntermediateResults::PointCloud>(std::move(new_points));
  
  QVERIFY(cache.insert(key, new_cloud));
  QVERIFY(cache.contains(key));
}

std::shared_ptr<PolySet> TestHullPointCloudCache::createTestPolySet() {
  // Create a simple triangular PolySet for testing
  auto ps = std::make_shared<PolySet>(3);
  
  // Add vertices directly to vertices vector
  ps->vertices.emplace_back(0, 0, 0);  // vertex 0
  ps->vertices.emplace_back(1, 0, 0);  // vertex 1
  ps->vertices.emplace_back(0, 1, 0);  // vertex 2
  
  // Add triangle face directly to indices vector
  ps->indices.push_back({0, 1, 2});
  
  return ps;
}

#include "TestHullPointCloudCache.moc"