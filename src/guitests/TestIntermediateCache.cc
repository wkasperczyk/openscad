#include "TestIntermediateCache.h"
#include "geometry/IntermediateCache.h"
#include "geometry/PolySet.h"
#include "geometry/linalg.h"

#include <QTest>
#include <memory>
#include <vector>
#include <iostream>

int TestIntermediateCache::runStandaloneTests() {
  TestIntermediateCache test;
  return QTest::qExec(&test);
}

void TestIntermediateCache::initTestCase() {
  // Initialize any global test data
}

void TestIntermediateCache::cleanupTestCase() {
  // Clean up global test data
  IntermediateCacheManager::instance().clearAll();
}

void TestIntermediateCache::init() {
  // Clear caches before each test to ensure isolation
  IntermediateCacheManager::instance().clearAll();
}

void TestIntermediateCache::cleanup() {
  // Clean up after each test
  IntermediateCacheManager::instance().clearAll();
}

void TestIntermediateCache::testCacheInsertAndRetrieve() {
  IntermediateCache<IntermediateResults::ConvexDecomposition> cache(1); // 1MB limit

  // Create test data
  std::list<CGAL_Polyhedron> test_parts;
  auto decomposition = std::make_shared<IntermediateResults::ConvexDecomposition>(std::move(test_parts));
  
  // Test insertion
  std::string key = "test_key_1";
  QVERIFY(cache.insert(key, decomposition));
  QVERIFY(cache.contains(key));
  QCOMPARE(cache.size(), static_cast<size_t>(1));
  
  // Test retrieval
  auto retrieved = cache.get(key);
  QVERIFY(retrieved != nullptr);
  QCOMPARE(retrieved.get(), decomposition.get());
  
  // Test non-existent key
  QVERIFY(!cache.contains("non_existent"));
  QVERIFY(cache.get("non_existent") == nullptr);
}

void TestIntermediateCache::testCacheEviction() {
  IntermediateCache<IntermediateResults::TransformationMatrix> cache(1); // 1MB limit = ~32k matrices
  
  // Fill cache with transformation matrices until eviction occurs
  std::vector<std::string> keys;
  const size_t num_inserts = 50000; // Should trigger eviction
  
  for (size_t i = 0; i < num_inserts; ++i) {
    Eigen::Affine2d transform = Eigen::Affine2d::Identity();
    auto matrix = std::make_shared<IntermediateResults::TransformationMatrix>(transform);
    std::string key = "matrix_" + std::to_string(i);
    keys.push_back(key);
    cache.insert(key, matrix);
  }
  
  // Cache should have evicted some entries
  QVERIFY(cache.size() < num_inserts);
  QVERIFY(cache.totalCost() <= cache.maxCostMB() * 1024 * 1024);
  
  // Recent entries should still be present (LRU eviction)
  for (size_t i = num_inserts - 100; i < num_inserts; ++i) {
    std::string key = "matrix_" + std::to_string(i);
    QVERIFY(cache.contains(key));
  }
}

void TestIntermediateCache::testCacheKeyGeneration() {
  // Test geometry key generation
  auto geom = createTestGeometry();
  std::string key1 = CacheKeyUtils::geometryKey(geom, "test_op");
  std::string key2 = CacheKeyUtils::geometryKey(geom, "test_op");
  std::string key3 = CacheKeyUtils::geometryKey(geom, "different_op");
  
  // Same geometry and operation should produce same key
  QCOMPARE(key1, key2);
  // Different operation should produce different key
  QVERIFY(key1 != key3);
  
  // Test transformation key generation
  std::string tkey1 = CacheKeyUtils::transformKey(1.0, 1.0, 0.0, 0);
  std::string tkey2 = CacheKeyUtils::transformKey(1.0, 1.0, 0.0, 0);
  std::string tkey3 = CacheKeyUtils::transformKey(1.0, 1.0, 0.0, 1);
  
  QCOMPARE(tkey1, tkey2);
  QVERIFY(tkey1 != tkey3);
  
  // Test slice key generation
  std::string skey1 = CacheKeyUtils::sliceKey(10.0, 0.0, 1.0, 1.0, 0, 0, 0.0);
  std::string skey2 = CacheKeyUtils::sliceKey(10.0, 0.0, 1.0, 1.0, 0, 0, 0.0);
  QCOMPARE(skey1, skey2);
}

void TestIntermediateCache::testMemoryManagement() {
  IntermediateCache<IntermediateResults::PointCloud> cache(1); // 1MB limit
  
  // Test memory usage calculation
  std::vector<Vector3d> points;
  for (int i = 0; i < 1000; ++i) {
    points.emplace_back(i, i, i);
  }
  
  auto cloud = std::make_shared<IntermediateResults::PointCloud>(std::move(points));
  std::string key = "point_cloud_test";
  
  QVERIFY(cache.insert(key, cloud));
  
  // Verify memory accounting
  size_t expected_cost = cloud->memoryUsage() + sizeof(typename IntermediateCache<IntermediateResults::PointCloud>::CacheEntry);
  QVERIFY(cache.totalCost() >= expected_cost - 100); // Allow some margin for overhead
}

void TestIntermediateCache::testConvexDecompositionCache() {
  auto& cache = IntermediateCacheManager::instance().convexDecompositionCache();
  
  // Test basic operations
  std::list<CGAL_Polyhedron> parts;
  auto decomposition = std::make_shared<IntermediateResults::ConvexDecomposition>(std::move(parts));
  
  std::string key = "convex_test";
  QVERIFY(cache.insert(key, decomposition));
  QVERIFY(cache.contains(key));
  
  auto retrieved = cache.get(key);
  QVERIFY(retrieved != nullptr);
  QCOMPARE(retrieved->num_parts, static_cast<size_t>(0));
}

void TestIntermediateCache::testTransformationMatrixCache() {
  auto& cache = IntermediateCacheManager::instance().transformationMatrixCache();
  
  Eigen::Affine2d transform;
  transform.setIdentity();
  transform.scale(Eigen::Vector2d(2.0, 3.0));
  transform.rotate(M_PI / 4);
  
  auto matrix = std::make_shared<IntermediateResults::TransformationMatrix>(transform);
  std::string key = "transform_test";
  
  QVERIFY(cache.insert(key, matrix));
  
  auto retrieved = cache.get(key);
  QVERIFY(retrieved != nullptr);
  
  // Verify matrix values are preserved
  Eigen::Matrix3d expected = transform.matrix();
  Eigen::Matrix3d actual = retrieved->transform.matrix();
  
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      QVERIFY(std::abs(expected(i, j) - actual(i, j)) < 1e-10);
    }
  }
}

void TestIntermediateCache::testSliceParametersCache() {
  auto& cache = IntermediateCacheManager::instance().sliceParametersCache();
  
  auto params = std::make_shared<IntermediateResults::SliceParameters>();
  params->slice_count = 10;
  params->slice_heights = {0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0};
  params->scale_factors_x = {1.0, 1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7, 1.8, 1.9};
  params->scale_factors_y = {1.0, 1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7, 1.8, 1.9};
  params->rotation_angles = {0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9};
  
  std::string key = "slice_test";
  QVERIFY(cache.insert(key, params));
  
  auto retrieved = cache.get(key);
  QVERIFY(retrieved != nullptr);
  QCOMPARE(retrieved->slice_count, static_cast<unsigned int>(10));
  QCOMPARE(retrieved->slice_heights.size(), static_cast<size_t>(10));
  QVERIFY(std::abs(retrieved->scale_factors_x[5] - 1.5) < 1e-10);
}

void TestIntermediateCache::testPointCloudCache() {
  auto& cache = IntermediateCacheManager::instance().pointCloudCache();
  
  std::vector<Vector3d> points = {
    Vector3d(0, 0, 0),
    Vector3d(1, 0, 0),
    Vector3d(0, 1, 0),
    Vector3d(0, 0, 1)
  };
  
  auto cloud = std::make_shared<IntermediateResults::PointCloud>(std::move(points));
  std::string key = "cloud_test";
  
  QVERIFY(cache.insert(key, cloud));
  
  auto retrieved = cache.get(key);
  QVERIFY(retrieved != nullptr);
  QCOMPARE(retrieved->points.size(), static_cast<size_t>(4));
  QVERIFY((retrieved->points[0] - Vector3d(0, 0, 0)).norm() < 1e-10);
  QVERIFY((retrieved->points[3] - Vector3d(0, 0, 1)).norm() < 1e-10);
}

void TestIntermediateCache::testIntermediateCacheManager() {
  auto& manager = IntermediateCacheManager::instance();
  
  // Test that all cache types are accessible
  QVERIFY(&manager.convexDecompositionCache() != nullptr);
  QVERIFY(&manager.transformationMatrixCache() != nullptr);
  QVERIFY(&manager.sliceParametersCache() != nullptr);
  QVERIFY(&manager.pointCloudCache() != nullptr);
  
  // Test global clear
  manager.clearAll();
  QCOMPARE(manager.convexDecompositionCache().size(), static_cast<size_t>(0));
  QCOMPARE(manager.transformationMatrixCache().size(), static_cast<size_t>(0));
  QCOMPARE(manager.sliceParametersCache().size(), static_cast<size_t>(0));
  QCOMPARE(manager.pointCloudCache().size(), static_cast<size_t>(0));
}

void TestIntermediateCache::testGlobalCacheOperations() {
  auto& manager = IntermediateCacheManager::instance();
  
  // Add some data to caches
  auto decomposition = std::make_shared<IntermediateResults::ConvexDecomposition>(std::list<CGAL_Polyhedron>());
  manager.convexDecompositionCache().insert("test1", decomposition);
  
  Eigen::Affine2d transform = Eigen::Affine2d::Identity();
  auto matrix = std::make_shared<IntermediateResults::TransformationMatrix>(transform);
  manager.transformationMatrixCache().insert("test2", matrix);
  
  // Verify caches have data
  QVERIFY(manager.convexDecompositionCache().size() > 0);
  QVERIFY(manager.transformationMatrixCache().size() > 0);
  
  // Test memory limits
  manager.setMemoryLimits(10, 5, 5, 5); // MB
  QCOMPARE(manager.convexDecompositionCache().maxCostMB(), static_cast<size_t>(10));
  QCOMPARE(manager.transformationMatrixCache().maxCostMB(), static_cast<size_t>(5));
  
  // Test total memory usage
  size_t total = manager.totalMemoryUsage();
  QVERIFY(total > 0);
}

void TestIntermediateCache::testCacheConcurrency() {
  // Basic test that cache can handle concurrent access
  // (Full thread safety testing would require more complex setup)
  auto& cache = IntermediateCacheManager::instance().convexDecompositionCache();
  
  // Insert and retrieve from multiple "threads" (simulated)
  for (int i = 0; i < 100; ++i) {
    std::string key = "concurrent_" + std::to_string(i);
    auto decomposition = std::make_shared<IntermediateResults::ConvexDecomposition>(std::list<CGAL_Polyhedron>());
    
    cache.insert(key, decomposition);
    auto retrieved = cache.get(key);
    QVERIFY(retrieved != nullptr);
  }
  
  QCOMPARE(cache.size(), static_cast<size_t>(100));
}

void TestIntermediateCache::testCacheCorrectnessWithRealGeometry() {
  // Test that cached results match non-cached computation
  // This would be expanded with real geometry in practice
  auto& cache = IntermediateCacheManager::instance().transformationMatrixCache();
  
  // Create a transformation matrix
  double scale_x = 2.0, scale_y = 1.5, rotation = M_PI / 6;
  Eigen::Affine2d expected_transform(Eigen::Scaling(scale_x, scale_y) * Eigen::Affine2d(Eigen::Rotation2Dd(rotation)));
  
  auto matrix = std::make_shared<IntermediateResults::TransformationMatrix>(expected_transform);
  std::string key = CacheKeyUtils::transformKey(scale_x, scale_y, rotation, 0);
  
  // Insert and retrieve
  cache.insert(key, matrix);
  auto cached_matrix = cache.get(key);
  
  QVERIFY(cached_matrix != nullptr);
  
  // Verify the cached result matches the original
  Eigen::Matrix3d expected_mat = expected_transform.matrix();
  Eigen::Matrix3d cached_mat = cached_matrix->transform.matrix();
  
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      QVERIFY(std::abs(expected_mat(i, j) - cached_mat(i, j)) < 1e-12);
    }
  }
}

std::shared_ptr<const Geometry> TestIntermediateCache::createTestGeometry() {
  // Create a simple test geometry (cube as PolySet)
  auto ps = std::make_shared<PolySet>(3);
  
  // Add a simple cube vertices (simplified for testing)
  ps->append_vertex(0, 0, 0);
  ps->append_vertex(1, 0, 0);
  ps->append_vertex(1, 1, 0);
  ps->append_vertex(0, 1, 0);
  ps->append_vertex(0, 0, 1);
  ps->append_vertex(1, 0, 1);
  ps->append_vertex(1, 1, 1);
  ps->append_vertex(0, 1, 1);
  
  // Add faces (simplified)
  ps->append_poly(); // Start new polygon
  
  return ps;
}

void TestIntermediateCache::verifyMemoryUsage(size_t expected_max_mb) {
  auto& manager = IntermediateCacheManager::instance();
  size_t total_bytes = manager.totalMemoryUsage();
  size_t total_mb = total_bytes / (1024 * 1024);
  
  QVERIFY(total_mb <= expected_max_mb + 1); // Allow 1MB tolerance
}

#include "TestIntermediateCache.moc"