#include "TestLinearExtrudeCache.h"
#include "geometry/IntermediateCache.h"
#include "geometry/linear_extrude.h"
#include "geometry/Polygon2d.h"
#include "geometry/PolySet.h"
#include "geometry/linalg.h"
#include "core/LinearExtrudeNode.h"

#include <QTest>
#include <memory>
#include <vector>
#include <iostream>

int TestLinearExtrudeCache::runStandaloneTests() {
  TestLinearExtrudeCache test;
  return QTest::qExec(&test);
}

void TestLinearExtrudeCache::initTestCase() {
  // Initialize any global test data
}

void TestLinearExtrudeCache::cleanupTestCase() {
  // Clean up global test data
  IntermediateCacheManager::instance().clearAll();
}

void TestLinearExtrudeCache::init() {
  // Clear caches before each test to ensure isolation
  IntermediateCacheManager::instance().clearAll();
}

void TestLinearExtrudeCache::cleanup() {
  // Clean up after each test
  IntermediateCacheManager::instance().clearAll();
}

void TestLinearExtrudeCache::testTransformationMatrixCaching() {
  auto& cache = IntermediateCacheManager::instance().transformationMatrixCache();
  
  // Create a test transformation matrix
  double scale_x = 2.0, scale_y = 1.5, rotation = M_PI / 4;
  Eigen::Affine2d transform(Eigen::Scaling(scale_x, scale_y) * Eigen::Affine2d(Eigen::Rotation2Dd(rotation)));
  
  auto matrix = std::make_shared<IntermediateResults::TransformationMatrix>(transform);
  std::string key = CacheKeyUtils::transformKey(scale_x, scale_y, rotation, 0);
  
  // Test insertion and retrieval
  QVERIFY(cache.insert(key, matrix));
  QVERIFY(cache.contains(key));
  
  auto retrieved = cache.get(key);
  QVERIFY(retrieved != nullptr);
  
  // Verify matrix values are preserved
  Eigen::Matrix3d expected = transform.matrix();
  Eigen::Matrix3d actual = retrieved->transform.matrix();
  
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      QVERIFY(std::abs(expected(i, j) - actual(i, j)) < 1e-12);
    }
  }
}

void TestLinearExtrudeCache::testTransformationMatrixKeyGeneration() {
  // Test that identical parameters produce identical keys
  std::string key1 = CacheKeyUtils::transformKey(1.0, 1.0, 0.0, 0);
  std::string key2 = CacheKeyUtils::transformKey(1.0, 1.0, 0.0, 0);
  QCOMPARE(key1, key2);
  
  // Test that different parameters produce different keys
  std::string key3 = CacheKeyUtils::transformKey(1.0, 1.0, 0.0, 1);
  std::string key4 = CacheKeyUtils::transformKey(1.0, 1.5, 0.0, 0);
  std::string key5 = CacheKeyUtils::transformKey(1.0, 1.0, M_PI/4, 0);
  
  QVERIFY(key1 != key3);
  QVERIFY(key1 != key4);
  QVERIFY(key1 != key5);
  
  // Test precision handling
  std::string key6 = CacheKeyUtils::transformKey(1.000001, 1.0, 0.0, 0);
  std::string key7 = CacheKeyUtils::transformKey(1.000002, 1.0, 0.0, 0);
  QVERIFY(key6 != key7); // Should be different due to precision
}

void TestLinearExtrudeCache::testLinearExtrudeWithoutCache() {
  // Clear cache to ensure we're testing without cache
  IntermediateCacheManager::instance().clearAll();
  auto& cache = IntermediateCacheManager::instance().transformationMatrixCache();
  
  auto polygon = createTestPolygon();
  auto node = createTestNode(45.0, 2.0, 1.5, 20.0); // twist, scale_x, scale_y, height
  
  // Record initial cache state
  size_t initial_size = cache.size();
  
  // Run linear extrude (this will be our baseline)
  auto result1 = extrudePolygon(node, *polygon);
  QVERIFY(result1 != nullptr);
  
  // For now, cache won't be populated since we haven't implemented caching yet
  QCOMPARE(cache.size(), initial_size);
}

void TestLinearExtrudeCache::testLinearExtrudeWithCache() {
  // This test will be updated after we implement the cache integration
  auto polygon = createTestPolygon();
  auto node = createTestNode(45.0, 2.0, 1.5, 20.0);
  auto& cache = IntermediateCacheManager::instance().transformationMatrixCache();
  
  // Run linear extrude twice with same parameters
  auto result1 = extrudePolygon(node, *polygon);
  size_t cache_size_after_first = cache.size();
  
  auto result2 = extrudePolygon(node, *polygon);
  size_t cache_size_after_second = cache.size();
  
  QVERIFY(result1 != nullptr);
  QVERIFY(result2 != nullptr);
  
  // Results should be equivalent
  compareGeometries(result1, result2);
}

void TestLinearExtrudeCache::testCacheHitRatios() {
  // Test that cache improves performance for repeated operations
  auto polygon = createTestPolygon();
  auto& cache = IntermediateCacheManager::instance().transformationMatrixCache();
  
  // Run same linear extrude multiple times
  auto node = createTestNode(90.0, 1.5, 2.0, 15.0);
  
  std::vector<std::shared_ptr<const Geometry>> results;
  for (int i = 0; i < 5; ++i) {
    results.push_back(extrudePolygon(node, *polygon));
  }
  
  // All results should be valid
  for (const auto& result : results) {
    QVERIFY(result != nullptr);
  }
  
  // Cache should have entries (when we implement caching)
  // For now, just verify the geometry is consistent
  for (size_t i = 1; i < results.size(); ++i) {
    compareGeometries(results[0], results[i]);
  }
}

void TestLinearExtrudeCache::testComplexLinearExtrude() {
  // Test complex linear extrude with multiple parameters
  auto polygon = createTestPolygon();
  
  // Test various parameter combinations
  std::vector<LinearExtrudeNode> nodes = {
    createTestNode(0.0, 1.0, 1.0, 10.0),      // No twist, no scale
    createTestNode(180.0, 1.0, 1.0, 10.0),    // Twist only
    createTestNode(0.0, 2.0, 0.5, 10.0),      // Scale only
    createTestNode(45.0, 1.5, 1.5, 10.0),     // Twist and scale
    createTestNode(90.0, 2.0, 0.25, 5.0),     // Complex case
  };
  
  for (const auto& node : nodes) {
    auto result = extrudePolygon(node, *polygon);
    QVERIFY(result != nullptr);
    QVERIFY(!result->isEmpty());
  }
}

void TestLinearExtrudeCache::testTwistScaleParameters() {
  // Test that different twist and scale parameters produce different results
  auto polygon = createTestPolygon();
  
  auto result1 = extrudePolygon(createTestNode(0.0, 1.0, 1.0, 10.0), *polygon);
  auto result2 = extrudePolygon(createTestNode(45.0, 1.0, 1.0, 10.0), *polygon);
  auto result3 = extrudePolygon(createTestNode(0.0, 2.0, 1.0, 10.0), *polygon);
  
  QVERIFY(result1 != nullptr);
  QVERIFY(result2 != nullptr);
  QVERIFY(result3 != nullptr);
  
  // Results should be different (can't easily compare geometry content, but verify they exist)
  QVERIFY(!result1->isEmpty());
  QVERIFY(!result2->isEmpty());
  QVERIFY(!result3->isEmpty());
}

void TestLinearExtrudeCache::testCacheEvictionOnLinearExtrude() {
  auto& cache = IntermediateCacheManager::instance().transformationMatrixCache();
  cache.setMaxCostMB(1); // Set very small limit to trigger eviction
  
  auto polygon = createTestPolygon();
  
  // Generate many different linear extrude operations to fill cache
  for (int i = 0; i < 100; ++i) {
    double twist = i * 3.6; // 0 to 356 degrees
    double scale = 1.0 + i * 0.01; // 1.0 to 2.0
    auto node = createTestNode(twist, scale, scale, 10.0);
    
    auto result = extrudePolygon(node, *polygon);
    QVERIFY(result != nullptr);
  }
  
  // Cache should have evicted some entries due to memory limit
  // Note: This test will be more meaningful after we implement caching
  QVERIFY(cache.totalCost() <= cache.maxCostMB() * 1024 * 1024);
}

void TestLinearExtrudeCache::compareGeometries(const std::shared_ptr<const Geometry>& geom1, 
                                              const std::shared_ptr<const Geometry>& geom2) {
  QVERIFY(geom1 != nullptr);
  QVERIFY(geom2 != nullptr);
  
  // Basic comparison - in a real implementation we'd compare vertex counts, bounds, etc.
  QCOMPARE(geom1->getDimension(), geom2->getDimension());
  QCOMPARE(geom1->isEmpty(), geom2->isEmpty());
  
  // Compare bounding boxes
  auto bbox1 = geom1->getBoundingBox();
  auto bbox2 = geom2->getBoundingBox();
  
  const double tolerance = 1e-10;
  QVERIFY((bbox1.min() - bbox2.min()).norm() < tolerance);
  QVERIFY((bbox1.max() - bbox2.max()).norm() < tolerance);
}

std::shared_ptr<const Polygon2d> TestLinearExtrudeCache::createTestPolygon() {
  auto poly = std::make_shared<Polygon2d>();
  
  // Create a simple square polygon
  Outline2d outline;
  outline.vertices = {
    Vector2d(0, 0),
    Vector2d(10, 0),
    Vector2d(10, 10),
    Vector2d(0, 10)
  };
  
  poly->addOutline(outline);
  poly->sanitize();
  
  return poly;
}

LinearExtrudeNode TestLinearExtrudeCache::createTestNode(double twist, double scale_x, 
                                                        double scale_y, double height) {
  LinearExtrudeNode node;
  node.twist = twist;
  node.scale_x = scale_x;
  node.scale_y = scale_y;
  node.height = Vector3d(0, 0, height);
  node.center = false;
  node.convexity = 1;
  node.fn = 0;
  node.fs = 2.0;
  node.fa = 12.0;
  node.has_segments = false;
  node.segments = 0;
  
  return node;
}

#include "TestLinearExtrudeCache.moc"