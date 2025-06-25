#pragma once

#include <QTest>
#include <QObject>
#include "geometry/IntermediateCache.h"

class TestIntermediateCache : public QObject
{
  Q_OBJECT

public:
  // Standalone test runner
  static int runStandaloneTests();

private slots:
  void initTestCase();
  void cleanupTestCase();
  void init(); // Called before each test method
  void cleanup(); // Called after each test method
  
  // Test basic cache functionality
  void testCacheInsertAndRetrieve();
  void testCacheEviction();
  void testCacheKeyGeneration();
  void testMemoryManagement();
  
  // Test specific intermediate result types
  void testConvexDecompositionCache();
  void testTransformationMatrixCache();
  void testSliceParametersCache();
  void testPointCloudCache();
  
  // Test cache manager
  void testIntermediateCacheManager();
  void testGlobalCacheOperations();
  
  // Test thread safety (basic)
  void testCacheConcurrency();
  
  // Test cache correctness for real geometry operations
  void testCacheCorrectnessWithRealGeometry();

private:
  // Helper methods for testing
  std::shared_ptr<const class Geometry> createTestGeometry();
  void verifyMemoryUsage(size_t expected_max_mb);
};