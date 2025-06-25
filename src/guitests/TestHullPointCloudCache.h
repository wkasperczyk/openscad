#pragma once

#include <QObject>
#include <memory>

class PolySet;

class TestHullPointCloudCache : public QObject
{
  Q_OBJECT

public:
  static int runStandaloneTests();

private slots:
  void initTestCase();
  void cleanupTestCase();
  void init();
  void cleanup();
  
  void testPointCloudExtractionFromPolySet();
  void testPointCloudCacheHitAndMiss();
  void testCacheKeyConsistency();
  void testCacheEvictionWithLargePointClouds();
  void testMemoryUsageCalculation();
  void testCacheIntegrationWithHullOperation();
  void testMultipleGeometryTypes();
  void testCacheConcurrencySimulation();
  void testCacheInvalidation();

private:
  std::shared_ptr<PolySet> createTestPolySet();
};