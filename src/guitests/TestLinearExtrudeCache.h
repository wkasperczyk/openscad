#pragma once

#include <QObject>

class TestLinearExtrudeCache : public QObject
{
  Q_OBJECT

public:
  static int runStandaloneTests();

private slots:
  void initTestCase();
  void cleanupTestCase();
  void init();
  void cleanup();

  // Test transformation matrix caching for linear extrude
  void testTransformationMatrixCaching();
  void testTransformationMatrixKeyGeneration();
  void testLinearExtrudeWithoutCache();
  void testLinearExtrudeWithCache();
  void testCacheHitRatios();
  void testComplexLinearExtrude();
  void testTwistScaleParameters();
  void testCacheEvictionOnLinearExtrude();

private:
  // Helper functions for testing
  void compareGeometries(const std::shared_ptr<const class Geometry>& geom1, 
                        const std::shared_ptr<const class Geometry>& geom2);
  std::shared_ptr<const class Polygon2d> createTestPolygon();
  class LinearExtrudeNode createTestNode(double twist = 0.0, double scale_x = 1.0, 
                                        double scale_y = 1.0, double height = 10.0);
};