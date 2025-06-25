// Empty
hull();
// No children
hull() { }

// Hull of hull (forces internal cache to be initialized; this has caused a crash earlier)
translate([25,0,0]) hull() hull3test();

module hull3test() {
  hull() {
    cylinder(r=10, h=1);
    translate([0,0,10]) cube([5,5,5], center=true);
  }
}
hull3test();

translate([50,0,0]) hull() {
  translate([0,0,10]) cylinder(r=3);
  difference() {
    cylinder(r=10, h=4, center=true);
    cylinder(r=5, h=5, center=true);
  }
}

// Don't Crash (issue 188)

translate([-5,-5,-5]) {
  hull() {
    intersection() {
      cube([1,1,1]);
      translate([-1,-1,-1]) cube([1,1,1]);
    }
  }
}

module hull3null() {
  hull() {
    cube(0);
    sphere(0);
  }
}
hull3null();

// Point cloud cache test - reuse same geometry in multiple hull operations
module cache_test_geometry() {
  cube([2,2,2]);
  translate([4,0,0]) cylinder(r=1, h=3);
}

// First hull operation - will populate point cloud cache
translate([0,25,0]) hull() {
  cache_test_geometry();
}

// Second hull operation - should reuse cached point clouds
translate([15,25,0]) hull() {
  cache_test_geometry();
  translate([0,6,0]) sphere(r=1.5);
}

