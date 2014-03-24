
#include "dubins_plus/dubins_plus.h"

#include <gtest/gtest.h>

using namespace dubins_plus;

void print(std::vector<Segment> &s) {
  for( int i=0; i<s.size(); i++ ) {
    printf("length: %f, curvature: %f\n", s[i].getLength(), s[i].getCurvature());
  }
}

TEST(DubinsTests, simple) {
  std::vector<Segment> a = dubins_path(1, 0, 0);
  EXPECT_EQ(a.size(), 3);
  EXPECT_FLOAT_EQ(a[0].getLength(),    0.0);
  EXPECT_FLOAT_EQ(a[1].getLength(),    1.0);
  EXPECT_FLOAT_EQ(a[1].getCurvature(), 0.0);
  EXPECT_FLOAT_EQ(a[2].getLength(),    0.0);
}

TEST(DubinsTests, simpleExamples) {
  double input[][3] = {
    1, 0, 0,
    1, 1, 0,
    1, -1, 0,
    1, 1, M_PI/2,
  };
  int input_sz = sizeof(input)/sizeof(double)/3;
  double output[][6] = {
    0.0,      1.0, 1.0,      0.0, 0.0,      1.0,
    0.785398, 1.0, 1.414214, 0.0, 5.497787, 1.0,
    5.497787, 1.0, 1.414214, 0.0, 0.785398, 1.0,
    M_PI/2,   1.0, 0.0,      0.0, 0.0,     -1.0,
  };

  for(int i=0; i<input_sz; i++) {
    std::vector<Segment> a = dubins_path(input[i][0], input[i][1], input[i][2]);
    EXPECT_EQ(a.size(), 3);
    EXPECT_FLOAT_EQ(a[0].getLength(), output[i][0]);
    EXPECT_FLOAT_EQ(a[0].getCurvature(), output[i][1]);
    EXPECT_FLOAT_EQ(a[1].getLength(), output[i][2]);
    EXPECT_FLOAT_EQ(a[1].getCurvature(), output[i][3]);
    EXPECT_FLOAT_EQ(a[2].getLength(), output[i][4]);
    EXPECT_FLOAT_EQ(a[2].getCurvature(), output[i][5]);
  }
}

TEST(DubinsTests, radiusExamples) {
  // Test that scaling the previous examples by radius/distance works as
  // expected
  double input[][4] = {
    2,   2, 0, 0,
    2,   2, 2, 0,
    0.5, 0.5, -0.5, 0,
    0.5, 0.5, 0.5, M_PI/2,
  };
  int input_sz = sizeof(input)/sizeof(double)/4;
  double output[][6] = {
    0.0,        0.5, 2.0,        0.0, 0.0,        0.5,
    2*0.785398, 0.5, 2*1.414214, 0.0, 2*5.497787, 0.5,
    5.497787/2, 2.0, 1.414214/2, 0.0, 0.785398/2, 2.0,
    M_PI/2/2,   2.0, 0.0,        0.0, 0.0,       -2.0,
  };

  for(int i=0; i<input_sz; i++) {
    std::vector<Segment> a = dubins_path(input[i][0], input[i][1], input[i][2],
        input[i][3]);
    EXPECT_EQ(a.size(), 3);
    EXPECT_FLOAT_EQ(a[0].getLength(), output[i][0]);
    EXPECT_FLOAT_EQ(a[0].getCurvature(), output[i][1]);
    EXPECT_FLOAT_EQ(a[1].getLength(), output[i][2]);
    EXPECT_FLOAT_EQ(a[1].getCurvature(), output[i][3]);
    EXPECT_FLOAT_EQ(a[2].getLength(), output[i][4]);
    EXPECT_FLOAT_EQ(a[2].getCurvature(), output[i][5]);
  }
}

TEST(DubinsTests, startAngle) {
  // Test that varied starting angles and positions give the correct results
  double input[][6] = {
    0, 0, M_PI/2, -1, 1, M_PI, // rotation only
    1, 1, 0, 2, 2, M_PI/2,     // translation only
    1, 1, M_PI/2, 0, 2, M_PI,  // translation and rotation
    0, 0, M_PI/4, 0, sqrt(2), 3*M_PI/4, 
  };
  int input_sz = sizeof(input)/sizeof(double)/6;
  //printf("Input size: %d\n", input_sz);
  double output[][6] = {
    M_PI/2, 1.0, 0.0, 0.0, 0.0, -1.0,
    M_PI/2, 1.0, 0.0, 0.0, 0.0, -1.0,
    M_PI/2, 1.0, 0.0, 0.0, 0.0, -1.0,
    M_PI/2, 1.0, 0.0, 0.0, 0.0, -1.0,
  };

  for(int i=0; i<input_sz; i++) {
    std::vector<Segment> a = dubins_path(1,
        input[i][0], input[i][1], input[i][2],
        input[i][3], input[i][4], input[i][5]);
    EXPECT_EQ(a.size(), 3);
    EXPECT_FLOAT_EQ(a[0].getLength(), output[i][0]);
    EXPECT_FLOAT_EQ(a[0].getCurvature(), output[i][1]);
    EXPECT_FLOAT_EQ(a[1].getLength(), output[i][2]);
    EXPECT_FLOAT_EQ(a[1].getCurvature(), output[i][3]);
    EXPECT_FLOAT_EQ(a[2].getLength(), output[i][4]);
    EXPECT_FLOAT_EQ(a[2].getCurvature(), output[i][5]);
  }
}

int main(int argc, char ** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
