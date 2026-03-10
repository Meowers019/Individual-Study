#include <unity.h>
#include <utils.h>

void setUp(void) {
  // set stuff up here
}

void tearDown(void) {
  // clean stuff up here
}

void test_clampFloat() {
  TEST_ASSERT_EQUAL_FLOAT(0.5f, clampFloat(0.5f, 0.0f, 1.0f));  // Within range
  TEST_ASSERT_EQUAL_FLOAT(0.0f, clampFloat(-0.5f, 0.0f, 1.0f)); // Below range
  TEST_ASSERT_EQUAL_FLOAT(1.0f, clampFloat(1.5f, 0.0f, 1.0f));  // Above range
  TEST_ASSERT_EQUAL_FLOAT(0.0f, clampFloat(0.0f, 0.0f, 1.0f)); // At lower bound
  TEST_ASSERT_EQUAL_FLOAT(1.0f, clampFloat(1.0f, 0.0f, 1.0f)); // At upper bound
}

int main(int argc, char **argv) {
  UNITY_BEGIN();
  RUN_TEST(test_clampFloat);
  UNITY_END();
}