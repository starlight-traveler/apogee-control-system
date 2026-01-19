#ifdef ARDUINO
#include <Arduino.h>
#endif

#include <math.h>
#include <unity.h>

#include "bno085_orientation.h"
#include "constants.h"

void setUp() {}
void tearDown() {}

static void TestVectorTransformationPreservesXAxis() {
    float outX = 0.0f;
    float outY = 0.0f;
    float outZ = 0.0f;
    bno085_orientation::TransformVector(1.0f, 0.0f, 0.0f, outX, outY, outZ);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.0f, outX);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, outY);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, outZ);
}

static void TestVectorTransformationFlipsGravity() {
    float outX = 0.0f;
    float outY = 0.0f;
    float outZ = 0.0f;
    bno085_orientation::TransformVector(0.0f, 0.0f, constants::kGravity, outX, outY, outZ);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, outX);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, outY);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, -constants::kGravity, outZ);
}

static void TestQuaternionIdentityBecomesOffset() {
    float out[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    bno085_orientation::AdjustQuaternion(1.0f, 0.0f, 0.0f, 0.0f, out);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, out[0]);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.0f, out[1]);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, out[2]);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, out[3]);
}

static void TestQuaternionPremultiplication() {
    const float rawW = 0.70710677f;
    const float rawX = 0.0f;
    const float rawY = 0.70710677f;
    const float rawZ = 0.0f;
    float out[4];
    bno085_orientation::AdjustQuaternion(rawW, rawX, rawY, rawZ, out);

    TEST_ASSERT_FLOAT_WITHIN(1e-5f, -rawX, out[0]);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, rawW, out[1]);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, -rawZ, out[2]);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, rawY, out[3]);

    const float norm = sqrtf(out[0] * out[0] + out[1] * out[1] + out[2] * out[2] + out[3] * out[3]);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 1.0f, norm);
}

#ifdef ARDUINO
void setup() {
    UNITY_BEGIN();
    RUN_TEST(TestVectorTransformationPreservesXAxis);
    RUN_TEST(TestVectorTransformationFlipsGravity);
    RUN_TEST(TestQuaternionIdentityBecomesOffset);
    RUN_TEST(TestQuaternionPremultiplication);
    UNITY_END();
}

void loop() {}
#else
int main() {
    UNITY_BEGIN();
    RUN_TEST(TestVectorTransformationPreservesXAxis);
    RUN_TEST(TestVectorTransformationFlipsGravity);
    RUN_TEST(TestQuaternionIdentityBecomesOffset);
    RUN_TEST(TestQuaternionPremultiplication);
    return UNITY_END();
}
#endif
