#ifndef TEST_FRAMEWORK_H_
#define TEST_FRAMEWORK_H_

#include <math.h>
#include <stdint.h>
#include <stdio.h>

static int test_failures = 0;

#define TEST_ASSERT_TRUE(condition)                                                     \
    do                                                                                  \
    {                                                                                   \
        if (!(condition))                                                               \
        {                                                                               \
            printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #condition);                 \
            test_failures++;                                                            \
        }                                                                               \
    } while (0)

#define TEST_ASSERT_INT_EQ(expected, actual)                                            \
    do                                                                                  \
    {                                                                                   \
        const int expected_value = (int)(expected);                                     \
        const int actual_value = (int)(actual);                                         \
        if (expected_value != actual_value)                                             \
        {                                                                               \
            printf("FAIL %s:%d: expected %d, got %d\n",                                \
                   __FILE__, __LINE__, expected_value, actual_value);                   \
            test_failures++;                                                            \
        }                                                                               \
    } while (0)

#define TEST_ASSERT_UINT_EQ(expected, actual)                                           \
    do                                                                                  \
    {                                                                                   \
        const unsigned int expected_value = (unsigned int)(expected);                   \
        const unsigned int actual_value = (unsigned int)(actual);                       \
        if (expected_value != actual_value)                                             \
        {                                                                               \
            printf("FAIL %s:%d: expected %u, got %u\n",                                \
                   __FILE__, __LINE__, expected_value, actual_value);                   \
            test_failures++;                                                            \
        }                                                                               \
    } while (0)

#define TEST_ASSERT_FLOAT_NEAR(expected, actual, tolerance)                             \
    do                                                                                  \
    {                                                                                   \
        const float expected_value = (float)(expected);                                 \
        const float actual_value = (float)(actual);                                     \
        const float tolerance_value = (float)(tolerance);                               \
        if (fabsf(expected_value - actual_value) > tolerance_value)                     \
        {                                                                               \
            printf("FAIL %s:%d: expected %.6f, got %.6f\n",                            \
                   __FILE__, __LINE__, expected_value, actual_value);                   \
            test_failures++;                                                            \
        }                                                                               \
    } while (0)

#define RUN_TEST(test_function)                                                         \
    do                                                                                  \
    {                                                                                   \
        printf("RUN  %s\n", #test_function);                                           \
        test_function();                                                                \
    } while (0)

#define TEST_REPORT()                                                                   \
    ((test_failures == 0) ? (printf("PASS\n"), 0) : (printf("FAILURES: %d\n", test_failures), 1))

#endif /* TEST_FRAMEWORK_H_ */
