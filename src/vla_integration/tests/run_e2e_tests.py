#!/usr/bin/env python3
"""
Runner script for VLA Integration End-to-End Tests

This script provides a convenient way to run the complete end-to-end test suite
for the Vision-Language-Action system, validating all user stories.
"""

import sys
import os
import unittest
from datetime import datetime


def run_tests():
    """Run the end-to-end test suite."""
    print(f"Starting VLA Integration End-to-End Tests")
    print(f"Run started at: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    print("-" * 60)

    # Discover and run tests
    loader = unittest.TestLoader()

    # Look for test files in the current directory
    start_dir = os.path.dirname(os.path.abspath(__file__))
    suite = loader.discover(start_dir, pattern='e2e_test_suite.py')

    # Run tests with detailed output
    runner = unittest.TextTestRunner(
        verbosity=2,
        stream=sys.stdout,
        buffer=True  # Capture stdout/stderr during tests
    )

    result = runner.run(suite)

    return result


def generate_test_report(result):
    """Generate a test report."""
    print("\n" + "="*60)
    print("TEST EXECUTION REPORT")
    print("="*60)
    print(f"Execution Time: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    print(f"Tests Run: {result.testsRun}")
    print(f"Failures: {len(result.failures)}")
    print(f"Errors: {len(result.errors)}")
    print(f"Success Rate: {((result.testsRun - len(result.failures) - len(result.errors)) / result.testsRun * 100):.1f}%")

    if result.failures:
        print("\nFAILURES:")
        for test, traceback in result.failures:
            print(f"  {test}")
            print(f"    {traceback}")

    if result.errors:
        print("\nERRORS:")
        for test, traceback in result.errors:
            print(f"  {test}")
            print(f"    {traceback}")

    # Performance metrics
    print("\nPERFORMANCE METRICS:")
    try:
        from vla_integration.utils.performance_monitor import get_performance_monitor
        perf_monitor = get_performance_monitor()
        summary = perf_monitor.get_performance_summary()

        print(f"  Average Response Time: {summary['average_response_time']:.3f}s")
        print(f"  Requests < 3s: {summary['requests_under_3s']}")
        print(f"  Requests > 3s: {summary['requests_over_3s']}")
        print(f"  3s Compliance Rate: {summary['success_rate_3s']:.1f}%")
        print(f"  Total Requests: {summary['total_requests']}")

        # Compliance check
        compliant = perf_monitor.is_meeting_performance_requirements()
        print(f"  Performance Compliant: {'YES' if compliant else 'NO'}")

    except ImportError:
        print("  Performance monitoring not available")

    print("="*60)


def main():
    """Main function to run tests."""
    try:
        result = run_tests()
        generate_test_report(result)

        # Exit with appropriate code
        if result.failures or result.errors:
            print("\nSome tests failed or encountered errors.")
            sys.exit(1)
        else:
            print("\nAll tests passed!")
            sys.exit(0)

    except KeyboardInterrupt:
        print("\n\nTest execution interrupted by user.")
        sys.exit(1)
    except Exception as e:
        print(f"\nError running tests: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()