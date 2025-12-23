#!/usr/bin/env python3
"""
Quick smoke test for VLA Integration System

This test verifies that the basic components of the VLA system can be imported
and initialized correctly, providing a quick check before running full tests.
"""

import sys
from datetime import datetime


def test_imports():
    """Test that all major components can be imported."""
    print("Testing imports...")

    try:
        from vla_integration import VLANode
        from vla_integration import SpeechRecognitionService
        from vla_integration import VLAPipeline
        from vla_integration import get_performance_monitor
        from vla_integration import get_security_validator
        print("  ✓ Core modules imported successfully")
    except ImportError as e:
        print(f"  ✗ Import error: {e}")
        return False

    try:
        from vla_integration.speech_recognition import WhisperInterface
        from vla_integration.nlu import IntentExtractor
        from vla_integration.cognitive_planning import ActionGenerator
        from vla_integration.pipeline import VLAPipeline
        print("  ✓ Component modules imported successfully")
    except ImportError as e:
        print(f"  ✗ Component import error: {e}")
        return False

    return True


def test_basic_initialization():
    """Test basic initialization of key components."""
    print("Testing basic initialization...")

    try:
        # Test pipeline initialization
        from vla_integration.pipeline.vla_pipeline import VLAPipeline
        pipeline = VLAPipeline()
        print("  ✓ VLAPipeline initialized successfully")

        # Test speech recognition service initialization
        from vla_integration.speech_recognition.speech_recognition_service import SpeechRecognitionService
        speech_service = SpeechRecognitionService(model_name="base", use_cache=True)
        print("  ✓ SpeechRecognitionService initialized successfully")

        # Test performance monitor
        from vla_integration.utils.performance_monitor import get_performance_monitor
        perf_monitor = get_performance_monitor()
        print("  ✓ Performance monitor accessible")

        # Test security validator
        from vla_integration.utils.security_validator import get_security_validator
        security_validator = get_security_validator()
        print("  ✓ Security validator accessible")

    except Exception as e:
        print(f"  ✗ Initialization error: {e}")
        return False

    return True


def test_config_validation():
    """Test configuration validation."""
    print("Testing configuration validation...")

    try:
        from vla_integration.utils.config_validator import ConfigValidator
        validator = ConfigValidator()

        # Test with minimal valid configuration
        config = {
            "speech_recognition": {
                "model": "base",
                "language": "en",
                "timeout": 5.0
            },
            "nlu": {
                "confidence_threshold": 0.7,
                "max_command_length": 200
            },
            "task_planning": {
                "max_action_sequence_length": 10,
                "action_timeout": 30.0,
                "retry_attempts": 3
            }
        }

        is_valid, errors = validator.validate_config(config)
        if is_valid:
            print("  ✓ Configuration validation passed")
        else:
            print(f"  ✗ Configuration validation failed: {errors}")
            return False

    except Exception as e:
        print(f"  ✗ Configuration validation error: {e}")
        return False

    return True


def test_security_validation():
    """Test security validation."""
    print("Testing security validation...")

    try:
        from vla_integration.utils.security_validator import get_security_validator
        validator = get_security_validator()

        # Test safe command
        is_valid, reason, level = validator.validate_command(
            "Go to the kitchen", "navigate", {"location": "kitchen"}
        )
        if is_valid:
            print("  ✓ Safe command validation passed")
        else:
            print(f"  ✗ Safe command validation failed: {reason}")
            return False

        # Test potentially dangerous command
        is_valid, reason, level = validator.validate_command(
            "Run system shutdown", "unknown", {"action": "shutdown"}
        )
        if not is_valid:
            print("  ✓ Dangerous command correctly blocked")
        else:
            print("  ✗ Dangerous command should have been blocked")
            return False

    except Exception as e:
        print(f"  ✗ Security validation error: {e}")
        return False

    return True


def main():
    """Run the smoke test."""
    print("VLA Integration System - Smoke Test")
    print(f"Run started at: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    print("-" * 50)

    tests = [
        ("Import Test", test_imports),
        ("Initialization Test", test_basic_initialization),
        ("Configuration Validation", test_config_validation),
        ("Security Validation", test_security_validation),
    ]

    passed = 0
    total = len(tests)

    for test_name, test_func in tests:
        print(f"\n{test_name}:")
        if test_func():
            passed += 1
            print(f"  Status: PASSED")
        else:
            print(f"  Status: FAILED")

    print("\n" + "="*50)
    print("SMOKE TEST RESULTS")
    print("="*50)
    print(f"Tests Passed: {passed}/{total}")
    print(f"Success Rate: {(passed/total)*100:.1f}%")

    if passed == total:
        print("✓ All smoke tests passed!")
        print("The VLA system appears to be properly set up.")
        sys.exit(0)
    else:
        print("✗ Some smoke tests failed!")
        print("Please check the errors above before running full tests.")
        sys.exit(1)


if __name__ == "__main__":
    main()