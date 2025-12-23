"""
Performance Monitoring Module
Provides performance metrics collection for the VLA system
"""

import time
from datetime import datetime
from typing import Dict, Any, Optional
import threading
import json


class PerformanceMonitor:
    """
    Monitors and collects performance metrics for the VLA system.
    """

    def __init__(self):
        """
        Initialize the performance monitor.
        """
        self.metrics = {}
        self.start_times = {}
        self.lock = threading.Lock()

        # Initialize default metrics
        self.metrics = {
            "total_requests": 0,
            "successful_requests": 0,
            "failed_requests": 0,
            "average_response_time": 0.0,
            "total_response_time": 0.0,
            "peak_memory_usage": 0,
            "current_memory_usage": 0,
            "max_response_time": 0.0,
            "min_response_time": float('inf'),
            "requests_under_3s": 0,  # Track requests under 3 seconds
            "requests_over_3s": 0,   # Track requests over 3 seconds
            "success_rate_3s": 0.0,  # Success rate for requests under 3 seconds
        }

    def start_timer(self, operation_name: str):
        """
        Start timing an operation.

        Args:
            operation_name: Name of the operation to time
        """
        with self.lock:
            self.start_times[operation_name] = time.time()

    def end_timer(self, operation_name: str) -> float:
        """
        End timing an operation and return the elapsed time.

        Args:
            operation_name: Name of the operation that was timed

        Returns:
            Elapsed time in seconds
        """
        with self.lock:
            if operation_name in self.start_times:
                elapsed = time.time() - self.start_times[operation_name]
                del self.start_times[operation_name]

                # Update metrics
                self.metrics["total_response_time"] += elapsed
                self.metrics["total_requests"] += 1

                # Update average response time
                if self.metrics["total_requests"] > 0:
                    self.metrics["average_response_time"] = (
                        self.metrics["total_response_time"] / self.metrics["total_requests"]
                    )

                # Update min/max response times
                if elapsed > self.metrics["max_response_time"]:
                    self.metrics["max_response_time"] = elapsed
                if elapsed < self.metrics["min_response_time"]:
                    self.metrics["min_response_time"] = elapsed

                # Track compliance with 3-second requirement
                if elapsed <= 3.0:
                    self.metrics["requests_under_3s"] += 1
                else:
                    self.metrics["requests_over_3s"] += 1

                # Update success rate for 3-second compliance
                if self.metrics["requests_under_3s"] + self.metrics["requests_over_3s"] > 0:
                    self.metrics["success_rate_3s"] = (
                        self.metrics["requests_under_3s"] /
                        (self.metrics["requests_under_3s"] + self.metrics["requests_over_3s"])
                    ) * 100.0

                return elapsed
            else:
                return 0.0

    def record_success(self):
        """
        Record a successful operation.
        """
        with self.lock:
            self.metrics["successful_requests"] += 1

    def record_failure(self):
        """
        Record a failed operation.
        """
        with self.lock:
            self.metrics["failed_requests"] += 1

    def update_memory_usage(self, current_usage: int, peak_usage: int = None):
        """
        Update memory usage metrics.

        Args:
            current_usage: Current memory usage
            peak_usage: Peak memory usage (optional)
        """
        with self.lock:
            self.metrics["current_memory_usage"] = current_usage
            if peak_usage and peak_usage > self.metrics["peak_memory_usage"]:
                self.metrics["peak_memory_usage"] = peak_usage

    def get_metrics(self) -> Dict[str, Any]:
        """
        Get current performance metrics.

        Returns:
            Dictionary of performance metrics
        """
        with self.lock:
            return self.metrics.copy()

    def get_formatted_metrics(self) -> str:
        """
        Get formatted performance metrics as a string.

        Returns:
            Formatted metrics string
        """
        metrics = self.get_metrics()
        return json.dumps(metrics, indent=2)

    def is_meeting_performance_requirements(self, target_avg_response_time: float = 3.0,
                                          target_success_rate: float = 80.0) -> bool:
        """
        Check if the system is meeting performance requirements.

        Args:
            target_avg_response_time: Target average response time in seconds
            target_success_rate: Target success rate for 3-second compliance (%)

        Returns:
            True if meeting requirements, False otherwise
        """
        with self.lock:
            if self.metrics["total_requests"] == 0:
                return False  # No data to evaluate

            avg_response_time = self.metrics["average_response_time"]
            success_rate_3s = self.metrics["success_rate_3s"]

            return (avg_response_time <= target_avg_response_time and
                    success_rate_3s >= target_success_rate)

    def get_performance_summary(self) -> Dict[str, Any]:
        """
        Get a summary of performance compliance with requirements.

        Returns:
            Dictionary with performance compliance information
        """
        with self.lock:
            total_requests = self.metrics["total_requests"]
            if total_requests == 0:
                return {
                    "compliant": False,
                    "message": "No requests processed yet",
                    "average_response_time": 0.0,
                    "requests_under_3s": 0,
                    "requests_over_3s": 0,
                    "success_rate_3s": 0.0,
                    "total_requests": 0
                }

            avg_response_time = self.metrics["average_response_time"]
            success_rate_3s = self.metrics["success_rate_3s"]
            requests_under_3s = self.metrics["requests_under_3s"]
            requests_over_3s = self.metrics["requests_over_3s"]

            compliant = avg_response_time <= 3.0 and success_rate_3s >= 80.0

            return {
                "compliant": compliant,
                "message": f"System {'is' if compliant else 'is NOT'} meeting performance requirements",
                "average_response_time": avg_response_time,
                "requests_under_3s": requests_under_3s,
                "requests_over_3s": requests_over_3s,
                "success_rate_3s": success_rate_3s,
                "total_requests": total_requests,
                "max_response_time": self.metrics["max_response_time"],
                "min_response_time": self.metrics["min_response_time"] if self.metrics["min_response_time"] != float('inf') else 0.0
            }

    def reset_metrics(self):
        """
        Reset all performance metrics to initial values.
        """
        with self.lock:
            self.metrics = {
                "total_requests": 0,
                "successful_requests": 0,
                "failed_requests": 0,
                "average_response_time": 0.0,
                "total_response_time": 0.0,
                "peak_memory_usage": 0,
                "current_memory_usage": 0,
                "max_response_time": 0.0,
                "min_response_time": float('inf'),
                "requests_under_3s": 0,  # Track requests under 3 seconds
                "requests_over_3s": 0,   # Track requests over 3 seconds
                "success_rate_3s": 0.0,  # Success rate for requests under 3 seconds
            }
            self.start_times = {}


# Global performance monitor instance
performance_monitor = PerformanceMonitor()


def get_performance_monitor() -> PerformanceMonitor:
    """
    Get the global performance monitor instance.

    Returns:
        PerformanceMonitor instance
    """
    return performance_monitor


# Example usage function
def example_usage():
    """Example of how to use the PerformanceMonitor."""
    print("Performance Monitor example usage:")
    print("-" * 35)

    # Get the global monitor instance
    monitor = get_performance_monitor()

    # Simulate multiple operations to test 3-second compliance tracking
    operations = [
        ("fast_operation", 0.5),      # Under 3 seconds
        ("medium_operation", 2.0),    # Under 3 seconds
        ("slow_operation", 4.0),      # Over 3 seconds
        ("fast_operation2", 1.0),     # Under 3 seconds
        ("slow_operation2", 5.0),     # Over 3 seconds
    ]

    for op_name, duration in operations:
        monitor.start_timer(op_name)
        time.sleep(duration)  # Simulate processing time
        elapsed = monitor.end_timer(op_name)

        # Record success/failure randomly for demonstration
        import random
        if random.random() > 0.3:  # 70% success rate
            monitor.record_success()
        else:
            monitor.record_failure()

        print(f"Operation '{op_name}' took {elapsed:.3f} seconds")

    # Print performance summary
    print("\nPerformance Summary:")
    summary = monitor.get_performance_summary()
    for key, value in summary.items():
        print(f"  {key}: {value}")

    # Check if system meets requirements
    meets_requirements = monitor.is_meeting_performance_requirements()
    print(f"\nSystem meets performance requirements: {meets_requirements}")

    # Print all metrics
    print("\nAll metrics:")
    print(monitor.get_formatted_metrics())


if __name__ == "__main__":
    example_usage()