"""
Caching Module
Implements caching mechanisms for repeated command patterns in the VLA system
"""

import time
import threading
from typing import Any, Optional, Dict
from datetime import datetime, timedelta


class VLACache:
    """
    Cache for storing results of repeated operations in the VLA system.
    """

    def __init__(self, max_size: int = 100, default_ttl: int = 300):  # 5 minutes default TTL
        """
        Initialize the cache.

        Args:
            max_size: Maximum number of items to store
            default_ttl: Default time-to-live for cached items in seconds
        """
        self.max_size = max_size
        self.default_ttl = default_ttl
        self.cache = {}
        self.access_times = {}
        self.lock = threading.RLock()  # Use reentrant lock for thread safety

    def set(self, key: str, value: Any, ttl: Optional[int] = None) -> bool:
        """
        Set a value in the cache with optional TTL.

        Args:
            key: Cache key
            value: Value to store
            ttl: Time-to-live in seconds (uses default if not specified)

        Returns:
            True if successfully set, False otherwise
        """
        with self.lock:
            # Check if we need to evict items
            if len(self.cache) >= self.max_size:
                self._evict_lru()

            # Set the value and access time
            self.cache[key] = value
            self.access_times[key] = time.time() + (ttl or self.default_ttl)
            return True

    def get(self, key: str) -> Optional[Any]:
        """
        Get a value from the cache.

        Args:
            key: Cache key

        Returns:
            Value if found and not expired, None otherwise
        """
        with self.lock:
            # Check if key exists
            if key not in self.cache:
                return None

            # Check if expired
            if time.time() > self.access_times[key]:
                # Remove expired item
                del self.cache[key]
                del self.access_times[key]
                return None

            # Update access time (for LRU)
            self.access_times[key] = time.time() + self.default_ttl
            return self.cache[key]

    def delete(self, key: str) -> bool:
        """
        Delete a key from the cache.

        Args:
            key: Cache key to delete

        Returns:
            True if key was deleted, False if key didn't exist
        """
        with self.lock:
            if key in self.cache:
                del self.cache[key]
                del self.access_times[key]
                return True
            return False

    def clear(self):
        """
        Clear all items from the cache.
        """
        with self.lock:
            self.cache.clear()
            self.access_times.clear()

    def _evict_lru(self):
        """
        Evict the least recently used item(s) to make space.
        """
        if not self.access_times:
            return

        # Find the oldest (earliest expiration time) item
        oldest_key = min(self.access_times, key=self.access_times.get)
        del self.cache[oldest_key]
        del self.access_times[oldest_key]

    def size(self) -> int:
        """
        Get the current size of the cache.

        Returns:
            Number of items in the cache
        """
        with self.lock:
            return len(self.cache)

    def keys(self) -> list:
        """
        Get all cache keys.

        Returns:
            List of cache keys
        """
        with self.lock:
            # Remove expired keys first
            current_time = time.time()
            expired_keys = [k for k, exp_time in self.access_times.items() if current_time > exp_time]
            for key in expired_keys:
                del self.cache[key]
                del self.access_times[key]

            return list(self.cache.keys())


class CommandPatternCache:
    """
    Specialized cache for storing repeated command patterns and their processed results.
    """

    def __init__(self, max_size: int = 50, default_ttl: int = 600):  # 10 minutes for command patterns
        """
        Initialize the command pattern cache.

        Args:
            max_size: Maximum number of command patterns to cache
            default_ttl: Default TTL for cached command patterns
        """
        self.cache = VLACache(max_size, default_ttl)
        self.pattern_hits = 0
        self.total_requests = 0

    def get_cached_result(self, command_text: str) -> Optional[Dict[str, Any]]:
        """
        Get a cached result for a command text.

        Args:
            command_text: The command text to look up

        Returns:
            Cached result if found, None otherwise
        """
        self.total_requests += 1
        result = self.cache.get(command_text)

        if result is not None:
            self.pattern_hits += 1

        return result

    def cache_result(self, command_text: str, result: Dict[str, Any], ttl: Optional[int] = None) -> bool:
        """
        Cache a result for a command text.

        Args:
            command_text: The command text to cache for
            result: The result to cache
            ttl: Optional TTL for this specific entry

        Returns:
            True if successfully cached, False otherwise
        """
        return self.cache.set(command_text, result, ttl)

    def get_hit_rate(self) -> float:
        """
        Get the cache hit rate.

        Returns:
            Cache hit rate as a percentage (0.0 to 1.0)
        """
        if self.total_requests == 0:
            return 0.0
        return self.pattern_hits / self.total_requests

    def get_cache_stats(self) -> Dict[str, Any]:
        """
        Get cache statistics.

        Returns:
            Dictionary with cache statistics
        """
        return {
            "size": self.cache.size(),
            "max_size": self.cache.max_size,
            "hit_rate": self.get_hit_rate(),
            "pattern_hits": self.pattern_hits,
            "total_requests": self.total_requests,
            "cached_keys": self.cache.keys()
        }


class WhisperResultCache:
    """
    Specialized cache for storing Whisper speech-to-text results.
    """

    def __init__(self, max_size: int = 100, default_ttl: int = 3600):  # 1 hour for Whisper results
        """
        Initialize the Whisper result cache.

        Args:
            max_size: Maximum number of Whisper results to cache
            default_ttl: Default TTL for cached Whisper results
        """
        self.cache = VLACache(max_size, default_ttl)

    def get_cached_transcription(self, audio_hash: str) -> Optional[Dict[str, Any]]:
        """
        Get a cached transcription result.

        Args:
            audio_hash: Hash of the audio data to look up

        Returns:
            Cached transcription result if found, None otherwise
        """
        return self.cache.get(audio_hash)

    def cache_transcription(self, audio_hash: str, transcription: Dict[str, Any]) -> bool:
        """
        Cache a transcription result.

        Args:
            audio_hash: Hash of the audio data
            transcription: Transcription result to cache

        Returns:
            True if successfully cached, False otherwise
        """
        return self.cache.set(audio_hash, transcription)


# Global cache instances
command_cache = CommandPatternCache()
whisper_cache = WhisperResultCache()


def get_command_cache() -> CommandPatternCache:
    """
    Get the global command pattern cache instance.

    Returns:
        CommandPatternCache instance
    """
    return command_cache


def get_whisper_cache() -> WhisperResultCache:
    """
    Get the global Whisper result cache instance.

    Returns:
        WhisperResultCache instance
    """
    return whisper_cache


# Example usage function
def example_usage():
    """Example of how to use the caching modules."""
    print("VLA Caching example usage:")
    print("-" * 25)

    # Get the command cache
    cmd_cache = get_command_cache()

    # Example: Cache a command result
    command_text = "go to the kitchen"
    result = {
        "intent": "navigate",
        "entities": {"location": "kitchen"},
        "confidence": 0.95
    }

    # Cache the result
    cmd_cache.cache_result(command_text, result)
    print(f"Cached result for command: '{command_text}'")

    # Retrieve the cached result
    cached_result = cmd_cache.get_cached_result(command_text)
    print(f"Retrieved cached result: {cached_result}")

    # Check cache stats
    stats = cmd_cache.get_cache_stats()
    print(f"Cache stats: {stats}")

    # Example: Cache a Whisper result
    whisper_cache_instance = get_whisper_cache()
    audio_hash = "audio_hash_123"
    transcription = {
        "text": "Hello, move to the table",
        "confidence": 0.92,
        "language": "en"
    }

    whisper_cache_instance.cache_transcription(audio_hash, transcription)
    cached_transcription = whisper_cache_instance.get_cached_transcription(audio_hash)
    print(f"Whisper cache result: {cached_transcription}")


if __name__ == "__main__":
    example_usage()