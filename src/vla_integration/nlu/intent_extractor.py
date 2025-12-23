"""
Intent Extractor Module
Implements intent extraction for natural language commands
"""

from typing import Dict, Any, Tuple
import re


class IntentExtractor:
    """
    Extracts intent from natural language commands using pattern matching and NLP techniques.
    """

    def __init__(self):
        """
        Initialize the intent extractor with known intents and patterns.
        """
        # Define intent patterns with more sophisticated matching
        self.intent_patterns = {
            "navigate": [
                (r"move to (?:the )?(?P<location>\w+)", ["location"]),
                (r"go to (?:the )?(?P<location>\w+)", ["location"]),
                (r"navigate to (?:the )?(?P<location>\w+)", ["location"]),
                (r"walk to (?:the )?(?P<location>\w+)", ["location"]),
                (r"travel to (?:the )?(?P<location>\w+)", ["location"]),
                (r"go (?:straight )?(forward|backward|left|right)", ["direction"]),
            ],
            "pick_up": [
                (r"pick up (?:the )?(?P<object>\w+(?: \w+)?)", ["object"]),
                (r"take (?:the )?(?P<object>\w+(?: \w+)?)", ["object"]),
                (r"grab (?:the )?(?P<object>\w+(?: \w+)?)", ["object"]),
                (r"get (?:the )?(?P<object>\w+(?: \w+)?)", ["object"]),
                (r"collect (?:the )?(?P<object>\w+(?: \w+)?)", ["object"]),
            ],
            "place": [
                (r"place (?:the )?(?P<object>\w+(?: \w+)?) in (?:the )?(?P<location>\w+)", ["object", "location"]),
                (r"put (?:the )?(?P<object>\w+(?: \w+)?) in (?:the )?(?P<location>\w+)", ["object", "location"]),
                (r"place (?:the )?(?P<object>\w+(?: \w+)?) on (?:the )?(?P<location>\w+)", ["object", "location"]),
                (r"put (?:the )?(?P<object>\w+(?: \w+)?) on (?:the )?(?P<location>\w+)", ["object", "location"]),
            ],
            "detect": [
                (r"find (?:the )?(?P<object>\w+(?: \w+)?)", ["object"]),
                (r"detect (?:the )?(?P<object>\w+(?: \w+)?)", ["object"]),
                (r"locate (?:the )?(?P<object>\w+(?: \w+)?)", ["object"]),
                (r"look for (?:the )?(?P<object>\w+(?: \w+)?)", ["object"]),
            ],
            "move": [
                (r"move (forward|backward|left|right)", ["direction"]),
                (r"go (forward|backward|left|right)", ["direction"]),
                (r"step (forward|backward|left|right)", ["direction"]),
            ],
            "stop": [
                (r"stop", []),
                (r"halt", []),
                (r"freeze", []),
            ],
            "follow": [
                (r"follow (?:me|the )?(?P<target>\w+)", ["target"]),
                (r"come with (?:me|the )?(?P<target>\w+)", ["target"]),
            ],
            "wait": [
                (r"wait", []),
                (r"pause", []),
                (r"hold on", []),
            ]
        }

    def extract_intent(self, text: str) -> Tuple[str, Dict[str, Any], float]:
        """
        Extract intent and entities from text with confidence scoring.

        Args:
            text: Input text to analyze

        Returns:
            Tuple of (intent, entities dictionary, confidence score)
        """
        text_lower = text.lower().strip()
        best_match = ("unknown", {}, 0.0)

        for intent, patterns in self.intent_patterns.items():
            for pattern, required_entities in patterns:
                match = re.search(pattern, text_lower)
                if match:
                    entities = match.groupdict()

                    # Calculate confidence based on match quality
                    confidence = self._calculate_confidence(text_lower, match, required_entities)

                    # Update best match if this one has higher confidence
                    if confidence > best_match[2]:
                        best_match = (intent, entities, confidence)

        return best_match

    def _calculate_confidence(self, text: str, match, required_entities: list) -> float:
        """
        Calculate confidence score for the intent match.

        Args:
            text: Original text
            match: Regex match object
            required_entities: List of required entity names

        Returns:
            Confidence score between 0.0 and 1.0
        """
        confidence = 0.5  # Base confidence

        # Increase confidence if required entities are found
        entities = match.groupdict()
        found_required = sum(1 for entity in required_entities if entity in entities and entities[entity])
        total_required = len(required_entities)

        if total_required > 0:
            confidence += 0.3 * (found_required / total_required)

        # Increase confidence for longer, more specific matches
        matched_text = match.group(0)
        if len(matched_text) > len(text) * 0.5:  # Match covers more than half of text
            confidence += 0.2

        return min(1.0, confidence)

    def get_supported_intents(self) -> list:
        """
        Get list of supported intents.

        Returns:
            List of supported intent names
        """
        return list(self.intent_patterns.keys())


# Example usage function
def example_usage():
    """Example of how to use the IntentExtractor."""
    extractor = IntentExtractor()

    # Test various commands
    test_commands = [
        "Go to the kitchen",
        "Pick up the red block",
        "Place the cup on the table",
        "Find the blue ball",
        "Move forward",
        "Stop immediately",
        "Wait here"
    ]

    print("Intent extraction examples:")
    print("-" * 40)

    for command in test_commands:
        intent, entities, confidence = extractor.extract_intent(command)
        print(f"Command: '{command}'")
        print(f"  Intent: {intent}")
        print(f"  Entities: {entities}")
        print(f"  Confidence: {confidence:.2f}")
        print()


if __name__ == "__main__":
    example_usage()