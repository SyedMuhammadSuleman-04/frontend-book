"""
Command Parser Service
Implements basic command parsing to extract intent and entities from transcribed text
"""

import re
from typing import Dict, Any, List, Tuple
from ..models.voice_command import VoiceCommand


class CommandParser:
    """
    Service class for parsing natural language commands and extracting intent and entities.
    """

    def __init__(self):
        """
        Initialize the command parser with known intents and entity patterns.
        """
        # Define known intents and their patterns
        self.intent_patterns = {
            "navigate": [
                r"move to (the )?(?P<location>\w+)",
                r"go to (the )?(?P<location>\w+)",
                r"navigate to (the )?(?P<location>\w+)",
                r"walk to (the )?(?P<location>\w+)",
                r"travel to (the )?(?P<location>\w+)",
            ],
            "pick_up": [
                r"pick up (the )?(?P<object>\w+)",
                r"take (the )?(?P<object>\w+)",
                r"grab (the )?(?P<object>\w+)",
                r"get (the )?(?P<object>\w+)",
                r"collect (the )?(?P<object>\w+)",
            ],
            "place": [
                r"place (the )?(?P<object>\w+) in (the )?(?P<location>\w+)",
                r"put (the )?(?P<object>\w+) in (the )?(?P<location>\w+)",
                r"place (the )?(?P<object>\w+) on (the )?(?P<location>\w+)",
                r"put (the )?(?P<object>\w+) on (the )?(?P<location>\w+)",
            ],
            "detect": [
                r"find (the )?(?P<object>\w+)",
                r"detect (the )?(?P<object>\w+)",
                r"locate (the )?(?P<object>\w+)",
                r"look for (the )?(?P<object>\w+)",
            ],
            "move": [
                r"move (forward|backward|left|right)",
                r"go (forward|backward|left|right)",
            ]
        }

        # Define common entity types and patterns
        self.entity_patterns = {
            "object": [
                r"(red|blue|green|yellow|large|small|big|tiny) (\w+)",
                r"(cup|block|box|ball|toy|book|pen|bottle|glass|plate)",
            ],
            "location": [
                r"(kitchen|living room|bedroom|bathroom|office|dining room|hallway|garage|garden|patio)",
                r"(table|chair|couch|bed|desk|shelf|counter|cupboard)",
            ],
            "direction": [
                r"(forward|backward|left|right|up|down|north|south|east|west)",
            ]
        }

    def parse_command(self, voice_command: VoiceCommand) -> Tuple[str, Dict[str, Any]]:
        """
        Parse a voice command to extract intent and entities.

        Args:
            voice_command: VoiceCommand object with transcribed text

        Returns:
            Tuple of (intent, entities dictionary)
        """
        text = voice_command.transcribed_text.lower().strip()

        # Extract intent
        intent = self._extract_intent(text)

        # Extract entities
        entities = self._extract_entities(text)

        return intent, entities

    def _extract_intent(self, text: str) -> str:
        """
        Extract intent from text using pattern matching.

        Args:
            text: Text to extract intent from

        Returns:
            Detected intent or "unknown" if no intent is detected
        """
        for intent, patterns in self.intent_patterns.items():
            for pattern in patterns:
                match = re.search(pattern, text)
                if match:
                    return intent

        return "unknown"

    def _extract_entities(self, text: str) -> Dict[str, Any]:
        """
        Extract entities from text using pattern matching.

        Args:
            text: Text to extract entities from

        Returns:
            Dictionary of entity types and their values
        """
        entities = {}

        # Use named groups from intent patterns first
        for intent, patterns in self.intent_patterns.items():
            for pattern in patterns:
                match = re.search(pattern, text)
                if match:
                    # Add named groups as entities
                    for name, value in match.groupdict().items():
                        if value:
                            entities[name] = value

        # Then use general entity patterns
        for entity_type, patterns in self.entity_patterns.items():
            if entity_type not in entities:  # Don't override named groups
                for pattern in patterns:
                    match = re.search(pattern, text)
                    if match:
                        # If pattern has named groups, use them
                        groups = match.groupdict()
                        if groups:
                            for name, value in groups.items():
                                if value:
                                    entities[name] = value
                        else:
                            # Otherwise, use the first capture group
                            if match.groups():
                                entities[entity_type] = match.group(1) if match.group(1) else match.group(0)

        return entities

    def update_voice_command(self, voice_command: VoiceCommand) -> VoiceCommand:
        """
        Update a VoiceCommand object with parsed intent and entities.

        Args:
            voice_command: VoiceCommand object to update

        Returns:
            Updated VoiceCommand object
        """
        intent, entities = self.parse_command(voice_command)

        voice_command.parsed_intent = intent
        voice_command.entities.update(entities)

        return voice_command

    def is_command_valid(self, voice_command: VoiceCommand) -> bool:
        """
        Check if the parsed command is valid (has intent and required entities).

        Args:
            voice_command: VoiceCommand object to validate

        Returns:
            True if command is valid, False otherwise
        """
        intent, entities = self.parse_command(voice_command)

        # For now, just check if we have a recognized intent
        # In a more sophisticated system, we'd check if required entities are present
        return intent != "unknown"


# Example usage function
def example_usage():
    """Example of how to use the CommandParser."""
    parser = CommandParser()

    # Example 1: Navigate command
    from ..models.voice_command import VoiceCommand
    from datetime import datetime

    voice_cmd1 = VoiceCommand(
        id="test1",
        raw_audio=b"",  # Empty for example
        transcribed_text="Go to the kitchen",
        parsed_intent="",
        entities={},
        timestamp=datetime.now(),
        confidence=0.9
    )

    intent, entities = parser.parse_command(voice_cmd1)
    print(f"Command: '{voice_cmd1.transcribed_text}'")
    print(f"Intent: {intent}")
    print(f"Entities: {entities}")
    print()

    # Example 2: Pick up command
    voice_cmd2 = VoiceCommand(
        id="test2",
        raw_audio=b"",  # Empty for example
        transcribed_text="Pick up the red block",
        parsed_intent="",
        entities={},
        timestamp=datetime.now(),
        confidence=0.85
    )

    intent, entities = parser.parse_command(voice_cmd2)
    print(f"Command: '{voice_cmd2.transcribed_text}'")
    print(f"Intent: {intent}")
    print(f"Entities: {entities}")
    print()

    # Example 3: Place command
    voice_cmd3 = VoiceCommand(
        id="test3",
        raw_audio=b"",  # Empty for example
        transcribed_text="Place the cup on the table",
        parsed_intent="",
        entities={},
        timestamp=datetime.now(),
        confidence=0.92
    )

    intent, entities = parser.parse_command(voice_cmd3)
    print(f"Command: '{voice_cmd3.transcribed_text}'")
    print(f"Intent: {intent}")
    print(f"Entities: {entities}")


if __name__ == "__main__":
    example_usage()