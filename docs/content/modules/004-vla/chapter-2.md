# Chapter 2: Voice-to-Action (Whisper) - Voice Capture, Transcription, Intent Extraction

## Objectives
- Understand voice processing for humanoid robot control
- Implement Whisper-based voice command transcription
- Extract intent from transcribed commands
- Integrate voice processing into the VLA pipeline
- Validate voice command accuracy and robustness

## Introduction to Voice Processing for Humanoid Robots

Voice interaction provides the most natural and intuitive interface for human-robot communication. For humanoid robots specifically, voice processing enables:
- Hands-free operation for users
- Natural communication aligned with human expectations
- Operation at a distance without requiring physical interfaces
- Multimodal interaction combining voice with gestures and visual cues

In the VLA (Vision-Language-Action) pipeline, voice processing serves as the first stage, converting human speech into text that can be processed by the language understanding system.

## Voice Capture and Preprocessing

### Audio Input Requirements

Humanoid robots need to capture high-quality audio in potentially challenging acoustic environments:

**Hardware Considerations**:
- Directional microphone arrays for noise reduction
- Multiple microphones for spatial audio processing
- Audio preprocessing capabilities for background noise reduction
- Integration with the robot's head for natural voice interaction

**Environmental Challenges**:
- Background noise from mechanical systems
- Room acoustics and reverberation
- Competing voices or sounds
- Distance variations between speaker and robot

### Audio Preprocessing Pipeline

```python
import rclpy
from rclpy.node import Node
from audio_common_msgs.msg import AudioData
import numpy as np
import pyaudio
import webrtcvad
from collections import deque
import threading
import time

class VoiceCaptureNode(Node):
    def __init__(self):
        super().__init__('voice_capture_node')
        
        # Audio configuration
        self.sample_rate = 16000  # Whisper works best at 16kHz
        self.chunk_size = 1024
        self.channels = 1
        self.format = pyaudio.paInt16  # 16-bit samples
        
        # VAD (Voice Activity Detection) for speech detection
        self.vad = webrtcvad.Vad()
        self.vad.set_mode(1)  # Aggressiveness level (0-3)
        
        # Audio buffer for voice activity detection
        self.audio_buffer = deque(maxlen=int(self.sample_rate * 2))  # 2 seconds buffer
        self.is_speaking = False
        self.speech_segments = []
        
        # Publishers for detected speech
        self.speech_pub = self.create_publisher(
            AudioData, 
            '/detected_speech', 
            10
        )
        
        # Publishers for transcription requests
        self.transcription_request_pub = self.create_publisher(
            String,
            '/transcription_request',
            10
        )
        
        # Start audio capture in a separate thread
        self.audio_thread = threading.Thread(target=self.capture_audio)
        self.audio_thread.daemon = True
        self.audio_thread.start()
        
        # Timer for monitoring voice activity
        self.voice_activity_timer = self.create_timer(0.1, self.check_voice_activity)
        
        self.get_logger().info('Voice Capture Node initialized')
    
    def capture_audio(self):
        """Capture audio from microphone in a separate thread"""
        audio = pyaudio.PyAudio()
        
        try:
            stream = audio.open(
                format=self.format,
                channels=self.channels,
                rate=self.sample_rate,
                input=True,
                frames_per_buffer=self.chunk_size
            )
            
            self.get_logger().info('Audio stream started')
            
            while rclpy.ok():
                # Read audio data from the microphone
                audio_data = stream.read(self.chunk_size, exception_on_overflow=False)
                
                # Convert to numpy array for processing
                audio_array = np.frombuffer(audio_data, dtype=np.int16)
                
                # Add to buffer for VAD analysis
                self.audio_buffer.extend(audio_array)
                
                # Check if voice activity is detected
                voice_active = self.is_voice_active(audio_data)
                
                if voice_active and not self.is_speaking:
                    # Start of speech detected
                    self.is_speaking = True
                    self.speech_segments = [audio_data]
                    self.get_logger().info('Voice activity started')
                
                elif not voice_active and self.is_speaking:
                    # End of speech detected
                    self.is_speaking = False
                    self.publish_speech_segment()
                    self.get_logger().info('Voice activity ended')
                
                elif self.is_speaking:
                    # Continue collecting speech
                    self.speech_segments.append(audio_data)
                
                time.sleep(0.01)  # 10ms sleep
                
        except Exception as e:
            self.get_logger().error(f'Audio capture error: {e}')
        finally:
            stream.stop_stream()
            stream.close()
            audio.terminate()
    
    def is_voice_active(self, audio_chunk):
        """Check if voice activity is present in the audio chunk"""
        try:
            # WebRTCVAD expects 10, 20, or 30ms frames at 8kHz, 16kHz, or 32kHz
            # For 16kHz and 1024 samples: 1024/16000 = 64ms (too long)
            # We'll break it down into smaller chunks
            chunk_duration = 10  # 10ms chunks for VAD
            samples_per_chunk = int(self.sample_rate * chunk_duration / 1000)
            
            # Process the audio in 10ms chunks
            bytes_per_sample = 2  # 16-bit = 2 bytes
            total_bytes = len(audio_chunk)
            
            for start in range(0, total_bytes, samples_per_chunk * bytes_per_sample):
                end = min(start + samples_per_chunk * bytes_per_sample, total_bytes)
                chunk = audio_chunk[start:end]
                
                # Pad if necessary
                if len(chunk) < samples_per_chunk * bytes_per_sample:
                    chunk += b'\x00' * ((samples_per_chunk * bytes_per_sample) - len(chunk))
                
                # Check VAD
                if self.vad.is_speech(chunk, self.sample_rate):
                    return True
            
            return False
        except Exception as e:
            self.get_logger().error(f'VAD error: {e}')
            return False
    
    def publish_speech_segment(self):
        """Publish collected speech segment for transcription"""
        if not self.speech_segments:
            return
        
        # Combine all speech chunks
        combined_audio = b''.join(self.speech_segments)
        
        # Publish to speech detection topic
        audio_msg = AudioData()
        audio_msg.data = combined_audio
        self.speech_pub.publish(audio_msg)
        
        # Also publish for transcription request
        request_msg = String()
        request_msg.data = combined_audio.hex()  # Send as hex string for processing
        self.transcription_request_pub.publish(request_msg)
        
        self.speech_segments = []  # Clear the segments
    
    def check_voice_activity(self):
        """Periodic check for voice activity status"""
        # This could be used for UI feedback or other status updates
        pass

def main(args=None):
    rclpy.init(args=args)
    node = VoiceCaptureNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Whisper Integration for Transcription

### Setting up Whisper for Robot Applications

OpenAI's Whisper model provides state-of-the-art automatic speech recognition (ASR) capabilities. For humanoid robot applications, Whisper offers several advantages:

- **Robustness**: Performs well in noisy environments
- **Multi-language support**: Can handle multiple languages and accents
- **Real-time capability**: Can be optimized for near real-time transcription
- **Open-source**: Available for deployment on robot hardware

### Whisper Implementation

```python
import torch
import whisper
from rclpy.node import Node
from std_msgs.msg import String
from audio_common_msgs.msg import AudioData
import io
import wave
import tempfile
import numpy as np
from scipy.io import wavfile

class WhisperTranscriptionNode(Node):
    def __init__(self):
        super().__init__('whisper_transcription_node')
        
        # Load Whisper model (use smaller model for robot deployment)
        self.get_logger().info('Loading Whisper model...')
        self.model = whisper.load_model("base.en")  # Use "base" for lower computational needs
        
        # Subscriptions for audio input
        self.audio_sub = self.create_subscription(
            AudioData,
            '/detected_speech',
            self.audio_callback,
            10
        )
        
        # Publishers for transcription results
        self.transcription_pub = self.create_publisher(
            String,
            '/transcription_result',
            10
        )
        
        # Publishers for VLA pipeline
        self.vla_command_pub = self.create_publisher(
            String,
            '/vla_language_command',
            10
        )
        
        self.get_logger().info('Whisper Transcription Node initialized')
    
    def audio_callback(self, msg):
        """Process incoming audio data with Whisper"""
        try:
            # Convert audio data from bytes back to waveform
            audio_bytes = bytes.fromhex(msg.data) if isinstance(msg.data, str) else msg.data
            
            # Write audio to temporary WAV file for Whisper
            with tempfile.NamedTemporaryFile(suffix='.wav', delete=False) as temp_wav:
                # Create WAV file from raw audio data
                self.save_audio_to_wav(audio_bytes, temp_wav.name)
                
                # Transcribe the audio
                result = self.model.transcribe(temp_wav.name)
                
                # Extract transcription and confidence
                transcription = result["text"].strip()
                confidence = self.estimate_confidence(result)
                
                self.get_logger().info(f'Transcribed: "{transcription}" (confidence: {confidence:.2f})')
                
                # Validate transcription quality
                if self.is_valid_transcription(transcription, confidence):
                    # Publish transcription result
                    transcription_msg = String()
                    transcription_msg.data = transcription
                    self.transcription_pub.publish(transcription_msg)
                    
                    # Also send to VLA pipeline
                    self.vla_command_pub.publish(transcription_msg)
                else:
                    self.get_logger().warn(f'Low quality transcription rejected: "{transcription}"')
        
        except Exception as e:
            self.get_logger().error(f'Whisper transcription error: {e}')
    
    def save_audio_to_wav(self, audio_bytes, filename):
        """Save raw audio bytes to WAV file for Whisper processing"""
        # Convert bytes back to numpy array
        audio_array = np.frombuffer(audio_bytes, dtype=np.int16)
        
        # Write to WAV file using scipy
        wavfile.write(filename, 16000, audio_array)
    
    def estimate_confidence(self, result):
        """Estimate confidence of transcription (simplified approach)"""
        # In a real implementation, this would use more sophisticated methods
        # based on token probabilities or other model outputs
        
        # For now, use a simplified approach based on:
        # - Number of tokens in the result
        # - Average probability if available
        # - Presence of common speech patterns
        
        text = result.get("text", "")
        if not text:
            return 0.0
        
        # Check for common non-speech patterns
        non_speech_patterns = ["you", "the", "and", "a", "to", "i", "it", "for", "is", "that", "on", "with", "as", "be", "at"]
        words = text.lower().split()
        
        # A very basic confidence metric
        # In practice, this should use model's internal probabilities
        valid_word_ratio = sum(1 for word in words if word not in non_speech_patterns) / len(words) if words else 0
        length_factor = min(1.0, len(text) / 100)  # Normalize by expected length
        
        # This is a simplified confidence measure
        # Real implementation should use Whisper's internal probability outputs
        return min(1.0, (valid_word_ratio + length_factor) / 2)
    
    def is_valid_transcription(self, text, confidence):
        """Validate if transcription meets quality thresholds"""
        if not text or len(text.strip()) < 2:
            return False
        
        if confidence < 0.5:  # Set confidence threshold
            return False
        
        # Additional validation could check for:
        # - Profanity filtering
        # - Command relevance
        # - Grammar patterns
        
        return True

def main(args=None):
    rclpy.init(args=args)
    node = WhisperTranscriptionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Intent Extraction from Transcribed Commands

### Natural Language Processing for Intent Recognition

Once speech is transcribed, the system must extract the user's intent. This involves:

- **Command Recognition**: Identifying the primary action requested
- **Entity Extraction**: Identifying objects, locations, or other parameters
- **Context Resolution**: Understanding references based on visual context
- **Ambiguity Resolution**: Clarifying unclear references

### Intent Extraction Implementation

```python
import spacy
import re
from typing import Dict, List, Tuple

class IntentExtractionNode(Node):
    def __init__(self):
        super().__init__('intent_extraction_node')
        
        # Load spaCy model for NLP processing
        try:
            self.nlp = spacy.load("en_core_web_sm")
        except OSError:
            self.get_logger().warn("spaCy model not found. Install with: python -m spacy download en_core_web_sm")
            self.nlp = None
        
        # Subscriptions
        self.transcription_sub = self.create_subscription(
            String,
            '/transcription_result',
            self.transcription_callback,
            10
        )
        
        # Publishers
        self.intent_pub = self.create_publisher(
            String,  # In practice, this would be a structured Intent message
            '/extracted_intent',
            10
        )
        
        self.vla_task_pub = self.create_publisher(
            String,
            '/vla_task_plan',
            10
        )
        
        # Define common commands and their patterns
        self.command_patterns = {
            'navigation': [
                r'go to (.+)',
                r'go to the (.+)',
                r'move to (.+)',
                r'walk to (.+)',
                r'go (.+)',
                r'head to (.+)',
                r'take me to (.+)'
            ],
            'fetch': [
                r'bring me (.+)',
                r'get (.+)',
                r'fetch (.+)',
                r'pick up (.+)',
                r'grab (.+)',
                r'give me (.+)'
            ],
            'manipulation': [
                r'pick (.+)',
                r'put (.+) (?:on|in) (.+)',
                r'move (.+) (?:to|on|in) (.+)',
                r'place (.+) (?:on|in) (.+)',
                r'open (.+)',
                r'close (.+)'
            ],
            'interaction': [
                r'wave to (.+)',
                r'say hello to (.+)',
                r'greet (.+)',
                r'talk to (.+)',
                r'follow (.+)'
            ]
        }
        
        self.get_logger().info('Intent Extraction Node initialized')
    
    def transcription_callback(self, msg):
        """Process transcribed text and extract intent"""
        text = msg.data
        
        self.get_logger().info(f'Processing transcription: "{text}"')
        
        # Extract intent using pattern matching
        intent = self.extract_intent_nlp(text)
        
        if intent:
            # Validate and refine the intent
            validated_intent = self.validate_intent(intent)
            
            if validated_intent:
                # Publish the extracted intent
                intent_msg = String()
                intent_msg.data = json.dumps(validated_intent)
                self.intent_pub.publish(intent_msg)
                
                # Also send to VLA pipeline
                self.vla_task_pub.publish(intent_msg)
                
                self.get_logger().info(f'Intent extracted: {validated_intent}')
            else:
                self.get_logger().warn('Intent validation failed')
        else:
            self.get_logger().warn(f'No intent recognized in: "{text}"')
    
    def extract_intent_nlp(self, text: str) -> Dict:
        """Extract intent using NLP techniques and pattern matching"""
        if not self.nlp:
            # Fallback to simple pattern matching if spaCy is not available
            return self.extract_intent_patterns(text)
        
        # Process text with spaCy
        doc = self.nlp(text.lower())
        
        # Extract tokens and their dependencies
        tokens = [(token.text, token.pos_, token.dep_) for token in doc]
        
        # Identify command type using pattern matching on tokens
        command_type = self.identify_command_type(text)
        
        if command_type:
            # Extract entities based on the command type
            entities = self.extract_entities(doc, command_type)
            
            return {
                'command_type': command_type,
                'entities': entities,
                'raw_text': text,
                'confidence': 0.8  # Placeholder confidence
            }
        
        return None
    
    def extract_intent_patterns(self, text: str) -> Dict:
        """Simple pattern matching for intent extraction (fallback)"""
        text_lower = text.lower()
        
        for command_type, patterns in self.command_patterns.items():
            for pattern in patterns:
                match = re.search(pattern, text_lower)
                if match:
                    groups = match.groups()
                    
                    # Handle different numbers of groups for different patterns
                    if len(groups) == 1:
                        entities = {'target': groups[0]}
                    elif len(groups) == 2:
                        entities = {'target': groups[0], 'destination': groups[1]}
                    else:
                        entities = {f'arg{i}': arg for i, arg in enumerate(groups)}
                    
                    return {
                        'command_type': command_type,
                        'entities': entities,
                        'raw_text': text,
                        'confidence': 0.7  # Lower confidence for pattern matching
                    }
        
        return None
    
    def identify_command_type(self, text: str) -> str:
        """Identify the type of command using pattern matching"""
        return self.extract_intent_patterns(text)['command_type'] if self.extract_intent_patterns(text) else None
    
    def extract_entities(self, doc, command_type: str) -> Dict:
        """Extract named entities and objects based on command type"""
        entities = {}
        
        # Extract noun phrases and named entities
        for ent in doc.ents:
            entities[ent.label_.lower()] = ent.text
        
        # Extract specific objects based on command type
        if command_type == 'fetch':
            # Look for direct objects (dobj) in the sentence
            for token in doc:
                if token.dep_ == 'dobj':  # Direct object
                    # Get the noun phrase for the object
                    entities['object'] = self.get_noun_phrase(token)
        
        elif command_type == 'navigation':
            # Look for locations (pobj - object of preposition)
            for token in doc:
                if token.dep_ == 'pobj':
                    # Check if this is a location
                    if token.pos_ == 'NOUN' or token.pos_ == 'PROPN':
                        entities['location'] = token.text
            
            # Also look for adverbial phrases that might indicate location
            for token in doc:
                if token.dep_ == 'advmod' and token.tag_ in ['NN', 'NNS']:
                    entities['destination'] = token.text
        
        elif command_type == 'manipulation':
            # Extract direct objects and prepositional objects
            for token in doc:
                if token.dep_ == 'dobj':
                    entities['object'] = self.get_noun_phrase(token)
                elif token.dep_ == 'pobj':
                    entities['target'] = self.get_noun_phrase(token)
        
        return entities
    
    def get_noun_phrase(self, token) -> str:
        """Get the complete noun phrase starting from the token"""
        # In a real implementation, this would collect the entire noun phrase
        # including adjectives, determiners, etc.
        return token.text
    
    def validate_intent(self, intent: Dict) -> Dict:
        """Validate and refine extracted intent"""
        if not intent:
            return None
        
        # Validate confidence threshold
        if intent.get('confidence', 0) < 0.5:
            return None
        
        # Validate entity completeness based on command type
        required_entities = self.get_required_entities(intent['command_type'])
        missing_entities = []
        
        for entity_type in required_entities:
            if entity_type not in intent['entities']:
                missing_entities.append(entity_type)
        
        if missing_entities and intent['command_type'] != 'interaction':
            # For critical commands, require all entities
            self.get_logger().warn(f'Missing required entities: {missing_entities}')
            return None
        
        # Refine entities using knowledge base
        intent['entities'] = self.refine_entities(intent['entities'])
        
        return intent
    
    def get_required_entities(self, command_type: str) -> List[str]:
        """Get required entities for different command types"""
        required = {
            'navigation': ['location'],
            'fetch': ['object'],
            'manipulation': ['object'],
            'interaction': []  # Interaction might not always require objects
        }
        
        return required.get(command_type, [])
    
    def refine_entities(self, entities: Dict) -> Dict:
        """Refine entities using knowledge base and context"""
        # In a real implementation, this would use:
        # - Visual context to resolve ambiguous references
        # - Knowledge base to resolve common object names
        # - Spatial context to resolve location references
        
        # Example: normalize object names
        if 'object' in entities:
            entities['object'] = self.normalize_object_name(entities['object'])
        
        return entities
    
    def normalize_object_name(self, name: str) -> str:
        """Normalize object names to standard forms"""
        # Map common variations to standard names
        name_map = {
            'coffee cup': 'coffee_mug',
            'water cup': 'glass',
            'water glass': 'glass',
            'book': 'book',
            'the book': 'book',
            'it': 'object'  # Context-dependent, would need visual resolution
        }
        
        return name_map.get(name, name)

def main(args=None):
    rclpy.init(args=args)
    node = IntentExtractionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Integration with the VLA Pipeline

### Voice-to-Action Pipeline

The complete voice-to-action pipeline integrates all components:

```python
class VoiceToActionNode(Node):
    def __init__(self):
        super().__init__('voice_to_action_node')
        
        # Initialize all pipeline components
        self.voice_capture = VoiceCaptureNode()
        self.whisper_transcription = WhisperTranscriptionNode()
        self.intent_extraction = IntentExtractionNode()
        
        # Subscriptions
        self.intent_sub = self.create_subscription(
            String,
            '/extracted_intent',
            self.intent_callback,
            10
        )
        
        # Publishers for downstream VLA components
        self.task_plan_pub = self.create_publisher(
            String,
            '/vla_task_plan',
            10
        )
        
        # Publishers for user feedback
        self.feedback_pub = self.create_publisher(
            String,
            '/voice_feedback',
            10
        )
        
        self.get_logger().info('Voice-to-Action Pipeline initialized')
    
    def intent_callback(self, msg):
        """Process extracted intent and generate task plan"""
        try:
            intent_data = json.loads(msg.data)
            
            # Generate task plan based on intent
            task_plan = self.generate_task_plan(intent_data)
            
            if task_plan:
                # Publish task plan to VLA pipeline
                plan_msg = String()
                plan_msg.data = json.dumps(task_plan)
                self.task_plan_pub.publish(plan_msg)
                
                # Provide feedback to user
                feedback_msg = String()
                feedback_msg.data = f"Executing: {intent_data.get('command_type', 'unknown')} command"
                self.feedback_pub.publish(feedback_msg)
                
                self.get_logger().info(f'Task plan generated: {task_plan}')
            else:
                self.get_logger().warn('Failed to generate task plan from intent')
                
        except Exception as e:
            self.get_logger().error(f'Error processing intent: {e}')
    
    def generate_task_plan(self, intent_data):
        """Generate executable task plan from extracted intent"""
        command_type = intent_data.get('command_type')
        entities = intent_data.get('entities', {})
        
        if command_type == 'navigation':
            return self.generate_navigation_plan(entities)
        elif command_type == 'fetch':
            return self.generate_fetch_plan(entities)
        elif command_type == 'manipulation':
            return self.generate_manipulation_plan(entities)
        elif command_type == 'interaction':
            return self.generate_interaction_plan(entities)
        else:
            return None
    
    def generate_navigation_plan(self, entities):
        """Generate navigation task plan"""
        location = entities.get('location') or entities.get('destination')
        
        if not location:
            return None
        
        return {
            'action': 'navigate',
            'target_location': location,
            'parameters': {
                'speed': 'normal',
                'avoidance_mode': 'cautious'
            }
        }
    
    def generate_fetch_plan(self, entities):
        """Generate fetch task plan"""
        object_name = entities.get('object')
        
        if not object_name:
            return None
        
        return {
            'action': 'fetch',
            'target_object': object_name,
            'parameters': {
                'approach_method': 'frontal',
                'grip_type': 'precision_pinch',
                'delivery_method': 'hand_off'
            }
        }
    
    def generate_manipulation_plan(self, entities):
        """Generate manipulation task plan"""
        object_name = entities.get('object') or entities.get('target')
        target_location = entities.get('destination') or entities.get('target')
        
        if not object_name or not target_location:
            return None
        
        return {
            'action': 'manipulation',
            'target_object': object_name,
            'target_location': target_location,
            'parameters': {
                'motion': 'place',
                'orientation': 'upright'
            }
        }
    
    def generate_interaction_plan(self, entities):
        """Generate interaction task plan"""
        target_entity = entities.get('target')
        
        if target_entity:
            if 'wave' in entities.get('raw_text', '').lower():
                action = 'wave'
            elif any(word in ['hello', 'hi', 'greet'] for word in entities.get('raw_text', '').lower().split()):
                action = 'greet'
            else:
                action = 'acknowledge'
        else:
            action = 'acknowledge'
        
        return {
            'action': action,
            'target': target_entity,
            'parameters': {
                'style': 'polite',
                'distance': 'arm_length'
            }
        }

def main(args=None):
    rclpy.init(args=args)
    node = VoiceToActionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Voice Command Validation and Error Handling

### Accuracy Validation

Voice-to-action systems must validate command accuracy to prevent incorrect robot behavior:

```python
class VoiceValidationNode(Node):
    def __init__(self):
        super().__init__('voice_validation_node')
        
        # Subscriptions
        self.intent_sub = self.create_subscription(
            String,
            '/extracted_intent',
            self.intent_callback,
            10
        )
        
        self.transcription_sub = self.create_subscription(
            String,
            '/transcription_result',
            self.transcription_callback,
            10
        )
        
        # Publishers
        self.validated_intent_pub = self.create_publisher(
            String,
            '/validated_intent',
            10
        )
        
        self.correction_request_pub = self.create_publisher(
            String,
            '/correction_request',
            10
        )
        
        # Internal state
        self.transcription_history = []
        self.validation_threshold = 0.7
        self.confidence_history = deque(maxlen=5)
        
    def intent_callback(self, msg):
        """Validate intent before passing it to the VLA pipeline"""
        try:
            intent_data = json.loads(msg.data)
            
            # Perform validation checks
            is_valid = self.validate_intent_completeness(intent_data)
            confidence = intent_data.get('confidence', 0)
            
            # Add confidence to history for trend analysis
            self.confidence_history.append(confidence)
            
            # Check for consistent low confidence
            avg_confidence = sum(self.confidence_history) / len(self.confidence_history) if self.confidence_history else 0
            
            if is_valid and confidence >= self.validation_threshold and avg_confidence > 0.6:
                # Publish validated intent
                self.validated_intent_pub.publish(msg)
                self.get_logger().info(f'Intent validated with confidence {confidence:.2f}')
            else:
                # Request clarification or reissuing
                self.request_clarification(intent_data, confidence < self.validation_threshold)
                
        except Exception as e:
            self.get_logger().error(f'Intent validation error: {e}')
    
    def validate_intent_completeness(self, intent_data):
        """Validate that the intent has sufficient information for execution"""
        command_type = intent_data.get('command_type')
        entities = intent_data.get('entities', {})
        
        # Check for required entities based on command type
        required_entities = {
            'navigation': ['location'],
            'fetch': ['object'],
            'manipulation': ['object', 'target'],  # Both needed for manipulation
            'interaction': ['target']  # For greeting/waving
        }
        
        if command_type in required_entities:
            required = required_entities[command_type]
            for req in required:
                if req not in entities:
                    self.get_logger().warn(f'Missing required entity "{req}" for command type "{command_type}"')
                    return False
        
        return True
    
    def request_clarification(self, intent_data, low_confidence):
        """Request clarification from the user"""
        command_type = intent_data.get('command_type')
        
        if low_confidence:
            # Ask user to repeat the command
            request_msg = String()
            request_msg.data = "I didn't catch that clearly. Could you please repeat your command?"
            self.correction_request_pub.publish(request_msg)
        else:
            # Ask for missing information
            missing_info = self.get_missing_info(intent_data)
            if missing_info:
                request_msg = String()
                request_msg.data = f"Could you specify {missing_info}?"
                self.correction_request_pub.publish(request_msg)
    
    def get_missing_info(self, intent_data):
        """Determine what information is missing from the intent"""
        command_type = intent_data.get('command_type')
        entities = intent_data.get('entities', {})
        
        if command_type == 'navigation':
            if 'location' not in entities:
                return "the location where you'd like me to go"
        elif command_type == 'fetch':
            if 'object' not in entities:
                return "which object you'd like me to bring"
        elif command_type == 'manipulation':
            missing = []
            if 'object' not in entities:
                missing.append("which object")
            if 'target' not in entities:
                missing.append("where you'd like to place it")
            return " and ".join(missing) + " for the manipulation task" if missing else None
        
        return None
    
    def transcription_callback(self, msg):
        """Track transcription accuracy over time"""
        self.transcription_history.append({
            'text': msg.data,
            'timestamp': self.get_clock().now().nanoseconds
        })

def main(args=None):
    rclpy.init(args=args)
    node = VoiceValidationNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Hands-on Exercise 2.1: Voice Processing Pipeline

1. Set up the voice capture system with appropriate audio hardware
2. Integrate Whisper for real-time transcription
3. Test the system with various voice commands
4. Evaluate the transcription accuracy under different acoustic conditions
5. Implement basic audio preprocessing for noise reduction

## Hands-on Exercise 2.2: Intent Recognition

1. Create a dataset of voice commands specific to your robot's capabilities
2. Train or configure the intent recognition system for your specific commands
3. Test the system with natural language commands
4. Evaluate the accuracy of intent extraction
5. Implement error handling and clarification requests

## Validation Checklist
- [ ] I understand the audio capture requirements for humanoid robots
- [ ] I can implement Whisper-based transcription for voice commands
- [ ] I can extract intent from transcribed commands using NLP techniques
- [ ] I have integrated voice processing into the VLA pipeline
- [ ] I have implemented validation and error handling for voice commands
- [ ] I have tested the system with various voice commands and acoustic conditions
- [ ] I understand how to improve voice command accuracy and robustness
- [ ] I have evaluated the overall performance of the voice-to-action pipeline

## Summary

This chapter covered the voice processing component of the VLA pipeline, focusing on capturing, transcribing, and understanding voice commands for humanoid robot control. We explored audio capture and preprocessing techniques, Whisper integration for transcription, and NLP-based intent extraction from transcribed commands.

The voice-to-action pipeline transforms human speech into structured commands that can be processed by the VLA system, enabling natural and intuitive interaction with humanoid robots. Proper validation and error handling ensure that voice commands are accurately interpreted and safely executed.

In the next chapter, we'll explore the complete VLA pipeline execution, including cognitive planning, ROS 2 action mapping, and the capstone implementation of the full voice-to-action autonomous task.