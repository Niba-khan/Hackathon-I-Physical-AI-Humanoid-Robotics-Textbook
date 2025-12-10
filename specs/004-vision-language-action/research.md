# Research: Vision-Language-Action Module

## Decision 1: Whisper Model Choice (tiny/base/small vs performance/accuracy)

**Decision**: Use Whisper "base" model
**Rationale**: 
- Good balance between performance and accuracy for educational purposes
- Reasonable computational requirements for student environments
- Adequate transcription accuracy (>80%) as required by success criteria
- Faster processing compared to larger models while maintaining quality

**Alternatives considered**:
- "tiny" model: Faster but likely insufficient accuracy for educational purposes
- "small" model: Better accuracy but higher computational requirements
- "medium" model: Even better accuracy but might be overkill for educational content

## Decision 2: Planning Method (prompt-based vs structured schema)

**Decision**: Use a hybrid approach combining structured schemas with prompt engineering
**Rationale**:
- Structured schemas provide reliable, predictable mappings for common actions
- Prompt engineering allows for flexibility in handling complex or novel commands
- Educational value in showing both approaches
- Token efficiency through structured schemas for common tasks
- Better handling of edge cases through prompt-based approaches

**Alternatives considered**:
- Pure prompt-based: Flexible but less predictable and potentially less token-efficient
- Pure structured schemas: More predictable but might not handle complex commands well

## Decision 3: ROS 2 Action Mapping Format

**Decision**: Use a standardized JSON-based format that maps natural language to ROS 2 action sequences
**Rationale**:
- Clear, human-readable format for educational purposes
- Easy to validate and debug for students
- Extensible for new action types
- Compatible with ROS 2 action interfaces
- Facilitates understanding of the NL→action transformation

**Alternatives considered**:
- Custom DSL (Domain Specific Language): Might be more concise but harder to learn
- Direct code generation: More complex and harder for beginners to understand

## Decision 4: Capstone Task Scope and Constraints

**Decision**: Implement a "fetch and deliver" task with simple navigation, object identification, and manipulation
**Rationale**:
- Covers all required components: voice → plan → navigate → identify → manipulate
- Complex enough to demonstrate the full pipeline
- Simple enough for beginner-intermediate students to understand and implement
- Clearly defined success criteria that can be validated
- Builds on concepts learned in previous modules

**Alternatives considered**:
- More complex tasks: Might be overwhelming for the target audience
- Simpler tasks: Might not adequately demonstrate the complete pipeline

## Decision 5: Token-Efficient LLM Usage Strategies

**Decision**: Implement a multi-tier approach with structured prompts and caching
**Rationale**:
- Uses structured, concise prompts to minimize token usage
- Implements caching for common query types
- Uses few-shot learning examples to reduce context length
- Aligns with the requirement for token-efficient LLM usage
- Educational value in demonstrating best practices for API usage

**Alternatives considered**:
- Simple direct API calls: Would be less token efficient
- Complex caching systems: Might be overkill for the educational context

## Additional Research Findings

### VLA Implementation Strategy
- Vision-Language-Action systems typically combine perception, language understanding, and motor control
- The pipeline involves converting natural language to robot actions through intermediate representations
- Effective VLA systems require careful integration of perception, planning, and control components

### Voice Processing with Whisper
- Whisper provides multiple models with different performance and accuracy characteristics
- Integration with ROS 2 requires converting voice commands to text first, then to actions
- Error handling is important when transcription accuracy is below 100%

### Cognitive Planning and LLM Integration
- LLMs can be used for high-level task planning and natural language understanding
- Structured approaches to planning can improve reliability and token efficiency
- Safety and validation checks are important when using LLMs for robot control

### Educational Content Structure
- 2-3 chapters as specified
- Each chapter includes hands-on tasks with validation checklists
- Progressive complexity from basic concepts to complete pipeline
- Follows modular MDX structure principle from constitution

### Quality Validation Approaches
- Transcription accuracy should be measured against ground truth
- NL→ROS action mapping should be validated for correctness
- Full pipeline tests should verify end-to-end functionality
- RAG integration will use section-based chunking to maintain context