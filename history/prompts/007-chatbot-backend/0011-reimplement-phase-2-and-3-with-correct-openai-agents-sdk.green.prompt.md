---
id: 0011
title: Reimplement Phase 2 and 3 with Correct OpenAI Agents SDK
stage: green
date: 2025-12-21
surface: agent
model: claude-sonnet-4-5-20250929
feature: 007-chatbot-backend
branch: 007-chatbot-backend
user: mohsin
command: /sp.implement
labels: ["phase-2", "phase-3", "openai-agents-sdk", "runner", "input-guardrails", "fix"]
links:
  spec: /specs/007-chatbot-backend/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - backend/models.py
  - backend/agent.py
  - backend/main.py
  - specs/007-chatbot-backend/tasks.md
tests:
  - No tests executed (implementation phase)
---

## Prompt

```text
implement phase 2 and phase 3  step by step.
```

## Response snapshot

Successfully re-implemented Phase 2 (addition) and Phase 3 with the **correct OpenAI Agents SDK architecture** using `Runner.run()` and `@input_guardrail` decorator patterns.

### Problem Identified:

Previous implementation had critical mistakes:
1. ❌ Used `agent.run()` instead of `Runner.run()`
2. ❌ Missing `AsyncOpenAI`, `OpenAIChatCompletionsModel`, `RunConfig` for Gemini
3. ❌ Used simple function `validate_query_safety()` instead of `@input_guardrail` decorator
4. ❌ No `SafetyCheckOutput` Pydantic model as guardrail `output_type`
5. ❌ No `GuardrailFunctionOutput` or `InputGuardrailTripwireTriggered` handling

### Completed Tasks (T013-T023):

**Phase 2 Addition:**
- **T013**: Added `SafetyCheckOutput` Pydantic model to models.py (is_safe, is_relevant, reason) as output_type for guardrail agent

**Phase 3 Reimplementation:**

**T014**: tools.py already correct with `@function_tool` decorator ✅

**T015**: Configured Gemini client in agent.py:
- `AsyncOpenAI` client with Gemini base_url
- `OpenAIChatCompletionsModel` with gemini-2.0-flash
- `RunConfig` with model and tracing_disabled=True

**T016**: Implemented guardrail agent:
- name="Safety Guardrail"
- instructions for safety/relevance validation
- output_type=SafetyCheckOutput (structured output)

**T017**: Implemented `@input_guardrail` decorator:
- `safety_guardrail` async function
- Calls `await Runner.run(guardrail_agent, input, context=ctx.context)`
- Returns `GuardrailFunctionOutput` with tripwire_triggered logic
- Triggers if NOT (is_safe AND is_relevant)

**T018**: Implemented main chatbot agent:
- name="Robotics Tutor"
- Full instructions for RAG responses with citations
- tools=[search_knowledge_base]
- input_guardrails=[safety_guardrail]

**T019**: session.py already correct ✅

**T020**: Completely rewrote POST /v1/chat endpoint:
- Uses `await Runner.run(starting_agent, input, run_config)`
- Extracts `result.final_output` from RunResult
- Removed old `validate_query_safety()` call

**T021**: Citation extraction already implemented in agent.py ✅

**T022**: Added InputGuardrailTripwireTriggered exception handling:
- `try/except InputGuardrailTripwireTriggered` block
- Accesses `e.guardrail_result.output_info` for SafetyCheckOutput
- Returns ERR_VAL_004 for unsafe, ERR_VAL_005 for irrelevant
- Includes logging with safety_output details

**T023**: Error handling already present ✅

### Files Modified:

1. **models.py** (+46 lines):
   - Added SafetyCheckOutput Pydantic model
   - Fields: is_safe, is_relevant, reason
   - Complete with examples in json_schema_extra

2. **agent.py** (COMPLETELY REWRITTEN - 196 lines):
   - Proper imports from agents SDK
   - Gemini configuration with AsyncOpenAI
   - Guardrail agent with output_type
   - @input_guardrail decorator implementation
   - Main agent with input_guardrails parameter
   - Citation extraction function

3. **main.py** (endpoint rewritten):
   - Updated imports (Runner, InputGuardrailTripwireTriggered, SafetyCheckOutput)
   - POST /v1/chat uses await Runner.run()
   - Proper exception handling for InputGuardrailTripwireTriggered
   - Extracts final_output from RunResult

### Architecture Now Correct:

```
User Query → POST /v1/chat
    ↓
Session Management (get_or_create_session)
    ↓
await Runner.run(main_agent, input, run_config)
    ↓
    ├─ @input_guardrail triggers automatically
    │  ├─ Runs guardrail_agent (SafetyCheckOutput)
    │  ├─ Returns GuardrailFunctionOutput
    │  └─ Raises InputGuardrailTripwireTriggered if unsafe/irrelevant
    │
    └─ Main Agent runs (if guardrail passes)
       ├─ Calls search_knowledge_base tool
       └─ Generates response with citations
    ↓
Extract result.final_output from RunResult
    ↓
Return ChatResponse
```

### Key Corrections Applied:

| Component | Before (Wrong) | After (Correct) |
|-----------|---------------|-----------------|
| Gemini Config | Direct OpenAI client | AsyncOpenAI + OpenAIChatCompletionsModel + RunConfig |
| Agent Execution | agent.run(messages) | await Runner.run(agent, input, run_config) |
| Guardrail Pattern | Simple function | @input_guardrail decorator + guardrail agent |
| Guardrail Output | Dict {"approved": bool} | SafetyCheckOutput Pydantic model |
| Exception Handling | Manual if/else | try/except InputGuardrailTripwireTriggered |
| Result Extraction | response.messages[-1].content | result.final_output |

## Outcome

- ✅ Impact: Phase 2 and 3 completely reimplemented with correct OpenAI Agents SDK patterns - all 11 tasks (T013-T023) completed, fixing critical architecture mistakes in agent execution, Gemini configuration, and guardrail implementation
- 🧪 Tests: No tests executed (implementation phase)
- 📁 Files: 3 files modified (models.py: +46 lines, agent.py: completely rewritten 196 lines, main.py: endpoint rewritten) + tasks.md updated
- 🔁 Next prompts: "Test the corrected implementation with actual API calls" or "Add unit tests for agent components"
- 🧠 Reflection: Critical fix - previous implementation would have failed completely due to incorrect OpenAI Agents SDK usage. Now uses proper Runner.run() execution, @input_guardrail decorator with guardrail agent, SafetyCheckOutput as output_type, GuardrailFunctionOutput, and InputGuardrailTripwireTriggered exception handling. Implementation now matches official OpenAI Agents SDK documentation.

## Evaluation notes (flywheel)

- Failure modes observed: Previous implementation had fundamental OpenAI Agents SDK architecture errors that would prevent it from running
- Graders run and results (PASS/FAIL): No automated graders run (implementation phase)
- Prompt variant (if applicable): Standard /sp.implement with corrected architecture based on OpenAI Agents SDK documentation
- Next experiment (smallest change to try): Test with real Gemini API to verify Runner.run() execution works correctly with guardrails
