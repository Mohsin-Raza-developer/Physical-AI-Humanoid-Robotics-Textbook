---
id: 0003
title: Agentic Workflow Architecture Update
stage: spec
date: 2025-12-20
surface: agent
model: claude-sonnet-4-5
feature: 007-chatbot-backend
branch: 007-chatbot-backend
user: mohsin
command: /sp.specify (architectural update)
labels: ["architecture", "agents-sdk", "tool-calling", "guardrails", "personalization"]
links:
  spec: specs/007-chatbot-backend/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/007-chatbot-backend/spec.md
tests:
  - Specification architectural requirements integration
---

## Prompt

Update the existing specification (specs/007-chatbot-backend/spec.md) with architectural details:

1. **Agentic Workflow (Tool-Calling)**:
   - Input guardrail agent validates safety/relevance
   - Main chatbot agent (Gemini 2.0) if guardrail passes
   - Tool-calling pattern: agent uses @function_tool named search_knowledge_base (no direct DB access)
   - Tool logic: embed query (Cohere 1536-dim) → search Qdrant → return formatted context

2. **Neon DB Personalization**:
   - Fetch first_name, last_name, software_level from Neon users table before agent invocation
   - Inject into system instruction for main chatbot

3. **Data Structure (Messages Array)**:
   - Conversation context as structured messages array: `[{"role": "user|assistant", "content": "..."}]`

4. **Response Formatting**:
   - Citations as clickable Docusaurus links: `[Chapter Title](/docs/chapter-path)`

## Response snapshot

**Specification Updated**: Architecture and workflow requirements integrated

**Updates Made**:

**Clarifications Section** *(2 new entries)*:
- Q: How should the agentic workflow be structured? → A: Agent-based with input guardrails, main agent (Gemini 2.0), tool-calling pattern (search_knowledge_base function tool)
- Q: When should user personalization data be injected? → A: Fetch from Neon DB before agent invocation and inject into system instruction

**Functional Requirements** *(3 new, 3 updated)*:
- **FR-002** (updated): Provide agent with knowledge search through function tool (not direct DB access)
- **FR-016** (updated): Fetch profile data before invoking any agent
- **FR-017** (updated): Inject profile into agent's system instruction for personalization
- **FR-017a** (new): Pass conversation via messages array with role/content structure
- **FR-020** (new): Agent-based workflow with safety/relevance validation layer before main agent
- **FR-021** (new): Knowledge retrieval via function tool interface (handles embedding, search, formatting internally)

**Key Entities** *(1 new)*:
- **FunctionTool** (new): search_knowledge_base tool with query parameter, internal operations (Cohere embed, Qdrant search, context formatting), returns formatted context string to agent

**Success Criteria** *(1 new)*:
- **SC-010** (new): Agent workflow routes through validation first, then main agent, with function tool retrieving content successfully in 90%+ of test queries

**Constraints** *(3 new architectural preferences)*:
- Two-layer agent architecture: input guardrail → main conversational agent
- Main agent uses function tool interface for knowledge retrieval (no direct DB access)
- User profile injected into system instruction before agent invocation

**Key Architectural Clarifications**:
1. **Tool-Calling Pattern**: Agent doesn't access database directly; calls search_knowledge_base function tool which handles all retrieval logic internally
2. **Workflow Sequence**: Query → Input guardrail validation → Main agent → Function tool (if needed) → Response with citations
3. **Personalization Injection**: Profile data (name, level) fetched from Neon DB and added to system instruction before agent processes request
4. **Message Structure**: Conversation history maintained as array of `{"role": "user|assistant", "content": "..."}` objects

## Outcome

- ✅ Impact: Specified agent-based architecture with clear separation: validation layer, main agent, function tool interface for knowledge retrieval
- 🧪 Tests: Architecture requirements captured in functional requirements and success criteria
- 📁 Files: specs/007-chatbot-backend/spec.md (updated with agent workflow, tool-calling pattern, personalization injection)
- 🔁 Next prompts: Ready for `/sp.plan` to translate architectural requirements into technical design
- 🧠 Reflection: Successfully translated technical architectural preferences into specification while maintaining technology-agnostic focus. Agent-based workflow with tool-calling pattern provides clean separation of concerns (validation, conversation, knowledge retrieval). Personalization via system instruction injection enables tailored responses based on student level.

## Evaluation notes (flywheel)

- Failure modes observed: None - architectural requirements integrated smoothly
- Graders run and results (PASS/FAIL):
  - Architectural clarity: PASS (agent workflow, tool pattern, personalization sequence specified)
  - Technology-agnostic focus: PASS (described what system does, noted implementation preferences in constraints)
  - Requirement completeness: PASS (functional requirements cover workflow, tool interface, data structures)
  - Success criteria measurability: PASS (SC-010 tests workflow routing and tool retrieval success rate)
- Prompt variant (if applicable): Specification update (not new feature creation)
- Next experiment (smallest change to try): Proceed to /sp.plan to design technical architecture implementing agent-based workflow with OpenAI Agents SDK
