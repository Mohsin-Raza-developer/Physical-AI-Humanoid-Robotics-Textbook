---
id: 0001
title: ChatKit Gemini Backend Specification
stage: spec
date: 2025-12-23
surface: agent
model: claude-sonnet-4-5-20250929
feature: 008-chatkit-gemini-backend
branch: 008-chatkit-gemini-backend
user: mohsin
command: /sp.specify
labels: ["chatkit", "gemini", "fastapi", "openai-agents-sdk", "specification"]
links:
  spec: specs/008-chatkit-gemini-backend/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/008-chatkit-gemini-backend/spec.md
  - specs/008-chatkit-gemini-backend/checklists/requirements.md
tests:
  - Specification quality validation checklist (all items passed)
---

## Prompt

Act as a Lead AI Engineer using the Spec-Kit Plus (SDD-RI) framework. We are starting the Specify Phase for a self-hosted Python backend for OpenAI ChatKit.

Objective: "Build a production-ready FastAPI backend for OpenAI ChatKit that uses Google Gemini as the primary LLM for inference while maintaining compatibility with ChatKit's features."

/sp.specify

Write a complete, testable specification following SMART criteria. The specification must include:

1. Project Overview: Building a production-ready FastAPI backend that integrates the ChatKit Python SDK and OpenAI Agents SDK.

2. Self-Hosted Requirements:
   - Detail our responsibility for storing messages, threads, and attachments.
   - Mechanism for User Authentication via custom fetch method and auth headers.

3. Core Features to Specify:
   - Response Streaming: Built-in real-time streaming for natural conversations.
   - Agentic Actions: Visualizing 'chain-of-thought' and tool integration.
   - Interactive Widgets: Supporting widgets with and without server-handled actions.
   - Progress Updates: Emitting ProgressUpdateEvent (e.g., 'Searching...') during long-running tool calls.
   - Client Effects: Logic to push UI changes directly from the server.

4. Advanced Logic:
   - Implementing a Title Agent to generate thread names after the first message.
   - Handling Attachments (upload/download URLs) and Entity Tagging (@-mentions).

5. Quality Standards: Ensure the spec focuses on capturing the 'Why' behind these architectural choices (for future ADRs).

After writing the spec, please run /sp.clarify to ask me any missing questions about database choices or specific tool constraints.

6. Core References:
   - ChatKit Python SDK: https://openai.github.io/chatkit-python/
   - OpenAI Agents SDK: https://openai.github.io/openai-agents-python/quickstart/
   - Reference Architecture (Patterns): https://github.com/openai/openai-chatkit-advanced-samples

7. Gemini with OpenAI_Agents_SDK: (Example code provided for Gemini + AsyncOpenAI configuration)

## Response snapshot

Created comprehensive specification for ChatKit backend with Google Gemini integration on branch 008-chatkit-gemini-backend.

**Key Deliverables**:

1. **Specification Document** (`specs/008-chatkit-gemini-backend/spec.md`):
   - 8 prioritized user stories (P1-P3) covering: streaming responses, agentic actions, widgets, progress updates, thread titles, attachments, @-mentions, client effects
   - 53 functional requirements organized by category (messaging, auth, streaming, widgets, tools, error handling, logging)
   - 14 measurable, technology-agnostic success criteria
   - 14 edge cases identified
   - Clear scope boundaries with detailed dependencies, constraints, and out-of-scope items

2. **Quality Validation Checklist** (`specs/008-chatkit-gemini-backend/checklists/requirements.md`):
   - All validation items passed
   - 3 clarification questions identified and resolved through user interaction

3. **Clarifications Resolved**:
   - **Q1 - Attachment Retention**: Thread lifetime (files deleted when thread deleted)
   - **Q2 - Invalid @-Mentions**: Ignore and treat as regular text (graceful degradation)
   - **Q3 - Compliance Requirements**: Leverage existing auth system's GDPR compliance

**Architectural Context Captured**:
- Integration with existing Docusaurus frontend + Better Auth authentication system
- Knowledge base tool (`search_knowledge_base`) using Cohere embeddings + Qdrant vector DB
- OpenAI Agents SDK with Gemini 2.0 Flash as LLM (via AsyncOpenAI adapter)
- Self-hosted storage for messages, threads, attachments
- Custom fetch method pattern for injecting auth tokens

**Quality Metrics**:
- Specification is technology-agnostic (focuses on WHAT and WHY, not HOW)
- All requirements testable and unambiguous
- Success criteria measurable with specific metrics
- User scenarios independently testable with clear acceptance criteria

## Outcome

- ✅ Impact: Complete, validated specification ready for planning phase with all clarifications resolved
- 🧪 Tests: Specification quality checklist validation (100% pass rate across content quality, requirement completeness, and feature readiness)
- 📁 Files: 2 files created (spec.md, requirements.md checklist)
- 🔁 Next prompts: Run `/sp.plan` to create technical architecture and design; then `/sp.tasks` for implementation breakdown
- 🧠 Reflection: Interactive clarification process successfully resolved ambiguities while keeping user engaged; specification maintains clear separation between business requirements (spec) and technical implementation (to be addressed in planning)

## Evaluation notes (flywheel)

- Failure modes observed: None - specification process completed successfully with user-driven clarifications
- Graders run and results (PASS/FAIL): Spec quality validation checklist - PASS (all criteria met)
- Prompt variant (if applicable): Standard /sp.specify workflow with interactive clarification questions
- Next experiment (smallest change to try): Proceed to `/sp.plan` phase to translate business requirements into technical architecture
