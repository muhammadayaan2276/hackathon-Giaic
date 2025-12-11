---
id: 0001
title: Create and Update Project Constitution
stage: constitution
date: 2025-12-08
surface: agent
model: gemini-pro
feature: none
branch: 001-physical-ai-textbook
user: Ayaan Zeeshan
command: /sp.constitution
labels: [constitution, principles, governance]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/constitution.md
  - .specify/templates/plan-template.md
  - .claude/commands/sp.constitution.md
  - .claude/commands/sp.plan.md
tests:
  - none
---

## Prompt

/sp.constitution Project: AI-Driven Book + RAG Chatbot + Physical AI & Humanoid RoboticsPrinciples:- Spec-first creation using Spec-Kit Plus & Claude Code- Technical accuracy (ROS 2, Gazebo, Isaac, VLA)- RAG-only answers; fully reproducible- Clear, concise CS/AI audience writing- Consistent English ↔ Urdu translationBook Standards:- Framework: Docusaurus + GitHub Pages- 4 modules + capstone (Physical AI & Humanoid Robotics)- Min 40,000 chars, structured via specs- Urdu translation toggle- Personalization toggle (beginner/intermediate/advanced)RAG Chatbot:- Stack: FastAPI + Neon Postgres + Qdrant + OpenAI Agents/ChatKit- Must answer book-wide & selected-text queries- Return exact chapter/section citations- Accuracy ≥ 90% on 20-query testBonus:- ≥3 Claude Code Subagents- ≥6 reusable Agent Skills- Signup/signin with better-auth; collect user backgroundRobotics Content:- ROS 2 nodes, topics, services- URDF for humanoids- Gazebo physics simulation- Unity interaction- Isaac Sim + Isaac ROS (VSLAM, Nav2)- VLA pipeline: Whisper → LLM → ROS 2 actions- Capstone: Autonomous humanoid (voice → plan → navigate → identify → manipulate)Success Criteria:- Book builds on GitHub Pages- Chatbot functional (RAG-only, citations)- Personalization + Urdu toggle work- Robotics content verified- Subagents/Skills reusable & testable

## Response snapshot

The project constitution was successfully created as v1.0.0, incorporating all provided principles and standards. Dependent templates (`plan-template.md`) were updated for consistency, and agent command definitions were sanitized. The new constitution is located at `.specify/memory/constitution.md`.

## Outcome

- ✅ Impact: The project constitution was created and ratified, establishing foundational principles for development.
- 🧪 Tests: none
- 📁 Files: 
  - `.specify/memory/constitution.md`: Replaced placeholder template with a fully defined v1.0.0 constitution.
  - `.specify/templates/plan-template.md`: Updated the 'Constitution Check' section to align with new principles.
  - `.claude/commands/sp.constitution.md`: Removed agent-specific name.
  - `.claude/commands/sp.plan.md`: Removed agent-specific parameter.
- 🔁 Next prompts: Run /sp.specify to begin a new feature.
- 🧠 Reflection: The constitution provides a strong foundation for the project. Propagating the changes to the templates ensures consistency. The fallback mechanism for PHR creation worked as expected.

## Evaluation notes (flywheel)

- Failure modes observed: The `create-phr.sh` script was not found, requiring the use of the fallback manual creation method.
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): Ensure the `create-phr` script is available and executable in the environment.
