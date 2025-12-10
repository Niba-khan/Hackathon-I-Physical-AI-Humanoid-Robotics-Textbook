## Sync Impact Report
<!--
Version change: 0.0.0 → 1.0.0
Added sections: Core Principles (6), Standards, Scope, Success Criteria
Removed sections: None
Modified principles: None (new document)
Templates requiring updates: ⚠ pending [.specify/templates/plan-template.md, .specify/templates/spec-template.md, .specify/templates/tasks-template.md]
Follow-up TODOs: None
-->

# Physical AI & Humanoid Robotics — AI-Native Textbook + RAG Chatbot Constitution

## Core Principles

### AI-Native, Spec-First Workflow
AI-native, spec-first workflow with auto-validation. All development begins with comprehensive specifications that are automatically validated against implementation. This ensures every feature is planned, documented, and verified before implementation, reducing technical debt and improving maintainability.

### Beginner-Intermediate Friendly Explanations
Beginner–intermediate friendly robotics/AI explanations. All content must be accessible to learners with varying skill levels, using clear language, progressive complexity, and intuitive analogies. Technical concepts should be explained with sufficient context to help readers understand without requiring extensive prior knowledge.

### Technical Content Verification
All technical content verified against ROS 2, Gazebo, Isaac, VLA docs. Every claim, code snippet, or tutorial must be validated against official documentation and tested implementations from ROS 2, Gazebo, NVIDIA Isaac, and Vision-Language-Action frameworks. This ensures accuracy and prevents the dissemination of incorrect technical information.

### Modular MDX Structure
Modular MDX chapters with consistent structure and terminology. Content must be organized into reusable, self-contained modules using MDX format. Each module follows standardized templates with consistent terminology, layout, and presentation to ensure a cohesive learning experience across all sections.

### Full Transparency in Decision Making
Full transparency: every decision captured in Spec-Kit. All architectural decisions, development choices, and content modifications must be documented using the Spec-Kit framework. This creates an auditable trail of reasoning that helps maintain consistency and enables future contributors to understand historical context.

### Zero Hallucination Safety Policy
Safety: zero hallucinated robotics/physics/kinematics claims. The textbook and RAG chatbot must never generate unverified technical information or make claims about physics, kinematics, or robotics behaviors that are not grounded in accurate models or real-world evidence. Factually correct information is paramount for safety in physical AI applications.

## Standards
All content and implementation must adhere to the following standards:
- MDX with reusable components: diagrams, callouts, code blocks
- RAG chatbot answers strictly from book content/user-selected text
- Infrastructure: Neon Postgres + Qdrant + FastAPI (typed) + OpenAI models
- Deployment: GitHub Pages with CI/CD defined in /sp.deploy
- UI: Tailwind + shadcn/ui only; accessible, dark/light mode
- Diagrams: AI-generated or open-license only

## Scope
The project encompasses five core modules:
1. Module 1 — ROS 2 Nervous System
2. Module 2 — Digital Twin (Gazebo + Unity)
3. Module 3 — AI-Robot Brain (NVIDIA Isaac)
4. Module 4 — Vision-Language-Action (VLA)
5. Capstone — Autonomous Humanoid Robot

Each module must include hands-on tasks and validation checklists to ensure learners can apply theoretical knowledge in practical scenarios.

## Success Criteria
The project achieves success when:
- Docusaurus build/deploy passes without errors
- RAG chatbot produces factual, book-grounded answers
- FastAPI + Neon + Qdrant fully operational and type-safe
- Each module includes hands-on tasks + validation checklists
- Capstone pipeline works: voice → plan → navigate → identify → manipulate
- All Spec-Kit artifacts validated: /sp.specify, /sp.plan, /sp.arch, /sp.schema, /sp.test

## Governance
This constitution serves as the governing document for all project activities. All development practices, code reviews, and content creation must comply with the stated principles. Amendments to this constitution require documentation of rationale, community review, and approval before implementation. This document supersedes all other practices and guidelines within the project.

**Version**: 1.0.0 | **Ratified**: 2025-12-08 | **Last Amended**: 2025-12-08
