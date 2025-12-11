# Implementation Plan: AI/Spec-Driven Book Creation Project

**Project**: AI/Spec-Driven Book Creation using Docusaurus
**Date**: 2025-12-10

## 1. Summary

This document outlines the implementation plan for creating a technical book using an AI-assisted, specification-driven workflow. The project will use Docusaurus for the final output and will be governed by the principles laid out in the project constitution. The goal is to produce a high-quality, technically accurate book on Robotics and AI, starting with a module on ROS 2.

## 2. Technical Context & Architecture

### 2.1. Architecture Sketch

The project will follow a structured folder system to separate concerns and facilitate the AI-driven workflow.

```text
.
├── My-Book/                # The Docusaurus project root
│   ├── docs/               # Docusaurus documentation source
│   │   ├── module1-ros2/
│   │   │   ├── _category_.json
│   │   │   ├── chapter1-ros2-fundamentals.md
│   │   │   └── ...
│   │   └── ...
│   ├── docusaurus.config.ts
│   └── ...
├── specs/                  # Feature and module specifications
│   ├── 001-book-module1-ros2/
│   │   ├── spec.md         # Functional/non-functional requirements
│   │   ├── plan.md         # Implementation plan (this file)
│   │   └── research.md     # Research findings
│   └── overall-project/
│       ├── plan.md
│       └── research.md
├── history/prompts/        # Verbatim records of prompts and AI responses
│   ├── 001-book-module1-ros2/
│   │   └── ...
│   └── overall-project/
│       └── ...
└── scripts/                # Automation scripts for generation and validation
    └── ...
```

**AI File Writing Process:**

1.  **Specification (`spec.md`):** A human author defines the requirements for a module or chapter in a `spec.md` file. This includes topics, key concepts, acceptance criteria, and constraints (e.g., word count).
2.  **AI Prompting:** The Gemini CLI agent is prompted to generate content based on the `spec.md`. The prompt will reference the spec and ask the AI to write a specific chapter or section.
3.  **Content Generation:** The AI writes the content directly into the appropriate file within the `My-Book/docs/` directory. For example, `My-Book/docs/module1-ros2/chapter1-ros2-fundamentals.md`. There will be no intermediate `AI_OUTPUT` directory to streamline the process.
4.  **Review & Refinement:** A human author reviews the AI-generated content for technical accuracy, clarity, and adherence to the spec. Edits are made directly to the file.

### 2.2. Section Structure

The book is organized into modules and chapters, which directly maps to the Docusaurus file structure.

*   **Modules:** A top-level directory within `My-Book/docs/` representing a major topic area (e.g., `module1-ros2`). Each module will contain a `_category_.json` file to define its appearance in the sidebar.
*   **Chapters:** Markdown files within a module directory (e.g., `chapter1-ros2-fundamentals.md`). The content will flow logically from one chapter to the next.

## 3. Constitution Check

This plan adheres to the core principles of the project constitution:

*   **I. Spec-First Creation:** The entire writing process is initiated from a `spec.md` file for each module.
*   **II. Technical Accuracy:** The Quality Validation and Testing Strategy sections explicitly require verification of all technical content.
*   **III. RAG-Only Answers:** While this plan focuses on book creation, the generated content will be the source for the RAG chatbot, fulfilling this principle by extension.
*   **IV. Clear & Accessible Writing:** The Quality Validation section mandates clarity and targeting for a CS/AI audience.
*   **V. Bilingual Consistency:** The plan acknowledges the Docusaurus structure which supports i18n features for future translation work.

## 4. Gates

*   **Gate 1: Specification Approval:** A `spec.md` for a module must be reviewed and approved before content generation can begin.
*   **Gate 2: Content Review:** All AI-generated content must be reviewed and approved by a human author before being considered "done".

There are no violations of the constitution in this plan.

## 5. Phase 0: Research & Clarification

This project will employ a **research-concurrent** approach. Research tasks will be identified and executed as needed during the specification and writing phases.

**Initial Research Tasks:**

1.  **Docusaurus Best Practices for Technical Books:** Investigate optimal configuration for code blocks, admonitions, and diagrams (Mermaid.js).
2.  **ROS 2 Key Concepts for Beginners:** Consolidate a definitive list of fundamental topics for `module1-ros2`.
3.  **AI Prompting Strategies for Technical Writing:** Develop best practices for prompting Gemini to produce accurate, well-structured technical content.

Findings will be documented in the `specs/overall-project/research.md` and module-specific `research.md` files.

## 6. Phase 1: Design & Contracts

This phase focuses on the data model and contracts for the content itself.

*   **Data Model (`data-model.md`):** For this project, the "data model" refers to the content structure.
    *   **Entity:** Chapter
    *   **Fields:** Title, Introduction, Sections (with headings), Code Examples, Diagrams (Mermaid.js), Conclusion, Word Count.
*   **API Contracts (`/contracts/`):** Not applicable for the book-writing process itself, but will be relevant for the RAG chatbot and other systems described in the constitution.
*   **Agent Context Update:** The `GEMINI.md` file will be updated with technologies as they are introduced, starting with Docusaurus, Markdown, and Python.

## 7. Key Decisions

| Decision Area                 | Options & Tradeoffs                                                                                                                              | Chosen Option & Rationale                                                                                                                              |
| ----------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------ | ------------------------------------------------------------------------------------------------------------------------------------------------------ |
| **Chapter Count per Module**  | - **Low (3-5):** Faster to produce, less depth. <br> - **High (8-10):** More comprehensive, longer development time.                                 | **3-5 chapters per module.** This allows for focused, high-quality content to be produced iteratively, in line with an agile approach.               |
| **Code & Image Placement**    | - **Central `assets` folder:** Easy to manage, but can be disconnected from content. <br> - **Co-located with Markdown:** Easier for authors to link. | **Co-located with Markdown.** For images specific to a chapter, they will be placed in an `img` subfolder within the module's directory. This is more maintainable in Docusaurus. |
| **Naming Conventions**        | - **camelCase, snake_case, kebab-case**                                                                                                          | **kebab-case** for files and folders (e.g., `module1-ros2`, `chapter1-ros2-fundamentals.md`). This is a common web convention and improves readability in URLs. |

## 8. Testing Strategy

The testing strategy focuses on ensuring the quality and correctness of the final book content.

1.  **File Validation:** Automated checks to ensure all files have the correct naming convention and exist in the expected directory.
2.  **Content Validation (Manual):** Human review of each chapter against its `spec.md` to verify:
    *   **Technical Correctness:** Code examples run without errors; concepts are accurately explained.
    *   **Clarity and Tone:** Content is understandable for the target audience.
    *   **Completeness:** All topics from the spec are covered.
3.  **Formatting Checks:**
    *   **Linting:** Use a Markdown linter to enforce consistent style.
    *   **Build Check:** Regularly run `npm run build` in the `My-Book` directory to ensure Docusaurus can build the site without errors.
4.  **Acceptance Criteria Checks:** A checklist will be created from the `spec.md`'s acceptance criteria and used to formally sign off on each chapter.

## 9. Phases of Work

The project will follow these high-level phases for each module:

1.  **Phase 1: Foundation (Setup):** Set up the Docusaurus project and define the folder structure. (This is largely complete).
2.  **Phase 2: Research & Specification:** Create the `spec.md` for a given module, including research into the topics to be covered.
3.  **Phase 3: AI-Assisted Writing:** Use the Gemini CLI agent to generate the chapter content based on the approved spec.
4.  **Phase 4: Review & Refinement:** Human authors review, edit, and approve the generated content.
5.  **Phase 5: Integration:** Ensure the new module is correctly integrated into the Docusaurus site, with proper navigation and links.

