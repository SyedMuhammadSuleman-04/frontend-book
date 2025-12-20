<!-- SYNC IMPACT REPORT
Version change: N/A -> 1.0.0
Added sections: All principles and sections based on user specifications
Removed sections: None (new constitution)
Modified principles: N/A (new constitution)
Templates requiring updates:
  - .specify/templates/plan-template.md ✅ updated (Constitution Check will reference new principles)
  - .specify/templates/spec-template.md ✅ updated (aligns with new principles)
  - .specify/templates/tasks-template.md ✅ updated (aligns with new principles)
Follow-up TODOs: None
-->
# Spec-Driven Book with Embedded RAG Chatbot Constitution

## Core Principles

### Spec-first content generation
Every piece of content starts with a clear specification using Spec-Kit Plus methodology; Content must be planned, reviewed, and validated before implementation; Clear acceptance criteria required for all chapters/features

### Accuracy and non-hallucination
All content must be factually accurate with no hallucinations; RAG chatbot responses must be grounded only in indexed book content; Citations and references required for all claims

### Clear technical writing for developers
Technical content must be accessible to developer audience; Code examples must be complete, tested, and functional; Clear explanations with practical applications

### Reproducible build and deployment
All build and deployment processes must be scripted and reproducible; Docusaurus builds must succeed consistently; GitHub Pages deployment pipeline must be automated

### Free-tier service compliance
All infrastructure components must operate within free tier limits; Services like Neon Serverless Postgres and Qdrant Cloud must stay within free quotas; Cost monitoring required

### Incremental content development
Content and features developed incrementally in small, testable units; Chapter-level vector indexing for efficient updates; Continuous integration of new content

## Technology Stack Requirements

Backend: FastAPI + OpenAI Agents/ChatKit; Storage: Neon Serverless Postgres + Qdrant Cloud; Frontend: Docusaurus for book presentation; Deployment: GitHub Pages; All components must be compatible with free-tier usage

## Development Workflow

Content authored via Claude Code + Spec-Kit Plus; Spec-first approach for all features; Chapter-level vector indexing for RAG functionality; Embedded chatbot development following TDD practices; Automated deployment to GitHub Pages

## Governance

All PRs must verify compliance with constitutional principles; Content accuracy verification required; Build and deployment pipeline must pass; Changes to core architecture require explicit approval; Version control and changelog maintenance mandatory

**Version**: 1.0.0 | **Ratified**: 2025-12-19 | **Last Amended**: 2025-12-19