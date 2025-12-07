<!-- Sync Impact Report -->
<!--
Version change: None (initial creation)
Modified principles: None
Added sections: All (initial creation)
Removed sections: None
Templates requiring updates:
- .specify/templates/plan-template.md: ✅ updated (implicit, by following structure)
- .specify/templates/spec-template.md: ✅ updated (implicit, by following structure)
- .specify/templates/tasks-template.md: ✅ updated (implicit, by following structure)
- .specify/templates/commands/*.md: ✅ updated (implicit, by following structure)
-->
<!-- Follow-up TODOs: None -->
<!-- End Sync Impact Report -->
# AI-native textbook project on Physical AI & Humanoid Robotics Constitution

## Core Principles

### I. High-Quality Educational Content
All educational content MUST be of high quality, accurate, and based on the provided course modules (ROS 2, Gazebo, NVIDIA Isaac, VLA). Content MUST be clear, concise, and pedagogically sound.

### II. Code Consistency
All Python and React code MUST adhere to established style guides, linting rules, and best practices. Code MUST be readable, maintainable, and consistent across the codebase.

### III. Full Test Coverage
All new and modified code MUST have comprehensive test coverage. Jest MUST be used for React components and Pytest for Python backend components. Tests MUST be automated and integrated into the CI/CD pipeline.

### IV. User-Friendly UX
The user experience MUST be intuitive, responsive, and visually appealing, leveraging Docusaurus themes. Design decisions MUST prioritize ease of use and clear navigation for learners.

### V. RAG Chatbot Performance
The Retrieval-Augmented Generation (RAG) chatbot MUST demonstrate high performance, with low latency and efficient resource utilization to provide a seamless interactive learning experience.

### VI. Accessibility
The platform MUST be designed and developed to be accessible to all users, adhering to relevant accessibility standards (e.g., WCAG).

### VII. Bonus Feature Integration
Integration of bonus features (authentication, personalization, translation) MUST be carefully planned, implemented, and tested to ensure they enhance the learning experience without compromising core functionality or performance.

### VIII. Executable Specifications
All specifications MUST be executable and directly verifiable, serving as living documentation that can be validated against the implemented system.

### IX. Prompt History Maintenance
A comprehensive and uncensored prompt history MUST be maintained for all AI interactions, serving as an immutable audit trail for learning, debugging, and traceability.

### X. Scalability with FastAPI
The backend architecture MUST be designed for scalability, leveraging FastAPI for efficient API development and deployment to support a growing user base and data volume.

## Additional Constraints

### Technology Stack
The primary backend framework MUST be FastAPI (Python). Frontend development MUST utilize React with Docusaurus for documentation and UI themes. Testing frameworks MUST include Jest for React and Pytest for Python.

### Security
All components MUST be developed with a "security-first" mindset. Authentication, data handling, and external integrations MUST follow best practices for data protection and vulnerability prevention.

## Development Workflow

### Code Review and Merging
All code changes MUST undergo a thorough peer code review process. Changes MUST only be merged into the main branch after approval from at least one other developer and passing all automated tests.

### Issue Tracking
All work MUST be tracked using an issue tracking system. Each feature, bug fix, or enhancement MUST be linked to a specific issue.

### Continuous Integration/Continuous Deployment (CI/CD)
Automated CI/CD pipelines MUST be in place to ensure code quality, run tests, and facilitate efficient deployments.

## Governance

All principles outlined in this constitution are non-negotiable and supersede all other practices. Any amendments to this constitution require a formal documentation process, team approval, and a clear migration plan for any affected systems or processes. All pull requests and code reviews MUST verify compliance with these principles. Complexity must always be justified and aligned with the principles of simplicity and smallest viable change.

**Version**: 1.0.0 | **Ratified**: 2025-12-04 | **Last Amended**: 2025-12-04
