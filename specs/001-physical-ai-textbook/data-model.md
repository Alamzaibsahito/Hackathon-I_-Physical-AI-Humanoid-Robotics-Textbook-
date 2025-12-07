# Data Model: AI-native Textbook RAG System

**Feature Branch**: `001-physical-ai-textbook` | **Date**: 2025-12-04 | **Spec**: [specs/001-physical-ai-textbook/spec.md](specs/001-physical-ai-textbook/spec.md)
**Input**: Feature specification (`spec.md`) and Implementation Plan (`plan.md`)

## Summary

This document outlines the data models for the AI-native textbook's RAG system, user authentication, and chat history, utilizing Neon Serverless Postgres for relational data and Qdrant Cloud Free Tier for vector embeddings.

## Neon Serverless Postgres Schemas

### `users` Table

Stores user authentication and background information, integrated with Better-Auth.

| Column Name            | Data Type          | Constraints                  | Description                                      |
| :--------------------- | :----------------- | :--------------------------- | :----------------------------------------------- |
| `id`                   | `UUID`             | `PRIMARY KEY`                | Unique identifier for the user.                  |
| `username`             | `VARCHAR(255)`     | `UNIQUE`, `NOT NULL`         | User's chosen username.                          |
| `email`                | `VARCHAR(255)`     | `UNIQUE`, `NOT NULL`         | User's email address (for Better-Auth).          |
| `hashed_password`      | `VARCHAR(255)`     | `NOT NULL`                   | Hashed password for authentication.              |
| `software_experience`  | `VARCHAR(50)`      | `NOT NULL`                   | 'beginner', 'intermediate', 'advanced'.          |
| `hardware_knowledge`   | `VARCHAR(50)`      | `NOT NULL`                   | 'none', 'basic', 'advanced'.                     |
| `created_at`           | `TIMESTAMP`        | `DEFAULT NOW()`              | Timestamp of user creation.                      |
| `updated_at`           | `TIMESTAMP`        | `DEFAULT NOW()`              | Last update timestamp.                           |

### `chapters` Table

Stores metadata about each textbook chapter.

| Column Name | Data Type          | Constraints           | Description                                      |
| :---------- | :----------------- | :-------------------- | :----------------------------------------------- |
| `id`        | `UUID`             | `PRIMARY KEY`         | Unique identifier for the chapter.               |
| `title`     | `VARCHAR(255)`     | `NOT NULL`            | Chapter title.                                   |
| `file_path` | `VARCHAR(255)`     | `UNIQUE`, `NOT NULL`  | Path to the MDX file (e.g., `docs/intro.mdx`).   |
| `content_hash` | `VARCHAR(64)`   | `NOT NULL`            | SHA256 hash of the chapter content for versioning. |
| `created_at`| `TIMESTAMP`        | `DEFAULT NOW()`       | Timestamp of chapter record creation.            |
| `updated_at`| `TIMESTAMP`        | `DEFAULT NOW()`       | Last update timestamp.                           |

### `chat_history` Table

Stores user interactions with the RAG chatbot.

| Column Name | Data Type          | Constraints           | Description                                      |
| :---------- | :----------------- | :-------------------- | :----------------------------------------------- |
| `id`        | `UUID`             | `PRIMARY KEY`         | Unique identifier for the chat message.          |
| `user_id`   | `UUID`             | `FOREIGN KEY`         | Reference to the `users` table.                  |
| `query_text`| `TEXT`             | `NOT NULL`            | The user's question.                             |
| `response_text` | `TEXT`           | `NOT NULL`            | The chatbot's answer.                            |
| `context_used`| `TEXT`           |                       | Text snippets from the textbook used for the response. |
| `timestamp`   | `TIMESTAMP`        | `DEFAULT NOW()`       | Timestamp of the chat interaction.               |

## Qdrant Cloud Free Tier Schema

### `textbook_chunks` Collection

Stores vector embeddings of textbook content chunks for RAG.

| Field Name    | Data Type  | Description                                      |
| :------------ | :--------- | :----------------------------------------------- |
| `vector`      | `float[]`  | OpenAI embedding vector of the content chunk.    |
| `payload`     | `JSON`     | Associated metadata for the chunk.               |
| `payload.chapter_id` | `UUID`   | ID of the chapter this chunk belongs to (links to `chapters` table). |
| `payload.chunk_id` | `UUID`    | Unique ID for the content chunk.                 |
| `payload.text`| `TEXT`     | The original text content of the chunk.          |
| `payload.page_number` | `INT`  | Optional: Page number if content is paginated.   |
| `payload.section_title` | `TEXT` | Optional: Section title within the chapter.      |

### Vector Configuration

-   **Vector Size**: Determined by the OpenAI embedding model (e.g., `text-embedding-ada-002` typically 1536 dimensions).
-   **Distance Metric**: Cosine similarity.

## Data Flow

1.  **Content Ingestion**: MDX chapter files are read, chunked. Each chunk's text is sent to OpenAI for embedding. The embedding vector and chunk metadata (chapter_id, chunk_id, text) are stored in the Qdrant `textbook_chunks` collection. Chapter metadata (id, title, file_path, content_hash) is stored in the Neon `chapters` table.
2.  **User Authentication**: User registration and login handled by Better-Auth, storing `id`, `username`, `email`, `hashed_password`, `software_experience`, `hardware_knowledge` in the Neon `users` table.
3.  **RAG Query**: User query is embedded using OpenAI. The vector is used to search Qdrant for relevant `textbook_chunks`. Retrieved chunks' text is combined with the original query and sent to OpenAI Agents/ChatKit for response generation. Chat interactions are logged in the Neon `chat_history` table.
4.  **Personalization/Translation**: Similar to RAG, content chunks are processed by Claude subagents (via OpenAI API) based on user background or language preference.
