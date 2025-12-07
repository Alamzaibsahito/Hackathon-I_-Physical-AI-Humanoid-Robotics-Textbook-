import os
import asyncio
from pathlib import Path
from dotenv import load_dotenv
from backend.app.services.openai_service import get_embedding
from backend.app.services.qdrant_service import create_collection_if_not_exists, upsert_vectors

# Load environment variables
load_dotenv()

DOCS_PATH = Path("frontend/my-website/docs")
COLLECTION_NAME = "book_content"
VECTOR_SIZE = 1536  # For text-embedding-ada-002

async def ingest_content():
    print(f"Starting content ingestion into Qdrant collection: {COLLECTION_NAME}")
    await create_collection_if_not_exists(COLLECTION_NAME, VECTOR_SIZE)

    text_chunks = []
    metadatas = []
    current_id = 0

    for md_file in DOCS_PATH.glob("*.md"):
        if md_file.name.endswith((".beginner.md", ".advanced.md")):
            continue # Skip personalized versions for main ingestion

        print(f"Processing file: {md_file.name}")
        content = md_file.read_text(encoding="utf-8")
        # Simple chunking for now, can be improved
        chunks = [chunk.strip() for chunk in content.split("\n\n") if chunk.strip()]

        for chunk in chunks:
            text_chunks.append(chunk)
            metadatas.append({"source": md_file.name, "content": chunk}) # Store original content in payload

    if not text_chunks:
        print("No Markdown files found or content to ingest.")
        return

    print(f"Generating embeddings for {len(text_chunks)} chunks...")
    # Generate embeddings in batches to avoid rate limits (if many chunks)
    # For simplicity, doing one by one for now. For production, batching is recommended.
    vectors = []
    ids = []
    for i, chunk in enumerate(text_chunks):
        try:
            embedding = await get_embedding(chunk)
            vectors.append(embedding)
            ids.append(current_id)
            current_id += 1
        except Exception as e:
            print(f"Failed to get embedding for chunk {i}: {e}")
            # Optionally skip this chunk or retry

    if vectors:
        await upsert_vectors(COLLECTION_NAME, ids, vectors, metadatas)
        print("Content ingestion complete.")
    else:
        print("No vectors generated, ingestion aborted.")

if __name__ == "__main__":
    asyncio.run(ingest_content())
