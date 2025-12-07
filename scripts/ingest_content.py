import os
import uuid
import hashlib
import re
import psycopg2
from qdrant_client import QdrantClient, models
from openai import OpenAI
from dotenv import load_dotenv

load_dotenv()

# Environment variables
OPENAI_API_KEY = os.getenv("OPENAI_API_KEY")
QDRANT_HOST = os.getenv("QDRANT_HOST")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
NEON_DB_URL = os.getenv("NEON_DB_URL")

if not all([OPENAI_API_KEY, QDRANT_HOST, QDRANT_API_KEY, NEON_DB_URL]):
    print("Error: Missing one or more environment variables. Please set OPENAI_API_KEY, QDRANT_HOST, QDRANT_API_KEY, and NEON_DB_URL.")
    exit(1)

# Initialize clients
openai_client = OpenAI(api_key=OPENAI_API_KEY)
qdrant_client = QdrantClient(
    host=QDRANT_HOST,
    api_key=QDRANT_API_KEY,
)

# Constants
COLLECTION_NAME = "textbook_chunks"
EMBEDDING_MODEL = "text-embedding-ada-002"
DOCS_DIR = "frontend/my-website/docs"

def get_db_connection():
    """Establishes a connection to the Neon Postgres database."""
    try:
        conn = psycopg2.connect(NEON_DB_URL)
        return conn
    except Exception as e:
        print(f"Error connecting to Neon Postgres: {e}")
        exit(1)

def create_tables_if_not_exists(conn):
    """Creates necessary tables in Neon Postgres if they don't exist."""
    with conn.cursor() as cur:
        cur.execute("""
            CREATE TABLE IF NOT EXISTS chapters (
                id UUID PRIMARY KEY,
                title VARCHAR(255) NOT NULL,
                file_path VARCHAR(255) UNIQUE NOT NULL,
                content_hash VARCHAR(64) NOT NULL,
                created_at TIMESTAMP DEFAULT NOW(),
                updated_at TIMESTAMP DEFAULT NOW()
            );

            CREATE TABLE IF NOT EXISTS users (
                id UUID PRIMARY KEY,
                username VARCHAR(255) UNIQUE NOT NULL,
                email VARCHAR(255) UNIQUE NOT NULL,
                hashed_password VARCHAR(255) NOT NULL,
                software_experience VARCHAR(50) NOT NULL,
                hardware_knowledge VARCHAR(50) NOT NULL,
                created_at TIMESTAMP DEFAULT NOW(),
                updated_at TIMESTAMP DEFAULT NOW()
            );

            CREATE TABLE IF NOT EXISTS chat_history (
                id UUID PRIMARY KEY,
                user_id UUID REFERENCES users(id),
                query_text TEXT NOT NULL,
                response_text TEXT NOT NULL,
                context_used TEXT,
                timestamp TIMESTAMP DEFAULT NOW()
            );
        """)
        conn.commit()

def create_qdrant_collection_if_not_exists():
    """Creates the Qdrant collection if it doesn't exist."""
    if not qdrant_client.collection_exists(collection_name=COLLECTION_NAME):
        qdrant_client.create_collection(
            collection_name=COLLECTION_NAME,
            vectors_config=models.VectorParams(size=1536, distance=models.Distance.COSINE),
        )
        print(f"Qdrant collection '{COLLECTION_NAME}' created.")
    else:
        print(f"Qdrant collection '{COLLECTION_NAME}' already exists.")

def get_embedding(text: str):
    """Generates an embedding for the given text using OpenAI."""
    response = openai_client.embeddings.create(
        input=text,
        model=EMBEDDING_MODEL
    )
    return response.data[0].embedding

def chunk_text(text: str, chapter_id: uuid.UUID):
    """
    Splits text into chunks, preserving paragraphs.
    Adds metadata for Qdrant payload.
    """
    chunks = []
    paragraphs = [p.strip() for p in text.split('\n\n') if p.strip()]

    for i, paragraph in enumerate(paragraphs):
        chunk_id = uuid.uuid4()
        chunks.append({
            "id": chunk_id,
            "text": paragraph,
            "metadata": {
                "chapter_id": str(chapter_id),
                "chunk_id": str(chunk_id),
                "text": paragraph,
                "chunk_order": i, # Keep track of original order
            }
        })
    return chunks

def ingest_chapter(file_path: str, conn):
    """Reads an MDX chapter, chunks it, generates embeddings, and stores in Qdrant and Neon."""
    try:
        with open(file_path, 'r', encoding='utf-8') as f:
            content = f.read()

        chapter_title = os.path.basename(file_path).replace('.mdx', '').replace('-', ' ').title()
        content_hash = hashlib.sha256(content.encode('utf-8')).hexdigest()

        with conn.cursor() as cur:
            # Check if chapter exists and content hash is the same
            cur.execute("SELECT id, content_hash FROM chapters WHERE file_path = %s", (file_path,))
            result = cur.fetchone()

            chapter_id = None
            if result:
                chapter_id, existing_hash = result
                if existing_hash == content_hash:
                    print(f"Chapter '{chapter_title}' ({file_path}) content unchanged. Skipping.")
                    return
                else:
                    print(f"Chapter '{chapter_title}' ({file_path}) content changed. Updating.")
                    # Delete existing chunks from Qdrant if content changed
                    qdrant_client.delete(
                        collection_name=COLLECTION_NAME,
                        points_selector=models.FilterSelector(
                            filter=models.Filter(
                                must=[models.FieldCondition(
                                    key="chapter_id",
                                    match=models.MatchValue(value=str(chapter_id))
                                )]
                            )
                        )
                    )
                    cur.execute(
                        "UPDATE chapters SET title = %s, content_hash = %s, updated_at = NOW() WHERE id = %s",
                        (chapter_title, content_hash, chapter_id)
                    )
            else:
                chapter_id = uuid.uuid4()
                print(f"New chapter '{chapter_title}' ({file_path}). Inserting.")
                cur.execute(
                    "INSERT INTO chapters (id, title, file_path, content_hash) VALUES (%s, %s, %s, %s)",
                    (chapter_id, chapter_title, file_path, content_hash)
                )
            conn.commit()

        # Chunk text and generate embeddings
        chunks_to_ingest = chunk_text(content, chapter_id)
        points = []
        for chunk in chunks_to_ingest:
            embedding = get_embedding(chunk["text"])
            points.append(
                models.PointStruct(
                    id=str(chunk["id"]),
                    vector=embedding,
                    payload=chunk["metadata"]
                )
            )

        if points:
            qdrant_client.upsert(
                collection_name=COLLECTION_NAME,
                wait=True,
                points=points
            )
            print(f"Successfully ingested {len(points)} chunks for chapter '{chapter_title}'.")

    except Exception as e:
        print(f"Error ingesting chapter {file_path}: {e}")

def main():
    conn = get_db_connection()
    create_tables_if_not_exists(conn)
    create_qdrant_collection_if_not_exists()

    for root, _, files in os.walk(DOCS_DIR):
        for file in files:
            if file.endswith(".mdx"):
                file_path = os.path.join(root, file)
                ingest_chapter(file_path, conn)

    conn.close()
    print("Content ingestion complete.")

if __name__ == "__main__":
    main()
