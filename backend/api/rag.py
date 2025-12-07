from fastapi import APIRouter, HTTPException
from pydantic import BaseModel
from typing import List
import os
from openai import OpenAI
from qdrant_client import QdrantClient, models
from dotenv import load_dotenv

load_dotenv()

router = APIRouter()

# Environment variables
OPENAI_API_KEY = os.getenv("OPENAI_API_KEY")
QDRANT_HOST = os.getenv("QDRANT_HOST")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")

if not all([OPENAI_API_KEY, QDRANT_HOST, QDRANT_API_KEY]):
    raise ValueError("Missing one or more environment variables for RAG. Please set OPENAI_API_KEY, QDRANT_HOST, QDRANT_API_KEY.")

# Initialize clients
openai_client = OpenAI(api_key=OPENAI_API_KEY)
qdrant_client = QdrantClient(
    host=QDRANT_HOST,
    api_key=QDRANT_API_KEY,
)

# Constants
COLLECTION_NAME = "textbook_chunks"
EMBEDDING_MODEL = "text-embedding-ada-002"

class RAGQuery(BaseModel):
    question: str
    context: str = None # Optional selected text context

@router.post("/query")
async def rag_query(query: RAGQuery):
    try:
        # 1. Generate embedding for the user's question
        query_embedding_response = openai_client.embeddings.create(
            input=query.question,
            model=EMBEDDING_MODEL
        )
        query_vector = query_embedding_response.data[0].embedding

        # 2. Search Qdrant for relevant chunks
        search_result = qdrant_client.search(
            collection_name=COLLECTION_NAME,
            query_vector=query_vector,
            limit=3, # Retrieve top 3 relevant chunks
            query_filter=None, # Add filters here if needed (e.g., filter by chapter)
            append_vectors=False,
        )

        context_chunks = [hit.payload["text"] for hit in search_result if hit.payload]

        # If optional context is provided by the user (selected text), prioritize it
        if query.context:
            context_chunks.insert(0, query.context) # Add user-selected context at the beginning

        # 3. Construct prompt for OpenAI with context
        full_context = "\n\n".join(context_chunks)
        if not full_context:
            return {"answer": "I cannot find relevant information in the textbook to answer your question.", "source_context": []}

        messages = [
            {"role": "system", "content": "You are an AI assistant specialized in Physical AI and Humanoid Robotics. Answer questions concisely based *only* on the provided textbook content. If the answer is not in the text, state that you cannot answer from the given information."},
            {"role": "user", "content": f"Question: {query.question}\n\nTextbook Content:\n{full_context}\n\nAnswer:"}
        ]

        # 4. Generate answer using OpenAI Chat Completions
        chat_response = openai_client.chat.completions.create(
            model="gpt-3.5-turbo", # Using a chat model for response generation
            messages=messages,
            max_tokens=300,
            temperature=0.7,
        )

        answer = chat_response.choices[0].message.content.strip()
        source_context = [hit.payload["text"] for hit in search_result if hit.payload] # Return text chunks as source

        return {"answer": answer, "source_context": source_context}

    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Internal server error: {e}")
