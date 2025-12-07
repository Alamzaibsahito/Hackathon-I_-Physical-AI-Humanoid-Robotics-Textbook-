from fastapi import APIRouter, HTTPException
from pydantic import BaseModel
from typing import List, Optional

from app.services.openai_service import get_embedding, get_chat_completion
from app.services.qdrant_service import search_qdrant

router = APIRouter()

class ChatRequest(BaseModel):
    query: str
    chat_history: Optional[List[dict]] = [] # For conversational context

class ChatResponse(BaseModel):
    response: str
    sources: List[str]

@router.post("/chat", response_model=ChatResponse)
async def chat_with_rag(request: ChatRequest):
    try:
        # 1. Generate embedding for request.query
        query_embedding = await get_embedding(request.query)

        # 2. Search Qdrant for relevant document chunks
        # Assuming a collection name 'book_content' and vector size of 1536 (for text-embedding-ada-002)
        # The actual creation of the collection will happen during content ingestion.
        search_results = await search_qdrant(collection_name="book_content", query_vector=query_embedding, limit=3)

        # Extract content from search results to use as context
        context_parts = [result["content"] for result in search_results]
        sources = list(set([result["source"] for result in search_results])) # Unique sources

        # 3. Construct prompt for OpenAI with context
        context_str = "\n\n".join(context_parts)
        full_prompt = f"""Context from the book:
{context_str}

User query: {request.query}

Based on the context provided, answer the user's query. If the answer is not available in the context, state that you don't have enough information from the book to answer.
"""

        # 4. Use OpenAI to generate a response
        chat_response = await get_chat_completion(full_prompt)

        return ChatResponse(response=chat_response, sources=sources)
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Chatbot error: {e}")
