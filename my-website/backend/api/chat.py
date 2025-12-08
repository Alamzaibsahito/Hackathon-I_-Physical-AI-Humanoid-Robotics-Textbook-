from fastapi import APIRouter, HTTPException, Request
from pydantic import BaseModel
from typing import List, Dict
import os
import re
from pathlib import Path
import json

router = APIRouter()

class ChatRequest(BaseModel):
    message: str
    history: List[Dict[str, str]] = []

# In-memory storage for indexed content
indexed_docs = {}
docs_loaded = False

def load_docs_from_directory(docs_dir: str = "docs") -> Dict[str, str]:
    """Load all markdown files from the docs directory"""
    global indexed_docs
    docs_dir_path = Path(docs_dir)
    docs_content = {}
    
    if not docs_dir_path.exists():
        print(f"Docs directory {docs_dir_path} does not exist!")
        return {}
    
    # Recursively find all markdown files
    for md_file in docs_dir_path.rglob("*.md"):
        try:
            with open(md_file, 'r', encoding='utf-8') as f:
                content = f.read()
                # Use relative path as the key
                relative_path = md_file.relative_to(docs_dir_path)
                docs_content[str(relative_path)] = content
        except Exception as e:
            print(f"Error reading {md_file}: {e}")
    
    return docs_content

def simple_search(query: str, documents: Dict[str, str], top_k: int = 3) -> List[Dict[str, str]]:
    """Simple keyword-based search in the documents"""
    query_lower = query.lower()
    scored_docs = []
    
    for doc_path, content in documents.items():
        # Simple scoring based on keyword matches
        content_lower = content.lower()
        score = 0
        
        # Count matches of query words in content
        query_words = query_lower.split()
        for word in query_words:
            if len(word) > 2:  # Ignore short words
                score += content_lower.count(word)
        
        # Also boost score if title/header exists in content
        header_match = re.search(r'#\s*(.+?)(?:\n|$)', content)
        if header_match:
            header = header_match.group(1).lower()
            for word in query_words:
                if word in header:
                    score *= 2  # Boost for header matches
        
        if score > 0:
            # Extract relevant context around the matches
            content_preview = content[:1000]  # Take first 1000 chars as context
            scored_docs.append({
                "path": doc_path,
                "content": content_preview,
                "score": score
            })
    
    # Sort by score and return top_k
    scored_docs.sort(key=lambda x: x["score"], reverse=True)
    return scored_docs[:top_k]

def generate_answer(query: str, context_docs: List[Dict[str, str]]) -> str:
    """Generate an answer based on the context from documents"""
    if not context_docs:
        return "I couldn't find any relevant information in the documentation to answer your question. Please try rephrasing your question."
    
    # Combine contexts from all relevant documents
    combined_context = ""
    sources = []
    
    for doc in context_docs:
        combined_context += doc["content"] + "\n\n"
        sources.append(doc["path"])
    
    # Simple response generation based on the context
    # In a real scenario, you might use a more sophisticated LLM
    # But for simplicity, we'll just return relevant info with sources
    
    # Find the most relevant parts of the content related to the query
    query_terms = query.lower().split()
    relevant_parts = []
    
    for doc in context_docs:
        content = doc["content"]
        best_match = ""
        best_score = 0
        
        # Split content into paragraphs and score each
        paragraphs = re.split(r'\n\s*\n', content)
        for para in paragraphs:
            if len(para.strip()) < 20:  # Skip very short paragraphs
                continue
                
            score = 0
            para_lower = para.lower()
            for term in query_terms:
                if len(term) > 2:  # Only consider words longer than 2 chars
                    score += para_lower.count(term)
            
            if score > best_score:
                best_score = score
                best_match = para.strip()
        
        if best_match:
            relevant_parts.append(best_match)
    
    if relevant_parts:
        answer = "Based on the documentation:\n\n"
        for i, part in enumerate(relevant_parts[:2]):  # Limit to 2 most relevant parts
            answer += f"{part}\n\n"
        
        if len(sources) == 1:
            answer += f"\nSource: {sources[0]}"
        else:
            answer += f"\nSources: {', '.join(sources)}"
    else:
        answer = "I found information in the documentation but couldn't extract a specific answer. Please refer to the documentation sections mentioned in the sources."
        answer += f"\n\nRelevant sections: {sources[0] if sources else 'Unknown'}"
    
    return answer.strip()

@router.on_event("startup")
async def startup_event():
    """Load docs when the app starts up"""
    global docs_loaded
    global indexed_docs
    
    docs_dir = os.getenv("DOCS_DIR", "./docs")
    print(f"Loading documentation from: {docs_dir}")
    
    indexed_docs = load_docs_from_directory(docs_dir)
    
    if indexed_docs:
        print(f"Loaded {len(indexed_docs)} documents")
        docs_loaded = True
    else:
        print("No documents loaded - check docs directory!")

@router.post("/chat/query")
async def chat_query(request: ChatRequest):
    global indexed_docs
    global docs_loaded
    
    if not docs_loaded or not indexed_docs:
        # Try to reload if not loaded
        docs_dir = os.getenv("DOCS_DIR", "./docs")
        indexed_docs = load_docs_from_directory(docs_dir)
        if indexed_docs:
            docs_loaded = True
        else:
            return {
                "response": "Chatbot is not yet ready. Documentation has not been loaded.",
                "sources": []
            }
    
    try:
        # Perform search in docs
        relevant_docs = simple_search(request.message, indexed_docs, top_k=3)
        
        # Generate answer based on relevant docs
        answer = generate_answer(request.message, relevant_docs)
        
        # Extract sources
        sources = [doc["path"] for doc in relevant_docs]
        
        return {
            "response": answer,
            "sources": sources
        }
    except Exception as e:
        print(f"Error processing query: {e}")
        return {
            "response": "Sorry, I encountered an error processing your query. Please try again.",
            "sources": []
        }

@router.get("/chat/health")
async def chat_health():
    """Health check endpoint"""
    return {
        "status": "healthy",
        "docs_loaded": docs_loaded,
        "docs_count": len(indexed_docs) if docs_loaded else 0
    }