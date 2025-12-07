from fastapi import FastAPI
from backend.src.api import rag

app = FastAPI(
    title="AI-native Textbook Backend",
    description="FastAPI backend for RAG chatbot, authentication, personalization, and translation.",
    version="0.0.1",
)

app.include_router(rag.router, prefix="/rag", tags=["RAG Chatbot"])

@app.get("/", tags=["Root"])
async def read_root():
    return {"message": "Welcome to the AI-native Textbook Backend!"}
