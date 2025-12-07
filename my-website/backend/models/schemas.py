from pydantic import BaseModel
from typing import List, Optional

class Document(BaseModel):
    id: Optional[str] = None
    content: str
    metadata: dict = {}

class QueryRequest(BaseModel):
    query: str
    selected_text: Optional[str] = None
    top_k: int = 4

class QueryResponse(BaseModel):
    answer: str
    sources: List[Document] = []

class ChatRequest(BaseModel):
    session_id: str
    message: str
    selected_text: Optional[str] = None

class ChatResponse(BaseModel):
    session_id: str
    response: str
    sources: List[Document] = []

class EmbedRequest(BaseModel):
    docs: List[Document]

class EmbedResponse(BaseModel):
    status: str
    details: Optional[str] = None
