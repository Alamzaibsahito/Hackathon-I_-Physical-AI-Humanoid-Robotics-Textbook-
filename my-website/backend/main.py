from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware

from api import embed, query, chat

app = FastAPI()

# Configure CORS
origins = [
    "http://localhost:3000",  # Docusaurus default port
    # Add your deployed Docusaurus frontend URL here
]

app.add_middleware(
    CORSMiddleware,
    allow_origins=origins,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

app.include_router(embed.router, prefix="/embed", tags=["embed"])
app.include_router(query.router, prefix="/query", tags=["query"])
app.include_router(chat.router, prefix="/chat", tags=["chat"])

@app.get("/")
async def read_root():
    return {"message": "Physical AI Chatbot Backend"}
