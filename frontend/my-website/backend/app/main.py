from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from dotenv import load_dotenv
import os

# Load environment variables from .env file
load_dotenv()

app = FastAPI(
    title="Physical AI & Humanoid Robotics Backend",
    description="API for RAG Chatbot, Authentication, and Translation services."
)

# Configure CORS
origins = [
    "http://localhost:3000",  # Docusaurus development server
    os.getenv("FRONTEND_URL", "*") # Frontend URL for production
]

app.add_middleware(
    CORSMiddleware,
    allow_origins=origins,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

@app.get("/", tags=["Root"])
async def read_root():
    return {"message": "Welcome to the Physical AI & Humanoid Robotics Backend!"}

from app.routers import chatbot, auth, translation
from app.services.db_service import create_translation_table_if_not_exists

app.include_router(chatbot.router, prefix="/api/chatbot", tags=["Chatbot"])
app.include_router(auth.router, prefix="/api/auth", tags=["Authentication"])
app.include_router(translation.router, prefix="/api/translate", tags=["Translation"])

@app.on_event("startup")
async def startup_event():
    await create_translation_table_if_not_exists()
