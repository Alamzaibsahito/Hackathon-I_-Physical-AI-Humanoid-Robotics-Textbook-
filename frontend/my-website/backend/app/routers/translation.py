from fastapi import APIRouter, HTTPException
from pydantic import BaseModel
from typing import Optional

from app.services.openai_service import get_translation
from app.services.db_service import get_cached_translation, save_translation

router = APIRouter()

class TranslationRequest(BaseModel):
    text: str
    target_language: str = "ur" # Default to Urdu

class TranslationResponse(BaseModel):
    translated_text: str
    cached: bool = False

@router.post("/translate", response_model=TranslationResponse)
async def translate_text(request: TranslationRequest):
    try:
        # 1. Check if translation is in cache
        cached_text = await get_cached_translation(request.text, request.target_language)
        if cached_text:
            return TranslationResponse(translated_text=cached_text, cached=True)

        # 2. If not, call OpenAI GPT-4o for translation
        translated_text = await get_translation(request.text, request.target_language)

        # 3. Save translation to cache (asynchronously)
        await save_translation(request.text, request.target_language, translated_text)

        return TranslationResponse(translated_text=translated_text, cached=False)
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Translation error: {e}")
