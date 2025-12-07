import os
from openai import OpenAI
from typing import List

# Initialize OpenAI client
client = OpenAI(api_key=os.getenv("OPENAI_API_KEY"))

async def get_embedding(text: str) -> List[float]:
    try:
        response = client.embeddings.create(
            input=text,
            model="text-embedding-ada-002" # Or a newer embedding model if available
        )
        return response.data[0].embedding
    except Exception as e:
        print(f"Error getting embedding: {e}")
        raise

async def get_chat_completion(prompt: str, model: str = "gpt-4o") -> str:
    try:
        response = client.chat.completions.create(
            model=model,
            messages=[
                {"role": "system", "content": "You are a helpful assistant for the Physical AI & Humanoid Robotics textbook. Answer questions based on the provided context."},
                {"role": "user", "content": prompt}
            ]
        )
        return response.choices[0].message.content
    except Exception as e:
        print(f"Error getting chat completion: {e}")
        raise

async def get_translation(text: str, target_language: str = "Urdu") -> str:
    try:
        # Using GPT-4o for translation
        response = client.chat.completions.create(
            model="gpt-4o",
            messages=[
                {"role": "system", "content": f"You are a highly skilled translator. Translate the following text into {target_language}."},
                {"role": "user", "content": text}
            ]
        )
        return response.choices[0].message.content
    except Exception as e:
        print(f"Error getting translation: {e}")
        raise
