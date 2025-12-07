import os
import psycopg2
from typing import Optional

def get_db_connection():
    conn = psycopg2.connect(os.getenv("NEON_DATABASE_URL"))
    return conn

async def create_translation_table_if_not_exists():
    conn = None
    try:
        conn = get_db_connection()
        cur = conn.cursor()
        cur.execute(
            """
            CREATE TABLE IF NOT EXISTS translations (
                id SERIAL PRIMARY KEY,
                original_text TEXT NOT NULL,
                target_language VARCHAR(10) NOT NULL,
                translated_text TEXT NOT NULL,
                created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                UNIQUE (original_text, target_language)
            );
            """
        )
        conn.commit()
        cur.close()
        print("Translations table checked/created successfully.")
    except Exception as e:
        print(f"Error creating translations table: {e}")
    finally:
        if conn:
            conn.close()

async def get_cached_translation(original_text: str, target_language: str) -> Optional[str]:
    conn = None
    try:
        conn = get_db_connection()
        cur = conn.cursor()
        cur.execute(
            "SELECT translated_text FROM translations WHERE original_text = %s AND target_language = %s",
            (original_text, target_language)
        )
        result = cur.fetchone()
        cur.close()
        return result[0] if result else None
    except Exception as e:
        print(f"Error getting cached translation: {e}")
        return None
    finally:
        if conn:
            conn.close()

async def save_translation(original_text: str, target_language: str, translated_text: str):
    conn = None
    try:
        conn = get_db_connection()
        cur = conn.cursor()
        cur.execute(
            "INSERT INTO translations (original_text, target_language, translated_text) VALUES (%s, %s, %s) ON CONFLICT (original_text, target_language) DO NOTHING",
            (original_text, target_language, translated_text)
        )
        conn.commit()
        cur.close()
    except Exception as e:
        print(f"Error saving translation: {e}")
    finally:
        if conn:
            conn.close()
