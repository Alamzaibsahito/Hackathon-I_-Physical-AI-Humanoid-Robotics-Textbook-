import os
from qdrant_client import QdrantClient, models
from typing import List, Dict

# Initialize Qdrant client
def get_qdrant_client():
    return QdrantClient(
        host=os.getenv("QDRANT_HOST"),
        api_key=os.getenv("QDRANT_API_KEY"),
    )

async def create_collection_if_not_exists(collection_name: str, vector_size: int):
    client = get_qdrant_client()
    try:
        client.get_collection(collection_name=collection_name)
        print(f"Collection '{collection_name}' already exists.")
    except Exception:
        client.recreate_collection(
            collection_name=collection_name,
            vectors_config=models.VectorParams(size=vector_size, distance=models.Distance.COSINE),
        )
        print(f"Collection '{collection_name}' created.")

async def upsert_vectors(collection_name: str, ids: List[int], vectors: List[List[float]], payloads: List[Dict]):
    client = get_qdrant_client()
    points = [
        models.PointStruct(id=ids[i], vector=vectors[i], payload=payloads[i])
        for i in range(len(ids))
    ]
    client.upsert(collection_name=collection_name, points=points, wait=True)
    print(f"Upserted {len(ids)} points to collection '{collection_name}'.")

async def search_qdrant(collection_name: str, query_vector: List[float], limit: int = 3) -> List[Dict]:
    client = get_qdrant_client()
    search_result = client.search(
        collection_name=collection_name,
        query_vector=query_vector,
        limit=limit,
        append_payload=True,
    )
    return [{"content": hit.payload["content"], "source": hit.payload.get("source", "unknown")} for hit in search_result]
